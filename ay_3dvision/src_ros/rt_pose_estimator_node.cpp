//-------------------------------------------------------------------------------------------
/*! \file    rt_pose_estimator_node.cpp
    \brief   Pose estimation using ray tracing
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Feb.10, 2015
*/
//-------------------------------------------------------------------------------------------
#include "ay_3dvision/pcl_util.h"
#include "ay_vision/rt_pose_estimator.h"
#include "ay_vision/vision_util.h"
#include "ay_cpp/cpp_util.h"
#include "ay_cpp/sys_util.h"
//-------------------------------------------------------------------------------------------
#include "ay_3dvision_msgs/CreateScene.h"
#include "ay_3dvision_msgs/RemoveScene.h"
#include "ay_3dvision_msgs/RTLabeledPoseOpt.h"
#include "ay_3dvision_msgs/LabeledPose.h"
#include "ay_3dvision_msgs/LabeledPoseOptReq.h"
#include "ay_3dvision_msgs/LabeledPoseRevision.h"
//-------------------------------------------------------------------------------------------
#include <string>
#include <map>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>  // filters
#include <ros/ros.h>
//-------------------------------------------------------------------------------------------
namespace trick
{

ros::Publisher PubLabeledPoseRevision;
// Camera model:
Imager::TCameraInfo CameraInfo;
// Map from a scene name to a pose estimator
std::map<std::string, TRayTracePoseEstimator> RayTracePoseEstimators;
// Map from a label to (scene name, object index)
std::map<std::string, std::pair<std::string, int> > LabelToSceneIdx;
// Current selection:
std::map<std::string, TRayTracePoseEstimator>::iterator CurrRTPoseEstimator;
int TargetObject(0);

cv::Mat DepthImg, NormalImg;
std_msgs::Header PointCloudHeader;
int Sequence(0);

bool NoRGBInPoints(false);
bool RemovePlaneFromCloud(false);
int NormalCalcStep(7);
int OptRenderStepX(7), OptRenderStepY(7);
// int OptRenderStepX(15), OptRenderStepY(15);
double DepthDiffThresh(0.2);
double NormalDiffThresh(0.4);
double NodataDiffDepth(0.2);
double NodataDiffNormal(0.4);
double ThGoodDepthRatio (0.3);
double ThBadDepthRatio  (0.7);
double ThGoodNormalRatio(0.1);
double ThBadNormalRatio (0.8);

double ResizeRatio(1.0);
int DisplayMode(0);  // 0: 0.5*original+render, 1: 0.25*original+render, 2: render, 3: original, 4: camera_info error.
TEasyVideoOut VideoOutD, VideoOutN;  // Video writer for depth and normal

}
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
using namespace trick;
//-------------------------------------------------------------------------------------------
// #define print(var) PrintContainer((var), #var"= ")
// #define print(var) std::cout<<#var"= "<<(var)<<std::endl
//-------------------------------------------------------------------------------------------

bool RemoveScene(const std::string &scene)
{
  std::map<std::string, TRayTracePoseEstimator>::iterator itr= RayTracePoseEstimators.find(scene);
  if(itr!=RayTracePoseEstimators.end())
  {
    RayTracePoseEstimators.erase(itr);
    CurrRTPoseEstimator= RayTracePoseEstimators.begin();
    return true;
  }
  return false;
}
//-------------------------------------------------------------------------------------------

bool CreateScene(ay_3dvision_msgs::CreateScene::Request &req, ay_3dvision_msgs::CreateScene::Response &res)
{
  /*
  req.name
  req.models[]
    label
    primitives[]
      kind
      param
      pose
    initial_pose
  */
  std::cerr<<"Making scene: "<<req.name<<std::endl;
  RemoveScene(req.name);
  RayTracePoseEstimators[req.name]= TRayTracePoseEstimator();
  for(int i_model(0),i_model_end(req.models.size()); i_model<i_model_end; ++i_model)
  {
    const ay_3dvision_msgs::RayTraceModel &model(req.models[i_model]);

    double pose[]= {0.0,0.0,0.0, 0.0,0.0,0.0,1.0};
    GPoseToX(model.initial_pose, pose);
    if(pose[3]==0.0&&pose[4]==0.0&&pose[5]==0.0&&pose[6]==0.0)  pose[6]= 1.0;

    std::cerr<<"  adding: "<<model.label<<" at ["; for(int d(0);d<7;++d)std::cerr<<(d==0?"":", ")<<pose[d]; std::cerr<<"]"<<std::endl;
    TRayTraceModel object;
    for(int i_prim(0),i_prim_end(model.primitives.size()); i_prim<i_prim_end; ++i_prim)
    {
      const ay_3dvision_msgs::RayTracePrimitive &prim(model.primitives[i_prim]);
      TRayTraceModel::TPrimitive primitive;
      primitive.Kind= StrToRTPrimitiveKind(prim.kind);
      for(int i_param(0),i_param_end(prim.param.size()); i_param<i_param_end; ++i_param)
        primitive.Param[i_param]= prim.param[i_param];
      GPoseToX(prim.pose, primitive.Pose);
      object.Primitives.push_back(primitive);
    }  // for each i_prim
    int idx= RayTracePoseEstimators[req.name].AddObject(object,pose);
    LabelToSceneIdx[model.label]= std::pair<std::string, int>(req.name,idx);
  }  // for each i_model

  RayTracePoseEstimators[req.name].SetCameraInfo(CameraInfo);
  RayTracePoseEstimators[req.name].SetSqDiffDepthThresh(Sq(DepthDiffThresh));
  RayTracePoseEstimators[req.name].SetSqDiffNormalThresh(Sq(NormalDiffThresh));
  RayTracePoseEstimators[req.name].SetNodataSqDiffDepth(Sq(NodataDiffDepth));
  RayTracePoseEstimators[req.name].SetNodataSqDiffNormal(Sq(NodataDiffNormal));
  RayTracePoseEstimators[req.name].SetThGoodDepthRatio (ThGoodDepthRatio );
  RayTracePoseEstimators[req.name].SetThBadDepthRatio  (ThBadDepthRatio  );
  RayTracePoseEstimators[req.name].SetThGoodNormalRatio(ThGoodNormalRatio);
  RayTracePoseEstimators[req.name].SetThBadNormalRatio (ThBadNormalRatio );

  CurrRTPoseEstimator= RayTracePoseEstimators.begin();
  return true;
}
//-------------------------------------------------------------------------------------------

bool RemoveScene(ay_3dvision_msgs::RemoveScene::Request &req, ay_3dvision_msgs::RemoveScene::Response &res)
{
  if(req.name=="")
  {
    RayTracePoseEstimators.clear();
    CurrRTPoseEstimator= RayTracePoseEstimators.begin();
    LabelToSceneIdx.clear();
    std::cerr<<"Removed: all scenes"<<std::endl;
  }
  else
  {
    RemoveScene(req.name);
    std::cerr<<"Removed: "<<req.name<<std::endl;
    // FIXME: We need to remove elements of LabelToSceneIdx whose scene name is req.name
    std::cerr<<"FIXME: need to remove elements of LabelToSceneIdx whose scene name is "<<req.name<<std::endl;
  }
  return true;
}
//-------------------------------------------------------------------------------------------

void CallbackPointCloud(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud= ConvertROSMsgToPointCloud(msg, NoRGBInPoints);

  if(RemovePlaneFromCloud)
    AssignInfToPlane(cloud,
        /*ransac_dist_thresh=*/0.01,
        /*ransac_max_iterations=*/50);

  cv::Mat rgb_img, depth_img, normal_img;
  ConvertPointCloudToRGBDImages(cloud, rgb_img, depth_img);
  ConvertPointCloudToNormalImage(cloud, normal_img, /*FS=*/NormalCalcStep);

  // cv::medianBlur(normal_img, normal_img, 5);
  // cv::medianBlur(normal_img, normal_img, 5);
  // normal_img*= 200.0;
  // normal_img.convertTo(normal_img,CV_8UC3);
  // cv::medianBlur(normal_img, normal_img, 9);
  // cv::Mat normal_img2;
  // cv::bilateralFilter(normal_img, normal_img2, 21, 21*2, 21/2);
  // normal_img= normal_img2;
  // cv::blur(normal_img, normal_img, cv::Size(11, 21), cv::Point(-1,-1));

  // normal_img*= 255.0;
  // normal_img.convertTo(normal_img,CV_8UC3);

  // depth_img*= 255.0;
  // depth_img.convertTo(depth_img,CV_8UC1);

  depth_img.copyTo(DepthImg);
  normal_img.copyTo(NormalImg);
  PointCloudHeader= msg->header;

  char c(cv::waitKey(10));
  if(c=='\x1b'||c=='q')  ros::shutdown();
  else if(CurrRTPoseEstimator!=RayTracePoseEstimators.end()
      && CurrRTPoseEstimator->second.HandleKeyEvent(TargetObject,c))
  {
    TRayTracePoseEstimator::TEvalDescription eval_desc, ed0;
    CurrRTPoseEstimator->second.GetEvalDescription(TargetObject, depth_img, normal_img,
        eval_desc, OptRenderStepX, OptRenderStepY, /*f_depth_normalize=*/CurrRTPoseEstimator->second.Pose(TargetObject).X[2]);
    CurrRTPoseEstimator->second.CompareEvalDescriptions(eval_desc,ed0,1.0,1.0);
    std::cerr<<"Quality,RMatchedDepth,RMatchedNormal,SqDiffDepth,SqDiffNormal= "
        <<eval_desc.Quality<<", "<<eval_desc.RMatchedDepth<<", "<<eval_desc.RMatchedNormal<<", "<<eval_desc.SqDiffDepth<<", "<<eval_desc.SqDiffNormal<<std::endl;
  }
  else if(CurrRTPoseEstimator!=RayTracePoseEstimators.end()
      && c==' ')  CurrRTPoseEstimator->second.OptimizeXYZ(TargetObject, depth_img, normal_img, OptRenderStepX, OptRenderStepY);
  else if(c=='0' || c=='1' || c=='2' || c=='3')
  {
    switch(c)
    {
    case '0': TargetObject= 0; break;
    case '1': TargetObject= 1; break;
    case '2': TargetObject= 2; break;
    case '3': TargetObject= 3; break;
    }
    std::cerr<<"Selected: "<<TargetObject<<std::endl;
  }
  else if(c=='>')
  {
    if(RayTracePoseEstimators.size()>0)
    {
      ++CurrRTPoseEstimator;
      if(CurrRTPoseEstimator==RayTracePoseEstimators.end())
        CurrRTPoseEstimator= RayTracePoseEstimators.begin();
    }
  }
  else if(c=='<')
  {
    if(RayTracePoseEstimators.size()>0)
    {
      if(CurrRTPoseEstimator==RayTracePoseEstimators.begin())
        CurrRTPoseEstimator= RayTracePoseEstimators.end();
      --CurrRTPoseEstimator;
    }
  }
  else if(c=='[')
  {
    if(ResizeRatio<=1.0)  ResizeRatio= 0.5;
    else  ResizeRatio-= 1.0;
    std::cerr<<"ResizeRatio: "<<ResizeRatio<<std::endl;
  }
  else if(c==']')
  {
    if(ResizeRatio<1.0)  ResizeRatio= 1.0;
    else  ResizeRatio+= 1.0;
    std::cerr<<"ResizeRatio: "<<ResizeRatio<<std::endl;
  }
  else if(c=='m')
  {
    ++DisplayMode;
    if(DisplayMode>4)  DisplayMode= 0;
    std::cerr<<"DisplayMode: "<<DisplayMode<<std::endl;
  }
  else if(c=='W')  // Video record/stop
  {
    VideoOutD.Switch();
    VideoOutN.Switch();
  }
  else if(c=='c')  // Calibration from point cloud
  {
    std::vector<double> cam_proj;
    cam_proj= GetCameraProjFromPointCloud(cloud);
    std::cerr<<"Done: GetCameraProjFromPointCloud"<<std::endl;
    std::cerr<<"Original Fx,Fy,Cx,Cy: "
        <<CameraInfo.Fx<<", "<<CameraInfo.Fy<<", "
        <<CameraInfo.Cx<<", "<<CameraInfo.Cy<<std::endl;
    std::cerr<<"Obtained Fx,Fy,Cx,Cy: "
        <<cam_proj[0]<<", "<<cam_proj[5]<<", "
        <<cam_proj[2]<<", "<<cam_proj[6]<<std::endl;
  }
  else if(c=='d')  // Print debug info
  {
    std::cerr<<"Camera info - Fx,Fy,Cx,Cy: "
        <<CameraInfo.Fx<<", "<<CameraInfo.Fy<<", "
        <<CameraInfo.Cx<<", "<<CameraInfo.Cy<<std::endl;

    int w(cloud->width), h(cloud->height);
    std::cerr<<"Sensor size (w x h): "<<w<<" x "<<h<<std::endl;
    const char* pickup_lab[5]= {"L","R","U","D","C"};
    int pickup[5][2]= {{w/4,h/2}, {(w*3)/4,h/2}, {w/2,h/4}, {w/2,(h*3)/4}, {w/2,h/2}};
    for(int i(0); i<5; ++i)
    {
      const pcl::PointXYZRGB &pt(cloud->at(pickup[i][0],pickup[i][1]));
      std::cerr<<pickup_lab[i]<<"("<<pickup[i][0]<<", "<<pickup[i][1]<<") - x,y,z: "<<pt.x<<", "<<pt.y<<", "<<pt.z<<std::endl;
    }
  }

  // Convert depth image for good look:
  cv::normalize(1.0/depth_img, depth_img, 0, 1.0, CV_MINMAX, CV_32FC1);

  // Rendering:
  // 0: 0.5*original+render, 1: 0.25*original+render, 2: render, 3: original, 4: camera_info error.
  if(DisplayMode==0 && RayTracePoseEstimators.size()>0)
  {
    depth_img*= 0.5;
    normal_img*= 0.5;
  }
  else if(DisplayMode==1 && RayTracePoseEstimators.size()>0)
  {
    depth_img*= 0.25;
    normal_img*= 0.25;
  }
  else if(DisplayMode==2)
  {
    depth_img.setTo(0.0);
    normal_img.setTo(cv::Scalar(0.0,0.0,0.0));
  }
  if(DisplayMode!=3 && DisplayMode!=4)
  {
    for(std::map<std::string, TRayTracePoseEstimator>::iterator
          itr(RayTracePoseEstimators.begin()),itr_end(RayTracePoseEstimators.end());
            itr!=itr_end; ++itr)
    {
      Imager::TROI2D<int> roi;
      if(!itr->second.GetImageROI(roi))  continue;
      cv::rectangle(depth_img,cv::Point(roi.Min[0],roi.Min[1]), cv::Point(roi.Max[0],roi.Max[1]), cv::Scalar(255), 1, 8, 0);
      cv::rectangle(normal_img,cv::Point(roi.Min[0],roi.Min[1]), cv::Point(roi.Max[0],roi.Max[1]), cv::Scalar(255,255,255), 1, 8, 0);
std::cerr<<"DEBUG-1:"<<roi.Min[0]<<","<<roi.Min[1]<<",  "<<roi.Max[0]<<","<<roi.Max[1]<<std::endl;
      itr->second.Render(depth_img, normal_img);
std::cerr<<"DEBUG-2:"<<std::endl;
    }
  }
  if(DisplayMode==4)
  {
    CameraProjErrorImgFromCloud(cloud, normal_img,
        CameraInfo.Fx, CameraInfo.Fy, CameraInfo.Cx, CameraInfo.Cy);
    normal_img*= 100.0;
  }

  VideoOutD.Step(depth_img);
  VideoOutN.Step(normal_img);
  if(ResizeRatio!=1.0)
  {
    cv::resize(depth_img, depth_img, cv::Size(0,0), ResizeRatio,ResizeRatio);
    cv::resize(normal_img, normal_img, cv::Size(0,0), ResizeRatio,ResizeRatio);
  }
  DrawCrossOnCenter(depth_img, 20, cv::Scalar(255,255,255));
  DrawCrossOnCenter(normal_img, 20, cv::Scalar(255,255,255));
  VideoOutD.VizRec(depth_img);
  VideoOutN.VizRec(normal_img);
  // cv::imshow("rgb", rgb_img);
  cv::imshow("depth", depth_img);
  cv::imshow("normal", normal_img);
}
//-------------------------------------------------------------------------------------------

void ExecLabeledPoseOptReq(
    const ay_3dvision_msgs::LabeledPoseOptReq &msg,
    ay_3dvision_msgs::LabeledPoseRevision &msg_pose_revision)
{
  // Read the message:
  std::pair<std::string, int> scene_idx= LabelToSceneIdx[msg.lpose.label];
  double pose[7];
  GPoseToX(msg.lpose.pose, pose);

  double pose_revised[7];
  for(int d(0); d<7; ++d)  pose_revised[d]= pose[d];
  TRayTracePoseEstimator::TEvalDescription eval_desc_revised;

  std::cerr<<"Revising "<<msg.lpose.label<<" in "<<scene_idx.first<<":"<<std::endl;
  int n_stage(msg.stages.size()), n_succeeded_stage(0);
  for(int st(0); st<n_stage; ++st)
  {
    const ay_3dvision_msgs::RayTraceOptReq1 &stage(msg.stages[st]);
    if(stage.type=="xyz_auto")
    {
      RayTracePoseEstimators[scene_idx.first].SetPose(scene_idx.second, pose);
      RayTracePoseEstimators[scene_idx.first].OptimizeXYZ(
          scene_idx.second, DepthImg, NormalImg, OptRenderStepX, OptRenderStepY,
          pose_revised, &eval_desc_revised);
    }
    else if(stage.type=="lin1d")
    {
      RayTracePoseEstimators[scene_idx.first].SetPose(scene_idx.second, pose);
      double axis_1[3];
      GPointToP(stage.axis1, axis_1);
      // Improve pose by ray tracing pose estimation:
      RayTracePoseEstimators[scene_idx.first].OptimizeLin1D(
          scene_idx.second, DepthImg, NormalImg, OptRenderStepX, OptRenderStepY,
          axis_1,
          stage.range[0], stage.num_div,
          stage.weight_depth, stage.weight_normal,
          /*opt_1=*/NULL, pose_revised, &eval_desc_revised);
    }
    else if(stage.type=="lin2d")
    {
      RayTracePoseEstimators[scene_idx.first].SetPose(scene_idx.second, pose);
      double axis_1[3], axis_2[3];
      GPointToP(stage.axis1, axis_1);
      GPointToP(stage.axis2, axis_2);
      // Improve pose by ray tracing pose estimation:
      RayTracePoseEstimators[scene_idx.first].OptimizeLin2D(
          scene_idx.second, DepthImg, NormalImg, OptRenderStepX, OptRenderStepY,
          axis_1, axis_2,
          stage.range[0], stage.range[1], stage.num_div,
          stage.weight_depth, stage.weight_normal,
          /*opt_12=*/NULL, pose_revised, &eval_desc_revised);
    }
    else if(stage.type=="rot1d")
    {
      RayTracePoseEstimators[scene_idx.first].SetPose(scene_idx.second, pose);
      double axis_1[3];
      GPointToP(stage.axis1, axis_1);
      // Improve pose by ray tracing pose estimation:
      RayTracePoseEstimators[scene_idx.first].OptimizeRot1D(
          scene_idx.second, DepthImg, NormalImg, OptRenderStepX, OptRenderStepY,
          axis_1,
          stage.range[0], stage.num_div,
          stage.weight_depth, stage.weight_normal,
          /*opt_1=*/NULL, pose_revised+3, &eval_desc_revised);
    }
    else if(stage.type=="rot2d")
    {
      RayTracePoseEstimators[scene_idx.first].SetPose(scene_idx.second, pose);
      double axis_1[3], axis_2[3];
      GPointToP(stage.axis1, axis_1);
      GPointToP(stage.axis2, axis_2);
      // Improve pose by ray tracing pose estimation:
      RayTracePoseEstimators[scene_idx.first].OptimizeRot2D(
          scene_idx.second, DepthImg, NormalImg, OptRenderStepX, OptRenderStepY,
          axis_1, axis_2,
          stage.range[0], stage.range[1], stage.num_div,
          stage.weight_depth, stage.weight_normal,
          /*opt_12=*/NULL, pose_revised+3, &eval_desc_revised);
    }
    else
    {
      std::cerr<<"  ERROR-unknown type: "<<stage.type<<std::endl;
      return;
    }

    // Check if the eval_desc_revised satisfies the threshold
    TRayTracePoseEstimator::TEvalDescription eval_desc_threshold;
    eval_desc_threshold.Quality= stage.th_quality;
    eval_desc_threshold.SqDiffDepth= Sq(stage.th_depth_diff);
    eval_desc_threshold.SqDiffNormal= Sq(stage.th_normal_diff);
    bool is_satisfied=
        RayTracePoseEstimators[scene_idx.first].CheckEvalDescriptions(
            eval_desc_revised, eval_desc_threshold,
            stage.weight_depth, stage.weight_normal);

    std::cerr<<"  Revised with "<<stage.type<<":";
    for(int d(0); d<7; ++d)  std::cerr<<" "<<pose[d]-pose_revised[d];
    std::cerr<<std::endl;
    std::cerr<<"  # Quality,RMatchedDepth,RMatchedNormal,SqDiffDepth,SqDiffNormal,is_satisfied= "<<std::endl
        <<"    "<<eval_desc_revised.Quality
        <<", "<<eval_desc_revised.RMatchedDepth
        <<", "<<eval_desc_revised.RMatchedNormal
        <<", "<<eval_desc_revised.SqDiffDepth
        <<", "<<eval_desc_revised.SqDiffNormal
        <<" : "<<is_satisfied<<std::endl;

    // TEST: Stop if eval_desc_revised does not satisfy a threshold:
    if(!is_satisfied)  break;
    else               ++n_succeeded_stage;

    // Copy a revised pose:
    for(int d(0); d<7; ++d)  pose[d]= pose_revised[d];

  }  // for st in n_stage

  // Assign result to the revised pose message:
  msg_pose_revision.lpose.header.seq= Sequence;  ++Sequence;
  msg_pose_revision.lpose.header.stamp= PointCloudHeader.stamp;
  msg_pose_revision.lpose.header.frame_id= PointCloudHeader.frame_id;
  msg_pose_revision.lpose.label= msg.lpose.label;
  XToGPose(pose, msg_pose_revision.lpose.pose);
  msg_pose_revision.errors.resize(5);
  msg_pose_revision.errors[0]= eval_desc_revised.SqDiffDepth;
  msg_pose_revision.errors[1]= eval_desc_revised.SqDiffNormal;
  msg_pose_revision.errors[2]= eval_desc_revised.Quality;
  msg_pose_revision.errors[3]= eval_desc_revised.RMatchedDepth;
  msg_pose_revision.errors[4]= eval_desc_revised.RMatchedNormal;
  if(n_succeeded_stage==n_stage)  msg_pose_revision.revision_status= 2;
  else if(n_succeeded_stage==0)   msg_pose_revision.revision_status= 0;
  else                            msg_pose_revision.revision_status= 1;
}
//-------------------------------------------------------------------------------------------

void CallbackLabeledPose(const ay_3dvision_msgs::LabeledPoseConstPtr &msg)
{
  ay_3dvision_msgs::LabeledPoseOptReq msg_opt_req;
  msg_opt_req.lpose= *msg;
  msg_opt_req.stages.resize(1);
  msg_opt_req.stages[0].type= "xyz_auto";

  ay_3dvision_msgs::LabeledPoseRevision msg_pose_revision;
  ExecLabeledPoseOptReq(msg_opt_req, msg_pose_revision);
  PubLabeledPoseRevision.publish(msg_pose_revision);
}
//-------------------------------------------------------------------------------------------

void CallbackLabeledPoseOptReq(const ay_3dvision_msgs::LabeledPoseOptReqConstPtr &msg)
{
  ay_3dvision_msgs::LabeledPoseRevision msg_pose_revision;
  ExecLabeledPoseOptReq(*msg, msg_pose_revision);
  PubLabeledPoseRevision.publish(msg_pose_revision);
}
//-------------------------------------------------------------------------------------------

bool RTLabeledPoseOpt(ay_3dvision_msgs::RTLabeledPoseOpt::Request &req, ay_3dvision_msgs::RTLabeledPoseOpt::Response &res)
{
  ExecLabeledPoseOptReq(req.req, res.res);
}
//-------------------------------------------------------------------------------------------


int main(int argc, char**argv)
{
  ros::init(argc, argv, "rt_pose_estimator");
  ros::NodeHandle node("~");

  std::string vout_base("/tmp/rt");

  std::string point_cloud_in;
  node.param("in_points", point_cloud_in, std::string("/camera/depth/points_xyzrgb"));
  std::string labeled_pose_in, labeled_pose_optreq_in, labeled_pose_out;
  node.param("in_pose", labeled_pose_in, std::string("labeled_pose"));
  node.param("in_poseoptreq", labeled_pose_optreq_in, std::string("labeled_pose_optreq"));
  node.param("out_pose", labeled_pose_out, std::string("labeled_pose_revision"));

  node.param("no_rgb_in_points", NoRGBInPoints, NoRGBInPoints);
  node.param("remove_plane_from_cloud", RemovePlaneFromCloud, RemovePlaneFromCloud);

  node.param("normal_calc_step", NormalCalcStep, NormalCalcStep);
  node.param("opt_render_dx", OptRenderStepX, OptRenderStepX);
  node.param("opt_render_dy", OptRenderStepY, OptRenderStepY);
  node.param("depth_diff_thresh", DepthDiffThresh, DepthDiffThresh);
  node.param("normal_diff_thresh", NormalDiffThresh, NormalDiffThresh);
  node.param("nodata_diff_depth", NodataDiffDepth, NodataDiffDepth);
  node.param("nodata_diff_normal", NodataDiffNormal, NodataDiffNormal);
  node.param("th_good_depth_ratio" , ThGoodDepthRatio , ThGoodDepthRatio );
  node.param("th_bad_depth_ratio"  , ThBadDepthRatio  , ThBadDepthRatio  );
  node.param("th_good_normal_ratio", ThGoodNormalRatio, ThGoodNormalRatio);
  node.param("th_bad_normal_ratio" , ThBadNormalRatio , ThBadNormalRatio );

  node.param("resize_ratio", ResizeRatio, ResizeRatio);
  node.param("display_mode", DisplayMode, DisplayMode);
  node.param("vout_base",vout_base,vout_base);

  int cam_width(640), cam_height(480);
  std::vector<double> cam_proj, cam_proj_default(12,0.0);  // 3x4 projection/camera matrix
  // TODO: These values should be taken from a topic like
  // /camera/depth/camera_info (Use height, width, and P)
  // P: [540.916992, 0.0, 317.022348, 0.0, 0.0, 542.752869, 230.86987, 0.0, 0.0, 0.0, 1.0, 0.0]
  {double tmp[]={540.916992, 0.0, 317.022348, 0.0, 0.0, 542.752869, 230.86987, 0.0, 0.0, 0.0, 1.0, 0.0};
  for(int i(0);i<12;++i) cam_proj_default[i]= tmp[i];}
  node.param("cam_width", cam_width, 640);
  node.param("cam_height", cam_height, 480);
  node.param("cam_proj", cam_proj, cam_proj_default);

  // cv::namedWindow("rgb",1);
  cv::namedWindow("depth",1);
  cv::namedWindow("normal",1);

  VideoOutD.SetfilePrefix(vout_base+"_depth");
  VideoOutN.SetfilePrefix(vout_base+"_normal");

  /*############Setup ray tracing############*/
  // Create object
  using namespace Imager;

  // Create camera model
  //   TODO: These values should be taken from a topic like
  //   /camera/depth/camera_info (Use height, width, and P)
  CameraInfo.Width= cam_width;
  CameraInfo.Height= cam_height;
  CameraInfo.Fx= cam_proj[0];
  CameraInfo.Fy= cam_proj[5];
  CameraInfo.Cx= cam_proj[2];
  CameraInfo.Cy= cam_proj[6];
  CurrRTPoseEstimator= RayTracePoseEstimators.begin();

  #if 0  // For DEBUG
  RayTracePoseEstimators["test"]= TRayTracePoseEstimator();
  TRayTraceModel container;
  #if 0  // Coke can test (b54)
  TRayTraceModel::TPrimitive container_1;
  container_1.Kind= rtpkCylinder;
  container_1.Param[0]= 0.033;
  container_1.Param[1]= 0.12;
  for(int d(0);d<7;++d)  container_1.Pose[d]= 0.0;
  container_1.Pose[2]= 0.06;
  container_1.Pose[6]= 1.0;
  container.Primitives.push_back(container_1);
  #endif
  #if 1  // Mag cup test (b99)
  TRayTraceModel::TPrimitive container_1;
  container_1.Kind= rtpkTube;
  container_1.Param[0]= 0.04;
  container_1.Param[1]= 0.037;
  container_1.Param[2]= 0.10;
  container_1.Param[3]= 0.0;
  container_1.Param[4]= 0.0;
  for(int d(0);d<7;++d)  container_1.Pose[d]= 0.0;
  container_1.Pose[2]= 0.05;
  container_1.Pose[6]= 1.0;
  container.Primitives.push_back(container_1);
  TRayTraceModel::TPrimitive container_2;
  container_2.Kind= rtpkCylinder;
  container_2.Param[0]= 0.04;
  container_2.Param[1]= 0.01;
  for(int d(0);d<7;++d)  container_2.Pose[d]= 0.0;
  container_2.Pose[2]= 0.005;
  container_2.Pose[6]= 1.0;
  container.Primitives.push_back(container_2);
  #endif
  double pose[]= {0.0461,0.1142,0.6272, 0.0,0.0,0.0,1.0};
  int idx= RayTracePoseEstimators["test"].AddObject(container,pose);
#if 0  // Add second object:
container.Primitives.clear();
TRayTraceModel::TPrimitive container_3;
container_3.Kind= rtpkCylinder;
container_3.Param[0]= 0.033;
container_3.Param[1]= 0.12;
for(int d(0);d<7;++d)  container_3.Pose[d]= 0.0;
container_3.Pose[2]= 0.06;
container_3.Pose[6]= 1.0;
container.Primitives.push_back(container_3);
RayTracePoseEstimators["test"].AddObject(container,pose);
#endif
  // LabelToSceneIdx["b99"]= std::pair<std::string, int>("test",idx);  // Mag cup test
  LabelToSceneIdx["b54"]= std::pair<std::string, int>("test",idx);  // Coke can test
  // cylinder->Move();
  // cylinder->RotateX(90.0);
  // cylinder->RotateY(0.0);
  // cylinder->RotateZ(0.0);
  RayTracePoseEstimators["test"].SetCameraInfo(CameraInfo);
  CurrRTPoseEstimator= RayTracePoseEstimators.begin();
  #endif  // TEST

  /*############Done############*/


  PubLabeledPoseRevision= node.advertise<ay_3dvision_msgs::LabeledPoseRevision>(labeled_pose_out, 1);
  ros::ServiceServer srv_create= node.advertiseService("create_scene", &CreateScene);
  ros::ServiceServer srv_remove= node.advertiseService("remove_scene", &RemoveScene);
  ros::ServiceServer srv_lpose_opt= node.advertiseService("lpose_opt", &RTLabeledPoseOpt);

  ros::Subscriber sub_point_cloud= node.subscribe(point_cloud_in, 1, &CallbackPointCloud);
  ros::Subscriber sub_labeled_pose= node.subscribe(labeled_pose_in, 1, &CallbackLabeledPose);
  ros::Subscriber sub_labeled_pose_optreq= node.subscribe(labeled_pose_optreq_in, 1, &CallbackLabeledPoseOptReq);
  ros::spin();

  return 0;
}
//-------------------------------------------------------------------------------------------
