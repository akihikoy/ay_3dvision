//-------------------------------------------------------------------------------------------
/*! \file    usb_stereo_node.cpp
    \brief   USB stereo integration with color, flow, and edge detections.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Apr.04, 2016

    cf. ay_vision/color_detector_node.cpp
*/
//-------------------------------------------------------------------------------------------
#include "ay_3dvision/pcl_util.h"
#include "ay_vision/color_detector.h"
#include "ay_vision/flow_finder.h"
#include "ay_vision/usb_stereo.h"
#include "ay_vision/edge_fit.h"
#include "ay_vision/vision_util.h"
#include "ay_cpp/geom_util.h"
//-------------------------------------------------------------------------------------------
#include "ay_3dvision_msgs/ROI_3DProj.h"
#include "ay_3dvision_msgs/FitEdge.h"
#include "ay_vision_msgs/ColDetSensor.h"
#include "ay_vision_msgs/ColDetViz.h"
#include "ay_vision_msgs/ColDetVizPrimitive.h"
//-------------------------------------------------------------------------------------------
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
//-------------------------------------------------------------------------------------------
namespace trick
{
// Get point cloud of flow by flow-stereo algorithm
struct TFlowStereo2
{
  int We;
  int Wd;
  int Wd0;
  int XFilter, YFilter;
  int XStep, YStep;
  int ThMatch;
  int TFilter;  // Temporal filter

  // Filter kernel:
  cv::Mat Kernel;

  // Temporary containers:
  cv::Mat frame1, frame2, mask1, mask2, fmask1, fmask2, frame01, frame02;
  cv::Mat seg1, seg2, tmp;
  std::vector<int> matched;
  std::vector<cv::Point2f> points1,points2;
  cv::Mat points4d;
  cv::Mat frame1c,frame2c;  // For visualization

  TFlowStereo2()
    {
      We= 2;
      Wd= 3;
      Wd0= 2;
      XFilter= 1;
      YFilter= 32;
      XStep= 1;
      YStep= 16;
      ThMatch= 16;
      TFilter= 5;
    }
  void Init();
  void operator()(cv::Mat flow_mask1, cv::Mat flow_mask2);  // implementation is below
};
//-------------------------------------------------------------------------------------------

const char *ColFileNames[]={
    "default_colors1.dat",
    "default_colors2.dat",
    "default_colors3.dat",
    "default_colors4.dat",
    "default_colors5.dat",
    "default_colors6.dat",
    "default_colors7.dat"};
std::string ColorFilesBase[]={"x","x2"};

std::vector<TCameraInfo> CamInfo;  // Using two elements only (0,1)
std::vector<TStereoInfo> StereoInfo;  // Using the first element only
std::vector<TFlowStereo2> StereoFInfo;  // Using the first element only

int  CameraIdx(0), CDIdx(0);  // Focused camera index, color index in ColDetector
bool Running(true), Shutdown(false);
TMultipleColorDetector ColDetector[2];
TFlowFinder FlowFinder[2];
TStereo Stereo;
TStereo Rectifier;
// TStereo StereoF;
TEdgeFit EdgeFit;

int NumColDetectors(1);
TEasyVideoOut VideoOut[2], VideoOutF[2];
int VizMode[]= {2,2};  // 0: camera only, 1: camera + detected, 2: 0.5*camera + detected, 3: 0.25*camera + detected, 4: detected only

int SendRawFlow(0);  // 1: Send raw flow sensing data as sensor_msg
std::string ImgWin("110011111");  // 1:show, 0:hide; order=color1,color2,stereo1,stereo2,disparity,flow1,flow2,edge1,edge2
double DispScale(1.0);  // Scaling factor to display.

std::vector<ay_vision_msgs::ColDetVizPrimitive> VizObjs[2];  // External visualization requests.

ros::Publisher CloudPub, FlowCloudPub, SensorPub[2];
cv::Mat Frame[2], FlowMask[2];
int FMQueueIdx[2];
std::vector<cv::Mat> FlowMaskQueue[2];  // Queue of FlowMask for filtering.
std::vector<int64_t> CapTime, FlowFindTime;  // Using two elements only (0,1)
boost::mutex MutCamCapture[2], MutFlowFind[2];
struct TIMShowStuff
{
  boost::shared_ptr<boost::mutex> Mutex;
  cv::Mat Frame;
};
std::map<std::string, TIMShowStuff> IMShowStuff;
#define IMSHOW(x_img,x_name)  \
  do{  \
  boost::mutex::scoped_lock lock(*IMShowStuff[x_name].Mutex);  \
  if(DispScale==1.0)  x_img.copyTo(IMShowStuff[x_name].Frame);  \
  else  cv::resize(x_img, IMShowStuff[x_name].Frame, cv::Size(0,0), DispScale,DispScale);  \
  } while(0)

ay_3dvision_msgs::ROI_3DProj ROIColDet, ROIStereoF;
}
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
using namespace trick;
//-------------------------------------------------------------------------------------------
// #define print(var) PrintContainer((var), #var"= ")
// #define print(var) std::cout<<#var"= "<<(var)<<std::endl
//-------------------------------------------------------------------------------------------

/*
  Shift+Left click: add a color
  Shift+Right click: reset colors
  Left double click: reset amount
  Right click: pause/resume
*/
void OnMouse(int event, int x, int y, int flags, void *data)
{
  if(event!=0)  CameraIdx= *(reinterpret_cast<int*>(data));

  if(flags==cv::EVENT_FLAG_SHIFTKEY)
  {
    ColDetector[CameraIdx].CameraWindowMouseCallback(CDIdx, event, x, y, flags);
  }
  else
  {
    ColDetector[CameraIdx].MaskWindowMouseCallback(CDIdx, event, x, y, flags);

    if(event == cv::EVENT_RBUTTONDOWN)
    {
      Running=!Running;
      std::cerr<<(Running?"Resume":"Pause (Hit space/R-click to resume)")<<std::endl;
    }
  }
}
/*
  Right click: pause/resume
*/
void OnMouseSimple(int event, int x, int y, int flags, void *data)
{
  if(flags!=0)  return;
  if(event == cv::EVENT_RBUTTONDOWN)
  {
    Running=!Running;
    std::cerr<<(Running?"Resume":"Pause (Hit space/R-click to resume)")<<std::endl;
  }
}
//-------------------------------------------------------------------------------------------

// return if continue
bool HandleKeyEvent()
{
  // keyboard interface:
  char c(cv::waitKey(1));
  if(c=='\x1b'||c=='q') return false;
  else if(c=='r')
  {
    for(int j(0);j<2;++j)
      ColDetector[j].Reset();
  }
  else if(c=='l')
  {
    ColDetector[CameraIdx].LoadColors(CDIdx, ColorFilesBase[CameraIdx]+ColFileNames[CDIdx]);
  }
  else if(c=='s')
  {
    ColDetector[CameraIdx].SaveColors(CDIdx, ColorFilesBase[CameraIdx]+ColFileNames[CDIdx]);
  }
  else if((c>='1' && c<='7') || c=='0')
  {
    int old_cd_idx(CDIdx);
    switch(c)
    {
    case '1':  CDIdx= 0; break;
    case '2':  CDIdx= 1; break;
    case '3':  CDIdx= 2; break;
    case '4':  CDIdx= 3; break;
    case '5':  CDIdx= 4; break;
    case '6':  CDIdx= 5; break;
    case '7':  CDIdx= 6; break;
    case '0':  CDIdx= -1; break;
    }
    if(CDIdx>=NumColDetectors)
      CDIdx= old_cd_idx;
    else
      std::cerr<<"###Selected: "<<(CDIdx+1)<<std::endl;
  }
  else if(c=='W')
  {
    for(int j(0);j<2;++j)  VideoOut[j].Switch();
    for(int j(0);j<2;++j)  VideoOutF[j].Switch();
  }
  else if(c==' ')
  {
    Running=!Running;
    std::cerr<<(Running?"Resume":"Pause (Hit space/R-click to resume)")<<std::endl;
  }
  else if(c=='m' || c=='M')
  {
    if(c=='m')  ++VizMode[CameraIdx];  else  --VizMode[CameraIdx];
    if(VizMode[CameraIdx]>4)  VizMode[CameraIdx]= 0;
    if(VizMode[CameraIdx]<0)  VizMode[CameraIdx]= 4;
    std::cerr<<"VizMode["<<CameraIdx<<"]: "<<VizMode[CameraIdx]<<std::endl;
  }

  return true;
}
//-------------------------------------------------------------------------------------------

void DrawExternalViz(int cam_idx, cv::Mat &disp_img)
{
  for(std::vector<ay_vision_msgs::ColDetVizPrimitive>::const_iterator itr(VizObjs[cam_idx].begin()),itr_end(VizObjs[cam_idx].end());
      itr!=itr_end; ++itr)
  {
    cv::Scalar col= CV_RGB(itr->color.r,itr->color.g,itr->color.b);
    const double &lw= itr->line_width;
    switch(itr->type)
    {
    case ay_vision_msgs::ColDetVizPrimitive::LINE :
      cv::line(disp_img, cv::Point2d(itr->param[0],itr->param[1]), cv::Point2d(itr->param[2],itr->param[3]), col, lw);
      break;
    case ay_vision_msgs::ColDetVizPrimitive::CIRCLE :
      cv::circle(disp_img, cv::Point2d(itr->param[0],itr->param[1]), itr->param[2], col, lw);
      break;
    case ay_vision_msgs::ColDetVizPrimitive::RECTANGLE :
      cv::rectangle(disp_img, cv::Rect(itr->param[0],itr->param[1],itr->param[2],itr->param[3]), col, lw);
      break;
    default:
      std::cerr<<"Unknown type:"<<itr->type<<std::endl;
      return;
    }
  }
}
//-------------------------------------------------------------------------------------------

void ProjectROIToMask(const ay_3dvision_msgs::ROI_3DProj &roi,
    const cv::Size &size1, const cv::Size &size2, const TCameraParams &cam_params,
    cv::Mat &mask1, cv::Mat &mask2)
{
  mask1.create(size1, CV_8U);
  mask2.create(size2, CV_8U);
  // std::cerr<<"Setting ROI "<<roi.type<<std::endl;
  if(roi.type==roi.NONE)
  {
    mask1.setTo(255);
    mask2.setTo(255);
  }
  else if(roi.type==roi.POLYGON)
  {
    cv::Mat points3d(roi.param.size()/3,3,CV_32F);
    // float pt[3];
    // cv::Mat points3d(0,3,CV_32F), ptm(1,3,CV_32F,pt);
    cv::Mat points2d1, points2d2/*, hull1, hull2*/;
    for(int i(0),i_end(roi.param.size()/3); i<i_end; ++i)
    {
      points3d.at<float>(i,0)= roi.param[3*i];
      points3d.at<float>(i,1)= roi.param[3*i+1];
      points3d.at<float>(i,2)= roi.param[3*i+2];
      // if(roi.param[3*i+2]>-1.0)
      // {
        // pt[0]= roi.param[3*i];
        // pt[1]= roi.param[3*i+1];
        // pt[2]= roi.param[3*i+2];
        // points3d.push_back(ptm);
      // }
    }
    ProjectPointsToRectifiedImg(points3d, cam_params.P1, points2d1);
    ProjectPointsToRectifiedImg(points3d, cam_params.P2, points2d2);
    cv::Mat clip= (cv::Mat_<short>(4,2)<<0,0, 0,10000, 10000,10000, 10000,0);
    clip.convertTo(clip, points2d1.type());
    points2d1= ClipPolygon(points2d1, clip);
    points2d2= ClipPolygon(points2d2, clip);
    //*DBG*/std::cerr<<"points2d1="<<points2d1<<std::endl;
    points2d1.convertTo(points2d1, CV_32S);
    points2d2.convertTo(points2d2, CV_32S);
    //*DBG*/std::cerr<<"points2d1="<<points2d1<<std::endl;

    // cv::convexHull(points2d1, hull1);
    // cv::convexHull(points2d2, hull2);
    mask1.setTo(0);
    mask2.setTo(0);
    std::vector<cv::Mat> ppt1(1),ppt2(1);
    ppt1[0]= points2d1;
    ppt2[0]= points2d2;
    cv::fillPoly(mask1, ppt1, cv::Scalar(255));
    cv::fillPoly(mask2, ppt2, cv::Scalar(255));
  }
  else if(roi.type==roi.CONE)
  {
    int N= 16;
    cv::Mat points3d(1+N,3,CV_32F), rot, points2d1, points2d2, hull1, hull2;
    points3d.at<float>(0,0)= 0.0;
    points3d.at<float>(0,1)= 0.0;
    points3d.at<float>(0,2)= 0.0;
    float rad= roi.param[7]*std::tan(roi.param[6]);
    for(int i(0); i<N; ++i)
    {
      float th=(float)i*M_PI*2.0/(float)(N);
      points3d.at<float>(1+i,0)= rad*std::cos(th);
      points3d.at<float>(1+i,1)= rad*std::sin(th);
      points3d.at<float>(1+i,2)= 0.0;
    }
    float v1[]={0.,0.,-1.}, v2[]={roi.param[3],roi.param[4],roi.param[5]};
    cv::Vec3f axis_angle;
    GetAxisAngle(v1, v2, axis_angle);
    cv::Rodrigues(axis_angle, rot);
    points3d= points3d*rot.t();
    points3d.at<float>(0,0)+= roi.param[0];
    points3d.at<float>(0,1)+= roi.param[1];
    points3d.at<float>(0,2)+= roi.param[2];
    cv::Mat_<float> center= points3d.row(0)+roi.param[7]*cv::Mat(cv::Vec3f(roi.param[3],roi.param[4],roi.param[5])).t();
    for(int i(0); i<N; ++i)
      points3d.row(1+i)+= center;
    ProjectPointsToRectifiedImg(points3d, cam_params.P1, points2d1);
    ProjectPointsToRectifiedImg(points3d, cam_params.P2, points2d2);
    cv::Mat clip= (cv::Mat_<short>(4,2)<<0,0, 0,10000, 10000,10000, 10000,0);
    clip.convertTo(clip, points2d1.type());
    points2d1= ClipPolygon(points2d1, clip);
    points2d2= ClipPolygon(points2d2, clip);
    points2d1.convertTo(points2d1, CV_32S);
    points2d2.convertTo(points2d2, CV_32S);

    cv::convexHull(points2d1, hull1);
    cv::convexHull(points2d2, hull2);
    mask1.setTo(0);
    mask2.setTo(0);
    std::vector<cv::Mat> ppt1(1),ppt2(1);
    ppt1[0]= hull1;
    ppt2[0]= hull2;
    cv::fillPoly(mask1, ppt1, cv::Scalar(255));
    cv::fillPoly(mask2, ppt2, cv::Scalar(255));
    // cv::imshow("mask1", mask1);
    // cv::imshow("mask2", mask2);
  }
}
//-------------------------------------------------------------------------------------------

class TColDetVizCallback
{
private:
  int cam_idx_;
public:
  TColDetVizCallback(int cam_idx) : cam_idx_(cam_idx) {}
  void operator()(const ay_vision_msgs::ColDetVizConstPtr &msg)
  {
    VizObjs[cam_idx_]= msg->objects;
  }
};
//-------------------------------------------------------------------------------------------

void ROICallback(const ay_3dvision_msgs::ROI_3DProjPtr &msg)
{
  if(msg->target=="col_det")
  {
    ROIColDet= *msg;
    // std::cerr<<"col_det: Setting ROI\n"<<ROIColDet<<std::endl;
  }
  else if(msg->target=="stereo_f")
  {
    ROIStereoF= *msg;
    // std::cerr<<"stereo_f: Setting ROI\n"<<ROIStereoF<<std::endl;
  }
  else
  {
    std::cerr<<"Failed to set ROI:\n"<<*msg<<std::endl;
  }
}
//-------------------------------------------------------------------------------------------

bool ResetAmount(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cerr<<"Resetting base amount..."<<std::endl;
  for(int j(0); j<2; ++j)
    ColDetector[j].Reset();
  return true;
}
//-------------------------------------------------------------------------------------------

bool Pause(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cerr<<"Paused..."<<std::endl;
  Running= false;
  return true;
}
//-------------------------------------------------------------------------------------------

bool Resume(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cerr<<"Resumed..."<<std::endl;
  Running= true;
  return true;
}
//-------------------------------------------------------------------------------------------

// Get point cloud by stereo
void ExecStereo()
{
  cv::Mat frame[2];
  cv::Mat disparity;
  while(!Shutdown)
  {
    if(Running)
    {
      {
        for(int j(0);j<2;++j)
        {
          boost::mutex::scoped_lock lock(MutCamCapture[j]);
          Frame[j].copyTo(frame[j]);
        }
      }
      Stereo.Proc(frame[0],frame[1]);
      // Stereo.StereoParams().GrayScale= false;
      // Stereo.StereoParams().StereoMethod= TStereoParams::smBM;
      // Stereo.Proc(FlowFinder[0].FlowMask(),FlowFinder[1].FlowMask());
      cv::normalize(Stereo.Disparity(), disparity, 0, 255, CV_MINMAX, CV_8U);

      // 1:show, 0:hide; order=color1,color2,stereo1,stereo2,disparity,flow1,flow2,edge1,edge2
      // if(ImgWin[2]=='1')  cv::imshow("stereo_frame_l", Stereo.FrameL());
      // if(ImgWin[3]=='1')  cv::imshow("stereo_frame_r", Stereo.FrameR());
      // if(ImgWin[4]=='1')  cv::imshow("stereo_disparity", disparity);
      if(ImgWin[2]=='1') {IMSHOW(Stereo.FrameL(),"stereo_frame_l");}
      if(ImgWin[3]=='1') {IMSHOW(Stereo.FrameR(),"stereo_frame_r");}
      if(ImgWin[4]=='1') {IMSHOW(disparity,"stereo_disparity");}

      // Publish as point cloud.
      Stereo.ReprojectTo3D();
      sensor_msgs::PointCloud2 cloud_msg;
      ConvertPointCloudToROSMsg<pcl::PointXYZRGB>(cloud_msg,
          ConvertXYZImageToPointCloud(Stereo.XYZ(),Stereo.RGB()),
          /*frame_id=*/"usb_stereo");
      CloudPub.publish(cloud_msg);
      usleep(50*1000);
    }  // Running
    else
    {
      usleep(200*1000);
    }
  }
}
//-------------------------------------------------------------------------------------------

#if 0
// Get point cloud of flow by stereo
void FlowStereo(int we, int wd)
{
  cv::Mat frame1, frame2, disparity;

  FlowFinder[0].FlowMask().copyTo(frame1);
  FlowFinder[1].FlowMask().copyTo(frame2);
  frame1*= 200;
  frame2*= 200;
  cv::erode(frame1,frame1,cv::Mat(),cv::Point(-1,-1), we);
  cv::dilate(frame1,frame1,cv::Mat(),cv::Point(-1,-1), wd);
  cv::erode(frame2,frame2,cv::Mat(),cv::Point(-1,-1), we);
  cv::dilate(frame2,frame2,cv::Mat(),cv::Point(-1,-1), wd);
  // cv::Mat mask1(frame1), mask2(frame2);
  // Frame[0].copyTo(frame1, mask1);
  // Frame[1].copyTo(frame2, mask2);

  StereoF.Proc(frame1, frame2);
  cv::normalize(StereoF.Disparity(), disparity, 0, 255, CV_MINMAX, CV_8U);
  cv::imshow("stereof_frame_l", StereoF.FrameL());
  cv::imshow("stereof_frame_r", StereoF.FrameR());
  cv::imshow("stereof_disparity", disparity);

  // Publish as point cloud.
  StereoF.ReprojectTo3D();
  sensor_msgs::PointCloud2 cloud_msg;
  ConvertPointCloudToROSMsg<pcl::PointXYZRGB>(cloud_msg,
      ConvertXYZImageToPointCloud(StereoF.XYZ(),StereoF.RGB()),
      /*frame_id=*/"usb_stereo");
  FlowCloudPub.publish(cloud_msg);
}
#endif
//-------------------------------------------------------------------------------------------


void TFlowStereo2::Init()
{
  Kernel.create(cv::Size(XFilter,YFilter),CV_32F);
  Kernel= cv::Mat::ones(Kernel.size(),CV_32F)/(float)(Kernel.rows*Kernel.cols);
}
//-------------------------------------------------------------------------------------------

void TFlowStereo2::operator()(cv::Mat flow_mask1, cv::Mat flow_mask2)
{
  ProjectROIToMask(ROIStereoF, flow_mask1.size(), flow_mask2.size(), Rectifier.CameraParams(), mask1, mask2);
  frame1.setTo(0);
  frame2.setTo(0);
  flow_mask1.copyTo(frame1, mask1);
  flow_mask2.copyTo(frame2, mask2);
  frame1*= 200;
  frame2*= 200;

  // // Remove noise, make remaining pixels bigger:
  // cv::erode(frame1,frame1,cv::Mat(),cv::Point(-1,-1), We);
  // cv::dilate(frame1,frame1,cv::Mat(),cv::Point(-1,-1), Wd);
  // cv::erode(frame2,frame2,cv::Mat(),cv::Point(-1,-1), We);
  // cv::dilate(frame2,frame2,cv::Mat(),cv::Point(-1,-1), Wd);
  // // Vertical filter to make detecting flow easier:
  // cv::filter2D(frame1, frame1, /*ddepth=*/-1, Kernel);
  // cv::filter2D(frame2, frame2, /*ddepth=*/-1, Kernel);

  // Remove noise, make remaining pixels bigger:
  cv::dilate(frame1,fmask1,cv::Mat(),cv::Point(-1,-1), Wd0);
  cv::dilate(frame2,fmask2,cv::Mat(),cv::Point(-1,-1), Wd0);
  cv::erode(fmask1,fmask1,cv::Mat(),cv::Point(-1,-1), We);
  cv::dilate(fmask1,fmask1,cv::Mat(),cv::Point(-1,-1), Wd);
  cv::erode(fmask2,fmask2,cv::Mat(),cv::Point(-1,-1), We);
  cv::dilate(fmask2,fmask2,cv::Mat(),cv::Point(-1,-1), Wd);
  // Vertical filter to make detecting flow easier:
  cv::filter2D(fmask1, fmask1, /*ddepth=*/-1, Kernel);
  cv::filter2D(fmask2, fmask2, /*ddepth=*/-1, Kernel);
  frame01.setTo(0);
  frame02.setTo(0);
  frame1.copyTo(frame01, fmask1);
  frame2.copyTo(frame02, fmask2);
  // frame1= frame01;
  // frame2= frame02;
  frame01.copyTo(frame1);
  frame02.copyTo(frame2);
  cv::filter2D(frame1, frame1, /*ddepth=*/-1, Kernel);
  cv::filter2D(frame2, frame2, /*ddepth=*/-1, Kernel);

  matched.resize(frame1.rows);
  for(int y(0),y_end(std::min(frame1.rows,frame2.rows)-YStep); y<y_end; y+=YStep)
  {
    cv::Mat seg1(frame1,cv::Rect(0,y,frame1.cols,1));
    cv::Mat seg2(frame2,cv::Rect(0,y,frame2.cols,1));
    int dx(0);
    double match(0.0), max_match(0.0), x_match(0);
    for(int x(0),x_end(frame1.cols); x<x_end; x+=XStep)
    {
      dx= frame1.cols-x;
      cv::bitwise_and(seg1(cv::Rect(x,0,dx,1)), seg2(cv::Rect(0,0,dx,1)), tmp);
      match= cv::sum(tmp)[0];
      if(match>max_match)  {x_match=x; max_match=match;}
    }
    if(max_match>ThMatch)
      for(int y2(y);y2<y+YStep;++y2)  matched[y2]= x_match;
    else
      for(int y2(y);y2<y+YStep;++y2)  matched[y2]= -1;
    // std::cerr<<" "<<matched[y];
  }
  // std::cerr<<std::endl;

  points1.clear();
  points2.clear();
  // cv::cvtColor(frame1, frame1c, CV_GRAY2BGR);
  // cv::cvtColor(frame2, frame2c, CV_GRAY2BGR);
  cv::Mat frame1cs[]= {0.0*frame1, 0.7*frame1, 0.7*frame1};
  cv::merge(frame1cs, 3, frame1c);
  cv::Mat frame2cs[]= {0.0*frame2, 0.7*frame2, 0.7*frame2};
  cv::merge(frame2cs, 3, frame2c);
  for(int y(0),y_end(matched.size()); y<y_end; y+=YStep)
  {
    int dx= matched[y];
    if(dx>=0)
    {
      for(int x(dx),x_end(frame1.cols); x<x_end; x+=XStep)
      {
        if(frame1.at<unsigned char>(y,x)>10 && frame2.at<unsigned char>(y,x-dx)>10)
        {
          for(int y2(y);y2<y+YStep;++y2)
          {
            frame1c.at<cv::Vec3b>(y2,x)+= 2*cv::Vec3b(x%128,0,128-(x%128));
            frame2c.at<cv::Vec3b>(y2,x-dx)+= 2*cv::Vec3b(x%128,0,128-(x%128));
          }
          points1.push_back(cv::Point2f(x,y));
          points2.push_back(cv::Point2f(x-dx,y));
        }
      }
    }
  }

  if(points1.size()>0)
  {
    cv::triangulatePoints(Rectifier.CameraParams().P1, Rectifier.CameraParams().P2,
        points1, points2, points4d);
    //*DBG*/std::cerr<<"points4d[0]="<<points4d.col(0)<<std::endl;
  }
  else
    points4d= cv::Mat::zeros(0,0,CV_32F);

  // Publish as point cloud.
  sensor_msgs::PointCloud2 cloud_msg;
  ConvertPointCloudToROSMsg<pcl::PointXYZ>(cloud_msg,
      Convert4DPointsToPointCloud(points4d),
      /*frame_id=*/"usb_stereo");
  FlowCloudPub.publish(cloud_msg);
}
//-------------------------------------------------------------------------------------------

void ExecFlowFind(int i_cam)
{
  cv::Mat frame, flow_mask;
  int64_t t_cap=0;
  TFPSEstimator fps_est;
  int show_fps(0);
  while(!Shutdown)
  {
    if(Running)
    {
      if(CapTime[i_cam]==t_cap)
      {
        usleep(10*1000);
        continue;
      }

// double t_start=GetCurrentTime();
// if(i_cam==0)std::cerr<<"DBG: a "<<1000.0*(GetCurrentTime()-t_start);
      {
        boost::mutex::scoped_lock lock(MutCamCapture[i_cam]);
        Frame[i_cam].copyTo(frame);
        t_cap= CapTime[i_cam];
      }

      // Rectify image before flow-find? OR after??
      // if(i_cam==0)       Rectifier.RectifyL(frame);
      // else if(i_cam==1)  Rectifier.RectifyR(frame);
// if(i_cam==0)std::cerr<<"DBG: b "<<1000.0*(GetCurrentTime()-t_start);
      // Flow detection from image
      FlowFinder[i_cam].Update(frame);
      FlowFinder[i_cam].FlowMask().copyTo(flow_mask);
      // if(i_cam==0)       Rectifier.RectifyL(flow_mask);
      // else if(i_cam==1)  Rectifier.RectifyR(flow_mask);
      // NOTE: Rectification is moved to ExecFlowStereo

// if(i_cam==0)std::cerr<<"DBG: c "<<1000.0*(GetCurrentTime()-t_start);
      {
        boost::mutex::scoped_lock lock(MutFlowFind[i_cam]);
        // flow_mask.copyTo(FlowMask[i_cam]);
        flow_mask.copyTo(FlowMaskQueue[i_cam][FMQueueIdx[i_cam]]);
        ++FMQueueIdx[i_cam];
        if(FMQueueIdx[i_cam]>=StereoFInfo[0].TFilter)  FMQueueIdx[i_cam]= 0;
        FlowFindTime[i_cam]= GetCurrentTimeL();
      }
// if(i_cam==0)std::cerr<<"DBG: d "<<1000.0*(GetCurrentTime()-t_start);

      fps_est.Step();
      if(show_fps==0)
      {
        std::cerr<<"FPS(flow "<<i_cam<<"): "<<fps_est.FPS<<std::endl;
        show_fps=fps_est.FPS*4;
      }
      --show_fps;
    }  // Running
    else
    {
      usleep(200*1000);
    }
  }
}
//-------------------------------------------------------------------------------------------

void ExecFlowStereo()
{
  TFlowStereo2 &stereo_f(StereoFInfo[0]);
  cv::Mat frame[2], disp_img[2];
  int show_fps(0), do_stereo_f(0);
  int64_t t_ff[2]={0,0};
  while(!Shutdown)
  {
    if(Running)
    {
      if(FlowFindTime[0]<0 || FlowFindTime[1]<0
        || FlowFindTime[0]==t_ff[0] || FlowFindTime[1]==t_ff[1])
      {
        usleep(5*1000);
// std::cerr<<"DBG: 10.0"<<std::endl;
        continue;
      }

// double t_start=GetCurrentTime();
// std::cerr<<"DBG: a "<<1000.0*(GetCurrentTime()-t_start);
// std::cerr<<"DBG: FMQueueIdx[0]="<<FMQueueIdx[0]<<std::endl;;
// std::cerr<<"DBG: FMQueueIdx[1]="<<FMQueueIdx[1]<<std::endl;;
// std::cerr<<"DBG: TFilter="<<StereoFInfo[0].TFilter<<std::endl;;
// std::cerr<<"DBG: FlowMaskQueue[0].size()="<<FlowMaskQueue[0].size()<<std::endl;;
// std::cerr<<"DBG: FlowMaskQueue[1].size()="<<FlowMaskQueue[1].size()<<std::endl;;
      {
        for(int i_cam(0);i_cam<2;++i_cam)
        {
          boost::mutex::scoped_lock lock(MutFlowFind[i_cam]);
          FlowMaskQueue[i_cam][0].copyTo(FlowMask[i_cam]);
          for(int i(1); i<StereoFInfo[0].TFilter; ++i)
            cv::bitwise_or(FlowMask[i_cam], FlowMaskQueue[i_cam][i], FlowMask[i_cam]);
          t_ff[i_cam]= FlowFindTime[i_cam];
          FlowMask[i_cam].copyTo(frame[i_cam]);
        }
      }
      Rectifier.RectifyL(frame[0]);
      Rectifier.RectifyR(frame[1]);

// std::cerr<<" c "<<1000.0*(GetCurrentTime()-t_start);
      if(do_stereo_f==0)
      {
        // stereo_f(FlowFinder[0].FlowMask(), FlowFinder[1].FlowMask());
        stereo_f(frame[0], frame[1]);
        // do_stereo_f= FlowFinder[0].FlowMaskFilterLen()/2;
        do_stereo_f= StereoFInfo[0].TFilter/2;
      }
      --do_stereo_f;

// std::cerr<<" d "<<1000.0*(GetCurrentTime()-t_start);
      // stereo_f.frame1c.copyTo(disp_img[0]);
      // stereo_f.frame2c.copyTo(disp_img[1]);
      // FlowFinder[0].DrawFlow(disp_img[0], CV_RGB(0,255,255), /*len=*/1.0, /*thickness=*/1);
      // FlowFinder[1].DrawFlow(disp_img[1], CV_RGB(0,255,255), /*len=*/1.0, /*thickness=*/1);
      disp_img[0]= ColorMask(frame[0], CV_RGB(0,80,80));
      disp_img[1]= ColorMask(frame[1], CV_RGB(0,80,80));
      disp_img[0]+= stereo_f.frame1c;
      disp_img[1]+= stereo_f.frame2c;

// std::cerr<<" e "<<1000.0*(GetCurrentTime()-t_start);
      VideoOutF[0].Step(disp_img[0]);
      VideoOutF[0].VizRec(disp_img[0]);
      VideoOutF[1].Step(disp_img[1]);
      VideoOutF[1].VizRec(disp_img[1]);

// std::cerr<<" f "<<1000.0*(GetCurrentTime()-t_start);
      // cv::normalize(disparity, disparity, 0, 255, CV_MINMAX, CV_8U);
      // 1:show, 0:hide; order=color1,color2,stereo1,stereo2,disparity,flow1,flow2,edge1,edge2
      // if(ImgWin[5]=='1')  cv::imshow("stereof_frame_l", disp_img[0]);
      // if(ImgWin[6]=='1')  cv::imshow("stereof_frame_r", disp_img[1]);
      // // cv::imshow("stereof_disparity", disparity);
      if(ImgWin[5]=='1') {IMSHOW(disp_img[0],"stereof_frame_l");}
      if(ImgWin[6]=='1') {IMSHOW(disp_img[1],"stereof_frame_r");}

// std::cerr<<" g "<<1000.0*(GetCurrentTime()-t_start)<<std::endl;
      if(show_fps==0)
      {
        std::cerr<<"FPS(flow-stereo): "<<VideoOutF[0].FPS()<<", "<<VideoOutF[1].FPS()<<std::endl;
        show_fps=VideoOutF[0].FPS()*4;
      }
      --show_fps;

    }  // Running
    else
    {
      usleep(200*1000);
    }
  }
}
//-------------------------------------------------------------------------------------------

void ReadFromYAML(std::vector<TFlowStereo2> &stereof_info, const std::string &file_name)
{
  stereof_info.clear();
  cv::FileStorage fs(file_name, cv::FileStorage::READ);
  cv::FileNode data= fs["FlowStereoInfo"];
  for(cv::FileNodeIterator itr(data.begin()),itr_end(data.end()); itr!=itr_end; ++itr)
  {
    TFlowStereo2 cf;
    #define PROC_VAR(x)  (*itr)[#x]>>cf.x;
    PROC_VAR(We      );
    PROC_VAR(Wd      );
    PROC_VAR(Wd0     );
    PROC_VAR(XFilter );
    PROC_VAR(YFilter );
    PROC_VAR(XStep   );
    PROC_VAR(YStep   );
    PROC_VAR(ThMatch );
    PROC_VAR(TFilter );
    #undef PROC_VAR
    stereof_info.push_back(cf);
  }
  fs.release();
}
//-------------------------------------------------------------------------------------------

void ExecColDet()
{
  cv::Size img_size[2]={cv::Size(CamInfo[0].Width,CamInfo[0].Height),
                        cv::Size(CamInfo[1].Width,CamInfo[1].Height)};
  cv::Mat frame_unrct[2], frame[2], disp_img[2], mask[2];
  for(int j(0);j<2;++j)
    ColDetector[j].SetCameraWindow(frame_unrct[j]);
  int show_fps(0);
  int64_t t_cap[2]={0,0};
  while(!Shutdown)
  {
    if(Running)
    {
      if(CapTime[0]==t_cap[0] || CapTime[1]==t_cap[1])
      {
        usleep(10*1000);
        continue;
      }

      {
        for(int i_cam(0);i_cam<2;++i_cam)
        {
          boost::mutex::scoped_lock lock(MutCamCapture[i_cam]);
          Frame[i_cam].copyTo(frame_unrct[i_cam]);
          t_cap[i_cam]= CapTime[i_cam];
        }
      }
      // The masks are drawn on rectified image plane.
      // So we should rectify the captured frames with stereo camera parameters.
      ProjectROIToMask(ROIColDet, img_size[0], img_size[1], Rectifier.CameraParams(), mask[0], mask[1]);
      Rectifier.RectifyL(frame_unrct[0]);
      Rectifier.RectifyR(frame_unrct[1]);
      frame[0].setTo(0);
      frame[1].setTo(0);
      frame_unrct[0].copyTo(frame[0], mask[0]);
      frame_unrct[1].copyTo(frame[1], mask[1]);


      for(int cam_idx(0); cam_idx<2; ++cam_idx)
      {
        // Visualization setup
        // 0: camera only, 1: camera + detected, 2: 0.5*camera + detected, 3: 0.25*camera + detected, 4: detected only
        if(VizMode[cam_idx]==0 || VizMode[cam_idx]==1)
        {
          frame[cam_idx].copyTo(disp_img[cam_idx]);
        }
        else if(VizMode[cam_idx]==2)
        {
          frame[cam_idx].copyTo(disp_img[cam_idx]);
          disp_img[cam_idx]*= 0.5;
        }
        else if(VizMode[cam_idx]==3)
        {
          frame[cam_idx].copyTo(disp_img[cam_idx]);
          disp_img[cam_idx]*= 0.25;
        }
        else if(VizMode[cam_idx]==4)
        {
          frame[cam_idx].copyTo(disp_img[cam_idx]);  // TODO: make this efficient (not need to copy)
          disp_img[cam_idx].setTo(cv::Scalar(0,0,0));
        }

        // Color detection from image
        ColDetector[cam_idx].Detect(frame[cam_idx], /*mode=*/2, /*verbose=*/false);
        if(VizMode[cam_idx]!=0)
        {
          ColDetector[cam_idx].Draw(disp_img[cam_idx]);
          if(CDIdx>=0)
            cv::rectangle(disp_img[cam_idx], ColDetector[cam_idx].Bound(CDIdx), CV_RGB(0,255,0), 2);
        }

        if(VizMode[cam_idx]!=0)
          DrawExternalViz(cam_idx, disp_img[cam_idx]);

        // TEST: show FlowMask only.
        // FlowFinder[cam_idx].FlowMask().copyTo(disp_img[cam_idx]);
        // disp_img[cam_idx]*= 200;

        // Send ROS topics
        {
          ay_vision_msgs::ColDetSensor  sensor_msg;

          sensor_msg.num_cols= ColDetector[cam_idx].Size();
          sensor_msg.col_filled_ratio= ColDetector[cam_idx].DataRatio();
          sensor_msg.col_center_xy= ColDetector[cam_idx].DataCenterXY();
          sensor_msg.col_area= ColDetector[cam_idx].DataArea();
          sensor_msg.col_bound= ColDetector[cam_idx].DataBound();

          sensor_msg.nums_blocks= ColDetector[cam_idx].NumsBlocks();
          sensor_msg.blocks_area.resize(ColDetector[cam_idx].BlocksArea().size());
          std::copy(ColDetector[cam_idx].BlocksArea().begin(),ColDetector[cam_idx].BlocksArea().end(), sensor_msg.blocks_area.begin());
          sensor_msg.blocks_center_xy.resize(ColDetector[cam_idx].BlocksCenterXY().size());
          std::copy(ColDetector[cam_idx].BlocksCenterXY().begin(),ColDetector[cam_idx].BlocksCenterXY().end(), sensor_msg.blocks_center_xy.begin());

          if(SendRawFlow)
          {
            std::list<TFlowElement> flow= FlowFinder[cam_idx].FlowElements();
            sensor_msg.num_flows= flow.size();
            sensor_msg.flows_xy     .resize(2*flow.size());
            sensor_msg.flows_vxy    .resize(2*flow.size());
            sensor_msg.flows_spddir .resize(2*flow.size());
            sensor_msg.flows_amount .resize(flow.size());
            int i(0);
            for(std::list<TFlowElement>::const_iterator itr(flow.begin()),itr_end(flow.end());
                itr!=itr_end; ++itr,++i)
            {
              sensor_msg.flows_xy[2*i+0]= itr->X;
              sensor_msg.flows_xy[2*i+1]= itr->Y;
              sensor_msg.flows_vxy[2*i+0]= itr->VX;
              sensor_msg.flows_vxy[2*i+1]= itr->VY;
              sensor_msg.flows_spddir[2*i+0]= itr->Speed;
              sensor_msg.flows_spddir[2*i+1]= itr->Angle;
              sensor_msg.flows_amount[i]= itr->Amount;
            }
          }

          // DEPRECATED: sensor_msg.{flow_avr_xy,flow_avr_vxy,flow_avr_spddir}

          SensorPub[cam_idx].publish(sensor_msg);
        }

        // VideoOut[cam_idx].Step(disp_img[cam_idx]);
        // VideoOut[cam_idx].VizRec(disp_img[cam_idx]);
      }  // for cam_idx


      std::cerr<<"ratio:";
      for(int cam_idx(0); cam_idx<2; ++cam_idx)
      {
        if(cam_idx>0)  std::cerr<<" | ";
        for(int i(0); i<ColDetector[cam_idx].Size(); ++i)
          std::cerr<<" "<<ColDetector[cam_idx].Ratio(i);
      }
      std::cerr<<std::endl;

      VideoOut[0].Step(disp_img[0]);
      VideoOut[0].VizRec(disp_img[0]);
      VideoOut[1].Step(disp_img[1]);
      VideoOut[1].VizRec(disp_img[1]);

      // 1:show, 0:hide; order=color1,color2,stereo1,stereo2,disparity,flow1,flow2,edge1,edge2
      // if(ImgWin[0]=='1')  cv::imshow("color_detector1", disp_img[0]);
      // if(ImgWin[1]=='1')  cv::imshow("color_detector2", disp_img[1]);
      if(ImgWin[0]=='1') {IMSHOW(disp_img[0],"color_detector1");}
      if(ImgWin[1]=='1') {IMSHOW(disp_img[1],"color_detector2");}

      if(show_fps==0)
      {
        std::cerr<<"FPS(coldet): "<<VideoOut[0].FPS()<<", "<<VideoOut[1].FPS()<<std::endl;
        show_fps=VideoOut[0].FPS()*4;
      }
      --show_fps;

    }  // Running
    else
    {
      usleep(200*1000);
    }
  }
}
//-------------------------------------------------------------------------------------------

void ExecCapture(cv::VideoCapture &cap, int cam_idx)
{
  cv::Mat frame;
  int64_t cap_time(0);
  TFPSEstimator fps_est;
  int show_fps(0);
  while(!Shutdown)
  {
    if(Running)
    {
      // Capture from cameras:
      cap >> frame; // get a new frame from camera
      if(CamInfo[cam_idx].CapWidth!=CamInfo[cam_idx].Width || CamInfo[cam_idx].CapHeight!=CamInfo[cam_idx].Height)
        cv::resize(frame,frame,cv::Size(CamInfo[cam_idx].Width,CamInfo[cam_idx].Height));
      Rotate90N(frame,frame,CamInfo[cam_idx].NRotate90);
      cap_time= GetCurrentTimeL();
      // Copy frames to global buffer:
      {
        boost::mutex::scoped_lock lock(MutCamCapture[cam_idx]);
        frame.copyTo(Frame[cam_idx]);
        CapTime[cam_idx]= cap_time;
      }

      fps_est.Step();
      if(show_fps==0)
      {
        std::cerr<<"FPS(capture "<<cam_idx<<"): "<<fps_est.FPS<<std::endl;
        show_fps=fps_est.FPS*4;
      }
      --show_fps;

    }  // Running
    else
    {
      usleep(200*1000);
    }
  }
}

bool ExecFitEdge(ay_3dvision_msgs::FitEdge::Request &req, ay_3dvision_msgs::FitEdge::Response &res)
{
  TEdgeFitParams &params(EdgeFit.Params());
  params.EdgeDetect.PreBlurSize  =   req.ED_PreBlurSize    ;
  params.EdgeDetect.PostBlurSize =   req.ED_PostBlurSize   ;

  params.EdgeEval.MinEdgeBrightness =   req.EE_MinEdgeBrightness  ;
  params.EdgeEval.MinMatchingRatio  =   req.EE_MinMatchingRatio   ;

  std::copy(req.XMin.begin(), req.XMin.end(), params.XMin);
  std::copy(req.XMax.begin(), req.XMax.end(), params.XMax);
  std::copy(req.Sig0.begin(), req.Sig0.end(), params.Sig0);

  params.LPoints3d.create(req.LPoints3d.size()/3,3,CV_32F);
  std::copy(req.LPoints3d.begin(), req.LPoints3d.end(), params.LPoints3d.begin<float>());

  double pose[7], quality(0.0);
  std::copy(req.pose0.begin(), req.pose0.end(), pose);

  cv::Mat frame[2], disp[2];
  {
    for(int i_cam(0);i_cam<2;++i_cam)
    {
      boost::mutex::scoped_lock lock(MutCamCapture[i_cam]);
      Frame[i_cam].copyTo(frame[i_cam]);
    }
  }
  Rectifier.RectifyL(frame[0]);
  Rectifier.RectifyR(frame[1]);

  std::cerr<<"FitEdge request. init pose="<<cv::Mat(1,7,CV_64F,pose)<<std::endl;
  EdgeFit.Run(frame[0], frame[1], pose, pose, &quality);
  EdgeFit.Viz(disp[0], disp[1], pose);
  std::cerr<<"FitEdge - Done. pose="<<cv::Mat(1,7,CV_64F,pose)<<", quality="<<quality<<std::endl;

  res.pose.resize(7);
  std::copy(pose, pose+7, res.pose.begin());
  res.quality= quality;

  // 1:show, 0:hide; order=color1,color2,stereo1,stereo2,disparity,flow1,flow2,edge1,edge2
  if(ImgWin[7]=='1') {IMSHOW(disp[0],"edge_fit_l");}
  if(ImgWin[8]=='1') {IMSHOW(disp[1],"edge_fit_r");}
  return true;
}
//-------------------------------------------------------------------------------------------

void FitEdgeDummy(void)
{
  cv::Mat frame[2], disp[2];
  {
    for(int i_cam(0);i_cam<2;++i_cam)
    {
      boost::mutex::scoped_lock lock(MutCamCapture[i_cam]);
      Frame[i_cam].copyTo(frame[i_cam]);
    }
  }
  Rectifier.RectifyL(frame[0]);
  Rectifier.RectifyR(frame[1]);

  EdgeFit.DetectEdges2(frame[0], frame[1]);
  EdgeFit.Viz(disp[0], disp[1], /*pose=*/NULL);

  // 1:show, 0:hide; order=color1,color2,stereo1,stereo2,disparity,flow1,flow2,edge1,edge2
  if(ImgWin[7]=='1') {IMSHOW(disp[0],"edge_fit_l");}
  if(ImgWin[8]=='1') {IMSHOW(disp[1],"edge_fit_r");}
}
//-------------------------------------------------------------------------------------------

int main(int argc, char**argv)
{
  ros::init(argc, argv, "usb_stereo_node");
  ros::NodeHandle node("~");
  std::string pkg_dir(".");
  std::string cam_config("config/ext_usbcam1.yaml");
  std::string stereo_config("config/ext_usbcam1.yaml");
  std::string stereof_config("config/ext_usbcam1.yaml");
  double block_area_min(10.0);
  double disp_fps(30.0);

  int    ff_ofl_win(3);  // FlowFinder.OptFlowWinSize
  double ff_ofl_spd_min(2.0);  // FlowFinder.OptFlowSpdThreshold
  int    ff_er_dl(1);  // FlowFinder.ErodeDilate
  double ff_amt_min(1.0), ff_amt_max(3000.0);  // FlowFinder.AmountRange
  double ff_spd_min(1.0), ff_spd_max(-1.0);  // FlowFinder.SpeedRange
  int    ff_mask_flen(/*5*/0);  // FlowFinder.FlowMaskFilterLen
  /* NOTE: We introduced TFlowStereo2.TFilter instead of
  FlowFinder.FlowMaskFilterLen to speed up.
  So, ff_mask_flen should be zero. */

  std::string vout_base("/tmp/vout"), voutf_base("/tmp/vout");

  node.param("pkg_dir",pkg_dir,pkg_dir);
  node.param("cam_config",cam_config,cam_config);
  node.param("stereo_config",stereo_config,stereo_config);
  node.param("stereof_config",stereof_config,stereof_config);

  node.param("viz_mode1",VizMode[0],VizMode[0]);
  node.param("viz_mode2",VizMode[1],VizMode[1]);
  node.param("num_detectors",NumColDetectors,NumColDetectors);
  node.param("block_area_min",block_area_min,block_area_min);
  node.param("color_files_base1",ColorFilesBase[0],ColorFilesBase[0]);
  node.param("color_files_base2",ColorFilesBase[1],ColorFilesBase[1]);
  node.param("img_win",ImgWin,ImgWin);
  node.param("disp_fps",disp_fps,disp_fps);
  node.param("disp_scale",DispScale,DispScale);

  node.param("ff_ofl_win",ff_ofl_win,ff_ofl_win);
  node.param("ff_ofl_spd_min",ff_ofl_spd_min,ff_ofl_spd_min);
  node.param("ff_er_dl",ff_er_dl,ff_er_dl);
  node.param("ff_amt_min",ff_amt_min,ff_amt_min);
  node.param("ff_amt_max",ff_amt_max,ff_amt_max);
  node.param("ff_spd_min",ff_spd_min,ff_spd_min);
  node.param("ff_spd_max",ff_spd_max,ff_spd_max);
  node.param("ff_mask_flen",ff_mask_flen,ff_mask_flen);
  node.param("send_raw_flow",SendRawFlow,SendRawFlow);

  node.param("vout_base",vout_base,vout_base);
  node.param("voutf_base",voutf_base,voutf_base);

  ReadFromYAML(CamInfo, pkg_dir+"/"+cam_config);
  ReadFromYAML(StereoInfo, pkg_dir+"/"+stereo_config);
  ReadFromYAML(StereoFInfo, pkg_dir+"/"+stereof_config);

  ROIColDet.type= ROIColDet.NONE;
  ROIStereoF.type= ROIStereoF.NONE;

  std::vector<cv::VideoCapture> cap(CamInfo.size());
  for(int i_cam(0), i_cam_end(CamInfo.size()); i_cam<i_cam_end; ++i_cam)
  {
    if(!CapOpen(CamInfo[i_cam], cap[i_cam]))  return -1;
    // MutCamCapture.push_back(boost::shared_ptr<boost::mutex>(new boost::mutex));
  }
  std::cerr<<"Opened camera(s)"<<std::endl;

  for(int j(0);j<2;++j)
    VideoOut[j].SetfilePrefix(vout_base);
  for(int j(0);j<2;++j)
    VideoOutF[j].SetfilePrefix(voutf_base);

  for(int j(0); j<2; ++j)
  {
    ColDetector[j].Setup(NumColDetectors);
    for(int i(0); i<NumColDetectors; ++i)
      ColDetector[j].LoadColors(i, ColorFilesBase[j]+ColFileNames[i]);
    ColDetector[j].SetBlockAreaMin(block_area_min);
  }
  CDIdx= 0;

  for(int j(0); j<2; ++j)
  {
    // 0: Full, 1: FlowMask only
    if(SendRawFlow)  FlowFinder[j].SetProcType(0);
    else             FlowFinder[j].SetProcType(1);
    FlowFinder[j].SetOptFlowWinSize(cv::Size(ff_ofl_win,ff_ofl_win));
    FlowFinder[j].SetOptFlowSpdThreshold(ff_ofl_spd_min);
    FlowFinder[j].SetErodeDilate(ff_er_dl);
    FlowFinder[j].SetAmountRange(/*min=*/ff_amt_min, /*max=*/ff_amt_max);
    FlowFinder[j].SetSpeedRange(/*min=*/ff_spd_min, /*max=*/ff_spd_max);
    FlowFinder[j].SetFlowMaskFilterLen(ff_mask_flen);
  }

  // Stereo.resize(StereoInfo.size());
  // Rectifier.resize(StereoInfo.size());
  for(int j(0);j<1;++j)
  {
    const TStereoInfo &info(StereoInfo[j]);
    Stereo.LoadCameraParametersFromYAML(pkg_dir+"/"+info.StereoParam);
    Stereo.SetImageSize(
        cv::Size(CamInfo[info.CamL].Width,CamInfo[info.CamL].Height),
        cv::Size(info.Width,info.Height) );
    Stereo.SetRecommendedStereoParams();
    Stereo.LoadConfigurationsFromYAML(pkg_dir+"/"+info.StereoConfig);
    Stereo.Init();

    Rectifier.LoadCameraParametersFromYAML(pkg_dir+"/"+info.StereoParam);
    Rectifier.SetImageSize(cv::Size(CamInfo[0].Width,CamInfo[0].Height),
                           cv::Size(CamInfo[0].Width,CamInfo[0].Height));
    Rectifier.SetRecommendedStereoParams();
    Rectifier.LoadConfigurationsFromYAML(pkg_dir+"/"+info.StereoConfig);
    Rectifier.Init();
  }
  // StereoF.LoadCameraParametersFromYAML(stereo_param_yaml);
  // StereoF.SetImageSize(img_size,img_size);
  // StereoF.SetRecommendedStereoParams();
  // StereoF.StereoParams().StereoMethod= TStereoParams::smBM;
  // StereoF.Init();
  TFlowStereo2 &stereo_f(StereoFInfo[0]);
  stereo_f.Init();
  for(int j(0); j<2; ++j)
  {
    FMQueueIdx[j]= 0;
    FlowMaskQueue[j].resize(StereoFInfo[0].TFilter);
    for(int i(0); i<StereoFInfo[0].TFilter; ++i)
      FlowMaskQueue[j][i]= cv::Mat::zeros(cv::Size(CamInfo[j].Width,CamInfo[j].Height), CV_8UC1);
  }

  EdgeFit.Params().P1= Rectifier.CameraParams().P1;
  EdgeFit.Params().P2= Rectifier.CameraParams().P2;


  SensorPub[0]= node.advertise<ay_vision_msgs::ColDetSensor>("sensor1", 1);
  SensorPub[1]= node.advertise<ay_vision_msgs::ColDetSensor>("sensor2", 1);
  CloudPub= node.advertise<sensor_msgs::PointCloud2>("point_cloud", 1);
  FlowCloudPub= node.advertise<sensor_msgs::PointCloud2>("flow_cloud", 1);

  ros::Subscriber sub_viz1= node.subscribe<ay_vision_msgs::ColDetViz>("viz1", 1, TColDetVizCallback(0));
  ros::Subscriber sub_viz2= node.subscribe<ay_vision_msgs::ColDetViz>("viz2", 1, TColDetVizCallback(1));
  ros::Subscriber sub_roi= node.subscribe("roi", 1, &ROICallback);

  ros::ServiceServer srv_reset= node.advertiseService("reset", &ResetAmount);
  ros::ServiceServer srv_pause= node.advertiseService("pause", &Pause);
  ros::ServiceServer srv_resume= node.advertiseService("resume", &Resume);
  ros::ServiceServer srv_fit_edge= node.advertiseService("fit_edge", &ExecFitEdge);

  // for(int j(0);j<2;++j)
    // ColDetector[j].SetCameraWindow(Frame[j]);

  // 1:show, 0:hide; order=color1,color2,stereo1,stereo2,disparity,flow1,flow2,edge1,edge2
  int camera_indexes[]= {0,1};
  if(ImgWin.length()<9)  ImgWin.resize(9,'0');
  if(ImgWin[0]=='1')  cv::namedWindow("color_detector1",1);
  if(ImgWin[0]=='1')  cv::setMouseCallback("color_detector1", OnMouse, &camera_indexes[0]);
  if(ImgWin[0]=='1')  IMShowStuff["color_detector1"].Mutex= boost::shared_ptr<boost::mutex>(new boost::mutex);
  if(ImgWin[1]=='1')  cv::namedWindow("color_detector2",1);
  if(ImgWin[1]=='1')  cv::setMouseCallback("color_detector2", OnMouse, &camera_indexes[1]);
  if(ImgWin[1]=='1')  IMShowStuff["color_detector2"].Mutex= boost::shared_ptr<boost::mutex>(new boost::mutex);

  if(ImgWin[2]=='1')  cv::namedWindow("stereo_frame_l",1);
  if(ImgWin[2]=='1')  cv::setMouseCallback("stereo_frame_l", OnMouseSimple);
  if(ImgWin[2]=='1')  IMShowStuff["stereo_frame_l"].Mutex= boost::shared_ptr<boost::mutex>(new boost::mutex);
  if(ImgWin[3]=='1')  cv::namedWindow("stereo_frame_r",1);
  if(ImgWin[3]=='1')  cv::setMouseCallback("stereo_frame_r", OnMouseSimple);
  if(ImgWin[3]=='1')  IMShowStuff["stereo_frame_r"].Mutex= boost::shared_ptr<boost::mutex>(new boost::mutex);
  if(ImgWin[4]=='1')  cv::namedWindow("stereo_disparity",1);
  if(ImgWin[4]=='1')  cv::setMouseCallback("stereo_disparity", OnMouseSimple);
  if(ImgWin[4]=='1')  IMShowStuff["stereo_disparity"].Mutex= boost::shared_ptr<boost::mutex>(new boost::mutex);

  if(ImgWin[5]=='1')  cv::namedWindow("stereof_frame_l",1);
  if(ImgWin[5]=='1')  cv::setMouseCallback("stereof_frame_l", OnMouseSimple);
  if(ImgWin[5]=='1')  IMShowStuff["stereof_frame_l"].Mutex= boost::shared_ptr<boost::mutex>(new boost::mutex);
  if(ImgWin[6]=='1')  cv::namedWindow("stereof_frame_r",1);
  if(ImgWin[6]=='1')  cv::setMouseCallback("stereof_frame_r", OnMouseSimple);
  if(ImgWin[6]=='1')  IMShowStuff["stereof_frame_r"].Mutex= boost::shared_ptr<boost::mutex>(new boost::mutex);

  if(ImgWin[7]=='1')  cv::namedWindow("edge_fit_l",1);
  if(ImgWin[7]=='1')  cv::setMouseCallback("edge_fit_l", OnMouseSimple);
  if(ImgWin[7]=='1')  IMShowStuff["edge_fit_l"].Mutex= boost::shared_ptr<boost::mutex>(new boost::mutex);
  if(ImgWin[8]=='1')  cv::namedWindow("edge_fit_r",1);
  if(ImgWin[8]=='1')  cv::setMouseCallback("edge_fit_r", OnMouseSimple);
  if(ImgWin[8]=='1')  IMShowStuff["edge_fit_r"].Mutex= boost::shared_ptr<boost::mutex>(new boost::mutex);
  // cv::namedWindow("stereof_disparity",1);
  // cv::setMouseCallback("stereof_disparity", OnMouseSimple);

  cv::Mat frame[2];
  std::vector<int64_t> cap_time(CamInfo.size());
  CapTime.resize(CamInfo.size());
  FlowFindTime.resize(CamInfo.size());
  FlowFindTime[0]= -1; FlowFindTime[1]= -1;

  // Dummy capture.
  for(int i_cam(0); i_cam<2; ++i_cam)
  {
    cap[i_cam] >> Frame[i_cam];
    if(CamInfo[i_cam].CapWidth!=CamInfo[i_cam].Width || CamInfo[i_cam].CapHeight!=CamInfo[i_cam].Height)
      cv::resize(Frame[i_cam],Frame[i_cam],cv::Size(CamInfo[i_cam].Width,CamInfo[i_cam].Height));
    Rotate90N(Frame[i_cam],Frame[i_cam],CamInfo[i_cam].NRotate90);
    CapTime[i_cam]= GetCurrentTimeL();
  }

  FitEdgeDummy();

  boost::thread th_capture0(boost::bind(&ExecCapture,cap[0],0));
  boost::thread th_capture1(boost::bind(&ExecCapture,cap[1],1));
  boost::thread th_col_det(&ExecColDet);
  boost::thread th_flow_find0(boost::bind(&ExecFlowFind,0));
  boost::thread th_flow_find1(boost::bind(&ExecFlowFind,1));
  boost::thread th_stereo(&ExecStereo);
  boost::thread th_stereo_f(&ExecFlowStereo);

  ros::Rate disp_rate(disp_fps);  // 30 Hz
  TFPSEstimator fps_est;
  int show_fps(0);
  for(int f(0);ros::ok();++f)
  {
    if(Running)
    {
      // // Capture from cameras:
      // for(int j(0);j<2;++j)
      // {
        // cap[j] >> frame[j]; // get a new frame from camera
        // if(CamInfo[j].CapWidth!=CamInfo[j].Width || CamInfo[j].CapHeight!=CamInfo[j].Height)
          // cv::resize(frame[j],frame[j],cv::Size(CamInfo[j].Width,CamInfo[j].Height));
        // Rotate90N(frame[j],frame[j],CamInfo[j].NRotate90);
        // cap_time[j]= GetCurrentTimeL();
      // }
      // // Copy frames to global buffer:
      // {
        // for(int j(0);j<2;++j)
        // {
          // boost::mutex::scoped_lock lock(MutCamCapture[j]);
          // frame[j].copyTo(Frame[j]);
          // CapTime[j]= cap_time[j];
        // }
      // }

      // Show windows
      for(std::map<std::string, TIMShowStuff>::iterator itr(IMShowStuff.begin()),itr_end(IMShowStuff.end()); itr!=itr_end; ++itr)
      {
        boost::mutex::scoped_lock lock(*itr->second.Mutex);
        if(itr->second.Frame.total()>0)
          cv::imshow(itr->first, itr->second.Frame);
      }

      fps_est.Step();
      if(show_fps==0)
      {
        std::cerr<<"FPS(main): "<<fps_est.FPS<<std::endl;
        show_fps=fps_est.FPS*4;
      }
      --show_fps;

    }  // Running
    else
    {
      usleep(200*1000);
    }

    if(!HandleKeyEvent())  break;

    ros::spinOnce();
    disp_rate.sleep();
  }
  Shutdown= true;
  th_capture0.join();
  th_capture1.join();
  th_col_det.join();
  th_flow_find0.join();
  th_flow_find1.join();
  th_stereo.join();
  th_stereo_f.join();


  usleep(500*1000);

  return 0;
}
//-------------------------------------------------------------------------------------------
