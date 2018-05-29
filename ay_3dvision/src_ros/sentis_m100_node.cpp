//-------------------------------------------------------------------------------------------
/*! \file    sentis_m100_node.cpp
    \brief   Load data from Sentis M100 and send it as a point cloud topic.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Apr.22, 2015
*/
//-------------------------------------------------------------------------------------------
#include "ay_3dvision/sentis_m100.h"
// #include "ay_cpp/geom_util.h"
//-------------------------------------------------------------------------------------------
#include "ay_3dvision_msgs/ReadRegister.h"
#include "ay_3dvision_msgs/WriteRegister.h"
#include "ay_3dvision_msgs/SetFrameRate.h"
//-------------------------------------------------------------------------------------------
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/filters/passthrough.h>
//-------------------------------------------------------------------------------------------
namespace trick
{

inline double GetCurrentTime(void)
{
  struct timeval time;
  gettimeofday (&time, NULL);
  return static_cast<double>(time.tv_sec) + static_cast<double>(time.tv_usec)*1.0e-6;
  // return ros::Time::now().toSec();
}

class TSentisM100Node : public TSentisM100
{
public:
  TSentisM100Node(ros::NodeHandle &node)
      :
        TSentisM100(),
        node_(node)
    {
      srv_write_register_= node_.advertiseService("write_register", &TSentisM100Node::SrvWriteRegister, this);
      srv_read_register_= node_.advertiseService("read_register", &TSentisM100Node::SrvReadRegister, this);
      srv_set_frame_rate_= node_.advertiseService("set_frame_rate", &TSentisM100Node::SrvSetFrameRate, this);
    }

  bool SrvWriteRegister(ay_3dvision_msgs::WriteRegister::Request &req, ay_3dvision_msgs::WriteRegister::Response &res)
    {
      res.success= WriteRegister(req.address, req.value);
      return true;
    }

  bool SrvReadRegister(ay_3dvision_msgs::ReadRegister::Request &req, ay_3dvision_msgs::ReadRegister::Response &res)
    {
      res.value= ReadRegister(req.address);
      res.success= IsNoError("");
      return true;
    }

  bool SrvSetFrameRate(ay_3dvision_msgs::SetFrameRate::Request &req, ay_3dvision_msgs::SetFrameRate::Response &res)
    {
      res.success= SetFrameRate(req.frame_rate);
      return true;
    }

private:
  ros::NodeHandle     &node_;
  ros::ServiceServer  srv_write_register_;
  ros::ServiceServer  srv_read_register_;
  ros::ServiceServer  srv_set_frame_rate_;

};
//-------------------------------------------------------------------------------------------


}
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
using namespace trick;
//-------------------------------------------------------------------------------------------
// #define print(var) PrintContainer((var), #var"= ")
// #define print(var) std::cout<<#var"= "<<(var)<<std::endl
//-------------------------------------------------------------------------------------------


int main(int argc, char**argv)
{
  ros::init(argc, argv, "sentis_m100");
  ros::NodeHandle node("~");

  int init_fps, tcp_port, udp_port, integ_time;
  std::string tcp_ip, udp_ip;
  node.param("init_fps",init_fps,1);
  node.param("tcp_ip",tcp_ip,std::string("192.168.0.10"));
  node.param("udp_ip",udp_ip,std::string("224.0.0.1"));
  node.param("tcp_port",tcp_port,10001);
  node.param("udp_port",udp_port,10002);
  node.param("integ_time",integ_time,573);

  // M100_IMAGE_WIDTH,M100_IMAGE_HEIGHT

  ros::Publisher pub_cloud= node.advertise<sensor_msgs::PointCloud2>("depth_non_filtered", 1);
  // ros::Publisher pub_cloud= node.advertise<pcl::PointCloud<pcl::PointXYZ> >("/depth_non_filtered", 1);
  TSentisM100Node tof_sensor(node);
  tof_sensor.Init(init_fps, /*data_format=*/XYZ_COORDS_DATA,
                  tcp_ip.c_str(), udp_ip.c_str(), tcp_port, udp_port,
                  integ_time);
  // tof_sensor.PrintRegisters(0);
  tof_sensor.PrintRegisters(1);
  // tof_sensor.SetFrameRate(40);
  double t_start= GetCurrentTime();
  ros::Rate loop_rate(40);  // 40 Hz
  for(int f(0); ros::ok(); ++f)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if(!tof_sensor.GetDataAsPointCloud(cloud))  continue;

    // const double max_depth(0.5);
    // pcl::PassThrough<pcl::PointXYZ> pass;
    // pass.setInputCloud(cloud);
    // pass.setFilterFieldName("z");
    // pass.setFilterLimits(-100.0, max_depth);
    // pass.filter(*cloud);

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud,cloud_msg);
    cloud_msg.header.frame_id= "tf_sentis_tof";
    cloud_msg.header.stamp= ros::Time::now();
    pub_cloud.publish(cloud_msg);

    if(f%100==0)
    {
      double duration= GetCurrentTime()-t_start;
      std::cerr<<"Duration: "<<duration<<std::endl;
      std::cerr<<"FPS: "<<double(100)/duration<<std::endl;
      t_start= GetCurrentTime();
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  tof_sensor.Sleep();


  // ros::spin();
  return 0;
}
//-------------------------------------------------------------------------------------------
