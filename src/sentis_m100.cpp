//-------------------------------------------------------------------------------------------
/*! \file    sentis_m100.cpp
    \brief   Sentis ToF m100 manager
    \author  Akihiko Yamaguchi, xakiyam@gmail.com
    \version 0.1
    \date    Oct.23, 2014
*/
//-------------------------------------------------------------------------------------------
#include "ay_3dvision/sentis_m100.h"
#include <cassert>
#include <fstream>  // for DEBUG
//-------------------------------------------------------------------------------------------
namespace trick
{
using namespace std;
// using namespace boost;


//===========================================================================================
// class TSentisM100
//===========================================================================================

/*! Establish the connection and initialize the sensor.
  \param [in]init_fps     Initial FPS (1-40 Hz).
  \param [in]data_format  Data format.  Choose from {DEPTH_AMP_DATA, XYZ_COORDS_DATA, XYZ_AMP_DATA}.
*/
bool TSentisM100::Init(
      unsigned short init_fps,
      unsigned short data_format,
      const char *tcp_ip,
      const char *udp_ip,
      unsigned short tcp_port,
      unsigned short udp_port,
      unsigned short integ_time)
{
  T_SENTIS_CONFIG config;
  config.tcp_ip= tcp_ip;
  config.udp_ip= udp_ip;
  config.tcp_port= tcp_port;
  config.udp_port= udp_port;
  config.flags= HOLD_CONTROL_ALIVE;

  data_format_= data_format;

  // Connect with the device
  std::cerr<<"Connecting to "<<tcp_ip<<std::endl;
  handle_= STSopen(&config, &error_);
  if(!IsNoError("Connection failed. "))  return false;

  if(!PrintRegister(Mode1,"Mode1"))  return false;
  std::cerr<<"Setting auto exposure."<<std::endl;
  if(!WriteRegister(Mode1, (ReadRegister(Mode1) | 0x8)))  return false;
  if(!PrintRegister(Mode1,"Mode1"))  return false;

  if(!PrintRegister(FrameRate,"FrameRate"))  return false;
  std::cerr<<"Setting frame rate."<<std::endl;
  if(!WriteRegister(FrameRate, init_fps))  return false;  // up to 40
  if(!PrintRegister(FrameRate,"FrameRate"))  return false;

  if(!PrintRegister(IntegrationTime,"IntegrationTime"))  return false;
  std::cerr<<"Setting integration time."<<std::endl;
  if(!WriteRegister(IntegrationTime, integ_time))  return false;
  if(!PrintRegister(IntegrationTime,"IntegrationTime"))  return false;

  if(!PrintRegister(ImageDataFormat,"ImageDataFormat"))  return false;
  std::cerr<<"Setting image data format."<<std::endl;
  if(!WriteRegister(ImageDataFormat, data_format_))  return false;
  if(!PrintRegister(ImageDataFormat,"ImageDataFormat"))  return false;

  return true;
}
//-------------------------------------------------------------------------------------------

bool TSentisM100::SetDHCP(bool using_dhcp)
{
  if(!PrintRegister(Eth0Config,"Eth0Config"))  return false;
  std::cerr<<"Setting DHCP mode."<<std::endl;
  unsigned short new_config= ReadRegister(Eth0Config);
  if(using_dhcp)  new_config= (new_config|0x0001);
  else            new_config= (new_config&0xFFFE);
  if(!WriteRegister(Eth0Config, new_config))  return false;
  if(!PrintRegister(Eth0Config,"Eth0Config"))  return false;
  std::cerr<<"  Note: in order to activate the DHCP mode, "
      "it is better to execute SaveToFlash, then restart the sensor."<<std::endl;
  return true;
}
//-------------------------------------------------------------------------------------------

bool TSentisM100::GetFrameRate(unsigned short &frame_rate)
{
  frame_rate= ReadRegister(FrameRate);
  return IsNoError();
}
//-------------------------------------------------------------------------------------------

//! Set the frame rate (1-40 Hz)
bool TSentisM100::SetFrameRate(unsigned short frame_rate)
{
  if(!PrintRegister(FrameRate,"FrameRate"))  return false;
  std::cerr<<"Setting frame rate."<<std::endl;
  if(!WriteRegister(FrameRate, frame_rate))  return false;
  if(!PrintRegister(FrameRate,"FrameRate"))  return false;
  return true;
}
//-------------------------------------------------------------------------------------------

bool TSentisM100::GetData()
{
  frame_size_= CalcFrameSize();
  // std::cerr<<"frame_size_:"<<frame_size_<<std::endl;
  error_= STSgetData(handle_, &header_,  (char*)buffer_, &frame_size_, 0, 0);
  // std::cerr<<"frame_size_:"<<frame_size_<<std::endl;
  if(!IsNoError("Failed to get data. "))  return false;
  if(header_.imageHeight!=M100_IMAGE_HEIGHT || header_.imageWidth!=M100_IMAGE_WIDTH)
  {
    std::cerr<<"Failed to get data (header is incorrect)."<<std::endl;
    return false;
  }
  return true;
}
//-------------------------------------------------------------------------------------------

#if SENTIS_M100_USING_PCL==1
bool TSentisM100::GetDataAsPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out)
{
  if(data_format_!=XYZ_COORDS_DATA)
  {
    std::cerr<<"In TSentisM100::GetDataAsPointCloud, error: data_format_!=XYZ_COORDS_DATA"<<std::endl;
    return false;
  }
  if(!GetData())  return false;
  BufferToPointCloud(cloud_out);
  return true;
}
//-------------------------------------------------------------------------------------------
void TSentisM100::BufferToPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out)
{
  if(data_format_!=XYZ_COORDS_DATA)
  {
    std::cerr<<"In TSentisM100::BufferToPointCloud, error: data_format_!=XYZ_COORDS_DATA"<<std::endl;
    return;
  }
  cloud_out->height= header_.imageHeight;
  cloud_out->width= header_.imageWidth;
  cloud_out->is_dense= true;
  cloud_out->points.resize(cloud_out->height * cloud_out->width);
  short *buffer(reinterpret_cast<short*>(buffer_));
  short *bx(buffer);
  short *by(buffer+M100_IMAGE_SIZE);
  short *bz(buffer+2*M100_IMAGE_SIZE);
  for(int i(0); i<M100_IMAGE_SIZE; ++bx,++by,++bz,++i)
  {
    pcl::PointXYZ &point(cloud_out->points[i]);
    point.x= double(*by)/1000.0;
    point.y= double(*bz)/1000.0;
    if((*bx)>=0.0)  point.z= double(*bx)/1000.0;
    else            point.z= nanf("");
  }
}
//-------------------------------------------------------------------------------------------
#endif

#if SENTIS_M100_USING_CV==1
bool TSentisM100::GetDataAsCVMat(cv::Mat *img_depth, cv::Mat *img_amp)
{
  if(!GetData())  return false;
  BufferToCVMat(img_depth, img_amp);
  return true;
}
//-------------------------------------------------------------------------------------------
void TSentisM100::BufferToCVMat(cv::Mat *img_depth, cv::Mat *img_amp)
{
  if(img_depth!=NULL)
  {
    if(img_depth->size()!=cv::Size(M100_IMAGE_WIDTH,M100_IMAGE_HEIGHT)
        || img_depth->type()!=CV_32FC1)
      img_depth->create(cv::Size(M100_IMAGE_WIDTH,M100_IMAGE_HEIGHT), CV_32FC1);
  }
  if(img_amp!=NULL)
  {
    if(img_amp->size()!=cv::Size(M100_IMAGE_WIDTH,M100_IMAGE_HEIGHT)
        || img_amp->type()!=CV_32FC1)
      img_amp->create(cv::Size(M100_IMAGE_WIDTH,M100_IMAGE_HEIGHT), CV_32FC1);
  }

  if(data_format_==DEPTH_AMP_DATA || data_format_==XYZ_AMP_DATA)
  {
    assert(img_depth!=NULL);
    assert(img_amp!=NULL);
    cv::MatIterator_<float> itr_depth(img_depth->begin<float>());
    cv::MatIterator_<float> itr_amp(img_amp->begin<float>());
    short *buffer(reinterpret_cast<short*>(buffer_));
    short *bx(buffer);
    short *ba(NULL);
    if(data_format_==DEPTH_AMP_DATA)     ba= buffer+M100_IMAGE_SIZE;
    else if(data_format_==XYZ_AMP_DATA)  ba= buffer+3*M100_IMAGE_SIZE;
    for(int i(0); i<M100_IMAGE_SIZE; ++bx,++ba,++itr_depth,++itr_amp,++i)
    {
      if((*bx)>=0.0)  (*itr_depth)= double(*bx)/1000.0;
      else            (*itr_depth)= 0;
      (*itr_amp)= (*ba);
    }
  }
  else if(data_format_==XYZ_COORDS_DATA)
  {
    assert(img_depth!=NULL);
    assert(img_amp==NULL);
    cv::MatIterator_<float> itr_depth(img_depth->begin<float>());
    short *buffer(reinterpret_cast<short*>(buffer_));
    short *bx(buffer);
    for(int i(0); i<M100_IMAGE_SIZE; ++bx,++itr_depth,++i)
    {
      if((*bx)>=0.0)  (*itr_depth)= double(*bx)/1000.0;
      else            (*itr_depth)= 0;
    }
  }
}
//-------------------------------------------------------------------------------------------
#endif


//! Save RegMap to flash
bool TSentisM100::SaveToFlash()
{
  if(!PrintRegister(CmdExec,"CmdExec"))  return false;
  if(!PrintRegister(CmdExecResult,"CmdExecResult"))  return false;

  std::cerr<<"###Save RegMap to flash..."<<std::endl;
  std::cerr<<"Setting CmdExecPassword."<<std::endl;
  if(!WriteRegister(CmdExecPassword, 0x4877))  return false;
  std::cerr<<"Save RegMap to flash."<<std::endl;
  if(!WriteRegister(CmdExec, 0xDD9E))  return false;
  if(!PrintRegister(CmdExecResult,"CmdExecResult"))  return false;

  if(!PrintRegister(CmdExec,"CmdExec"))  return false;
  if(!PrintRegister(CmdExecResult,"CmdExecResult"))  return false;

  return true;
}
//-------------------------------------------------------------------------------------------

/*! Print a part of registers.
    \param [in] preset  0: major registers, 1: Ethernet related. */
bool TSentisM100::PrintRegisters(int preset=0)
{
  if(handle_==NULL)  return false;

  #define PRINT_REGISTER(x_address) \
    if(!PrintRegister(x_address, #x_address))  {return false;} \

  if(preset==0)
  {
    PRINT_REGISTER(Mode0)
    PRINT_REGISTER(Status)
    PRINT_REGISTER(ImageDataFormat)
    PRINT_REGISTER(IntegrationTime)  // min 50 max 25000, current 573
    PRINT_REGISTER(DeviceType)
    PRINT_REGISTER(DeviceInfo)
    PRINT_REGISTER(FirmwareInfo)
    PRINT_REGISTER(ModulationFrequency)
    PRINT_REGISTER(FrameRate)
    PRINT_REGISTER(HardwareConfiguration)
    PRINT_REGISTER(SerialNumberLowWord)
    PRINT_REGISTER(SerialNumberHighWord)
    PRINT_REGISTER(FrameCounter)
    PRINT_REGISTER(CalibrationCommand)
    PRINT_REGISTER(ConfidenceThresLow)
    PRINT_REGISTER(ConfidenceThresHig)
    PRINT_REGISTER(Mode1)
    PRINT_REGISTER(CalculationTime)
    PRINT_REGISTER(LedboardTemp)
    PRINT_REGISTER(MainboardTemp)
    PRINT_REGISTER(LinearizationAmplitude)
    PRINT_REGISTER(LinearizationPhasseShift)
    PRINT_REGISTER(FrameTime)
    PRINT_REGISTER(CalibrationExtended)
    PRINT_REGISTER(MaxLedTemp)
    PRINT_REGISTER(HorizontalFov)
    PRINT_REGISTER(VerticalFov)
    PRINT_REGISTER(TriggerDelay)
    PRINT_REGISTER(BootloaderStatus)
    PRINT_REGISTER(TemperatureCompensationGradient)
    PRINT_REGISTER(ApplicationVersion)
    PRINT_REGISTER(DistCalibGradient)
    PRINT_REGISTER(TempCompGradient2)
    PRINT_REGISTER(CmdExec)
    PRINT_REGISTER(CmdExecResult)
    PRINT_REGISTER(FactoryMacAddr2)
    PRINT_REGISTER(FactoryMacAddr1)
    PRINT_REGISTER(FactoryMacAddr0)
    PRINT_REGISTER(FactoryYear)
    PRINT_REGISTER(FactoryMonthDay)
    PRINT_REGISTER(FactoryHourMinute)
    PRINT_REGISTER(FactoryTimezone)
    PRINT_REGISTER(TempCompGradient3)
    PRINT_REGISTER(BuildYearMonth)
    PRINT_REGISTER(BuildDayHour)
    PRINT_REGISTER(BuildMinuteSecond)
    PRINT_REGISTER(UpTimeLow)
    PRINT_REGISTER(UpTimeHigh)
    PRINT_REGISTER(AkfPlausibilityCheckAmpLimit)
    PRINT_REGISTER(CommKeepAliveTimeout)
    PRINT_REGISTER(CommKeepAliveReset)
    PRINT_REGISTER(AecAvgWeight0)
    PRINT_REGISTER(AecAvgWeight1)
    PRINT_REGISTER(AecAvgWeight2)
    PRINT_REGISTER(AecAvgWeight3)
    PRINT_REGISTER(AecAvgWeight4)
    PRINT_REGISTER(AecAvgWeight5)
    PRINT_REGISTER(AecAvgWeight6)
    PRINT_REGISTER(AecAmpTarget)
    PRINT_REGISTER(AecTintStepMax)
    PRINT_REGISTER(AecTintMax)
    PRINT_REGISTER(AecKp)
    PRINT_REGISTER(AecKi)
    PRINT_REGISTER(AecKd)
  }
  else if(preset==1)
  {
    PRINT_REGISTER(Eth0Config)
    if(!PrintMacAddrRegister(Eth0Mac0, Eth0Mac1, Eth0Mac2, "Eth0Mac"))  return false;
    if(!PrintIPAddrRegister(Eth0Ip0, Eth0Ip1, "Eth0Ip"))  return false;
    if(!PrintIPAddrRegister(Eth0Snm0, Eth0Snm1, "Eth0Snm"))  return false;
    if(!PrintIPAddrRegister(Eth0Gateway0, Eth0Gateway1, "Eth0Gateway"))  return false;
    PRINT_REGISTER(Eth0TcpStreamPort)
    PRINT_REGISTER(Eth0TcpConfigPort)
    if(!PrintIPAddrRegister(Eth0UdpStreamIp0, Eth0UdpStreamIp1, "Eth0UdpStreamIp"))  return false;
    PRINT_REGISTER(Eth0UdpStreamPort)
  }

  #undef PRINT_REGISTER

  return true;
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------

