//-------------------------------------------------------------------------------------------
/*! \file    sentis_m100.h
    \brief   Sentis ToF m100 manager
    \author  Akihiko Yamaguchi, xakiyam@gmail.com
    \version 0.1
    \date    Oct.23, 2014
*/
//-------------------------------------------------------------------------------------------
#ifndef sentis_m100_h
#define sentis_m100_h
//-------------------------------------------------------------------------------------------
#ifndef SENTIS_M100_USING_PCL
  #define SENTIS_M100_USING_PCL 1
#endif
#ifndef SENTIS_M100_USING_CV
  #define SENTIS_M100_USING_CV 1
#endif
//-------------------------------------------------------------------------------------------
#include <m100api.h>
#include <iostream>
#include <iomanip>
//-------------------------------------------------------------------------------------------
/* Modify the definition of XYZ_COORDS_DATA, which is originally defined in
  libm100/apitypes.h as 0x0010 but according to Sentis-ToF-M100_UM_V1.pdf
  Sec 8.6.1 (General registers - ImageDataFormat), it should be 0x0018.
  So, this seems to be a bug.  */
#undef XYZ_COORDS_DATA
#define XYZ_COORDS_DATA 0x0018
//-------------------------------------------------------------------------------------------
#define M100_IMAGE_WIDTH 160
#define M100_IMAGE_HEIGHT 120
#define M100_IMAGE_SIZE (M100_IMAGE_WIDTH*M100_IMAGE_HEIGHT)
//-------------------------------------------------------------------------------------------
// m100 registers.  See Sentis-ToF-M100_UM_V1.pdf
#define ModulationFrequency     0x0009
#define LedboardTemp      0x001B
#define CalibrationCommand      0x000F
#define CalibrationExtended     0x0021
#define FrameTime         0x001F
#define TempCompGradient2       0x0030
#define TempCompGradient3       0x003C
#define BuildYearMonth    0x003D
#define BuildDayHour      0x003E
#define BuildMinuteSecond       0x003F
#define UpTimeLow         0x0040
#define UpTimeHigh        0x0041
#define AkfPlausibilityCheckAmpLimit  0x0042
#define CommKeepAliveTimeout    0x004E
#define CommKeepAliveReset      0x004F
#define AecAvgWeight0     0x01A9
#define AecAvgWeight1     0x01AA
#define AecAvgWeight2     0x01AB
#define AecAvgWeight3     0x01AC
#define AecAvgWeight4     0x01AD
#define AecAvgWeight5     0x01AE
#define AecAvgWeight6     0x01AF
#define AecAmpTarget      0x01B0
#define AecTintStepMax    0x01B1
#define AecTintMax        0x01B2
#define AecKp       0x01B3
#define AecKi       0x01B4
#define AecKd       0x01B5
//-------------------------------------------------------------------------------------------
// m100 registers.  See Sentis-ToF-M100_UM_V1.pdf
#define CmdExecPassword     0x0022
//-------------------------------------------------------------------------------------------
// m100 registers.  See Sentis-ToF-M100_UM_V1.pdf
#define Eth0Config    0x0240
#define Eth0Mac2      0x0241
#define Eth0Mac1      0x0242
#define Eth0Mac0      0x0243
#define Eth0Ip0       0x0244
#define Eth0Ip1       0x0245
#define Eth0Snm0      0x0246
#define Eth0Snm1      0x0247
#define Eth0Gateway0        0x0248
#define Eth0Gateway1        0x0249
#define Eth0TcpStreamPort   0x024A
#define Eth0TcpConfigPort   0x024B
#define Eth0UdpStreamIp0    0x024C
#define Eth0UdpStreamIp1    0x024D
#define Eth0UdpStreamPort   0x024E
//-------------------------------------------------------------------------------------------
#if SENTIS_M100_USING_PCL==1
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#endif
#if SENTIS_M100_USING_CV==1
#include <opencv2/core/core.hpp>
#endif
//-------------------------------------------------------------------------------------------
namespace trick
{
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TSentisM100
//===========================================================================================
{
public:
  #define HEX2(x_val) std::hex<<std::setfill('0')<<std::setw(2)<<(x_val)<<std::dec
  #define HEX4(x_val) std::hex<<std::setfill('0')<<std::setw(4)<<(x_val)<<std::dec

  TSentisM100() : handle_(NULL), error_(0), data_format_(DEPTH_AMP_DATA) {}

  /*! Establish the connection and initialize the sensor.
    \param [in]init_fps     Initial FPS (1-40 Hz).
    \param [in]data_format  Data format.  Choose from {DEPTH_AMP_DATA, XYZ_COORDS_DATA, XYZ_AMP_DATA}.
  */
  bool Init(
      unsigned short init_fps= 1,
      unsigned short data_format= DEPTH_AMP_DATA,
      const char *tcp_ip= "192.168.0.10",
      const char *udp_ip= "224.0.0.1",
      unsigned short tcp_port= 10001,
      unsigned short udp_port= 10002,
      unsigned short integ_time= 573);

  // FIXME TODO implement close using  STSclose(handle_)

  bool SetDHCP(bool using_dhcp);

  bool GetFrameRate(unsigned short &frame_rate);

  //! Set the frame rate (1-40 Hz; 45 Hz seems to work)
  bool SetFrameRate(unsigned short frame_rate);

  //! Set the frame rate to 1 Hz (lowest FPS) to cool down
  bool Sleep()  {return SetFrameRate(1);}

  //! Save RegMap to flash
  bool SaveToFlash();

  bool GetData();

  #if SENTIS_M100_USING_PCL==1
  bool GetDataAsPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out);
  void BufferToPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out);
  #endif
  #if SENTIS_M100_USING_CV==1
  bool GetDataAsCVMat(cv::Mat *img_depth=NULL, cv::Mat *img_amp=NULL);
  void BufferToCVMat(cv::Mat *img_depth=NULL, cv::Mat *img_amp=NULL);
  #endif

  //! Print a part of registers.
  bool PrintRegisters(int preset);

  bool IsNoError(const char *msg=NULL)
    {
      if(error_==0)  return true;
      if(msg!=NULL)  std::cerr<<msg<<"Error: "<<error_<<std::endl;
      return false;
    }

  int CalcFrameSize()
    {
      switch(data_format_)
      {
      case DEPTH_AMP_DATA:   return 2*M100_IMAGE_SIZE*sizeof(unsigned short);
      case XYZ_COORDS_DATA:  return 3*M100_IMAGE_SIZE*sizeof(unsigned short);
      case XYZ_AMP_DATA:     return 4*M100_IMAGE_SIZE*sizeof(unsigned short);
      }
      return -1;
    }

  unsigned short ReadRegister(unsigned short address)
    {
      unsigned short res(0);
      error_= STSreadRegister(handle_, address, &res, 0, 0);
      return res;
    }
  bool WriteRegister(unsigned short address, unsigned short value)
    {
      error_= STSwriteRegister(handle_, address, value);
      return IsNoError();
    }
  bool PrintRegister(unsigned short address, const char *name="")
    {
      unsigned short res= ReadRegister(address);
      if(!IsNoError())  return false;
      std::cerr<<name<<": 0x"<<HEX4(res)
          <<", "<<res
          <<std::endl;
      return true;
    }
  bool PrintIPAddrRegister(unsigned short addr0, unsigned short addr1, const char *name="")
    {
      unsigned short res1= ReadRegister(addr1);
      if(!IsNoError())  return false;
      unsigned short res0= ReadRegister(addr0);
      if(!IsNoError())  return false;
      std::cerr<<name<<": "
          <<((res1&0xFF00)>>8)<<"."<<(res1&0x00FF)
          <<"."<<((res0&0xFF00)>>8)<<"."<<(res0&0x00FF)
          <<std::endl;
      return true;
    }
  bool PrintMacAddrRegister(unsigned short addr0, unsigned short addr1, unsigned short addr2, const char *name="")
    {
      unsigned short res2= ReadRegister(addr2);
      if(!IsNoError())  return false;
      unsigned short res1= ReadRegister(addr1);
      if(!IsNoError())  return false;
      unsigned short res0= ReadRegister(addr0);
      if(!IsNoError())  return false;
      std::cerr<<name<<": "
          <<HEX2((res2&0xFF00)>>8)<<":"<<HEX2(res2&0x00FF)
          <<":"<<HEX2((res1&0xFF00)>>8)<<":"<<HEX2(res1&0x00FF)
          <<":"<<HEX2((res0&0xFF00)>>8)<<":"<<HEX2(res0&0x00FF)
          <<std::dec<<std::endl;
      return true;
    }
public:
  // Accessors
  T_SENTIS_HANDLE& Handle() {return handle_;}
  const T_ERROR_CODE& ErrorCode() const {return error_;}
  unsigned short DataFormat() const {return data_format_;}
  const T_SENTIS_DATA_HEADER& DataHeader() const {return header_;}
  int FrameSize() const {return frame_size_/sizeof(unsigned short);}
  const unsigned short *const Buffer() const {return buffer_;}

private:
  T_SENTIS_HANDLE handle_;
  T_ERROR_CODE error_;
  unsigned short data_format_;

  T_SENTIS_DATA_HEADER header_;
  int frame_size_;
  /*! Buffer to store the observed data.  Its side depends on the data format.
      See Sentis-ToF-M100_UM_V1.pdf Sec 6.3 Camera Data Format.
      DEPTH_AMP_DATA: 2*M100_IMAGE_SIZE,
      XYZ_COORDS_DATA: 3*M100_IMAGE_SIZE,
      XYZ_AMP_DATA: 4*M100_IMAGE_SIZE. */
  unsigned short buffer_[4*M100_IMAGE_SIZE];

  #undef HEX2
  #undef HEX4
};
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // sentis_m100_h
//-------------------------------------------------------------------------------------------
