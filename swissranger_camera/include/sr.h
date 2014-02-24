
#ifndef SR_HH
#define SR_HH

#include <libMesaSR.h>

// ROS include
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#define USE_SR4K 1  // Comment this out for SR3K
#define USE_FILTER 1

#define SR_IMG_DISTANCE   0
#define SR_IMG_AMPLITUDE  1
#define SR_IMG_CONFIDENCE 2

#define SR4K_MODE (AM_CONF_MAP | AM_CONV_GRAY | AM_COR_FIX_PTRN | AM_DENOISE_ANF)
#define SR3K_MODE (AM_COR_FIX_PTRN | AM_MEDIAN)

#ifdef USE_SR4K
#define MODE SR4K_MODE
#else
#define MODE SR3K_MODE
#endif

namespace sr
{
 
  using namespace std;
  //! Macro for defining an exception with a given parent (std::runtime_error should be top parent)
  // code borrowed from drivers/laser/hokuyo_driver/hokuyo.h
#define DEF_EXCEPTION(name, parent)		\
  class name  : public parent {			\
  public:					\
    name (const char* msg) : parent (msg) {}	\
  }
  
  //! A standard SR exception
  DEF_EXCEPTION(Exception, std::runtime_error);
  
  const int SR_COLS = 176;
  const int SR_ROWS = 144;
#ifdef USE_SR4K 
  const int SR_IMAGES = 3; 
#else 
  const int SR_IMAGES = 2; 
#endif  

  class SR
  {
  public:
    SR (bool use_filter=USE_FILTER);
    ~SR ();
    
    int open (int auto_exposure, int integration_time, 
       int modulation_freq, int amp_threshold, std::string &ether_addr);
    int close();
    
    void readData (sensor_msgs::PointCloud &cloud,
		   sensor_msgs::PointCloud2 &cloud2,
		   sensor_msgs::Image &image_d,
		   sensor_msgs::Image &image_i,
		   sensor_msgs::Image &image_c,
                   sensor_msgs::Image &image_d16);
    
    int setAutoExposure (bool on);
    int setIntegrationTime (int time);
    int getIntegrationTime ();
    int setModulationFrequency (int freq);
    int getModulationFrequency ();
    int setAmplitudeThreshold (int thresh);
    int getAmplitudeThreshold ();
    
    std::string device_id_;
    std::string lib_version_;
    
  private:
    // device identifier
    CMesaDevice* srCam_;

    ImgEntry* imgEntryArray_;
    float *buffer_, *xp_, *yp_, *zp_;
    
    int integration_time_, modulation_freq_;

    bool use_filter_;
    
    std::string getDeviceString ();
    std::string getLibraryVersion ();
      
    void SafeCleanup();
  
  };
};

#endif
