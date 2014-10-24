// $Id: sr.cpp 39414 2012-05-10 19:52:30Z pbeeson $

/*
 * Copyright (c) 2010 TRACLabs Inc. 
 * This node uses the libmesasr API to support both SR 3000 and 4000
 * devices.  It extends the 2008 ROS swissranger (3000 only) node that
 * used the older libUSB API.  This verison also works with ros-core
 * >= v0.9.
 * Author: Patrick Beeson (pbeeson@traclabs.com) 
 *
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * Author: Radu Bogdan Rusu <rusu@cs.tum.edu>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <signal.h>
#include <ros/ros.h>
#include <boost/format.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>
#include <driver_base/driver.h>

#include <sr.h>
#include <swissranger_camera/SwissRangerConfig.h>
/** @file

@brief swissranger_camera is a ROS driver for Mesa Imaging SwissRangercamera_name_
3k and 4k devices

This is an extension of the older ROS swissranger node. It provides a
reliable driver with minimal dependencies, intended to fill a role in
the ROS image pipeline similar to the other ROS camera drivers.

The ROS image pipeline provides Bayer filtering at a higher level (in
image_proc).  In some cases it is useful to run the driver without the
entire image pipeline, so some image normalization is provided.

@par Advertises

 - \b distance/image_raw topic (sensor_msgs/Image) raw 2D camera image
   corresponding to measured range depth

 - \b intensity/image_raw topic (sensor_msgs/Image) raw 2D camera
   images corresponding to measured intensity (time synched with
   distance)

 - \b confidence/image_raw topic (sensor_msgs/Image) raw 2D camera
   images corresponding to measurement confidence (time synched with
   distance)

 - \b camera/camera_info topic (sensor_msgs/CameraInfo) Calibration
   information for each timestep.

 
@par Subscribes

 - None


@par Parameters

- \b frame_id : @b [string] camera frame of reference (Default: device
     node name)

- \b auto_exposure : @b [int] Whether to turn on auto exposure or
  not. 0 == off; > 0 == on; < 0 == use current settings. (Default: 1
  (on))

- \b integration_time : @b [int] Set integration time [SR3k:
  (integration_time+1)*0.200 ms; Sr 0.300ms+(integration_time)*0.100
  ms].  value < 0 results in no change to current settings.  Note:
  auto exposure adapts integration time online, so it is advisabel not
  to set this if using auto exposure.

- \b modulation_freq : @b [int] Set modulation frequency.  (Default:
  no value -- uses factory settings).  value < 0 results in no change
  to current settings.
  The devices employ the following values:
                    0  == 40MHz,  SR3k: maximal range 3.75m
                    1  == 30MHz,  SR3k, Sr: maximal range 5m
                    2  == 21MHz,  SR3k: maximal range 7.14m
                    3  == 20MHz,  SR3k: maximal range 7.5m
                    4  == 19MHz,  SR3k: maximal range 7.89m
                    5  == 60MHz,  Sr: maximal range 2.5m
                    6  == 15MHz,  Sr: maximal range 10m
                    7  == 10MHz,  Sr: maximal range 15m
                    8  == 29MHz,  Sr: maximal range 5.17m
                    9  == 31MHz,  Sr: maximal range 4.84m
                    10 == 14.5MHz, Sr: maximal range 10.34m
                    11 == 15.5MHz, Sr: maximal range 9.68m

- \b amp_threshold : @b [int] Setting this value will set all distance
  values to 0 if their amplitude is lower than the amplitude
  threshold. value < 0 results in no change to current settings.

- \b use_filter : @b [bool] Same as setting the USE_FILTER define in
  sr.h

**/

void sigsegv_handler(int sig);

class SRNode
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_d_, it_i_, it_c_;
  //std::string frame_id_;
  std::string camera_name_;
  std::string ether_addr_;
  sensor_msgs::Image image_d_, image_i_, image_c_, image_d16_;
  sensor_msgs::PointCloud cloud_;
  sensor_msgs::PointCloud2 cloud2_;
  sensor_msgs::CameraInfo cam_info_;

  camera_info_manager::CameraInfoManager *cinfo_;
  std::string camera_info_url_;

  // reconfigurable parameters
  // None right now, all are set once at runtime.


  ros::Publisher info_pub_;
  image_transport::Publisher image_pub_d_;
  image_transport::Publisher image_pub_i_;
  image_transport::Publisher image_pub_c_;
  image_transport::Publisher image_pub_d16_;
  ros::Publisher cloud_pub_;
  ros::Publisher cloud_pub2_;

  //int auto_exposure_;
  //int integration_time_;
  int modulation_freq_;
  //int amp_threshold_;
  bool use_filter_;

  static bool device_open_;

  dynamic_reconfigure::Server<swissranger_camera::SwissRangerConfig > dynsrv;
  swissranger_camera::SwissRangerConfig config_;
public:
  static sr::SR* dev_;

  SRNode(const ros::NodeHandle& nh): nh_(nh), it_d_(nh), it_i_(nh), it_c_(nh)
  {
    signal(SIGSEGV, &sigsegv_handler);

    info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
    image_pub_d_ = it_d_.advertise("distance/image_raw", 1);
    image_pub_i_ = it_i_.advertise("intensity/image_raw", 1);
    image_pub_c_ = it_c_.advertise("confidence/image_raw", 1);
    image_pub_d16_ = it_c_.advertise("distance/image_raw16", 1);
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("pointcloud_raw", 1);
    cloud_pub2_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud2_raw", 1);

    // set reconfigurable parameter defaults (all to automatic)
    // None right now

    try
    {
      dev_ = new sr::SR(use_filter_);
    }
    catch (sr::Exception& e)
    {
      ROS_ERROR_STREAM("Exception thrown while constructing camera driver: " << e.what ());
      nh_.shutdown();
      return;
    }

    getInitParams();
  } 

  ~SRNode()
  {
    if (dev_)
    {
      dev_->close();
      delete dev_;
      dev_ = NULL;
    }

    delete cinfo_;
  }

  /** get initial parameters (only when node starts). */
  void getInitParams(void)
  {
    nh_.param("modulation_freq", modulation_freq_, -1);
    std::string default_addr("");
    nh_.param("ether_addr",  ether_addr_, default_addr);
    nh_.param("camera_name",  camera_name_, std::string("swissranger"));

    cinfo_ = new camera_info_manager::CameraInfoManager(nh_, camera_name_);

    nh_.param("use_filter", use_filter_, static_cast<bool>(USE_FILTER));
    dynamic_reconfigure::Server<swissranger_camera::SwissRangerConfig>::CallbackType f = boost::bind(&SRNode::reconfig, this, _1, _2);
    dynsrv.setCallback(f);
  }

  /** Update reconfigurable parameter.
   *
   * This is done every frame, so we use getParamCached() to avoid
   * unnecessary parameter server requests.
   *
   * @param name ROS parameter private name
   * @param val current parameter value; updated on exit
   * @return true if @a val changed this time
   */
  bool inline updateParam(const std::string &name, int &val)
  {
    bool changed = false;
    int prev = val;
    if (nh_.getParamCached(name, val) && (val != prev))
      changed = true;
    return changed;
  }

  /** Update reconfigurable parameter (for strings). */
  bool inline updateParam(const std::string &name, std::string &val)
  {
    bool changed = false;
    std::string prev = val;
    if (nh_.getParamCached(name, val) && (val != prev))
      changed = true;
    return changed;
  }

  /** Check for changes in reconfigurable parameters. */
  void getParameters()
  {
    //    static bool first_cycle = true;

//     if (updateParam("brightness", brightness_) || first_cycle)
//       {
//         if (dev_->setBrightness(brightness_) >= 0)
//           {
//             if (brightness_ >= 0)
//               ROS_INFO ("[SRNode] Brightness set to %d", brightness_);
//             else
//               ROS_INFO ("[SRNode] Auto Brightness set");
//           }
//       }

//    first_cycle = false;
  }

  void reconfig(swissranger_camera::SwissRangerConfig & newconfig, uint32_t level)
  {
    ROS_DEBUG("dynamic reconfigure level 0x%x", level);

    // resolve frame ID using tf_prefix parameter
    if (newconfig.frame_id == "")
      newconfig.frame_id = camera_name_;
    if (config_.frame_id != newconfig.frame_id)
    {
      std::string tf_prefix = tf::getPrefixParam(nh_);
      ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
      newconfig.frame_id = tf::resolve(tf_prefix, newconfig.frame_id);
      config_.frame_id = newconfig.frame_id;
    }

    if (config_.camera_info_url != newconfig.camera_info_url)
    {
      // set the new URL and load CameraInfo (if any) from it
      if (cinfo_->validateURL(newconfig.camera_info_url))
      {
        // Create camera_info message:
        if (not cinfo_->loadCameraInfo(newconfig.camera_info_url))
        {
          cam_info_.header.frame_id = camera_name_;
          cam_info_.height = 144;
          cam_info_.width = 176;

          cam_info_.K[0] = 149.0;
          cam_info_.K[2] = 88.0;
          cam_info_.K[4] = 149.0;
          cam_info_.K[5] = 72.0;
          cam_info_.K[8] = 1.0;

          cam_info_.R[0] = 1.0;
          cam_info_.R[4] = 1.0;
          cam_info_.R[8] = 1.0;

          cam_info_.P[0] = cam_info_.K[0];
          cam_info_.P[2] = cam_info_.K[2];
          cam_info_.P[5] = cam_info_.K[4];
          cam_info_.P[6] = cam_info_.K[5];
          cam_info_.P[10] = cam_info_.K[8];

          cinfo_->setCameraInfo(cam_info_);
          newconfig.camera_info_url = "";
        }

      }
      else
      {
        // new URL not valid, use the old one
        newconfig.camera_info_url = config_.camera_info_url;
      }
    }

    if ((config_.auto_exposure != newconfig.auto_exposure) && device_open_)
    {
      if( newconfig.auto_exposure == 1)
      {
        SRNode::dev_->setAutoExposure(true);
      }
      else
      {
        SRNode::dev_->setAutoExposure(false);
      }
    }
    if ((config_.integration_time != newconfig.integration_time) && device_open_)
    {
      if( newconfig.auto_exposure != 1 )
      {
        SRNode::dev_->setIntegrationTime(newconfig.integration_time);
      }
    }
    if ((config_.amp_threshold != newconfig.amp_threshold) && device_open_)
    {
      if( newconfig.amp_threshold >= 0 )
      {
        SRNode::dev_->setAmplitudeThreshold(newconfig.amp_threshold);
      }
      else
      {
        SRNode::dev_->setAmplitudeThreshold(0);
      }
    }

    config_ = newconfig;                // save new parameters

    ROS_DEBUG_STREAM("[" << camera_name_  << "] reconfigured: frame_id " << newconfig.frame_id
                     << ", camera_info_url " << newconfig.camera_info_url);
  }

  /** Main driver loop */
  bool spin()
  {
    while (nh_.ok())
    {
      getParameters();                // check reconfigurable parameters

      // get current CameraInfo data
      cam_info_ = cinfo_->getCameraInfo();
      cloud2_.header.frame_id = cloud_.header.frame_id =
          image_d_.header.frame_id = image_i_.header.frame_id =
          image_c_.header.frame_id = image_d16_.header.frame_id =
          cam_info_.header.frame_id = camera_name_;//config_.frame_id;

      if(!device_open_)
      {
        try
        {
          if (dev_->open(config_.auto_exposure, config_.integration_time,
                         modulation_freq_, config_.amp_threshold, ether_addr_) == 0)
          {
            ROS_INFO_STREAM("[" << camera_name_ << "] Connected to device with ID: " << dev_->device_id_);
            ROS_INFO_STREAM("[" << camera_name_ << "] libmesasr version: " << dev_->lib_version_);
            device_open_ = true;
          }
          else
          {
            ros::Duration(3.0).sleep();
          }
        }
        catch (sr::Exception& e)
        {
          ROS_ERROR_STREAM("Exception thrown while connecting to the camera: " << e.what());
          ros::Duration(3.0).sleep();
        }
      }
      else
      {
        try
        {
          // Read data from the Camera
          dev_->readData(cloud_,cloud2_,image_d_, image_i_, image_c_, image_d16_);

          cam_info_.header.stamp = image_d_.header.stamp;
          cam_info_.height = image_d_.height;
          cam_info_.width = image_d_.width;

          // Publish it via image_transport
          if (info_pub_.getNumSubscribers() > 0)
            info_pub_.publish(cam_info_);
          if (image_pub_d_.getNumSubscribers() > 0)
            image_pub_d_.publish(image_d_);
          if (image_pub_i_.getNumSubscribers() > 0)
            image_pub_i_.publish(image_i_);
          if (image_pub_c_.getNumSubscribers() > 0)
            image_pub_c_.publish(image_c_);
          if (image_pub_d16_.getNumSubscribers() > 0)
            image_pub_d16_.publish(image_d16_);
          if (cloud_pub_.getNumSubscribers() > 0)
            cloud_pub_.publish (cloud_);
          if (cloud_pub2_.getNumSubscribers() > 0)
            cloud_pub2_.publish (cloud2_);
        }
        catch (sr::Exception & e)
        {
          ROS_WARN_STREAM("Exception thrown trying to read data: " << e.what());
          dev_->close();
          device_open_ = false;
          ros::Duration(3.0).sleep();
        }
      }
      ros::spinOnce();
    }

    return true;
  }
};

// TODO: figure out a clean way to do this inside SRNode:
sr::SR * SRNode::dev_ = NULL;
bool SRNode::device_open_ = false;

/** Segfault signal handler */
void sigsegv_handler(int sig)
{
  signal(SIGSEGV, SIG_DFL);
  fprintf(stderr, "Segmentation fault, stopping camera driver.\n");
  if (SRNode::dev_)
  {
    SRNode::dev_->close();
  }
}

/** Main entry point */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "sr");
  ros::NodeHandle nh("~");

  SRNode cm(nh);

  cm.spin();
  return 0;
}
