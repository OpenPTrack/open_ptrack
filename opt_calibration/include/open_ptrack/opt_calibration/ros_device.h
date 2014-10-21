/*
 *  Copyright (c) 2013- Filippo Basso, Riccardo Levorato, Matteo Munaro
 *  Copyright (c) 2014-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *     3. Neither the name of the copyright holder(s) nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: Filippo Basso [bassofil@dei.unipd.it]
 *          Riccardo Levorato [levorato@dei.unipd.it]
 *          Matteo Munaro [matteo.munaro@dei.unipd.it]
 */

#ifndef OPEN_PTRACK_OPT_CALIBRATION_ROS_DEVICE_H_
#define OPEN_PTRACK_OPT_CALIBRATION_ROS_DEVICE_H_

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <swissranger_camera/utility.h>

#include <calibration_common/pinhole/sensor.h>
#include <kinect/depth/sensor.h>
#include <kinect/depth/polynomial_matrix_model.h>

namespace open_ptrack
{
namespace opt_calibration
{

namespace cb = calibration;

/** @brief class containing ROS device information */
class ROSDevice
{

public:

  typedef boost::shared_ptr<ROSDevice> Ptr;
  typedef boost::shared_ptr<const ROSDevice> ConstPtr;

  ROSDevice()
    : has_new_messages_(false)
  {
    // Do nothing
  }

  inline bool hasNewMessages() const
  {
    return has_new_messages_;
  }

  virtual void createSubscribers(ros::NodeHandle & nh,
                                 image_transport::ImageTransport & image_transport_nh,
                                 const std::string & main_topic) = 0;

  virtual bool allSensorsSet() const = 0;

protected:

  inline void setHasNewMessages(bool has_new_messages) const
  {
    has_new_messages_ = has_new_messages;
  }

private:

  mutable bool has_new_messages_;

};

class PinholeRGBDevice : public ROSDevice
{

public:

  typedef boost::shared_ptr<PinholeRGBDevice> Ptr;
  typedef boost::shared_ptr<const PinholeRGBDevice> ConstPtr;

  typedef ROSDevice Base;

  struct Messages
  {
    sensor_msgs::Image::ConstPtr image_msg;
    sensor_msgs::CameraInfo::ConstPtr camera_info_msg;
  };

  struct Data
  {
    typedef boost::shared_ptr<Data> Ptr;
    typedef boost::shared_ptr<const Data> ConstPtr;
    cv::Mat image;
  };

  PinholeRGBDevice(const std::string & frame_id)
    : Base(),
      frame_id_(frame_id)
  {
    // Do nothing
  }

  virtual ~PinholeRGBDevice() {}

  inline const std::string & frameId() const
  {
    return frame_id_;
  }

  inline const Messages & lastMessages() const
  {
    if (hasNewMessages())
      setHasNewMessages(false);
    return last_messages_;
  }

  inline const Data::Ptr & convertLastMessages()
  {
    if (hasNewMessages())
    {
      setHasNewMessages(false);
      last_data_ = convertMessages(last_messages_);
    }
    return lastData();
  }

  inline const Data::Ptr & lastData() const
  {
    return last_data_;
  }

  inline const cb::PinholeSensor::Ptr & sensor() const
  {
    return sensor_;
  }

  inline bool isSensorSet() const
  {
    return sensor_;
  }

  virtual bool allSensorsSet() const
  {
    return isSensorSet();
  }

  void imageCallback(const sensor_msgs::Image::ConstPtr & image_msg);

  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr & camera_info_msg);

  virtual void createSubscribers(ros::NodeHandle & nh,
                                 image_transport::ImageTransport & image_transport_nh,
                                 const std::string & main_topic);

  Data::Ptr convertMessages(const Messages & messages);

private:

  std::string frame_id_;

  image_transport::Subscriber image_sub_;
  ros::Subscriber camera_info_sub_;
  Messages last_messages_;
  Data::Ptr last_data_;

  cb::PinholeSensor::Ptr sensor_;

};


class KinectDevice : public ROSDevice
{

public:

  typedef boost::shared_ptr<KinectDevice> Ptr;
  typedef boost::shared_ptr<const KinectDevice> ConstPtr;

  typedef ROSDevice Base;

  struct Messages
  {
    sensor_msgs::Image::ConstPtr image_msg;
    sensor_msgs::CameraInfo::ConstPtr camera_info_msg;
//    sensor_msgs::Image::ConstPtr depth_image_msg;
//    sensor_msgs::CameraInfo::ConstPtr depth_camera_info_msg;
    sensor_msgs::PointCloud2::ConstPtr cloud_msg;
  };

  struct Data
  {
    typedef boost::shared_ptr<Data> Ptr;
    typedef boost::shared_ptr<const Data> ConstPtr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    cv::Mat image;
  };

  KinectDevice(const std::string & color_frame_id,
               const std::string & depth_frame_id)
    : Base(),
      color_frame_id_(color_frame_id),
      depth_frame_id_(depth_frame_id),
      depth_sensor_(boost::make_shared<cb::DepthSensor>(cb::Vector3(0.0, 0.0, 0.0035)))
  {
    // Do nothing
  }

  virtual ~KinectDevice() {}

  inline const std::string & colorFrameId() const
  {
    return color_frame_id_;
  }

  inline const std::string & depthFrameId() const
  {
    return depth_frame_id_;
  }

  inline const Messages & lastMessages() const
  {
    if (hasNewMessages())
      setHasNewMessages(false);
    return last_messages_;
  }

  inline const Data::Ptr & convertLastMessages()
  {
    if (hasNewMessages())
    {
      setHasNewMessages(false);
      last_data_ = convertMessages(last_messages_);
    }
    return lastData();
  }

  inline const Data::Ptr & lastData() const
  {
    return last_data_;
  }

  inline const cb::PinholeSensor::Ptr & colorSensor() const
  {
    return color_sensor_;
  }

  inline const cb::DepthSensor::Ptr & depthSensor() const
  {
    return depth_sensor_;
  }

  inline bool isDepthSensorSet() const
  {
    return depth_sensor_;
  }

  inline bool isColorSensorSet() const
  {
    return color_sensor_;
  }

  virtual bool allSensorsSet() const
  {
    return isColorSensorSet() and isDepthSensorSet();
  }

  void imageCallback(const sensor_msgs::Image::ConstPtr & image_msg,
                     const sensor_msgs::CameraInfo::ConstPtr & camera_info_msg);

//  void depthCallback(const sensor_msgs::Image::ConstPtr & image_msg,
//                     const sensor_msgs::CameraInfo::ConstPtr & camera_info_msg);

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr & cloud_msg);

  virtual void createSubscribers(ros::NodeHandle & nh,
                                 image_transport::ImageTransport & image_transport_nh,
                                 const std::string & main_topic);

  Data::Ptr convertMessages(const Messages & messages);

private:

  std::string color_frame_id_;
  std::string depth_frame_id_;

  image_transport::CameraSubscriber camera_sub_;
//  image_transport::CameraSubscriber depth_sub_;
  ros::Subscriber cloud_sub_;
  Messages last_messages_;
  Data::Ptr last_data_;

  cb::DepthSensor::Ptr depth_sensor_;
  cb::PinholeSensor::Ptr color_sensor_;

};

class SwissRangerDevice : public ROSDevice
{

public:

  typedef boost::shared_ptr<SwissRangerDevice> Ptr;
  typedef boost::shared_ptr<const SwissRangerDevice> ConstPtr;

  typedef ROSDevice Base;

  struct Messages
  {
    sensor_msgs::PointCloud2::ConstPtr cloud_msg;
    sensor_msgs::Image::ConstPtr intensity_msg;
    sensor_msgs::CameraInfo::ConstPtr camera_info_msg;
  };

  struct Data
  {
    typedef boost::shared_ptr<Data> Ptr;
    typedef boost::shared_ptr<const Data> ConstPtr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    cv::Mat intensity_image;
    cv::Mat confidence_image;
  };

  SwissRangerDevice(const std::string & frame_id)
    : Base(),
      frame_id_(frame_id),
      depth_sensor_(boost::make_shared<cb::DepthSensor>(cb::Vector3(0.02, 0.0, 0.0))),
      confidence_threshold_(0.95f)
  {
    // Do nothing
  }

  virtual ~SwissRangerDevice() {}

  inline const std::string & frameId() const
  {
    return frame_id_;
  }

  inline const Messages & lastMessages() const
  {
    if (hasNewMessages())
      setHasNewMessages(false);
    return last_messages_;
  }

  inline const Data::Ptr & convertLastMessages()
  {
    if (hasNewMessages())
    {
      setHasNewMessages(false);
      last_data_ = convertMessages(last_messages_);
    }
    return lastData();
  }

  inline const Data::Ptr & lastData() const
  {
    return last_data_;
  }

  inline void setConfidenceThreshold(float threshold)
  {
    confidence_threshold_ = threshold;
  }

  inline const cb::PinholeSensor::Ptr & intensitySensor() const
  {
    return intensity_sensor_;
  }

  inline const cb::DepthSensor::Ptr & depthSensor() const
  {
    return depth_sensor_;
  }

  inline bool isDepthSensorSet() const
  {
    return depth_sensor_;
  }

  inline bool isIntensitySensorSet() const
  {
    return intensity_sensor_;
  }

  virtual bool allSensorsSet() const
  {
    return isIntensitySensorSet() and isDepthSensorSet();
  }

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr & cloud_msg);

  void imageCallback(const sensor_msgs::Image::ConstPtr & image_msg,
                     const sensor_msgs::CameraInfo::ConstPtr & camera_info_msg);

  virtual void createSubscribers(ros::NodeHandle & nh,
                                 image_transport::ImageTransport & image_transport_nh,
                                 const std::string & main_topic);

  Data::Ptr convertMessages(const Messages & messages);

private:

  std::string frame_id_;

  cb::PinholeSensor::Ptr intensity_sensor_ ;
  cb::DepthSensor::Ptr depth_sensor_;

  ros::Subscriber cloud_sub_;
  image_transport::CameraSubscriber camera_sub_;
  Messages last_messages_;
  Data::Ptr last_data_;
  float confidence_threshold_;

};


} /* namespace opt_calibration */
} /* namespace open_ptrack */

#endif /* OPEN_PTRACK_OPT_CALIBRATION_ROS_DEVICE_H_ */
