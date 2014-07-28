/*
 *  Copyright (c) 2013-2014, Filippo Basso <bassofil@dei.unipd.it>,
 *                           Riccardo Levorato <riccardo.levorato@dei.unipd.it>
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
 */

#ifndef OPEN_PTRACK_OPT_CALIBRATION_CALIBRATION_NODE_H_
#define OPEN_PTRACK_OPT_CALIBRATION_CALIBRATION_NODE_H_

#include <ros/ros.h>
#include <ros/package.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <std_msgs/String.h>

#include <calibration_common/calibration_common.h>
#include <camera_info_manager/camera_info_manager.h>

#include <open_ptrack/opt_calibration/opt_calibration.h>

using namespace camera_info_manager;
using namespace calibration;

namespace open_ptrack
{
namespace opt_calibration
{

/** @brief class containing sensor information */
class SensorROS
{

public:

  typedef boost::shared_ptr<SensorROS> Ptr;
  typedef boost::shared_ptr<const SensorROS> ConstPtr;

  /**
   * @brief SensorROS
   * @param[in] frame_id The sensor frame.
   * @param[in] type The type of the sensor.
   */
  SensorROS(const std::string & frame_id,
            SensorNode::SensorType type)
    : frame_id_(frame_id),
      type_(type),
      new_image_(false)
  {
    // Do nothing
  }

  /**
   * @brief Callback for images.
   * @param[in] msg Message containing the image.
   */
  void imageCallback(const sensor_msgs::Image::ConstPtr & msg)
  {
    image_msg_ = msg;
    new_image_ = true;
  }

  /**
   * @brief Callback for camera_info topic, containing intrinsic sensor calibration.
   * @param[in] msg Message containing camera info (intrinsic parameters).
   */
  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr & msg)
  {
    if (not sensor_)
    {
      if (type_ == SensorNode::PINHOLE_RGB)
      {
        sensor_ = boost::make_shared<PinholeSensor>();
        sensor_->setFrameId(frame_id_);
        PinholeCameraModel::ConstPtr cm = boost::make_shared<PinholeCameraModel>(*msg);
        boost::static_pointer_cast<PinholeSensor>(sensor_)->setCameraModel(cm);
      }
      else
      {
        sensor_ = boost::make_shared<KinectDepthSensor<UndistortionModel> >();
        sensor_->setFrameId(frame_id_);
        KinectDepthCameraModel::ConstPtr cm = boost::make_shared<KinectDepthCameraModel>(*msg);
        boost::static_pointer_cast<KinectDepthSensor<UndistortionModel> >(sensor_)->setCameraModel(cm);
      }
    }
  }

  const std::string & frameId() const
  {
    return frame_id_;
  }

  void setImageSubscriber(const image_transport::Subscriber & image_sub)
  {
    image_sub_ = image_sub;
  }

  void setCameraInfoSubscriber(const ros::Subscriber & camera_info_sub)
  {
    camera_info_sub_ = camera_info_sub;
  }

  const sensor_msgs::Image::ConstPtr & lastImage()
  {
    assert(new_image_);
    new_image_ = false;
    return image_msg_;
  }

  bool hasNewImage() const
  {
    return new_image_;
  }

  const Sensor::Ptr & sensor() const
  {
    return sensor_;
  }

  bool isSensorSet() const
  {
    return sensor_;
  }

  SensorNode::SensorType type() const
  {
    return type_;
  }

private:

  std::string frame_id_;                    ///< Camera frame.

  image_transport::Subscriber image_sub_;   ///< ROS subscriber to the topic where the sensor publishes the image.
  ros::Subscriber camera_info_sub_;         ///< ROS subscriber to the topic where the sensor publishes the intrinsic calibration information.
  sensor_msgs::Image::ConstPtr image_msg_;  ///< Message containing the image.

  Sensor::Ptr sensor_;                      ///< Sensor object related to this SensorROS
  SensorNode::SensorType type_;             ///< Sensor type.

  bool new_image_;

};

class OPTCalibrationNode
{
public:

  /**
   * @brief Constructor.
   */
  OPTCalibrationNode(const ros::NodeHandle & node_handle);

  /**
   * @brief Callback for string messages which enables saving options.
   * @param[in] msg Message containing the command as a string.
   */
  void actionCallback(const std_msgs::String::ConstPtr & msg);

  /**
   * @brief Calibration initialization.
   * @return @c true whether the initialization has succeded, @c false otherwise.
   */
  bool initialize();

  /**
   * @brief Calibration main loop.
   */
  void spin();

private:

  ros::NodeHandle node_handle_;                             ///< @brief Handle to the ROS node.
  image_transport::ImageTransport image_transport_;         ///< @brief Handle to ImageTransport, which advertise and subscribe to image topics.

  ros::Subscriber action_sub_;                              ///< @brief Subscriber to topic where action commands are sent.
  Checkerboard::Ptr checkerboard_;                          ///< @brief Object representing a checkerboard.

  std::vector<SensorROS::Ptr> sensor_vec_;
  int num_sensors_;                                         ///< @brief Number of cameras connected to the network.

//  std::vector<std::string> sensor_frame_id_vec_;
//  std::map<std::string, SensorROS::Ptr> sensor_map_;
//  std::map<std::string, std::string> sensor_corr_map_;

  OPTCalibration::Ptr calibration_;                         ///< @brief Calibration object.

  bool calibration_with_serials_;                           ///< @brief Flag stating if sensors are identified with serial numbers (true) or with names (false).

};

} /* namespace opt_calibration */
} /* namespace open_ptrack */

#endif /* OPEN_PTRACK_OPT_CALIBRATION_CALIBRATION_NODE_H_ */
