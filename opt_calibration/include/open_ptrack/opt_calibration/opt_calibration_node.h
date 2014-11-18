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

#ifndef OPEN_PTRACK_OPT_CALIBRATION_OPT_CALIBRATION_NODE_H_
#define OPEN_PTRACK_OPT_CALIBRATION_OPT_CALIBRATION_NODE_H_

#include <ros/ros.h>
#include <ros/package.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <std_msgs/String.h>
#include <opt_msgs/CalibrationStatus.h>

#include <calibration_common/calibration_common.h>
#include <camera_info_manager/camera_info_manager.h>

#include <open_ptrack/opt_calibration/opt_calibration.h>
#include <open_ptrack/opt_calibration/ros_device.h>

using namespace camera_info_manager;
using namespace calibration;

namespace open_ptrack
{
namespace opt_calibration
{

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

  /**
   * @brief Save calibration results.
   * @return @c true whether the save has succeded, @c false otherwise.
   */
  bool save();

private:

  enum WorldComputation
  {
    FIRST_SENSOR,
    LAST_CHECKERBOARD,
    UPDATE
  };

  ros::NodeHandle node_handle_;                             ///< @brief Handle to the ROS node.
  image_transport::ImageTransport image_transport_;         ///< @brief Handle to ImageTransport, which advertise and subscribe to image topics.

  ros::Subscriber action_sub_;                              ///< @brief Subscriber to topic where action commands are sent.
  Checkerboard::Ptr checkerboard_;                          ///< @brief Object representing a checkerboard.

  ros::Publisher status_pub_;
  std::map<std::string, int> images_acquired_map_;
  opt_msgs::CalibrationStatus status_msg_;

  WorldComputation world_computation_;
  Sensor::Ptr fixed_sensor_;
  Pose fixed_sensor_pose_;

  std::vector<PinholeRGBDevice::Ptr> pinhole_vec_;
  std::vector<KinectDevice::Ptr> kinect_vec_;
  std::vector<SwissRangerDevice::Ptr> swiss_ranger_vec_;

  std::vector<Sensor::Ptr> sensor_vec_;

  int num_sensors_;                                         ///< @brief Number of sensors connected to the network.

  OPTCalibration::Ptr calibration_;                         ///< @brief Calibration object.

};

} /* namespace opt_calibration */
} /* namespace open_ptrack */

#endif /* OPEN_PTRACK_OPT_CALIBRATION_OPT_CALIBRATION_NODE_H_ */
