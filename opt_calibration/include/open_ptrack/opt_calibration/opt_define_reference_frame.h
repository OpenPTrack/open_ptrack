/*
 *  Copyright (c) 2015- Open Perception, Inc.
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
 *  Author: Matteo Munaro [matteo.munaro@dei.unipd.it]
 *          Filippo Basso [bassofil@dei.unipd.it]
 */

#ifndef OPEN_PTRACK_OPT_DEFINE_REFERENCE_FRAME_H_
#define OPEN_PTRACK_OPT_DEFINE_REFERENCE_FRAME_H_

#include <ros/ros.h>
#include <ros/package.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <std_msgs/String.h>

#include <calibration_common/calibration_common.h>
#include <camera_info_manager/camera_info_manager.h>

#include <open_ptrack/opt_calibration/opt_calibration.h>
#include <open_ptrack/opt_calibration/ros_device.h>

#include <Eigen/Geometry>

using namespace camera_info_manager;
using namespace calibration;

namespace open_ptrack
{
namespace opt_calibration
{

class OPTDefineReferenceFrame
{
public:

  /**
   * @brief Constructor.
   */
  OPTDefineReferenceFrame(const ros::NodeHandle & node_handle);

  /**
   * @brief Calibration initialization.
   * @return @c true whether the initialization has succeeded, @c false otherwise.
   */
  bool initialize();

  /**
   * @brief Compute and save the new reference frame.
   * @param[in] reference_points_2d Reference points selected from the images.
   * @param[in] reference_points_3d Reference points coordinates in the new (user) reference frame.
   * @param[in] reference_camera_poses Vector containing all camera poses.
   * @param[in] reference_camera_info Vector containing camera_info messages of all cameras.
   */
  bool compute_and_save(const std::vector<Eigen::Vector2d>& reference_points_2d,
      const std::vector<Eigen::Vector3d>& reference_points_3d,
      std::vector<Pose> reference_camera_poses,
      std::vector<sensor_msgs::CameraInfo::ConstPtr> reference_camera_info);

  /**
   * @brief Save calibration results.
   * @param[in] correction_transform Transformation from original to user reference frame.
   * @return @c true whether the save has succeded, @c false otherwise.
   */
  bool save(Pose correction_transform);

  /**
   * @brief Main loop for defining user reference frame.
   */
  void spin();


private:

  ros::NodeHandle node_handle_;                             ///< @brief Handle to the ROS node.
  image_transport::ImageTransport image_transport_;         ///< @brief Handle to ImageTransport, which advertise and subscribe to image topics.

  std::map<std::string, int> images_acquired_map_;

  std::vector<PinholeRGBDevice::Ptr> pinhole_vec_;
  std::vector<KinectDevice::Ptr> kinect_vec_;
  std::vector<SwissRangerDevice::Ptr> swiss_ranger_vec_;

  std::vector<Sensor::Ptr> sensor_vec_;

  int num_sensors_;                                         ///< @brief Number of sensors connected to the network.

  OPTCalibration::Ptr calibration_;                         ///< @brief Calibration object.

  std::vector<Pose> camera_poses_;                          ///< @brief Vector containing all camera poses.

  std::vector<Pose> inverse_camera_poses_;                  ///< @brief Vector containing all inverse camera poses.

  std::vector<std::string> camera_frame_ids_;               ///< @brief Vector containing the frame_id of every camera in the network.

};

} /* namespace opt_calibration */
} /* namespace open_ptrack */

#endif /* OPEN_PTRACK_OPT_DEFINE_REFERENCE_FRAME_H_ */
