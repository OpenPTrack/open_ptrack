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

#include <fstream>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

#include <ceres/ceres.h>

#include <pcl/filters/filter_indices.h>
#include <pcl/filters/random_sample.h>

#include <calibration_common/pcl/utils.h>

#include <open_ptrack/opt_calibration/calibration_node.h>

namespace open_ptrack
{
namespace opt_calibration
{

OPTCalibrationNode::OPTCalibrationNode(const ros::NodeHandle & node_handle)
  : node_handle_(node_handle),
    image_transport_(node_handle)
{
  action_sub_ = node_handle_.subscribe("action", 1, &OPTCalibrationNode::actionCallback, this);

  node_handle_.param("num_sensors", num_sensors_, 0);

  double cell_width, cell_height;
  int rows, cols;

  bool cb_ok = true;
  cb_ok = cb_ok and node_handle_.getParam("cell_width", cell_width);
  cb_ok = cb_ok and node_handle_.getParam("cell_height", cell_height);
  cb_ok = cb_ok and node_handle_.getParam("rows", rows);
  cb_ok = cb_ok and node_handle_.getParam("cols", cols);
  if (not cb_ok)
    ROS_FATAL("Checkerboard parameter missing! Please set \"rows\", \"cols\", \"cell_width\" and \"cell_height\".");

  checkerboard_ = boost::make_shared<Checkerboard>(cols, rows, cell_width, cell_height);
  checkerboard_->setFrameId("/checkerboard");

  for (int i = 0; i < num_sensors_; ++i)
  {
    std::stringstream ss;

    std::string type_s;
    ss.str("");
    ss << "sensor_" << i << "/type";
    if (not node_handle_.getParam(ss.str(), type_s))
      ROS_FATAL_STREAM("No \"" << ss.str() << "\" parameter found!!");

    SensorNode::SensorType type;
    if (type_s == "pinhole_rgb")
      type = SensorNode::PINHOLE_RGB;
    else if (type_s == "kinect_depth")
      type = SensorNode::KINECT_DEPTH;
    else
      ROS_FATAL_STREAM("\"" << ss.str() << "\" parameter value not valid. Please use \"pinhole_rgb\" or \"kinect_depth\".");

    ss.str("");
    ss << "/sensor_" << i;
    std::string frame_id = ss.str();

    ss.str("");
    ss << "sensor_" << i << "/name";
    node_handle_.param(ss.str(), frame_id, frame_id);
    SensorROS::Ptr sensor = boost::make_shared<SensorROS>(frame_id, type);

    ss.str("");
    ss << "sensor_" << i << "/image";
    sensor->setImageSubscriber(image_transport_.subscribe(ss.str(), 1, &SensorROS::imageCallback, sensor));

    ss.str("");
    ss << "sensor_" << i << "/camera_info";
    sensor->setCameraInfoSubscriber(node_handle_.subscribe<sensor_msgs::CameraInfo>(ss.str(),
                                                                                    1,
                                                                                    &SensorROS::cameraInfoCallback,
                                                                                    sensor));

    sensor_vec_.push_back(sensor);
  }

}

bool OPTCalibrationNode::initialize()
{

  bool all_messages_received = false;
  ros::Rate rate(1.0);
  while (ros::ok() and not all_messages_received)
  {
    ros::spinOnce();
    all_messages_received = true;
    for (size_t i = 0; i < sensor_vec_.size(); ++i)
    {
      SensorROS::Ptr & sensor_ros = sensor_vec_[i];
      if (not sensor_ros->sensor())
      {
        ROS_WARN_THROTTLE(5, "Not all messages received. Waiting...");
        all_messages_received = false;
        break;
      }
    }
    rate.sleep();
  }

  ROS_INFO("All sensors connected.");

  calibration_ = boost::make_shared<OPTCalibration>(node_handle_);
  calibration_->setCheckerboard(checkerboard_);

  for (size_t i = 0; i < sensor_vec_.size(); ++i)
  {
    SensorROS::Ptr & sensor_ros = sensor_vec_[i];
    if (sensor_ros->type() == SensorNode::PINHOLE_RGB)
      calibration_->addSensor(boost::static_pointer_cast<PinholeSensor>(sensor_ros->sensor()));
  }

  return true;
}

void OPTCalibrationNode::actionCallback(const std_msgs::String::ConstPtr & msg)
{

  if (msg->data == "save" or msg->data == "saveExtrinsicCalibration")
  {
    calibration_->optimize();
    save();
  }
  else if (msg->data == "stop")
  {
    calibration_->optimize();
  }
}

void OPTCalibrationNode::spin()
{
  ros::Rate rate(5.0);

  while (ros::ok())
  {
    ros::spinOnce();

    calibration_->nextAcquisition();

    for (int i = 0; i < num_sensors_; ++i)
    {
      const SensorROS::Ptr & sensor_ros = sensor_vec_[i];

      if (not sensor_ros->hasNewImage())
        continue;
      try
      {
        if (sensor_ros->type() == SensorNode::PINHOLE_RGB)
        {
          cv_bridge::CvImage::Ptr image_ptr = cv_bridge::toCvCopy(sensor_ros->lastImage(),
                                                                  sensor_msgs::image_encodings::BGR8);
          calibration_->addData(boost::static_pointer_cast<PinholeSensor>(sensor_ros->sensor()), image_ptr->image);
        }
      }
      catch (cv_bridge::Exception & ex)
      {
        ROS_ERROR("cv_bridge exception: %s", ex.what());
        return;
      }
      catch (std::runtime_error & ex)
      {
        ROS_ERROR("exception: %s", ex.what());
        return;
      }

    }

    calibration_->perform();
    calibration_->publish();

    rate.sleep();

  }

}

bool OPTCalibrationNode::save()
{
  // Save tfs between sensors and world coordinate system (last checherboard) to file
  std::string file_name = ros::package::getPath("opt_calibration") + "/conf/camera_poses.yaml";
  std::ofstream file;
  file.open(file_name.c_str());

  if (file.is_open())
  {
    ros::Time time = ros::Time::now();
    file << "# Auto generated file." << std::endl;
    file << "calibration_id: " << time.sec << std::endl << std::endl;

    AngleAxis rotation(M_PI, Vector3(1.0, 1.0, 0.0).normalized());
    Pose new_world_pose = calibration_->getLastCheckerboardPose().inverse();

    // Write TF transforms between cameras and world frame
    file << "# Poses w.r.t. the \"world\" reference frame" << std::endl;
    file << "poses:" << std::endl;
    for (size_t i = 0; i < sensor_vec_.size(); ++i)
    {
      const SensorROS::Ptr & sensor_ros = sensor_vec_[i];
      const Sensor::Ptr & sensor = sensor_ros->sensor();

      Pose pose = rotation * new_world_pose * sensor->pose();

      file << "  " << sensor->frameId().substr(1) << ":" << std::endl;

      file << "    translation:" << std::endl
           << "      x: " << pose.translation().x() << std::endl
           << "      y: " << pose.translation().y() << std::endl
           << "      z: " << pose.translation().z() << std::endl;

      Quaternion rotation(pose.rotation());
      file << "    rotation:" << std::endl
           << "      x: " << rotation.x() << std::endl
           << "      y: " << rotation.y() << std::endl
           << "      z: " << rotation.z() << std::endl
           << "      w: " << rotation.w() << std::endl;

    }

    file << std::endl << "# Inverse poses" << std::endl;
    file << "inverse_poses:" << std::endl;
    for (size_t i = 0; i < sensor_vec_.size(); ++i)
    {
      const SensorROS::Ptr & sensor_ros = sensor_vec_[i];
      const Sensor::Ptr & sensor = sensor_ros->sensor();

      Pose pose = rotation * new_world_pose * sensor->pose();
      pose = pose.inverse();

      file << "  " << sensor->frameId().substr(1) << ":" << std::endl;

      file << "    translation:" << std::endl
           << "      x: " << pose.translation().x() << std::endl
           << "      y: " << pose.translation().y() << std::endl
           << "      z: " << pose.translation().z() << std::endl;

      Quaternion rotation(pose.rotation());
      file << "    rotation:" << std::endl
           << "      x: " << rotation.x() << std::endl
           << "      y: " << rotation.y() << std::endl
           << "      z: " << rotation.z() << std::endl
           << "      w: " << rotation.w() << std::endl;

    }

  }
  file.close();

  ROS_INFO_STREAM(file_name << " created!");

  return true;

}

} /* namespace opt_calibration */
} /* namespace open_ptrack */

int main(int argc,
         char ** argv)
{
  ros::init(argc, argv, "opt_calibration");
  ros::NodeHandle node_handle("~");

  try
  {
    open_ptrack::opt_calibration::OPTCalibrationNode calib_node(node_handle);
    if (not calib_node.initialize())
      return 0;
    calib_node.spin();
  }
  catch (std::runtime_error & error)
  {
    ROS_FATAL("Calibration error: %s", error.what());
    return 1;
  }

  return 0;
}
