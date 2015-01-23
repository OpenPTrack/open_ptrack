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
#include <omp.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

#include <ceres/ceres.h>

#include <pcl/filters/filter_indices.h>
#include <pcl/filters/random_sample.h>

#include <calibration_common/pcl/utils.h>

#include <open_ptrack/opt_calibration/opt_calibration_node.h>
#include <opt_msgs/CalibrationStatus.h>

namespace open_ptrack
{
namespace opt_calibration
{

OPTCalibrationNode::OPTCalibrationNode(const ros::NodeHandle & node_handle)
  : node_handle_(node_handle),
    image_transport_(node_handle),
    world_computation_(FIRST_SENSOR),
    fixed_sensor_pose_(Pose::Identity())
{
  action_sub_ = node_handle_.subscribe("action", 1, &OPTCalibrationNode::actionCallback, this);
  status_pub_ = node_handle_.advertise<opt_msgs::CalibrationStatus>("status", 1);

  std::string world_computation_s;
  node_handle_.param("world_computation", world_computation_s, std::string("first_sensor"));
  if (world_computation_s == "first_sensor")
    world_computation_ = FIRST_SENSOR;
  else if (world_computation_s == "last_checkerboard")
    world_computation_ = LAST_CHECKERBOARD;
  else if (world_computation_s == "update")
    world_computation_ = UPDATE;
  else
    ROS_FATAL_STREAM("\"world_computation\" parameter value not valid. Please use \"first_sensor\", \"last_checkerboard\" or \"manual\".");

  std::string fixed_sensor_frame_id;
  if (world_computation_ == UPDATE)
  {
    if (not node_handle_.getParam("fixed_sensor/name", fixed_sensor_frame_id))
      ROS_FATAL_STREAM("No \"fixed_sensor/name\" parameter found!!");

    std::string pose_s = "/poses/" + fixed_sensor_frame_id;

    if (not node_handle_.hasParam(pose_s))
      ROS_FATAL_STREAM("No \"" << pose_s << "\" parameter found!! Has \"conf/camera_poses.yaml\" been loaded with \"rosparam load ...\"?");

    Translation3 t;
    node_handle_.getParam(pose_s + "/translation/x", t.x());
    node_handle_.getParam(pose_s + "/translation/y", t.y());
    node_handle_.getParam(pose_s + "/translation/z", t.z());

    Quaternion q;
    node_handle_.getParam(pose_s + "/rotation/x", q.x());
    node_handle_.getParam(pose_s + "/rotation/y", q.y());
    node_handle_.getParam(pose_s + "/rotation/z", q.z());
    node_handle_.getParam(pose_s + "/rotation/w", q.w());

    fixed_sensor_pose_.linear() = q.toRotationMatrix();
    fixed_sensor_pose_.translation() = t.vector();
  }

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

//    SensorNode::SensorType type;
//    if (type_s == "pinhole_rgb")
//      type = SensorNode::PINHOLE_RGB;
//    else if (type_s == "kinect_depth")
//      type = SensorNode::KINECT_DEPTH;
//    else if (type_s == "tof_depth")
//      type = SensorNode::TOF_DEPTH;
//    else
//      ROS_FATAL_STREAM("\"" << ss.str() << "\" parameter value not valid. Please use \"pinhole_rgb\", \"kinect_depth\" or \"tof_depth\".");

    ss.str("");
    ss << "/sensor_" << i;
    std::string frame_id = ss.str();

    ss.str("");
    ss << "sensor_" << i << "/name";
    node_handle_.param(ss.str(), frame_id, frame_id);

    ROSDevice::Ptr ros_device;

    if (type_s == "pinhole_rgb")
    {
      PinholeRGBDevice::Ptr device = boost::make_shared<PinholeRGBDevice>(frame_id);
      pinhole_vec_.push_back(device);
      ros_device = device;
      ROS_INFO_STREAM(device->frameId() << " added.");
      if (world_computation_ == UPDATE)
      {
        ROS_INFO_STREAM(frame_id << " == " << fixed_sensor_frame_id << "?");
        if (frame_id == fixed_sensor_frame_id)
           fixed_sensor_ = device->sensor();
      }
    }
    else if (type_s == "kinect1")
    {
      KinectDevice::Ptr device = boost::make_shared<KinectDevice>(frame_id, frame_id + "_depth");
      kinect_vec_.push_back(device);
      ros_device = device;
      if (world_computation_ == UPDATE)
      {
        ROS_INFO_STREAM(frame_id << " == " << fixed_sensor_frame_id << "?");
        if (frame_id == fixed_sensor_frame_id)
           fixed_sensor_ = device->colorSensor();
      }
    }
    else if (type_s == "sr4500")
    {
      SwissRangerDevice::Ptr device = boost::make_shared<SwissRangerDevice>(frame_id);
      swiss_ranger_vec_.push_back(device);
      ros_device = device;
      if (world_computation_ == UPDATE)
      {
        ROS_INFO_STREAM(frame_id << " == " << fixed_sensor_frame_id << "?");
        if (frame_id == fixed_sensor_frame_id)
           fixed_sensor_ = device->depthSensor();
      }
    }
    else
    {
      ROS_FATAL_STREAM("\"" << ss.str() << "\" parameter value not valid. Please use \"pinhole_rgb\", \"kinect\" or \"swiss_ranger\".");
    }

    ss.str("");
    ss << "sensor_" << i;
    ros_device->createSubscribers(node_handle_, image_transport_, ss.str());

  }

  if (world_computation_ == UPDATE and not fixed_sensor_)
    ROS_FATAL_STREAM("Wrong \"fixed_sensor/name\" parameter provided: " << fixed_sensor_frame_id);
}

bool OPTCalibrationNode::initialize()
{
  bool all_messages_received = false;
  ros::Rate rate(1.0);
  while (ros::ok() and not all_messages_received)
  {
    ros::spinOnce();
    all_messages_received = true;
    for (size_t i = 0; all_messages_received and i < pinhole_vec_.size(); ++i)
      all_messages_received = pinhole_vec_[i]->hasNewMessages();
    for (size_t i = 0; all_messages_received and i < kinect_vec_.size(); ++i)
      all_messages_received = kinect_vec_[i]->hasNewMessages();
    for (size_t i = 0; all_messages_received and i < swiss_ranger_vec_.size(); ++i)
      all_messages_received = swiss_ranger_vec_[i]->hasNewMessages();

    if (not all_messages_received)
      ROS_WARN_THROTTLE(5, "Not all messages received. Waiting...");
    rate.sleep();
  }

  ROS_INFO("All sensors connected.");

  calibration_ = boost::make_shared<OPTCalibration>(node_handle_);
  calibration_->setCheckerboard(checkerboard_);

  for (size_t i = 0; i < pinhole_vec_.size(); ++i)
  {
    const PinholeRGBDevice::Ptr & device = pinhole_vec_[i];
    calibration_->addSensor(device->sensor(), true);
    sensor_vec_.push_back(device->sensor());
    images_acquired_map_[device->frameId()] = 0;
    status_msg_.sensor_ids.push_back(device->frameId());
  }

  for (size_t i = 0; i < kinect_vec_.size(); ++i) // TODO Add flags
  {
    const KinectDevice::Ptr & device = kinect_vec_[i];
    calibration_->addSensor(device->colorSensor(), true);
    calibration_->addSensor(device->depthSensor(), true);
    sensor_vec_.push_back(device->colorSensor());
    sensor_vec_.push_back(device->depthSensor());
    images_acquired_map_[device->colorFrameId()] = 0;
    images_acquired_map_[device->depthFrameId()] = 0;
    status_msg_.sensor_ids.push_back(device->colorFrameId());
    status_msg_.sensor_ids.push_back(device->depthFrameId());
  }

  for (size_t i = 0; i < swiss_ranger_vec_.size(); ++i) // TODO Add flags
  {
    const SwissRangerDevice::Ptr & device = swiss_ranger_vec_[i];
    calibration_->addSensor(device->intensitySensor(), true);
    calibration_->addSensor(device->depthSensor(), true);
    sensor_vec_.push_back(device->intensitySensor());
    sensor_vec_.push_back(device->depthSensor());
    images_acquired_map_[device->frameId()] = 0;
    status_msg_.sensor_ids.push_back(device->frameId());
  }

  status_msg_.images_acquired.resize(status_msg_.sensor_ids.size(), 0);
  status_msg_.header.stamp = ros::Time::now();
  status_msg_.header.seq = 0;
  status_pub_.publish(status_msg_);

  return true;
}

void OPTCalibrationNode::actionCallback(const std_msgs::String::ConstPtr & msg)
{
  if (msg->data == "save" or msg->data == "saveExtrinsicCalibration")
  {
    ROS_INFO("Saving calibration results...");
    calibration_->optimize();
    save();
  }
  else if (msg->data == "start floor")
  {
    if (calibration_->floorAcquisition())
    {
      ROS_WARN("Floor acquisition already started.");
    }
    else
    {
      ROS_INFO("Floor acquisition started.");
      calibration_->startFloorAcquisition();
    }
  }
  else if (msg->data == "stop floor")
  {
    if (not calibration_->floorAcquisition())
      ROS_WARN("Floor acquisition not started.");
    else
      calibration_->stopFloorAcquisition();
  }
  else
  {
    ROS_ERROR_STREAM("Unknown action: \"" << msg->data << "\"!");
  }
}

void OPTCalibrationNode::spin()
{
  ros::Rate rate(5.0);

  while (ros::ok())
  {
    ros::spinOnce();
    calibration_->nextAcquisition();

    int count = 0;

    try
    {
#pragma omp parallel for
      for (size_t i = 0; i < pinhole_vec_.size(); ++i)
      {
        const PinholeRGBDevice::Ptr & device = pinhole_vec_[i];
        if (device->hasNewMessages())
        {
          device->convertLastMessages();
          PinholeRGBDevice::Data::Ptr data = device->lastData();
          OPTCalibration::CheckerboardView::Ptr cb_view;
          ROS_DEBUG_STREAM("[" << device->frameId() << "] analysing image generated at: " << device->lastMessages().image_msg->header.stamp);
#pragma omp critical
          if (calibration_->analyzeData(device->sensor(), data->image, cb_view))
          {
            calibration_->addData(device->sensor(), cb_view);
            images_acquired_map_[device->frameId()]++;
            ROS_INFO_STREAM("[" << device->frameId() << "] checkerboard detected");
            ++count;
          }
        }
      }
      for (size_t i = 0; i < kinect_vec_.size(); ++i)
      {
        const KinectDevice::Ptr & device = kinect_vec_[i];
        if (device->hasNewMessages())
        {
          device->convertLastMessages();
          KinectDevice::Data::Ptr data = device->lastData();
          OPTCalibration::CheckerboardView::Ptr color_cb_view;
          OPTCalibration::CheckerboardView::Ptr depth_cb_view;
          ROS_DEBUG_STREAM("[" << device->colorFrameId() << "] analysing image generated at: " << device->lastMessages().image_msg->header.stamp);
          ROS_DEBUG_STREAM("[" << device->depthFrameId() << "] analysing cloud generated at: " << device->lastMessages().cloud_msg->header.stamp);
          if (calibration_->analyzeData(device->colorSensor(), device->depthSensor(),
                                        data->image, data->cloud,
                                        color_cb_view, depth_cb_view))
          {
            calibration_->addData(device->colorSensor(), color_cb_view);
            calibration_->addData(device->depthSensor(), depth_cb_view);
            images_acquired_map_[device->colorFrameId()]++;
            ROS_INFO_STREAM("[" << device->colorFrameId() << "] checkerboard detected");
            ++count;
          }
        }
      }
      for (size_t i = 0; i < swiss_ranger_vec_.size(); ++i)
      {
        const SwissRangerDevice::Ptr & device = swiss_ranger_vec_[i];
        if (device->hasNewMessages())
        {
          device->convertLastMessages();
          SwissRangerDevice::Data::Ptr data = device->lastData();
          OPTCalibration::CheckerboardView::Ptr color_cb_view;
          OPTCalibration::CheckerboardView::Ptr depth_cb_view;
          ROS_DEBUG_STREAM("[" << device->frameId() << "] analysing image generated at: " << device->lastMessages().intensity_msg->header.stamp);
          ROS_DEBUG_STREAM("[" << device->frameId() << "] analysing cloud generated at: " << device->lastMessages().cloud_msg->header.stamp);
          if (calibration_->analyzeData(device->intensitySensor(), device->depthSensor(),
                                        data->intensity_image, data->cloud,
                                        color_cb_view, depth_cb_view))
          {
            calibration_->addData(device->intensitySensor(), color_cb_view);
            calibration_->addData(device->depthSensor(), depth_cb_view);
            images_acquired_map_[device->frameId()]++;
            ROS_INFO_STREAM("[" << device->frameId() << "] checkerboard detected");
            ++count;
          }
        }
      }
    }
    catch (cv_bridge::Exception & ex)
    {
      ROS_ERROR_STREAM("cv_bridge exception: " << ex.what());
      return;
    }
    catch (std::runtime_error & ex)
    {
      ROS_ERROR_STREAM("exception: " << ex.what());
      return;
    }

    status_msg_.header.stamp = ros::Time::now();
    status_msg_.header.seq++;
    for (size_t i = 0; i < status_msg_.sensor_ids.size(); ++i)
    {
      status_msg_.images_acquired[i] = images_acquired_map_[status_msg_.sensor_ids[i]];
    }
    status_pub_.publish(status_msg_);

    calibration_->perform();
    calibration_->publish();

    if (count > 0)
      ROS_INFO("-----------------------------------------------");

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

    Pose new_world_pose = Pose::Identity();

    if (world_computation_ == LAST_CHECKERBOARD)
    {
      new_world_pose = calibration_->getLastCheckerboardPose().inverse();
      for (size_t i = 0; i < sensor_vec_.size(); ++i)
      {
        const Sensor::Ptr & sensor = sensor_vec_[i];
        Pose pose = new_world_pose * sensor->pose();
        if(pose.translation().z() < 0)
        {
          ROS_INFO_STREAM("[" << sensor->frameId() << "] z < 0. Flipping /world orientation.");
          AngleAxis rotation(M_PI, Vector3(1.0, 1.0, 0.0).normalized());
          new_world_pose = rotation * calibration_->getLastCheckerboardPose().inverse();
          break;
        }
      }
    }
    else if (world_computation_ == UPDATE)
    {
      new_world_pose = fixed_sensor_pose_ * fixed_sensor_->pose().inverse();
    }

    // Write TF transforms between cameras and world frame
    file << "# Poses w.r.t. the \"world\" reference frame" << std::endl;
    file << "poses:" << std::endl;
    for (size_t i = 0; i < sensor_vec_.size(); ++i)
    {
      const Sensor::Ptr & sensor = sensor_vec_[i];

      Pose pose = new_world_pose * sensor->pose();

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
      const Sensor::Ptr & sensor = sensor_vec_[i];

      Pose pose = new_world_pose * sensor->pose();
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
      return 1;
    calib_node.spin();
  }
  catch (std::runtime_error & error)
  {
    ROS_FATAL("Calibration error: %s", error.what());
    return 2;
  }

  return 0;
}
