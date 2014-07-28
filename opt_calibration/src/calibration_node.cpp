/*
 *  Copyright (c) 2013-2014, Filippo Basso <bassofil@dei.unipd.it>
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

#include <open_ptrack/opt_calibration/calibration_node.h>

#include <fstream>

#include <pcl/filters/filter_indices.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

#include <ceres/ceres.h>

#include <calibration_common/pcl/utils.h>

#include <pcl/filters/random_sample.h>

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
//  if (not node_handle_.getParam("base_sensor", base_sensor_frame_id_))
//    ROS_FATAL("No base_sensor parameter found!!");

  node_handle_.param("calibration_with_serials", calibration_with_serials_, false);

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

  for (int id = 0; id < num_sensors_; ++id)
  {
    std::stringstream ss;

    std::string type_s;
    ss.str("");
    ss << "sensor_" << id << "/type";
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
    ss << "/sensor_" << id;
    std::string frame_id = ss.str();

    ss << "/name";
    node_handle_.param(ss.str(), frame_id, frame_id);
    SensorROS::Ptr sensor = boost::make_shared<SensorROS>(frame_id, type);

    ss.str("");
    ss << "sensor_" << id << "/image";
    sensor->setImageSubscriber(image_transport_.subscribe(ss.str(), 1, &SensorROS::imageCallback, sensor));

    ss.str("");
    ss << "sensor_" << id << "/camera_info";
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
        ROS_WARN("Not all messages received. Waiting...");
        all_messages_received = false;
        break;
      }
    }
    rate.sleep();
  }

  calibration_ = boost::make_shared<OPTCalibration>(node_handle_, calibration_with_serials_);
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
//    for (size_t i = 0; i < sensor_vec_.size(); ++i)
//    {
//      if (not sensor_vec_[i].sensor()->parent())
//      {
//        ROS_WARN("Not all camera poses estimated!!! File not saved!!!");
//        return;
//      }
//      else
//      {
//        ROS_INFO("Saving...");
//      }
//    }
//    ROS_INFO("Optimizing...");
    calibration_->optimize();
    calibration_->save();
  }
  else if (msg->data == "stop")
  {
    ROS_INFO("Optimizing...");
    calibration_->optimize();
  }
}

void OPTCalibrationNode::spin()
{
  ros::Rate rate(2.0);

  while (ros::ok())
  {
    ros::spinOnce();

    size_t n = calibration_->nextAcquisition();

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

          std::stringstream image_file_name;
          image_file_name << "/tmp/cam" << i << "_image" << std::setw(4) << std::setfill('0') << n << ".png";
          cv::imwrite(image_file_name.str(), image_ptr->image);

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
      }

    }

    calibration_->perform();

    rate.sleep();

  }

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
