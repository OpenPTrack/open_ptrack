/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * publish_stereo.cpp
 * Created on: Aug 08, 2014
 * Author: Matteo Munaro
 *
 */

// ROS includes:
#include <ros/ros.h>

//Publish Messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

using namespace sensor_msgs;

ros::Publisher stereo_left_info_pub, stereo_right_info_pub;
sensor_msgs::CameraInfo stereo_left_camera_info_msg, stereo_right_camera_info_msg;
std::string stereo_name;

void
leftImageCb (const sensor_msgs::ImageConstPtr& left_image)
{
  // Update stamp and seq at every iteration
  stereo_left_camera_info_msg.header = left_image->header;
  stereo_left_camera_info_msg.header.frame_id = stereo_name;

	// Send camera info:
  stereo_left_info_pub.publish(stereo_left_camera_info_msg);
}

void
rightImageCb (const sensor_msgs::ImageConstPtr& right_image)
{
  // Update stamp and seq at every iteration
  stereo_right_camera_info_msg.header = right_image->header;
  stereo_right_camera_info_msg.header.frame_id = stereo_name;

  // Send camera info:
  stereo_right_info_pub.publish(stereo_right_camera_info_msg);
}

void
fillCameraInfoMessage (ros::NodeHandle nh, std::string base_name, sensor_msgs::CameraInfo& camera_info_msg )
{
  // Read calibration data and fill camera_info message:
  int width, height;
  nh.param(base_name + "/image_width", width, 640);
  nh.param(base_name + "/image_height", height, 480);
  camera_info_msg.width = width;
  camera_info_msg.height = height;

  std::string distortion_model;
  if (not nh.getParam (base_name + "/distortion_model", distortion_model))
  {
    ROS_ERROR("Stereo calibration parameters not found!");
  }
  camera_info_msg.distortion_model = distortion_model;

  std::vector<float> camera_matrix;
  if (not nh.getParam (base_name + "/camera_matrix/data", camera_matrix))
  {
    ROS_ERROR("Stereo calibration parameters not found!");
  }
  for (unsigned int i = 0; i < camera_matrix.size(); i++)
  {
    camera_info_msg.K[i] = camera_matrix[i];
  }

  std::vector<float> rectification_matrix;
  if (not nh.getParam (base_name + "/rectification_matrix/data", rectification_matrix))
  {
    ROS_ERROR("Stereo calibration parameters not found!");
  }
  for (unsigned int i = 0; i < rectification_matrix.size(); i++)
  {
    camera_info_msg.R[i] = rectification_matrix[i];
  }

  std::vector<float> projection_matrix;
  if (not nh.getParam (base_name + "/projection_matrix/data", projection_matrix))
  {
    ROS_ERROR("Stereo calibration parameters not found!");
  }
  for (unsigned int i = 0; i < projection_matrix.size(); i++)
  {
    camera_info_msg.P[i] = projection_matrix[i];
  }

  std::vector<float> distortion_coefficients;
  if (not nh.getParam (base_name + "/distortion_coefficients/data", distortion_coefficients))
  {
    ROS_ERROR("Stereo calibration parameters not found!");
  }
  for (unsigned int i = 0; i < distortion_coefficients.size(); i++)
  {
    camera_info_msg.D.push_back(distortion_coefficients[i]);
  }
}

int
main (int argc, char** argv)
{
	ros::init(argc, argv, "stereo_publisher");
	ros::NodeHandle nh("~");

	// Read some parameters from launch file:
	std::string left_image_topic, right_image_topic, calibration_file_left, calibration_file_right;
	nh.param("stereo_name", stereo_name, std::string("blackfly_stereo"));
	nh.param("left_image_topic", left_image_topic, "/" + stereo_name + std::string("/left/image_raw"));
	nh.param("right_image_topic", right_image_topic, "/" + stereo_name + std::string("/right/image_raw"));
	double rate_value;
	nh.param("rate", rate_value, 30.0);

	// Define output topics:
	std::string stereo_left_camera_info_topic = "/" + stereo_name + std::string("/left/camera_info");
	std::string stereo_right_camera_info_topic = "/" + stereo_name + std::string("/right/camera_info");

	// Subscribers:
	ros::Subscriber left_sub = nh.subscribe(left_image_topic, 1, leftImageCb);
	ros::Subscriber right_sub = nh.subscribe(right_image_topic, 1, rightImageCb);

	// Publishers:
	stereo_left_info_pub = nh.advertise<CameraInfo>(stereo_left_camera_info_topic, 3);
	stereo_right_info_pub = nh.advertise<CameraInfo>(stereo_right_camera_info_topic, 3);

  // Read stereo calibration files and fill camera_info messages:
	fillCameraInfoMessage (nh, "left", stereo_left_camera_info_msg);
	fillCameraInfoMessage (nh, "right", stereo_right_camera_info_msg);

	ros::Rate rate(rate_value);
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}

