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
 * main_image_publisher_sr.cpp
 * Created on: Mar 03, 2014
 * Author: Matteo Munaro
 *
 */

// ROS includes:
#include <ros/ros.h>

// PCL includes:
#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

//Publish Messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace sensor_msgs;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

enum { COLS = 176, ROWS = 144 };

cv::Mat intensity_image;
cv::Mat confidence_image;
ros::Publisher image_pub, info_pub, image_rect_pub;
sensor_msgs::CameraInfo camera_info_msg;
std::string swissranger_frame_id;
int scale_factor;

void
cloud_cb (const PointCloud2ConstPtr& callback_cloud)
{
  // Point Cloud Fields:
  //  name: x  //  offset: 0  //  datatype: 7  //  count: 1
  //  name: y  //  offset: 4  //  datatype: 7  //  count: 1
  //  name: z  //  offset: 8  //  datatype: 7  //  count: 1
  //  name: intensity  //  offset: 12  //  datatype: 7  //  count: 1
  //  name: confidence //  offset: 16  //  datatype: 7  //  count: 1

  // Create intensity and confidence images:
  int intensity_offset = 12;
  int confidence_offset = 16;

  float point_step = callback_cloud->point_step;
  for (unsigned int i = 0; i < ROWS; i++)
  {
    for (unsigned int j = 0; j < COLS; j++)
    {
      float * intensity_ptr = (float*)&callback_cloud->data.at(intensity_offset + (j + i*COLS)*point_step);
      float * confidence_ptr = (float*)&callback_cloud->data.at(confidence_offset + (j + i*COLS)*point_step);
      intensity_image.at<unsigned char>(i,j) = (unsigned char)(intensity_ptr[0] / 255);
      confidence_image.at<unsigned char>(i,j) = (unsigned char)(confidence_ptr[0] / 255);
    }
  }

  // Find min and max of intensity image:
  double minVal, maxVal;
  cv::Point minLoc, maxLoc;
  cv::minMaxLoc( intensity_image, &minVal, &maxVal, &minLoc, &maxLoc, confidence_image);

  // Rescale image intensity:
  for (unsigned int i = 0; i < ROWS; i++)
  {
    for (unsigned int j = 0; j < COLS; j++)
    {
      float * intensity_ptr = (float*)&callback_cloud->data.at(intensity_offset + (j + i*COLS)*point_step);
      intensity_image.at<unsigned char>(i,j) = (intensity_ptr[0] / 255 - minVal) * 255 / (maxVal - minVal);
    }
  }

  cv::equalizeHist(intensity_image, intensity_image);

  // Rescale image:
  cv::Mat output_intensity_image;
  if (scale_factor != 1)
  {
    cv::resize(intensity_image, output_intensity_image, cv::Size(), scale_factor, scale_factor);
  }
  else
  {
    output_intensity_image = intensity_image;
  }

  // Send intensity image:
  if (image_pub.getNumSubscribers() > 0)
  {
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    cv_ptr->encoding = "mono8";
    cv_ptr->image = output_intensity_image;
    cv_ptr->header = callback_cloud->header;
    cv_ptr->header.frame_id = swissranger_frame_id;
    image_pub.publish(cv_ptr->toImageMsg());
  }

  //  // Undistort image:
  //  cv::Mat undistorted_image;
  //  cv::Mat new_camera_matrix;
  //  cv::Mat distortion_coeffs(5,1,CV_32F);
  //  distortion_coeffs.at<float>(0) = -0.348799905748622;
  //  distortion_coeffs.at<float>(1) = 0.133373516437116;
  //  distortion_coeffs.at<float>(2) = 0.00579477365472071;
  //  distortion_coeffs.at<float>(3) = 0.00178143189450469;
  //  distortion_coeffs.at<float>(4) = 0;
  //
  //  cv::Mat old_camera_matrix(3,3,CV_32F);
  //  old_camera_matrix.at<float>(0) = camera_info_msg.K[0];
  //  old_camera_matrix.at<float>(1) = camera_info_msg.K[1];
  //  old_camera_matrix.at<float>(2) = camera_info_msg.K[2];
  //  old_camera_matrix.at<float>(3) = camera_info_msg.K[3];
  //  old_camera_matrix.at<float>(4) = camera_info_msg.K[4];
  //  old_camera_matrix.at<float>(5) = camera_info_msg.K[5];
  //  old_camera_matrix.at<float>(6) = camera_info_msg.K[6];
  //  old_camera_matrix.at<float>(7) = camera_info_msg.K[7];
  //  old_camera_matrix.at<float>(8) = camera_info_msg.K[8];
  //
  //  cv::undistort(output_intensity_image, undistorted_image, old_camera_matrix, distortion_coeffs, new_camera_matrix);
  //
  //  // Send undistorted image:
  //  if (image_pub.getNumSubscribers() > 0)
  //  {
  //    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
  //    cv_ptr->encoding = "mono8";
  //    cv_ptr->image = undistorted_image;
  //    cv_ptr->header = callback_cloud->header;
  //    cv_ptr->header.frame_id = "/swissranger";
  //    image_rect_pub.publish(cv_ptr->toImageMsg());
  //  }


  // Send camera info:
  if (info_pub.getNumSubscribers() > 0)
  {
    camera_info_msg.header.seq = callback_cloud->header.seq;
    camera_info_msg.header.stamp = callback_cloud->header.stamp;
    info_pub.publish(camera_info_msg);
  }

  //  cv::imshow("Confidence map", confidence_image);
  //  cv::waitKey(1);
}

int
main (int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher_sr");
  ros::NodeHandle nh("~");

  // Read some parameters from launch file:
  nh.param("scale_factor", scale_factor, 1);
  nh.param("frame_id", swissranger_frame_id, std::string("/swissranger"));
  std::string pointcloud_topic;
  nh.param("pointcloud_topic", pointcloud_topic, std::string("/swissranger/pointcloud2_raw"));
  std::string image_resized_topic;
  nh.param("image_resized_topic", image_resized_topic, std::string("/swissranger/intensity/image_resized"));
  std::string camera_info_topic;
  nh.param("camera_info_topic", camera_info_topic, std::string("/swissranger/intensity/image_resized/camera_info"));
  double rate_value;
  nh.param("rate", rate_value, 30.0);

  // Subscribers:
  //ros::Subscriber sub = nh.subscribe(pointcloud_topic, 1, cloud_cb);
  ros::Subscriber sub = nh.subscribe(pointcloud_topic, 1, cloud_cb);

  // Publishers:
  image_pub = nh.advertise<Image>(image_resized_topic, 3);
  info_pub = nh.advertise<CameraInfo>(camera_info_topic, 3);

  // Initialize confidence and intensity images
  intensity_image = cv::Mat(ROWS, COLS, CV_8U, cv::Scalar(255));
  confidence_image = cv::Mat(ROWS, COLS, CV_8U, cv::Scalar(255));

  // Create camera_info message:
  camera_info_msg.header.frame_id = swissranger_frame_id;
  camera_info_msg.height = ROWS;
  camera_info_msg.width = COLS;
  camera_info_msg.K[0] = 148.906628757021;
  camera_info_msg.K[2] = 87.307592764537;
  camera_info_msg.K[4] = 148.634545599915;
  camera_info_msg.K[5] = 70.8860920144388;
  camera_info_msg.K[8] = 1.0;

  // Update camera_info according to the scale factor:
  if (scale_factor != 1)
  {
    camera_info_msg.K[0] = camera_info_msg.K[0] * scale_factor;
    camera_info_msg.K[2] = camera_info_msg.K[2] * scale_factor;
    camera_info_msg.K[4] = camera_info_msg.K[4] * scale_factor;
    camera_info_msg.K[5] = camera_info_msg.K[5] * scale_factor;
    camera_info_msg.height = camera_info_msg.height * scale_factor;
    camera_info_msg.width = camera_info_msg.width * scale_factor;
  }

  //	cv::namedWindow("Confidence map", CV_WINDOW_NORMAL);

  ros::Rate rate(rate_value);
  while(ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

