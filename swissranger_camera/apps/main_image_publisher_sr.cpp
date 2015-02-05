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

#include <camera_info_manager/camera_info_manager.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//#include <swissranger_camera/utility.h>

using namespace sensor_msgs;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

enum { COLS = 176, ROWS = 144 };

cv::Mat intensity_image;
cv::Mat confidence_image;
ros::Publisher image_pub, info_pub, image_rect_pub;
sensor_msgs::CameraInfo camera_info_msg;
std::string swissranger_name;
int scale_factor;

int seq_;

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
      intensity_image.at<unsigned char>(i,j) = (unsigned char)(intensity_ptr[0] / 257);
      confidence_image.at<unsigned char>(i,j) = (unsigned char)(confidence_ptr[0] / 257);
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
      intensity_image.at<unsigned char>(i,j) = (intensity_ptr[0] / 257 - minVal) * 255 / (maxVal - minVal);
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

  ++seq_;
  ros::Time now = ros::Time::now();
  // Send intensity image:
  if (image_pub.getNumSubscribers() > 0)
  {
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    cv_ptr->encoding = "mono8";
    cv_ptr->image = output_intensity_image;
    cv_ptr->header = callback_cloud->header;
    cv_ptr->header.frame_id = "/" + swissranger_name;
    cv_ptr->header.seq = callback_cloud->header.seq;
    cv_ptr->header.stamp = callback_cloud->header.stamp;
//    cv_ptr->header.seq = seq_;
//    cv_ptr->header.stamp = now;
    image_pub.publish(cv_ptr->toImageMsg());
  }

//  sr::Utility sr_utility;
//  sr_utility.setConfidenceThreshold(0.0f);
//  sr_utility.setInputCloud(callback_cloud);
//  sr_utility.setIntensityType(sr::Utility::INTENSITY_8BIT);
//  sr_utility.setConfidenceType(sr::Utility::CONFIDENCE_8BIT);
//  sr_utility.setNormalizeIntensity(true);
//  sr_utility.split(sr::Utility::ALL);

//  if (image_pub.getNumSubscribers() > 0)
//  {
//    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
//    cv_ptr->encoding = "mono8";
//    cv_ptr->image = sr_utility.getIntensity();
//    cv_ptr->header = callback_cloud->header;
//    cv_ptr->header.frame_id = swissranger_frame_id;
//    image_pub.publish(cv_ptr->toImageMsg());
//  }

  // Send camera info:
  if (info_pub.getNumSubscribers() > 0)
  {
    camera_info_msg.header.seq = callback_cloud->header.seq;
    camera_info_msg.header.stamp = callback_cloud->header.stamp;
//    camera_info_msg.header.seq = seq_;
//    camera_info_msg.header.stamp = now;
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
  nh.param("camera_name", swissranger_name, std::string("swissranger"));

  std::string pointcloud_topic, image_resized_topic, camera_info_topic;
  nh.param("pointcloud_topic", pointcloud_topic, std::string("/swissranger/pointcloud2_raw"));
  nh.param("image_resized_topic", image_resized_topic, std::string("/swissranger/intensity/image_resized"));
  nh.param("camera_info_topic", camera_info_topic, std::string("/swissranger/intensity/image_resized/camera_info"));

  double rate_value;
  nh.param("rate", rate_value, 30.0);

  std::string camera_info_url;
  nh.param("camera_info_url", camera_info_url, std::string(""));

  camera_info_manager::CameraInfoManager manager(nh, swissranger_name, camera_info_url);

  seq_ = 0;

  // Subscribers:
  //ros::Subscriber sub = nh.subscribe(pointcloud_topic, 1, cloud_cb);
  ros::Subscriber sub = nh.subscribe(pointcloud_topic, 1, cloud_cb);

  // Publishers:
  image_pub = nh.advertise<Image>(image_resized_topic, 3);
  info_pub = nh.advertise<CameraInfo>(camera_info_topic, 3);


  // Create camera_info message:
  if (manager.loadCameraInfo(camera_info_url))
  {
    camera_info_msg = manager.getCameraInfo();
  }
  else
  {
    ROS_INFO_STREAM("[" << swissranger_name << "] loading default parameters.");

    camera_info_msg.header.frame_id = "/" + swissranger_name;
    camera_info_msg.height = ROWS;
    camera_info_msg.width = COLS;

    camera_info_msg.K[0] = 148.906628757021;
    camera_info_msg.K[2] = 87.307592764537;
    camera_info_msg.K[4] = 148.634545599915;
    camera_info_msg.K[5] = 70.8860920144388;
    camera_info_msg.K[8] = 1.0;

    camera_info_msg.R[0] = 1.0;
    camera_info_msg.R[4] = 1.0;
    camera_info_msg.R[8] = 1.0;

    camera_info_msg.P[0] = camera_info_msg.K[0];
    camera_info_msg.P[2] = camera_info_msg.K[2];
    camera_info_msg.P[5] = camera_info_msg.K[4];
    camera_info_msg.P[6] = camera_info_msg.K[5];
    camera_info_msg.P[10] = camera_info_msg.K[8];

    camera_info_msg.distortion_model = "plumb_bob";
    camera_info_msg.D.resize(5, 0);
  }

  // Initialize confidence and intensity images
  intensity_image = cv::Mat(camera_info_msg.height, camera_info_msg.width, CV_8U, cv::Scalar(255));
  confidence_image = cv::Mat(camera_info_msg.height, camera_info_msg.width, CV_8U, cv::Scalar(255));

  // Update camera_info according to the scale factor:
  if (scale_factor != 1)
  {
    camera_info_msg.height *= scale_factor;
    camera_info_msg.width *= scale_factor;

    camera_info_msg.K[0] *= scale_factor;
    camera_info_msg.K[2] *= scale_factor;
    camera_info_msg.K[4] *= scale_factor;
    camera_info_msg.K[5] *= scale_factor;

    camera_info_msg.P[0] *= scale_factor;
    camera_info_msg.P[2] *= scale_factor;
    camera_info_msg.P[5] *= scale_factor;
    camera_info_msg.P[6] *= scale_factor;
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
