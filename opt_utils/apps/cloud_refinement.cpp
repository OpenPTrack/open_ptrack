/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2016-, Matteo Munaro [matteo.munaro@dei.unipd.it]
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
 * cloud_refinement.cpp
 * Created on: Mar 24, 2016
 * Author: Matteo Munaro
 *
 */

// ROS includes:
#include <ros/ros.h>
#include <ros/package.h>

// PCL includes:
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <fstream>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// Refinement matrix:
Eigen::Matrix4d registration_matrix;
// Output cloud:
PointCloudT::Ptr refined_cloud(new PointCloudT);
// Output publisher:
ros::Publisher refined_cloud_pub;

void
cloud_cb (const PointCloudT::ConstPtr& callback_cloud)
{
  *refined_cloud = *callback_cloud;
  Eigen::Matrix4d trans = registration_matrix.inverse();
  pcl::transformPointCloud (*refined_cloud, *refined_cloud, trans);
  refined_cloud->header = callback_cloud->header;
  //refined_cloud->header.stamp = ros::Time::now();
  ros::Time curr_time = ros::Time::now();
  refined_cloud->header.stamp = curr_time.toNSec() / 1000;
  refined_cloud_pub.publish(*refined_cloud);
}

Eigen::Matrix4d
readMatrixFromFile (std::string filename)
{
  Eigen::Matrix4d matrix;
  std::string line;
  std::ifstream myfile (filename.c_str());
  if (myfile.is_open())
  {
    int k = 0;
    std::string number;
    while (myfile >> number)
    {
      matrix(int(k/4), int(k%4)) = std::atof(number.c_str());
      k++;
    }
    myfile.close();
  }

  std::cout << matrix << std::endl;

  return matrix;
}

int
main (int argc, char** argv)
{
  ros::init(argc, argv, "cloud_refinement");
  ros::NodeHandle nh("~");

  // Read some parameters from launch file:
  std::string input_topic;
  nh.param("input_topic", input_topic, std::string("input"));
  std::string output_topic;
  nh.param("output_topic", output_topic, std::string("output"));
  std::string refinement_file;
  nh.param("refinement_file", refinement_file, std::string("registration_kinect2_head_ir_optical_frame.txt"));
  // Main loop rate:
  double rate_value;
  nh.param("rate", rate_value, 30.0);

  // Load refinement matrix:
  std::cout << "Reading refinement matrix in " << refinement_file << std::endl;
  std::string refinement_filename = ros::package::getPath("opt_calibration") + "/conf/" + refinement_file + ".txt";
  std::ifstream f(refinement_filename.c_str());
  if (f.good()) // if the file exists
  {
    f.close();
    registration_matrix = readMatrixFromFile (refinement_filename);
  }
  else  // if the file does not exist
  {
    // insert the identity matrix
    std::cout << "Refinement file not found! Not doing refinement for this sensor." << std::endl;
    registration_matrix = Eigen::Matrix4d::Identity();
  }

  // Subscribers:
  ros::Subscriber sub = nh.subscribe(input_topic, 1, cloud_cb);

  // Publishers:
  refined_cloud_pub = nh.advertise<PointCloudT>(output_topic, 1);

  ros::Rate rate(rate_value);
  while(ros::ok())
  {
    ros::spinOnce();

    rate.sleep();
  }

  return 0;
}

