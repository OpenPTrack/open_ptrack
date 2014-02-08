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
 * Author: Matteo Munaro [matteo.munaro@dei.unipd.it]
 *
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>

int
main(int argc, char ** argv)
{
  ros::init(argc, argv, "calibration_initializer");
  ros::NodeHandle nh("~");

  // Read some parameters from launch file:
  int num_cameras;
  nh.param("num_cameras", num_cameras, 1);
  int base_camera;
  nh.param("base_camera", base_camera, 0);
  bool calibration_with_serials;
  nh.param("calibration_with_serials", calibration_with_serials, false);
  int rows;
  nh.param("rows", rows, 6);
  int cols;
  nh.param("cols", cols, 5);
  double cell_width;
  nh.param("cell_width", cell_width, 0.12);
  double cell_height;
  nh.param("cell_height", cell_height, 0.12);
  std::string driver_name;
  nh.param("driver", driver_name, std::string("openni"));

  // Read ID of cameras:
  std::vector<std::string> camera_id_vector;
  for (unsigned int i = 0; i < num_cameras; i++)
  {
    std::stringstream ss;
    ss << "camera" << i << "_id";
    std::string camera_id;
    nh.param(ss.str(), camera_id, std::string("./"));
    camera_id_vector.push_back(camera_id);
  }

  // Write launch file to be used for extrinsic calibration of the camera network:
  std::string file_name = ros::package::getPath("opt_calibration") + "/launch/opt_calibration_master.launch";
  std::ofstream launch_file;
  launch_file.open(file_name.c_str());

  if (launch_file.is_open())
  {
    launch_file << "<launch>" << std::endl << std::endl;

    // Network parameters:
    launch_file << "  <!-- Network parameters -->" << std::endl
                << "  <arg name=\"num_cameras\" value=\"" << num_cameras << "\" />" << std::endl;

    for (unsigned int i = 0; i < num_cameras; i++)
    {
      launch_file << "  <arg name=\"camera" << i << "_id\" value=\"" << camera_id_vector[i] << "\" />" << std::endl;
      launch_file << "  <arg name=\"camera" << i << "_name\" default=\"$(arg camera" << i << "_id)\" />" << std::endl;
    }
    launch_file << "  <arg name=\"base_camera\" value=\"$(arg camera" << base_camera << "_name)\" />" << std::endl << std::endl;

    // Checkerboard parameters:
    launch_file << "  <!-- Checkerboard parameters -->" << std::endl
        << "  <arg name=\"rows\" default=\"" << rows << "\" />" << std::endl
        << "  <arg name=\"cols\" default=\"" << cols << "\" />" << std::endl
        << "  <arg name=\"cell_width\" default=\"" << cell_width << "\" />" << std::endl
        << "  <arg name=\"cell_height\" default=\"" << cell_height << "\" />" << std::endl << std::endl;

    // RViz and calibration node:
    launch_file << "  <!-- Opening Rviz for visualization-->" << std::endl
        << "  <node name=\"rviz\" pkg=\"rviz\" type=\"rviz\" args=\"-d $(find opt_calibration)/conf/opt_calibration.rviz\"/>" << std::endl << std::endl
        << "  <!-- Launching calibration -->" << std::endl
        << "  <node pkg=\"opt_calibration\" type=\"opt_calibration\" name=\"opt_calibration\" output=\"screen\">" << std::endl
        << "    <param name=\"num_cameras\" value=\"$(arg num_cameras)\" /> " << std::endl
        << "    <param name=\"rows\" value=\"$(arg rows)\" />" << std::endl
        << "    <param name=\"cols\" value=\"$(arg cols)\" />" << std::endl
        << "    <param name=\"cell_width\" value=\"$(arg cell_width)\" />" << std::endl
        << "    <param name=\"cell_height\" value=\"$(arg cell_height)\" />" << std::endl << std::endl
        << "    <param name=\"base_camera\" value=\"$(arg base_camera)\" />" << std::endl << std::endl;

    // Parameters and remapping for every camera:
    for (unsigned int i = 0; i < num_cameras; i++)
    {
      launch_file << "    <param name=\"camera_" << i << "/name\" value=\"/$(arg camera" << i << "_name)\" />" << std::endl;
      launch_file << "    <remap from=\"~camera_" << i << "/image\" to=\"/$(arg camera" << i << "_name)/rgb/image_rect_color\" />" << std::endl;
      launch_file << "    <remap from=\"~camera_" << i << "/camera_info\" to=\"/$(arg camera" << i << "_name)/rgb/camera_info\" />" << std::endl << std::endl;
    }

    launch_file << "  </node>" << std::endl << std::endl;

    launch_file << "</launch>" << std::endl;
  }
  launch_file.close();
  ROS_INFO_STREAM(file_name << " created!");

  // Write a launch file for every camera:
  for (unsigned int i = 0; i < num_cameras; i++)
  {
    file_name = ros::package::getPath("opt_calibration") + "/launch/sensor_" + camera_id_vector[i] + ".launch";
    std::ofstream launch_file;
    launch_file.open(file_name.c_str());

    if (launch_file.is_open())
    {
      launch_file << "<launch>" << std::endl << std::endl;

      launch_file << "  <!-- Camera parameters -->" << std::endl
          << "  <arg name=\"camera_id\" value=\"" << camera_id_vector[i] << "\" />" << std::endl
          << "  <arg name=\"camera_name\" default=\"$(arg camera_id)\" />" << std::endl << std::endl;

      launch_file << "  <!-- Launch sensor -->" << std::endl
          << "  <include file=\"$(find detection)/launch/" << driver_name << ".launch\">" << std::endl;

      // If serial numbers can be used to identify cameras, they are added to the launch file:
      if (calibration_with_serials)
        launch_file << "    <arg name=\"device_id\" value=\"$(arg camera_id)\" />" << std::endl;

      launch_file << "    <arg name=\"camera\" value=\"$(arg camera_name)\" />" << std::endl
          << "  </include>" << std::endl << std::endl;

      launch_file << "  <!-- Publish a further transform -->" << std::endl
          << "  <node pkg=\"tf\" type=\"static_transform_publisher\" name=\"$(arg camera_name)_broadcaster\" args=\"-0.045 0 0 1.57079 -1.57079 0 /$(arg camera_name) /$(arg camera_name)_link  100\" />" << std::endl << std::endl;

      launch_file << "</launch>" << std::endl;
    }
    launch_file.close();
    ROS_INFO_STREAM(file_name << " created!");
  }

  return 0;
}
