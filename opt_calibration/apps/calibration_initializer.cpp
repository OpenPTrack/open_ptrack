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
#include <string.h>

int
main(int argc, char ** argv)
{
  ros::init(argc, argv, "calibration_initializer");
  ros::NodeHandle nh("~");

  // Read some parameters from launch file:
  int num_sensors;
  nh.param("num_sensors", num_sensors, 1);
  int base_sensor;
  nh.param("base_sensor", base_sensor, 0);
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

  // Read ID of sensors:
  std::vector<std::string> sensor_id_vector;
  for (unsigned int i = 0; i < num_sensors; i++)
  {
    std::stringstream ss;
    ss << "sensor" << i << "_id";
    std::string sensor_id;
    nh.param(ss.str(), sensor_id, std::string("./"));
    sensor_id_vector.push_back(sensor_id);
  }

  // Write launch file to be used for extrinsic calibration of the sensor network:
  std::string file_name = ros::package::getPath("opt_calibration") + "/launch/opt_calibration_master.launch";
  std::ofstream launch_file;
  launch_file.open(file_name.c_str());

  if (launch_file.is_open())
  {
    launch_file << "<launch>" << std::endl << std::endl;

    // Network parameters:
    launch_file << "  <!-- Network parameters -->" << std::endl
                << "  <arg name=\"num_sensors\" value=\"" << num_sensors << "\" />" << std::endl;

    for (unsigned int i = 0; i < num_sensors; i++)
    {
      if (!std::strcmp(sensor_id_vector[i].substr(0,1).c_str(), "1"))    // if SwissRanger
      {
        std::string sensor_name = sensor_id_vector[i];
        replace(sensor_name.begin(), sensor_name.end(), '.', '_');
        sensor_name = "SR_" + sensor_name;
        launch_file << "  <arg name=\"sensor" << i << "_id\" value=\"" << sensor_name << "\" />" << std::endl;
      }
      else
      {
        launch_file << "  <arg name=\"sensor" << i << "_id\" value=\"" << sensor_id_vector[i] << "\" />" << std::endl;
      }
      launch_file << "  <arg name=\"sensor" << i << "_name\" default=\"$(arg sensor" << i << "_id)\" />" << std::endl;
    }
    launch_file << "  <arg name=\"base_sensor\" value=\"$(arg sensor" << base_sensor << "_name)\" />" << std::endl << std::endl;

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
        << "    <param name=\"num_sensors\" value=\"$(arg num_sensors)\" /> " << std::endl
        << "    <param name=\"rows\" value=\"$(arg rows)\" />" << std::endl
        << "    <param name=\"cols\" value=\"$(arg cols)\" />" << std::endl
        << "    <param name=\"cell_width\" value=\"$(arg cell_width)\" />" << std::endl
        << "    <param name=\"cell_height\" value=\"$(arg cell_height)\" />" << std::endl << std::endl
        << "    <param name=\"base_sensor\" value=\"$(arg base_sensor)\" />" << std::endl << std::endl;

    if (calibration_with_serials)
      launch_file << "    <param name=\"calibration_with_serials\" value=\"true\" />" << std::endl << std::endl;

    // Parameters and remapping for every sensor:
    for (unsigned int i = 0; i < num_sensors; i++)
    {
      launch_file << "    <param name=\"sensor_" << i << "/name\" value=\"/$(arg sensor" << i << "_name)\" />" << std::endl;

      if (!std::strcmp(sensor_id_vector[i].substr(0,1).c_str(), "1"))    // if SwissRanger (sensor_id is an IP address)
      {
        launch_file << "    <param name=\"sensor_" << i << "/type\" value=\"pinhole_rgb\" />" << std::endl;
        launch_file << "    <remap from=\"~sensor_" << i << "/image\" to=\"/$(arg sensor" << i << "_name)/intensity/image_resized\" />" << std::endl;
        launch_file << "    <remap from=\"~sensor_" << i << "/camera_info\" to=\"/$(arg sensor" << i << "_name)/intensity/camera_info\" />" << std::endl << std::endl;
      }
      else                                                              // if Kinect
      {
        launch_file << "    <param name=\"sensor_" << i << "/type\" value=\"pinhole_rgb\" />" << std::endl;
        launch_file << "    <remap from=\"~sensor_" << i << "/image\" to=\"/$(arg sensor" << i << "_name)/rgb/image_rect_color\" />" << std::endl;
        launch_file << "    <remap from=\"~sensor_" << i << "/camera_info\" to=\"/$(arg sensor" << i << "_name)/rgb/camera_info\" />" << std::endl << std::endl;
      }
    }

    launch_file << "  </node>" << std::endl << std::endl;

    launch_file << "</launch>" << std::endl;
  }
  launch_file.close();
  ROS_INFO_STREAM(file_name << " created!");

  // Write a launch file for every sensor:
  for (unsigned int i = 0; i < num_sensors; i++)
  {
    std::string sensor_name = sensor_id_vector[i];
    if (!std::strcmp(sensor_id_vector[i].substr(0,1).c_str(), "1"))    // if SwissRanger (sensor_id is an IP address)
    {
      replace(sensor_name.begin(), sensor_name.end(), '.', '_');
      sensor_name = "SR_" + sensor_name;
    }

    file_name = ros::package::getPath("opt_calibration") + "/launch/sensor_" + sensor_name + ".launch";
    std::ofstream launch_file;
    launch_file.open(file_name.c_str());

    if (launch_file.is_open())
    {
      launch_file << "<launch>" << std::endl << std::endl;

      launch_file << "  <!-- sensor parameters -->" << std::endl
          << "  <arg name=\"sensor_id\" value=\"" << sensor_name << "\" />" << std::endl
          << "  <arg name=\"sensor_name\" default=\"$(arg sensor_id)\" />" << std::endl << std::endl;

      if (!std::strcmp(sensor_id_vector[i].substr(0,1).c_str(), "1"))    // if SwissRanger (sensor_id is an IP address)
      {
        launch_file << "  <!-- Launch sensor -->" << std::endl
            << "  <include file=\"$(find swissranger_camera)/launch/sr_eth.launch\">" << std::endl
            << "    <arg name=\"camera_id\" value=\"$(arg sensor_id)\" />" << std::endl
            << "    <arg name=\"device_ip\" value=\"" << sensor_id_vector[i] << "\" />" << std::endl
            << "  </include>" << std::endl << std::endl
            << "  <include file=\"$(find swissranger_camera)/launch/publisher_for_calibration.launch\">" << std::endl
            << "    <arg name=\"camera_id\" value=\"$(arg sensor_id)\" />" << std::endl
            << "  </include>" << std::endl << std::endl;
      }
      else                                                               // if Kinect
      {
        launch_file << "  <!-- Launch sensor -->" << std::endl
            << "  <include file=\"$(find detection)/launch/" << driver_name << ".launch\">" << std::endl;

        // If serial numbers can be used to identify sensors, they are added to the launch file:
        if (calibration_with_serials)
          launch_file << "    <arg name=\"device_id\" value=\"$(arg sensor_id)\" />" << std::endl;

        launch_file << "    <arg name=\"camera\" value=\"$(arg sensor_name)\" />" << std::endl
            << "  </include>" << std::endl << std::endl;

        launch_file << "  <!-- Publish a further transform -->" << std::endl
            << "  <node pkg=\"tf\" type=\"static_transform_publisher\" name=\"$(arg sensor_name)_broadcaster\" args=\"-0.045 0 0 1.57079 -1.57079 0 /$(arg sensor_name) /$(arg sensor_name)_link  100\" />" << std::endl << std::endl;
      }
      launch_file << "</launch>" << std::endl;
    }
    launch_file.close();
    ROS_INFO_STREAM(file_name << " created!");
  }

  return 0;
}
