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
#include <stdlib.h>     /* system, NULL, EXIT_FAILURE */
#include <tf/transform_listener.h>

int
main(int argc, char ** argv)
{
  ros::init(argc, argv, "calibration_saver");
  ros::NodeHandle nh("~");

  // Read some parameters from launch file:
  int num_cameras;
  nh.param("num_cameras", num_cameras, 1);
  int base_camera;
  nh.param("base_camera", base_camera, 0);
  bool calibration_with_serials;
  nh.param("calibration_with_serials", calibration_with_serials, false);

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

  // Kill calibration node before saving camera poses file:
  int ret_value = system ("killall -9 opt_calibration");

  // Wait a bit:
  ros::Duration(1).sleep();

  // Save file with all cameras position with respect to the world frame:
  std::string file_name = ros::package::getPath("detection") + "/launch/camera_poses.txt";
  std::ofstream poses_file;
  poses_file.open(file_name.c_str());
  for (unsigned int i = 0; i < num_cameras; i++)
  {
    std::string camera_name;
    if (!std::strcmp(camera_id_vector[i].substr(0,1).c_str(), "1"))    // if SwissRanger
    {
      camera_name = camera_id_vector[i];
      replace(camera_name.begin(), camera_name.end(), '.', '_');
      camera_name = "SR_" + camera_name;
    }
    else
    {
      camera_name = camera_id_vector[i];
    }

    // Read transform between world and camera reference frame:
    tf::TransformListener tfListener;
    tf::StampedTransform worldToCamTransform;
    try
    {
      tfListener.waitForTransform(camera_name, "/world", ros::Time(0), ros::Duration(3.0), ros::Duration(0.01));
      tfListener.lookupTransform(camera_name, "/world", ros::Time(0), worldToCamTransform);

      // Extract rotation angles from camera pose:
      tf::Quaternion q = worldToCamTransform.getRotation();

      // Write transform between camera and world frame to file:
      poses_file << camera_name << ": " << worldToCamTransform.getOrigin().x() << " " << worldToCamTransform.getOrigin().y() << " " << worldToCamTransform.getOrigin().z()
          << " " << q.getX() << " " << q.getY() << " " << q.getZ() << " " << q.getW() << std::endl;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
    }
  }

  poses_file.close();
  ROS_INFO_STREAM(file_name << " created!");

  return 0;
}
