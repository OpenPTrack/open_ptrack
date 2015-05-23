/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015-, Open Perception, Inc.
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
#include <Eigen/Eigen>
#include <tf/tf.h>
#include <list>
#include <sstream>
#include <fstream>
#include <string.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <std_msgs/String.h>
#include <calibration_common/calibration_common.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <pcl_conversions/pcl_conversions.h>

using namespace calibration;

int line_width = 4;

void
generateColors (int colors_number, std::vector<cv::Vec3f>& colors)
{
  for (unsigned int i = 0; i < colors_number; i++)
  {
    colors.push_back(cv::Vec3f(
        float(rand() % 256) / 255,
        float(rand() % 256) / 255,
        float(rand() % 256) / 255));
  }
}

pcl::PointXYZ
Eigen2PointXYZ (Eigen::Vector3d v)
{
  pcl::PointXYZ p(v[0],v[1],v[2]);
  return p;
}

Eigen::Vector3d
cameraRayWithGroundIntersection (Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Hyperplane<double, 3> ground_plane)
{
  Eigen::ParametrizedLine<double,3> camera_ray = Eigen::ParametrizedLine<double,3>::Through(p1, p2);
  double intersection = camera_ray.intersection(ground_plane);
  if ((intersection > 0) && (intersection <= (p2-p1).norm()))
  {
    return (intersection*((p2-p1).normalized()) + p1);
  }
  else
  {
    return (p2);
  }
}

Eigen::Vector3d
computeIntermediatePoint (Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Hyperplane<double, 3> ground_plane, Eigen::Vector3d degenerate_point)
{
  Eigen::ParametrizedLine<double,3> camera_ray = Eigen::ParametrizedLine<double,3>::Through(p1, p2);
  double intersection = camera_ray.intersection(ground_plane);
  if ((intersection > 0) && (intersection <= (p2-p1).norm()))
  {
    return (intersection*((p2-p1).normalized()) + p1);
  }
  else
  {
    return (degenerate_point);
  }
}

void
plotCameraFieldOfView (Pose& camera_pose, float cx, float cy, float focal, float max_distance,
    cv::Vec3f& color, std::string frame_id, pcl::visualization::PCLVisualizer& viewer)
{
  // Find extreme points in meters:
  cx = cx * max_distance / focal;
  cy = cy * max_distance / focal;

  std::stringstream ss_tl, ss_tr, ss_bl, ss_br, ss_diag1, ss_diag2, s1, s2, s3, s4, s5, s6, s7, s8, text;
  ss_tl << frame_id << "_tl";
  ss_tr << frame_id << "_tr";
  ss_bl << frame_id << "_bl";
  ss_br << frame_id << "_br";
  ss_diag1 << frame_id << "_diag1";
  ss_diag2 << frame_id << "_diag2";
  s1 << frame_id << "_s1";
  s2 << frame_id << "_s2";
  s3 << frame_id << "_s3";
  s4 << frame_id << "_s4";
  s5 << frame_id << "_s5";
  s6 << frame_id << "_s6";
  s7 << frame_id << "_s7";
  s8 << frame_id << "_s8";
  text << frame_id << "_text";

  pcl::PointCloud<pcl::PointXYZ>::Ptr final_points(new pcl::PointCloud<pcl::PointXYZ>);

  // Define ground plane:
  Eigen::Hyperplane<double, 3> ground_plane(Eigen::Vector3d(0.0, 0.0, 1.0), 0.0);

  // Define end points of the pyramid:
  Eigen::Vector3d p0 = camera_pose * Eigen::Vector3d(0.0, 0.0, 0.0);
  pcl::PointXYZ p0_pcl = Eigen2PointXYZ(p0);
  Eigen::Vector3d p1 = camera_pose * Eigen::Vector3d(-cx, -cy, max_distance);
  Eigen::Vector3d p2 = camera_pose * Eigen::Vector3d(cx, -cy, max_distance);
  Eigen::Vector3d p3 = camera_pose * Eigen::Vector3d(cx, cy, max_distance);
  Eigen::Vector3d p4 = camera_pose * Eigen::Vector3d(-cx, cy, max_distance);

  // Compute intersection of camera rays with the ground plane:
  Eigen::Vector3d int1, int2, int3, int4;
  int1 = cameraRayWithGroundIntersection (p0, p1, ground_plane);
  int2 = cameraRayWithGroundIntersection (p0, p2, ground_plane);
  int3 = cameraRayWithGroundIntersection (p0, p3, ground_plane);
  int4 = cameraRayWithGroundIntersection (p0, p4, ground_plane);

  // Find p12, intersection of p1p2 with ground:
  Eigen::Vector3d p12 = computeIntermediatePoint (p1, p2, ground_plane, int1);
  Eigen::Vector3d p23 = computeIntermediatePoint (p2, p3, ground_plane, int2);
  Eigen::Vector3d p34 = computeIntermediatePoint (p3, p4, ground_plane, int3);
  Eigen::Vector3d p41 = computeIntermediatePoint (p4, p1, ground_plane, int4);

//  final_points->push_back(Eigen2PointXYZ(p12));
//  final_points->push_back(Eigen2PointXYZ(p23));
//  final_points->push_back(Eigen2PointXYZ(p34));
//  final_points->push_back(Eigen2PointXYZ(p41));

  // Visualization:
  viewer.addLine(p0_pcl, Eigen2PointXYZ(int1), color(0), color(1), color(2), ss_tl.str());
  viewer.addLine(p0_pcl, Eigen2PointXYZ(int2), color(0), color(1), color(2), ss_tr.str());
  viewer.addLine(p0_pcl, Eigen2PointXYZ(int3), color(0), color(1), color(2), ss_br.str());
  viewer.addLine(p0_pcl, Eigen2PointXYZ(int4), color(0), color(1), color(2), ss_bl.str());
  viewer.addLine(Eigen2PointXYZ(p12), Eigen2PointXYZ(int1), color(0), color(1), color(2), s1.str());
  viewer.addLine(Eigen2PointXYZ(p12), Eigen2PointXYZ(int2), color(0), color(1), color(2), s2.str());
  viewer.addLine(Eigen2PointXYZ(p23), Eigen2PointXYZ(int2), color(0), color(1), color(2), s3.str());
  viewer.addLine(Eigen2PointXYZ(p23), Eigen2PointXYZ(int3), color(0), color(1), color(2), s4.str());
  viewer.addLine(Eigen2PointXYZ(p34), Eigen2PointXYZ(int3), color(0), color(1), color(2), s5.str());
  viewer.addLine(Eigen2PointXYZ(p34), Eigen2PointXYZ(int4), color(0), color(1), color(2), s6.str());
  viewer.addLine(Eigen2PointXYZ(p41), Eigen2PointXYZ(int4), color(0), color(1), color(2), s7.str());
  viewer.addLine(Eigen2PointXYZ(p41), Eigen2PointXYZ(int1), color(0), color(1), color(2), s8.str());
  viewer.addText3D(frame_id, p0_pcl, 0.2, 1.0, 1.0, 1.0, text.str());
//  viewer.addLine(Eigen2PointXYZ(int1), Eigen2PointXYZ(int3), color(0), color(1), color(2), ss_diag1.str());
//  viewer.addLine(Eigen2PointXYZ(int2), Eigen2PointXYZ(int4), color(0), color(1), color(2), ss_diag2.str());

  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width-2, ss_tl.str());
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width-2, ss_tr.str());
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width-2, ss_br.str());
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width-2, ss_bl.str());
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, s1.str());
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, s2.str());
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, s3.str());
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, s4.str());
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, s5.str());
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, s6.str());
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, s7.str());
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, s8.str());
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.7, ss_tl.str());
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.7, ss_tr.str());
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.7, ss_br.str());
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.7, ss_bl.str());

//  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, ss_diag1.str());
//  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, ss_diag2.str());

//  final_points->push_back(p0_pcl);
//  final_points->push_back(Eigen2PointXYZ(int1));
//  final_points->push_back(Eigen2PointXYZ(int2));
//  final_points->push_back(Eigen2PointXYZ(int3));
//  final_points->push_back(Eigen2PointXYZ(int4));
//
//  final_points->push_back(Eigen2PointXYZ(p1));
//  final_points->push_back(Eigen2PointXYZ(p2));
//  final_points->push_back(Eigen2PointXYZ(p3));
//  final_points->push_back(Eigen2PointXYZ(p4));

//  viewer.addPointCloud(final_points, frame_id);
//  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, frame_id);

  viewer.addCoordinateSystem(1.0);
}

void
plotCameraLegend (std::vector<std::string> camera_frame_ids, std::vector<cv::Vec3f> camera_colors)
{
  // Compose camera legend:
  cv::Mat legend_image = cv::Mat::zeros(500, 500, CV_8UC3);
  for(unsigned int i = 0; i < camera_frame_ids.size(); i++)
  {
    cv::Vec3f color = camera_colors[i];
    int y_coord = i * legend_image.rows / (camera_frame_ids.size()+1) + 0.5 * legend_image.rows / (camera_frame_ids.size()+1);
    cv::line(legend_image, cv::Point(0,y_coord), cv::Point(100,y_coord), cv::Scalar(255*color(2), 255*color(1), 255*color(0)), 8);
    cv::putText(legend_image, camera_frame_ids[i], cv::Point(110,y_coord), 1, 1, cv::Scalar(255, 255, 255), 1);
  }

  // Display the cv image
  cv::imshow("Camera legend", legend_image);
}

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "network_assessment");
  ros::NodeHandle nh("~");

  // Intrinsic parameters:
  float f_k1 = 525;
  int cx_k1 = 320;
  int cy_k1 = 240;
  float f_k2 = 365;
  int cx_k2 = 256;
  int cy_k2 = 212;
  float f_sr = 148;
  int cx_sr = 86;
  int cy_sr = 72;
  float f_pg = 525;
  int cx_pg = 320;
  int cy_pg = 240;

  // Read max distance parameter:
  double max_distance_k1, max_distance_k2, max_distance_sr, max_distance_pg;
  nh.param("kinect1/max_distance", max_distance_k1, 15.0);
  nh.param("kinect2/max_distance", max_distance_k2, 15.0);
  nh.param("sr4500/max_distance", max_distance_sr, 15.0);
  nh.param("stereo_pg/max_distance", max_distance_pg, 15.0);

  pcl::visualization::PCLVisualizer viewer("Network assessment");

  // Read number of sensors in the network:
  int num_cameras = 0;
  XmlRpc::XmlRpcValue network;
  nh.getParam("network", network);
  std::map<std::string, XmlRpc::XmlRpcValue>::iterator i;
  for (unsigned i = 0; i < network.size(); i++)
  {
    num_cameras += network[i]["sensors"].size();
  }

  // Generate colors used to identify different cameras:
  std::vector<cv::Vec3f> camera_colors;     // vector containing colors to use to identify cameras in the network
  generateColors(num_cameras, camera_colors);

  // Read frame_id of all sensors and corresponding pose and plot cameras:
  std::vector<Pose> camera_poses;
  std::vector<std::string> camera_frame_ids;
  nh.getParam("network", network);
  int k = 0;
  float cx, cy, f, max_distance;
  int color_index;
  for (unsigned i = 0; i < network.size(); i++)
  {
    for (unsigned j = 0; j < network[i]["sensors"].size(); j++)
    {
      std::string frame_id = network[i]["sensors"][j]["id"];
      camera_frame_ids.push_back(frame_id);

      std::string pose_s = "/poses/" + frame_id;
      if (not nh.hasParam(pose_s))
        ROS_FATAL_STREAM("No \"" << pose_s << "\" parameter found!! Has \"conf/camera_poses.yaml\" been loaded with \"rosparam load ...\"?");

      // Read pose:
      calibration::Translation3 t;
      nh.getParam(pose_s + "/translation/x", t.x());
      nh.getParam(pose_s + "/translation/y", t.y());
      nh.getParam(pose_s + "/translation/z", t.z());

      calibration::Quaternion q;
      nh.getParam(pose_s + "/rotation/x", q.x());
      nh.getParam(pose_s + "/rotation/y", q.y());
      nh.getParam(pose_s + "/rotation/z", q.z());
      nh.getParam(pose_s + "/rotation/w", q.w());

      camera_poses.push_back(Pose::Identity());
      camera_poses[k].linear() = q.toRotationMatrix();
      camera_poses[k].translation() = t.vector();

      // Select parameters relative to the type of sensor:
      std::string type = network[i]["sensors"][j]["type"];
      if (strcmp(type.c_str(), "kinect1") == 0)
      {
        cx = cx_k1;
        cy = cy_k1;
        f = f_k1;
        max_distance = max_distance_k1;
      }
      else if (strcmp(type.c_str(), "kinect2") == 0)
      {
        cx = cx_k2;
        cy = cy_k2;
        f = f_k2;
        max_distance = max_distance_k2;
      }
      else if (strcmp(type.c_str(), "sr4500") == 0)
      {
        cx = cx_sr;
        cy = cy_sr;
        f = f_sr;
        max_distance = max_distance_sr;
      }
      else if (strcmp(type.c_str(), "stereo_pg") == 0)
      {
        cx = cx_pg;
        cy = cy_pg;
        f = f_pg;
        max_distance = max_distance_pg;
      }

      // Plot camera field of view:
      plotCameraFieldOfView (camera_poses[k], cx, cy, f, max_distance, camera_colors[k], frame_id, viewer);

      k++;
    }
  }

  // Plot ground plane:
  pcl::PointCloud<pcl::PointXYZ>::Ptr groundplane(new pcl::PointCloud<pcl::PointXYZ>);
  for (float x = -10; x < 10; x = x+0.2)
  {
    for (float y = -10; y < 10; y = y+0.2)
    {
      groundplane->points.push_back(pcl::PointXYZ(x, y, 0));
    }
  }
  viewer.addPointCloud(groundplane, "groundplane");

  // Plot camera legend:
  plotCameraLegend (camera_frame_ids, camera_colors);

  // Update loop:
  ros::Rate hz(30);
  ros::Time last_camera_legend_update = ros::Time::now();           // last time when the camera legend has been updated
  while (ros::ok)
  {
    viewer.spinOnce();

    // Update camera legend every second:
    ros::Time now = ros::Time::now();
    if ((now - last_camera_legend_update) > ros::Duration(1.0))     // if more than one second passed since last update
    { // update OpenCV image with a waitKey:
      cv::waitKey(1);
      last_camera_legend_update = now;
    }

    hz.sleep();
  }

  return 0;
}
