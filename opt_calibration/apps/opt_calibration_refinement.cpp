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
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <list>
#include <sstream>
#include <fstream>
#include <string.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
#include <std_msgs/String.h>

//#include <open_ptrack/opt_utils/conversions.h>
#include <open_ptrack/detection/detection.h>
#include <open_ptrack/detection/detection_source.h>
#include <open_ptrack/opt_calibration/trajectory_registration.h>
#include <opt_msgs/Detection.h>
#include <opt_msgs/DetectionArray.h>
#include <opt_msgs/TrackArray.h>

// Global variables:
std::map<std::string, open_ptrack::detection::DetectionSource*> detection_sources_map;
tf::TransformListener* tf_listener;
std::string world_frame_id;
int detection_history_size;
bool output_detection_results;  // Enables/disables the publishing of detection positions to be visualized in RViz
ros::Publisher detection_marker_pub;
ros::Publisher detection_trajectory_pub;
size_t detection_insert_index;
tf::Transform camera_frame_to_world_transform;
tf::Transform world_to_camera_frame_transform;
double period;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr detection_history_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
double min_confidence;
std::vector<cv::Vec3f> camera_colors;     // vector containing colors to use to identify cameras in the network
std::map<std::string, int> color_map;     // map between camera frame_id and color
double voxel_size;
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > cloud_vector;
std::vector<std::vector<double> > timestamp_vector;
ros::Time start_time;
ros::Time start_time_playback;
bool start_time_set;
double time_scale_factor;
bool clouds_saved;
ros::Time latest_time;
bool save_detection_clouds;
double time_bin_size;                   // bin size for the time variable
double icp_max_correspondence_distance; // max correspondence distance for ICP
int N_iter;
bool doing_calibration_refinement;

void
saveRegistrationMatrix (std::string filename, Eigen::Matrix4d transformation)
{
  std::ofstream myfile;
  myfile.open (filename.c_str());
  myfile << transformation;
  myfile.close();
}

void
plotCameraLegend (std::map<std::string, int> curr_color_map)
{
  // Compose camera legend:
  cv::Mat legend_image = cv::Mat::zeros(500, 500, CV_8UC3);
  for(std::map<std::string, int>::iterator colormap_iterator = curr_color_map.begin(); colormap_iterator != curr_color_map.end(); colormap_iterator++)
  {
    int color_index = colormap_iterator->second;
    cv::Vec3f color = camera_colors[color_index];
    int y_coord = color_index * legend_image.rows / (curr_color_map.size()+1) + 0.5 * legend_image.rows / (curr_color_map.size()+1);
    cv::line(legend_image, cv::Point(0,y_coord), cv::Point(100,y_coord), cv::Scalar(255*color(2), 255*color(1), 255*color(0)), 8);
    cv::putText(legend_image, colormap_iterator->first, cv::Point(110,y_coord), 1, 1, cv::Scalar(255, 255, 255), 1);
  }

  // Display the cv image
  cv::imshow("Camera legend", legend_image);
}

int
performCalibrationRefinement (std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > curr_cloud_vector,
    std::vector<std::vector<double> > curr_timestamp_vector, std::map<std::string, int> curr_color_map)
{
  // Remove empty entries (cameras with no detections):
  unsigned int num_empty_entries = curr_cloud_vector.size() - curr_color_map.size();
  for (unsigned int i = 0; i < num_empty_entries; i++)
  {
    curr_cloud_vector.pop_back();
    curr_timestamp_vector.pop_back();
  }

  // Initialize TrajectoryRegistration object:
  open_ptrack::opt_calibration::TrajectoryRegistration registrator;
  registrator.setTimeBinSize (time_bin_size);
  registrator.setICPMaxCorrespondenceDistance (icp_max_correspondence_distance);
  registrator.setTimestamps (curr_timestamp_vector);
  registrator.setColormap (curr_color_map);
  registrator.setNIterations (N_iter);

  // Perform calibration refinement:
  std::map<std::string, Eigen::Matrix4d> registration_matrices;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > registered_cloud_vector;
  for (unsigned int i = 0; i < curr_cloud_vector.size(); i++)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    registered_cloud_vector.push_back(empty_cloud_ptr);
  }
  registrator.compute (curr_cloud_vector, time_scale_factor, registration_matrices, registered_cloud_vector);

  // Save registration matrices:
  for(std::map<std::string, int>::iterator colormap_iterator = curr_color_map.begin(); colormap_iterator != curr_color_map.end(); colormap_iterator++)
  {
    saveRegistrationMatrix(ros::package::getPath("opt_calibration") + "/conf/registration_" + colormap_iterator->first + ".txt", registration_matrices[colormap_iterator->first]);
  }

//  // Visualization of clouds in registered_cloud_vector:
//  registrator.visualizeClouds (registered_cloud_vector, "Seeked registration", true);

  std::cout << "Calibration refinement finished. Press 'Ctrl+C' to exit." << std::endl;

  // Visualize input and output of registration:
  pcl::visualization::PCLVisualizer final_viewer("Input clouds and final registration");
  registrator.visualizeFinalRegistration (curr_cloud_vector, registration_matrices, final_viewer);
  while (not final_viewer.wasStopped())
  {
    final_viewer.spinOnce();
    cv::waitKey(1);
  }

  doing_calibration_refinement = false;

  return 0;
}

void
actionCallback(const std_msgs::String::ConstPtr & msg)
{
  if (msg->data == "save" or msg->data == "saveExtrinsicCalibration")
  {
    ROS_INFO("Performing calibration refinement...");
    doing_calibration_refinement = true;
    performCalibrationRefinement (cloud_vector, timestamp_vector, color_map);
  }
  else
  {
    ROS_ERROR_STREAM("Action " << msg->data << " unknown!");
  }
}

void
saveTimestampFile (std::string filename, std::vector<double> timestamps)
{
  std::ofstream myfile;
  myfile.open (filename.c_str());
  for (unsigned int i = 0; i < timestamps.size(); i++)
  {
    myfile << timestamps[i] << "\n";
  }
  myfile.close();
}

/**
 * \brief Create marker to be visualized in RViz
 *
 * \param[in] id The marker ID.
 * \param[in] frame_id The marker reference frame.
 * \param[in] position The marker position.
 * \param[in] color The marker color.
 */
visualization_msgs::Marker
createMarker (int id, std::string frame_id, ros::Time stamp, Eigen::Vector3d position, cv::Vec3f color)
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = world_frame_id;
  marker.header.stamp = stamp;
  marker.ns = frame_id;
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = position(0);
  marker.pose.position.y = position(1);
  marker.pose.position.z = position(2);
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.r = color(0);
  marker.color.g = color(1);
  marker.color.b = color(2);
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(0.2);

  return marker;
}

/**
 * \brief Read the DetectionArray message and use the detections for creating/updating/deleting tracks
 *
 * \param[in] msg the DetectionArray message.
 */
void
detection_cb(const opt_msgs::DetectionArray::ConstPtr& msg)
{
  if (not doing_calibration_refinement)
  {
    if (not start_time_set)
    {
      start_time = msg->header.stamp;
      start_time_playback = ros::Time::now();
      start_time_set = true;
    }

    // Read message header information:
    std::string frame_id = msg->header.frame_id;
    ros::Time frame_time = msg->header.stamp;

    // Compute delay of detection message, if any:
    double time_delay = 0.0;
    if (frame_time > latest_time)
    {
      latest_time = frame_time;
      time_delay = 0.0;
    }
    else
    {
      time_delay = (latest_time - frame_time).toSec();
    }

    tf::StampedTransform transform;
    tf::StampedTransform inverse_transform;

    try
    {
      //Calculate direct and inverse transforms between camera and world frame:
      tf_listener->lookupTransform(world_frame_id, frame_id, ros::Time(0), transform);
      tf_listener->lookupTransform(frame_id, world_frame_id, ros::Time(0), inverse_transform);

      // Read camera intrinsic parameters:
      Eigen::Matrix3d intrinsic_matrix;
      for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
          intrinsic_matrix(i, j) = msg->intrinsic_matrix[i * 3 + j];

      // Add a new DetectionSource or update an existing one:
      if(detection_sources_map.find(frame_id) == detection_sources_map.end())
      {
        detection_sources_map[frame_id] = new open_ptrack::detection::DetectionSource(cv::Mat(0, 0, CV_8UC3),
            transform, inverse_transform, intrinsic_matrix, frame_time, frame_id);
      }
      else
      {
        detection_sources_map[frame_id]->update(cv::Mat(0, 0, CV_8UC3), transform, inverse_transform,
            intrinsic_matrix, frame_time, frame_id);
        double d = detection_sources_map[frame_id]->getDuration().toSec() / period;
        int lostFrames = int(round(d)) - 1;
      }
      open_ptrack::detection::DetectionSource* source = detection_sources_map[frame_id];

      // Create a Detection object for every detection in the detection message:
      std::vector<open_ptrack::detection::Detection> detections_vector;
      for(std::vector<opt_msgs::Detection>::const_iterator it = msg->detections.begin();
          it != msg->detections.end(); it++)
      {
        detections_vector.push_back(open_ptrack::detection::Detection(*it, source));
      }

      // Convert HOG+SVM confidences to HAAR+ADABOOST-like people detection confidences:
      if (not std::strcmp(msg->confidence_type.c_str(), "hog+svm"))
      {
        for(unsigned int i = 0; i < detections_vector.size(); i++)
        {
          double new_confidence = detections_vector[i].getConfidence();
          new_confidence = (new_confidence - (-3)) / 3 * 4 + 2;
          detections_vector[i].setConfidence(new_confidence);
        }
      }

      // If at least one detection has been received:
      if(detections_vector.size() > 0)
      {
        // Create message for showing detection positions in RViz:
        if (output_detection_results)
        {
          visualization_msgs::MarkerArray::Ptr marker_msg(new visualization_msgs::MarkerArray);
          detection_history_pointcloud->header.stamp = frame_time.toNSec() / 1e3;  // Convert from ns to us
          detection_history_pointcloud->header.frame_id = world_frame_id;
          std::string camera_name = detections_vector[0].getSource()->getFrameId();
          if (strcmp(camera_name.substr(0,1).c_str(), "/") == 0)
          {
            camera_name = camera_name.substr(1, camera_name.size() - 1);
          }

          // Define color:
          int color_index;
          std::map<std::string, int>::iterator colormap_iterator = color_map.find(camera_name);
          if (colormap_iterator != color_map.end())
          { // camera already present
            color_index = colormap_iterator->second;
          }
          else
          { // camera not present
            color_index = color_map.size();
            color_map.insert(std::pair<std::string, int> (camera_name, color_index));
          }
          ros::Duration time_offset = (msg->header.stamp - start_time);
          for (unsigned int i = 0; i < detections_vector.size(); i++)
          {
            // Create marker and add it to message:
            Eigen::Vector3d centroid = detections_vector[i].getWorldCentroid();

//            Eigen::Vector2d badPoint(-3.43483, 2.18623);
//            Eigen::Vector2d currPoint(centroid(0), centroid(1));
//            if ((currPoint - badPoint).norm() > 0.2)
//            {
              visualization_msgs::Marker marker = createMarker (i, frame_id, frame_time, centroid, camera_colors[color_index]);
              marker_msg->markers.push_back(marker);

              // Point cloud:
              pcl::PointXYZRGB point;
              point.x = marker.pose.position.x;
              point.y = marker.pose.position.y;
              point.z = marker.pose.position.z;
              point.r = marker.color.r * 255.0f;
              point.g = marker.color.g * 255.0f;
              point.b = marker.color.b * 255.0f;
              detection_insert_index = (detection_insert_index + 1) % detection_history_size;
              detection_history_pointcloud->points[detection_insert_index] = point;
              //            point.z =  (time_offset.toSec() / time_scale_factor);
              cloud_vector[color_index]->push_back(point);
              timestamp_vector[color_index].push_back(time_offset.toSec());
//            }
          }

          //        std::cout << color_index << " " << (ros::Time::now() - start_time_playback).toSec() << " " << time_delay << std::endl;

          detection_marker_pub.publish(marker_msg); // publish marker message
          detection_trajectory_pub.publish(detection_history_pointcloud); // publish trajectory message

          // Plot legend with camera names and colors:
          plotCameraLegend (color_map);

//                      std::cout << colormap_iterator->first << " " << colormap_iterator->second << std::endl;
        }
      }
      else // if no detections have been received
      {
      }
    }
    catch(tf::TransformException& ex)
    {
      ROS_ERROR("transform exception: %s", ex.what());
    }
  }
}

void
generateColors(int colors_number, std::vector<cv::Vec3f>& colors)
{
  for (unsigned int i = 0; i < colors_number; i++)
  {
    colors.push_back(cv::Vec3f(
        float(rand() % 256) / 255,
        float(rand() % 256) / 255,
        float(rand() % 256) / 255));
  }
}

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "opt_calibration_refinement");
  ros::NodeHandle nh("~");

  tf_listener = new tf::TransformListener();

  // Read tracking parameters:
  nh.param("world_frame_id", world_frame_id, std::string("/odom"));
  nh.param("voxel_size", voxel_size, 0.06);
  double rate;
  nh.param("rate", rate, 30.0);
  nh.param("min_confidence_initialization", min_confidence, -3.0);
  nh.param("detection_debug", output_detection_results, true);
  bool debug_mode;
  nh.param("debug_active", debug_mode, false);
  nh.param("save_detection_clouds", save_detection_clouds, false);
  nh.param("time_bin_size", time_bin_size, 0.5);
  nh.param("icp_max_correspondence_distance", icp_max_correspondence_distance, 0.5);
  nh.param("time_scale_factor", time_scale_factor, 1.0);
  nh.param("calibration_refinement_iterations", N_iter, 4);

  doing_calibration_refinement = false;

  // Read number of sensors in the network:
  int num_cameras = 0;
  XmlRpc::XmlRpcValue network;
  nh.getParam("network", network);
  std::map<std::string, XmlRpc::XmlRpcValue>::iterator i;
  for (unsigned i = 0; i < network.size(); i++)
  {
    num_cameras += network[i]["sensors"].size();
  }

  // Subscribers/Publishers:
  ros::Subscriber input_sub = nh.subscribe("/detector/detections", num_cameras, detection_cb);
  ros::Subscriber action_sub = nh.subscribe("/opt_calibration/action", 1, actionCallback);
  detection_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/detector/markers_array", 1);
  detection_trajectory_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGBA> >("/detector/history", 1);

  // Create a detection point cloud for every sensor:
  std::vector<double> empty_vector;
  for (unsigned int i = 0; i < num_cameras; i++)
  {
    cloud_vector.push_back(pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>));
    timestamp_vector.push_back(empty_vector);
  }

  // Compute additional parameters:
  period = 1.0 / rate;

  // Generate colors used to identify different cameras:
  generateColors(num_cameras, camera_colors);

  // Initialize point cloud containing detections trajectory:
  detection_history_size = 10000*num_cameras;
  pcl::PointXYZRGB nan_point;
  nan_point.x = std::numeric_limits<float>::quiet_NaN();
  nan_point.y = std::numeric_limits<float>::quiet_NaN();
  nan_point.z = std::numeric_limits<float>::quiet_NaN();
  detection_history_pointcloud->points.resize(detection_history_size, nan_point);

  ros::Rate hz(num_cameras*rate);

  time_scale_factor = 1;
  start_time_set = false;

  // Spin and execute callbacks:
  ros::Time last_camera_legend_update = ros::Time::now();           // last time when the camera legend has been updated

  while (ros::ok)
  {
    ros::spinOnce();

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
