/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013-, Matteo Munaro [matteo.munaro@dei.unipd.it]
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
 * ground_based_people_detector_node.cpp
 * Created on: Jul 07, 2013
 * Author: Matteo Munaro
 *
 * ROS node which performs people detection assuming that people stand/walk on a ground plane.
 * As a first step, the ground is manually initialized, then people detection is performed with the GroundBasedPeopleDetectionApp class,
 * which implements the people detection algorithm described here:
 * M. Munaro, F. Basso and E. Menegatti,
 * Tracking people within groups with RGB-D data,
 * In Proceedings of the International Conference on Intelligent Robots and Systems (IROS) 2012, Vilamoura (Portugal), 2012.
 */

// ROS includes:
#include <ros/ros.h>

// PCL includes:
#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl_conversions/pcl_conversions.h>

// Open PTrack includes:
#include <open_ptrack/detection/ground_based_people_detection_app.h>

//Publish Messages
#include "opt_msgs/RoiRect.h"
#include "opt_msgs/Rois.h"
#include "std_msgs/String.h"

using namespace opt_msgs;
using namespace sensor_msgs;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool new_cloud_available_flag = false;
PointCloudT::Ptr cloud(new PointCloudT);

// PCL viewer //
pcl::visualization::PCLVisualizer viewer("PCL Viewer");

enum { COLS = 640, ROWS = 480 };

void cloud_cb(const PointCloudT::ConstPtr& callback_cloud)
{
  *cloud = *callback_cloud;
  new_cloud_available_flag = true;
}

struct callback_args{
    // structure used to pass arguments to the callback function
    PointCloudT::Ptr clicked_points_3d;
    pcl::visualization::PCLVisualizer* viewerPtr;
};

void
pp_callback (const pcl::visualization::PointPickingEvent& event, void* args)
{
  struct callback_args* data = (struct callback_args *)args;
  if (event.getPointIndex () == -1)
    return;
  PointT current_point;
  event.getPoint(current_point.x, current_point.y, current_point.z);
  data->clicked_points_3d->points.push_back(current_point);
  // Draw clicked points in red:
  pcl::visualization::PointCloudColorHandlerCustom<PointT> red (data->clicked_points_3d, 255, 0, 0);
  data->viewerPtr->removePointCloud("clicked_points");
  data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
  data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
  std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "ground_based_people_detector");
  ros::NodeHandle nh("~");

  // Read some parameters from launch file:
  std::string svm_filename;
  nh.param("classifier_file", svm_filename, std::string("./"));
  bool use_rgb;
  nh.param("use_rgb", use_rgb, false);
  double min_confidence;
  nh.param("HogSvmThreshold", min_confidence, -1.5);
  double min_height;
  nh.param("minimum_person_height", min_height, 1.3);
  double max_height;
  nh.param("maximum_person_height", max_height, 2.3);
  int sampling_factor;
  nh.param("sampling_factor", sampling_factor, 1);
  std::string pointcloud_topic;
  nh.param("pointcloud_topic", pointcloud_topic, std::string("/camera/depth_registered/points"));
  double rate_value;
  nh.param("rate", rate_value, 30.0);

  // Fixed parameters:
  float voxel_size = 0.06;
  Eigen::Matrix3f rgb_intrinsics_matrix;
  rgb_intrinsics_matrix << 525, 0.0, 319.5, 0.0, 525, 239.5, 0.0, 0.0, 1.0; // Kinect RGB camera intrinsics

  // Subscribers:
  ros::Subscriber sub = nh.subscribe(pointcloud_topic, 1, cloud_cb);

  // Publishers:
  ros::Publisher pub_rois_;
  pub_rois_= nh.advertise<Rois>("GroundBasedPeopleDetectorOutputRois",3);

  Rois output_rois_;

  ros::Rate rate(rate_value);
  while(ros::ok() && !new_cloud_available_flag)
  {
    ros::spinOnce();
    rate.sleep();
  }

  // Display pointcloud:
  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
  viewer.addPointCloud<PointT> (cloud, rgb, "input_cloud");
  viewer.setCameraPosition(0,0,-2,0,-1,0,0);

  // Add point picking callback to viewer:
  struct callback_args cb_args;
  PointCloudT::Ptr clicked_points_3d (new PointCloudT);
  cb_args.clicked_points_3d = clicked_points_3d;
  cb_args.viewerPtr = &viewer;
  viewer.registerPointPickingCallback (pp_callback, (void*)&cb_args);
  std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;

  // Spin until 'Q' is pressed:
  viewer.spin();
  std::cout << "done." << std::endl;

  // Ground plane estimation:
  Eigen::VectorXf ground_coeffs;
  ground_coeffs.resize(4);
  std::vector<int> clicked_points_indices;
  for (unsigned int i = 0; i < clicked_points_3d->points.size(); i++)
    clicked_points_indices.push_back(i);
  pcl::SampleConsensusModelPlane<PointT> model_plane(clicked_points_3d);
  model_plane.computeModelCoefficients(clicked_points_indices,ground_coeffs);
  ROS_ERROR("Ground plane coefficients: %f, %f, %f, %f.", ground_coeffs(0), ground_coeffs(1), ground_coeffs(2), ground_coeffs(3));

  // Create classifier for people detection:
  pcl::people::PersonClassifier<pcl::RGB> person_classifier;
  person_classifier.loadSVMFromFile(svm_filename);   // load trained SVM

  // People detection app initialization:
  open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT> people_detector;    // people detection object
  people_detector.setVoxelSize(voxel_size);                        // set the voxel size
  people_detector.setIntrinsics(rgb_intrinsics_matrix);            // set RGB camera intrinsic parameters
  people_detector.setClassifier(person_classifier);                // set person classifier
  people_detector.setHeightLimits(min_height, max_height);         // set person classifier
  people_detector.setSamplingFactor(sampling_factor);              // set sampling factor

  // Main loop:
  while(ros::ok())
  {
    if (new_cloud_available_flag)
    {
      new_cloud_available_flag = false;

      // Convert PCL cloud header to ROS header:
      std_msgs::Header cloud_header = pcl_conversions::fromPCL(cloud->header);

      // Perform people detection on the new cloud:
      std::vector<pcl::people::PersonCluster<PointT> > clusters;   // vector containing persons clusters
      people_detector.setInputCloud(cloud);
      people_detector.setGround(ground_coeffs);                    // set floor coefficients
      people_detector.compute(clusters);                           // perform people detection
      people_detector.setUseRGB(false);                            // set if RGB should be used or not

      ground_coeffs = people_detector.getGround();                 // get updated floor coefficients

      // Write ROIs message and publish it:
      output_rois_.rois.clear();
      output_rois_.header = cloud_header;
      for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
      {
        if((!use_rgb) | (it->getPersonConfidence() > min_confidence))            // keep only people with confidence above a threshold
        {
          // theoretical person centroid:
          Eigen::Vector3f centroid = rgb_intrinsics_matrix * (it->getTCenter());
          centroid /= centroid(2);
          // theoretical person top point:
          Eigen::Vector3f top = rgb_intrinsics_matrix * (it->getTTop());
          top /= top(2);

          // Define RoiRect and make sure it is not out of the image:
          RoiRect R;
          R.height = centroid(1) - top(1);
          R.width  = R.height * 2 / 3.0;
          R.x      = std::max(0, int(centroid(0) - R.width / 2.0));
          R.y      = std::max(0, int(top(1)));
          R.height = std::min(int(ROWS - R.y), int(R.height));
          R.width = std::min(int(COLS - R.x), int(R.width));
          R.label  = 1;
          R.confidence  = it->getPersonConfidence();
          output_rois_.rois.push_back(R);
        }
      }
      pub_rois_.publish(output_rois_);  // publish message
    }

    // Execute callbacks:
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

