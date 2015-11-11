/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013-, Open Perception, Inc.
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
 * ground_based_people_detection_app.hpp
 * Created on: Nov 30, 2012
 * Author: Matteo Munaro
 */

#ifndef OPEN_PTRACK_DETECTION_GROUND_BASED_PEOPLE_DETECTION_APP_HPP_
#define OPEN_PTRACK_DETECTION_GROUND_BASED_PEOPLE_DETECTION_APP_HPP_

#include <open_ptrack/detection/ground_based_people_detection_app.h>

template <typename PointT>
open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT>::GroundBasedPeopleDetectionApp ()
{
//  denoising_viewer_ = pcl::visualization::PCLVisualizer::Ptr (new pcl::visualization::PCLVisualizer("filtering_viewer"));

  rgb_image_ = pcl::PointCloud<pcl::RGB>::Ptr(new pcl::PointCloud<pcl::RGB>);

  // set default values for optional parameters:
  sampling_factor_ = 1;
  voxel_size_ = 0.06;
  max_distance_ = 50.0;
  vertical_ = false;
  head_centroid_ = true;
  min_height_ = 1.3;
  max_height_ = 2.3;
  min_points_ = 30;     // this value is adapted to the voxel size in method "compute"
  max_points_ = 5000;   // this value is adapted to the voxel size in method "compute"
  dimension_limits_set_ = false;
  heads_minimum_distance_ = 0.3;
  use_rgb_ = true;
  mean_luminance_ = 0.0;
  sensor_tilt_compensation_ = false;
  background_subtraction_ = false;

  // set flag values for mandatory parameters:
  sqrt_ground_coeffs_ = std::numeric_limits<float>::quiet_NaN();
  person_classifier_set_flag_ = false;
  frame_counter_ = 0;
}

template <typename PointT> void
open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT>::setInputCloud (PointCloudPtr& cloud)
{
  cloud_ = cloud;
}

template <typename PointT> void
open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT>::setGround (Eigen::VectorXf& ground_coeffs)
{
  ground_coeffs_ = ground_coeffs;
  sqrt_ground_coeffs_ = (ground_coeffs - Eigen::Vector4f(0.0f, 0.0f, 0.0f, ground_coeffs(3))).norm();
}

template <typename PointT> void
open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT>::setSamplingFactor (int sampling_factor)
{
  sampling_factor_ = sampling_factor;
}

template <typename PointT> void
open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT>::setVoxelSize (float voxel_size)
{
  voxel_size_ = voxel_size;
}

template <typename PointT> void
open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT>::setDenoisingParameters (bool apply_denoising, int mean_k_denoising, float std_dev_denoising)
{
  apply_denoising_ = apply_denoising;
  mean_k_denoising_ = mean_k_denoising;
  std_dev_denoising_ = std_dev_denoising;
}

template <typename PointT> void
open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT>::setMaxDistance (float max_distance)
{
  max_distance_ = max_distance;
}

template <typename PointT> void
open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT>::setIntrinsics (Eigen::Matrix3f intrinsics_matrix)
{
  intrinsics_matrix_ = intrinsics_matrix;
}

template <typename PointT> void
open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT>::setClassifier (open_ptrack::detection::PersonClassifier<pcl::RGB> person_classifier)
{
  person_classifier_ = person_classifier;
  person_classifier_set_flag_ = true;
}

template <typename PointT> void
open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT>::setSensorPortraitOrientation (bool vertical)
{
  vertical_ = vertical;
}

template <typename PointT> void
open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT>::setHeightLimits (float min_height, float max_height)
{
  min_height_ = min_height;
  max_height_ = max_height;
}

template <typename PointT> void
open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT>::setDimensionLimits (int min_points, int max_points)
{
  min_points_ = min_points;
  max_points_ = max_points;
  dimension_limits_set_ = true;
}

template <typename PointT> void
open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT>::setMinimumDistanceBetweenHeads (float heads_minimum_distance)
{
  heads_minimum_distance_= heads_minimum_distance;
}

template <typename PointT> void
open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT>::setHeadCentroid (bool head_centroid)
{
  head_centroid_ = head_centroid;
}

template <typename PointT> void
open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT>::setSensorTiltCompensation (bool sensor_tilt_compensation)
{
  sensor_tilt_compensation_ = sensor_tilt_compensation;
}

template <typename PointT> void
open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT>::setUseRGB (bool use_rgb)
{
  use_rgb_ = use_rgb;
}

template <typename PointT> void
open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT>::setBackground (bool background_subtraction, float background_octree_resolution, PointCloudPtr& background_cloud)
{
  background_subtraction_ = background_subtraction;

  background_octree_ = new pcl::octree::OctreePointCloud<PointT>(background_octree_resolution);
  background_octree_->defineBoundingBox(-max_distance_/2, -max_distance_/2, 0.0, max_distance_/2, max_distance_/2, max_distance_);
  background_octree_->setInputCloud (background_cloud);
  background_octree_->addPointsFromInputCloud ();
}

template <typename PointT> void
open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT>::getHeightLimits (float& min_height, float& max_height)
{
  min_height = min_height_;
  max_height = max_height_;
}

template <typename PointT> void
open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT>::getDimensionLimits (int& min_points, int& max_points)
{
  min_points = min_points_;
  max_points = max_points_;
}

template <typename PointT> float
open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT>::getMinimumDistanceBetweenHeads ()
{
  return (heads_minimum_distance_);
}

template <typename PointT> Eigen::VectorXf
open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT>::getGround ()
{
  if (sqrt_ground_coeffs_ != sqrt_ground_coeffs_)
  {
    PCL_ERROR ("[open_ptrack::detection::GroundBasedPeopleDetectionApp::getGround] Floor parameters have not been set or they are not valid!\n");
  }
  return (ground_coeffs_);
}

template <typename PointT> typename open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT>::PointCloudPtr
open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT>::getNoGroundCloud ()
{
  return (no_ground_cloud_);
}

template <typename PointT> float
open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT>::getMeanLuminance ()
{
  return (mean_luminance_);
}

template <typename PointT> void
open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT>::getTiltCompensationTransforms (Eigen::Affine3f& transform, Eigen::Affine3f& anti_transform)
{
  transform = transform_;
  anti_transform = anti_transform_;
}

template <typename PointT> void
open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT>::extractRGBFromPointCloud (PointCloudPtr input_cloud, pcl::PointCloud<pcl::RGB>::Ptr& output_cloud)
{
  // Extract RGB information from a point cloud and output the corresponding RGB point cloud  
  output_cloud->points.resize(input_cloud->height*input_cloud->width);
  output_cloud->width = input_cloud->width;
  output_cloud->height = input_cloud->height;

  pcl::RGB rgb_point;
  for (int j = 0; j < input_cloud->width; j++)
  {
    for (int i = 0; i < input_cloud->height; i++)
    { 
      rgb_point.r = (*input_cloud)(j,i).r;
      rgb_point.g = (*input_cloud)(j,i).g;
      rgb_point.b = (*input_cloud)(j,i).b;    
      (*output_cloud)(j,i) = rgb_point; 
    }
  }
}

template <typename PointT> void
open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT>::swapDimensions (pcl::PointCloud<pcl::RGB>::Ptr& cloud)
{
  pcl::PointCloud<pcl::RGB>::Ptr output_cloud(new pcl::PointCloud<pcl::RGB>);
  output_cloud->points.resize(cloud->height*cloud->width);
  output_cloud->width = cloud->height;
  output_cloud->height = cloud->width;
  for (int i = 0; i < cloud->width; i++)
  {
    for (int j = 0; j < cloud->height; j++)
    {
      (*output_cloud)(j,i) = (*cloud)(cloud->width - i - 1, j);
    }
  }
  cloud = output_cloud;
}

template <typename PointT> typename open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT>::PointCloudPtr
open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT>::rotateCloud(PointCloudPtr cloud, Eigen::Affine3f transform )
{
  PointCloudPtr rotated_cloud (new PointCloud);
  pcl::transformPointCloud(*cloud, *rotated_cloud, transform);
  rotated_cloud->header.frame_id = cloud->header.frame_id;

  return rotated_cloud;

}

template <typename PointT> Eigen::VectorXf
open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT>::rotateGround(Eigen::VectorXf ground_coeffs, Eigen::Affine3f transform){

  Eigen::VectorXf ground_coeffs_new;

  // Create a cloud with three points on the input ground plane:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr dummy (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PointXYZRGB first = pcl::PointXYZRGB(0.0,0.0,0.0);
  first.x = 1.0;
  pcl::PointXYZRGB second = pcl::PointXYZRGB(0.0,0.0,0.0);
  second.y = 1.0;
  pcl::PointXYZRGB third = pcl::PointXYZRGB(0.0,0.0,0.0);
  third.x = 1.0;
  third.y = 1.0;

  dummy->points.push_back( first );
  dummy->points.push_back( second );
  dummy->points.push_back( third );

  for(uint8_t i = 0; i < dummy->points.size(); i++ )
  { // Find z given x and y:
    dummy->points[i].z = (double) ( -ground_coeffs_(3) -(ground_coeffs_(0) * dummy->points[i].x) - (ground_coeffs_(1) * dummy->points[i].y) ) / ground_coeffs_(2);
  }

  // Rotate them:
  dummy = rotateCloud(dummy, transform);

  // Compute new ground coeffs:
  std::vector<int> indices;
  for(unsigned int i = 0; i < dummy->points.size(); i++)
  {
    indices.push_back(i);
  }
  pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> model_plane(dummy);
  model_plane.computeModelCoefficients(indices, ground_coeffs_new);

  return ground_coeffs_new;
}

template <typename PointT> typename open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT>::PointCloudPtr
open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT>::preprocessCloud (PointCloudPtr& input_cloud)
{
  // Downsample of sampling_factor in every dimension:
  PointCloudPtr cloud_downsampled(new PointCloud);
  PointCloudPtr cloud_denoised(new PointCloud);
  if (sampling_factor_ != 1)
  {
    cloud_downsampled->width = (input_cloud->width)/sampling_factor_;
    cloud_downsampled->height = (input_cloud->height)/sampling_factor_;
    cloud_downsampled->points.resize(cloud_downsampled->height*cloud_downsampled->width);
    cloud_downsampled->is_dense = input_cloud->is_dense;
    cloud_downsampled->header = input_cloud->header;
    for (int j = 0; j < cloud_downsampled->width; j++)
    {
      for (int i = 0; i < cloud_downsampled->height; i++)
      {
        (*cloud_downsampled)(j,i) = (*input_cloud)(sampling_factor_*j,sampling_factor_*i);
      }
    }
  }

  if (apply_denoising_)
  {
    // Denoising with statistical filtering:
    pcl::StatisticalOutlierRemoval<PointT> sor;
    if (sampling_factor_ != 1)
      sor.setInputCloud (cloud_downsampled);
    else
      sor.setInputCloud (input_cloud);
    sor.setMeanK (mean_k_denoising_);
    sor.setStddevMulThresh (std_dev_denoising_);
    sor.filter (*cloud_denoised);
  }

  //  // Denoising viewer
  //  int v1(0);
  //  int v2(0);
  //  denoising_viewer_->removeAllPointClouds(v1);
  //  denoising_viewer_->removeAllPointClouds(v2);
  //  denoising_viewer_->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
  //  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(input_cloud);
  //  denoising_viewer_->addPointCloud<PointT> (input_cloud, rgb, "original", v1);
  //  denoising_viewer_->createViewPort (0.5, 0.0, 1.0, 1.0, v2);
  //  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb2(cloud_denoised);
  //  denoising_viewer_->addPointCloud<PointT> (cloud_denoised, rgb2, "denoised", v2);
  //  denoising_viewer_->spinOnce();

  // Voxel grid filtering:
  PointCloudPtr cloud_filtered(new PointCloud);
  pcl::VoxelGrid<PointT> voxel_grid_filter_object;
  if (apply_denoising_)
    voxel_grid_filter_object.setInputCloud(cloud_denoised);
  else
  {
    if (sampling_factor_ != 1)
      voxel_grid_filter_object.setInputCloud(cloud_downsampled);
    else
      voxel_grid_filter_object.setInputCloud(input_cloud);
  }
  voxel_grid_filter_object.setLeafSize (voxel_size_, voxel_size_, voxel_size_);
  voxel_grid_filter_object.setFilterFieldName("z");
  voxel_grid_filter_object.setFilterLimits(0.0, max_distance_);
  voxel_grid_filter_object.filter (*cloud_filtered);

  return cloud_filtered;
}

template <typename PointT> bool
open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT>::compute (std::vector<pcl::people::PersonCluster<PointT> >& clusters)
{
  frame_counter_++;

  // Define if debug info should be written or not for this frame:
  bool debug_flag = false;
  if ((frame_counter_ % 60) == 0)
  {
    debug_flag = true;
  }

  // Check if all mandatory variables have been set:
  if (debug_flag)
  {
    if (sqrt_ground_coeffs_ != sqrt_ground_coeffs_)
    {
      PCL_ERROR ("[open_ptrack::detection::GroundBasedPeopleDetectionApp::compute] Floor parameters have not been set or they are not valid!\n");
      return (false);
    }
    if (cloud_ == NULL)
    {
      PCL_ERROR ("[open_ptrack::detection::GroundBasedPeopleDetectionApp::compute] Input cloud has not been set!\n");
      return (false);
    }
    if (intrinsics_matrix_(0) == 0)
    {
      PCL_ERROR ("[open_ptrack::detection::GroundBasedPeopleDetectionApp::compute] Camera intrinsic parameters have not been set!\n");
      return (false);
    }
    if (!person_classifier_set_flag_)
    {
      PCL_ERROR ("[open_ptrack::detection::GroundBasedPeopleDetectionApp::compute] Person classifier has not been set!\n");
      return (false);
    }
  }

  if (!dimension_limits_set_)    // if dimension limits have not been set by the user
  {
    // Adapt thresholds for clusters points number to the voxel size:
    max_points_ = int(float(max_points_) * std::pow(0.06/voxel_size_, 2));
    if (voxel_size_ > 0.06)
      min_points_ = int(float(min_points_) * std::pow(0.06/voxel_size_, 2));
  }

  // Fill rgb image:
  rgb_image_->points.clear();                            // clear RGB pointcloud
  extractRGBFromPointCloud(cloud_, rgb_image_);          // fill RGB pointcloud

  // Point cloud pre-processing (downsampling and filtering):
  PointCloudPtr cloud_filtered(new PointCloud);
  cloud_filtered = preprocessCloud (cloud_);

  if (use_rgb_)
  {
    // Compute mean luminance:
    int n_points = cloud_filtered->points.size();
    double sumR, sumG, sumB = 0.0;
    for (int j = 0; j < cloud_filtered->width; j++)
    {
      for (int i = 0; i < cloud_filtered->height; i++)
      {
        sumR += (*cloud_filtered)(j,i).r;
        sumG += (*cloud_filtered)(j,i).g;
        sumB += (*cloud_filtered)(j,i).b;
      }
    }
    mean_luminance_ = 0.3 * sumR/n_points + 0.59 * sumG/n_points + 0.11 * sumB/n_points;
    //    mean_luminance_ = 0.2126 * sumR/n_points + 0.7152 * sumG/n_points + 0.0722 * sumB/n_points;
  }

  // Ground removal and update:
  pcl::IndicesPtr inliers(new std::vector<int>);
  boost::shared_ptr<pcl::SampleConsensusModelPlane<PointT> > ground_model(new pcl::SampleConsensusModelPlane<PointT>(cloud_filtered));
  ground_model->selectWithinDistance(ground_coeffs_, voxel_size_, *inliers);
  no_ground_cloud_ = PointCloudPtr (new PointCloud);
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*no_ground_cloud_);
  if (inliers->size () >= (300 * 0.06 / voxel_size_ / std::pow (static_cast<double> (sampling_factor_), 2)))
    ground_model->optimizeModelCoefficients (*inliers, ground_coeffs_, ground_coeffs_);
  else
  {
    if (debug_flag)
    {
      PCL_INFO ("No groundplane update!\n");
    }
  }

  // Background Subtraction (optional):
  if (background_subtraction_)
  {
    PointCloudPtr foreground_cloud(new PointCloud);
    for (unsigned int i = 0; i < no_ground_cloud_->points.size(); i++)
    {
      if (not (background_octree_->isVoxelOccupiedAtPoint(no_ground_cloud_->points[i].x, no_ground_cloud_->points[i].y, no_ground_cloud_->points[i].z)))
      {
        foreground_cloud->points.push_back(no_ground_cloud_->points[i]);
      }
    }
    no_ground_cloud_ = foreground_cloud;
  }

  if (no_ground_cloud_->points.size() > 0)
  {
    // Euclidean Clustering:
    std::vector<pcl::PointIndices> cluster_indices;
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud(no_ground_cloud_);
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(2 * 0.06);
    ec.setMinClusterSize(min_points_);
    ec.setMaxClusterSize(max_points_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(no_ground_cloud_);
    ec.extract(cluster_indices);

    // Sensor tilt compensation to improve people detection:
    PointCloudPtr no_ground_cloud_rotated(new PointCloud);
    Eigen::VectorXf ground_coeffs_new;
    if(sensor_tilt_compensation_)
    {
      // We want to rotate the point cloud so that the ground plane is parallel to the xOz plane of the sensor:
      Eigen::Vector3f input_plane, output_plane;
      input_plane << ground_coeffs_(0), ground_coeffs_(1), ground_coeffs_(2);
      output_plane << 0.0, -1.0, 0.0;

      Eigen::Vector3f axis = input_plane.cross(output_plane);
      float angle = acos( input_plane.dot(output_plane)/ ( input_plane.norm()/output_plane.norm() ) );
      transform_ = Eigen::AngleAxisf(angle, axis);

      // Setting also anti_transform for later
      anti_transform_ = transform_.inverse();
      no_ground_cloud_rotated = rotateCloud(no_ground_cloud_, transform_);
      ground_coeffs_new.resize(4);
      ground_coeffs_new = rotateGround(ground_coeffs_, transform_);
    }
    else
    {
      transform_ = transform_.Identity();
      anti_transform_ = transform_.inverse();
      no_ground_cloud_rotated = no_ground_cloud_;
      ground_coeffs_new = ground_coeffs_;
    }

    // To avoid PCL warning:
    if (cluster_indices.size() == 0)
      cluster_indices.push_back(pcl::PointIndices());

    // Head based sub-clustering //
    pcl::people::HeadBasedSubclustering<PointT> subclustering;
    subclustering.setInputCloud(no_ground_cloud_rotated);
    subclustering.setGround(ground_coeffs_new);
    subclustering.setInitialClusters(cluster_indices);
    subclustering.setHeightLimits(min_height_, max_height_);
    subclustering.setMinimumDistanceBetweenHeads(heads_minimum_distance_);
    subclustering.setSensorPortraitOrientation(vertical_);
    subclustering.subcluster(clusters);

//    for (unsigned int i = 0; i < rgb_image_->points.size(); i++)
//    {
//      if ((rgb_image_->points[i].r < 0) | (rgb_image_->points[i].r > 255) | isnan(rgb_image_->points[i].r))
//      {
//        std::cout << "ERROR in RGB data!" << std::endl;
//      }
//    }

    if (use_rgb_) // if RGB information can be used
    {
      // Person confidence evaluation with HOG+SVM:
      if (vertical_)  // Rotate the image if the camera is vertical
      {
        swapDimensions(rgb_image_);
      }
      for(typename std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
      {
        //Evaluate confidence for the current PersonCluster:
        Eigen::Vector3f centroid = intrinsics_matrix_ * (anti_transform_ * it->getTCenter());
        centroid /= centroid(2);
        Eigen::Vector3f top = intrinsics_matrix_ * (anti_transform_ * it->getTTop());
        top /= top(2);
        Eigen::Vector3f bottom = intrinsics_matrix_ * (anti_transform_ * it->getTBottom());
        bottom /= bottom(2);

        it->setPersonConfidence(person_classifier_.evaluate(rgb_image_, bottom, top, centroid, vertical_));
      }
    }
    else
    {
      for(typename std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
      {
        it->setPersonConfidence(-100.0);
      }
    }
  }

  return (true);
}

template <typename PointT>
open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT>::~GroundBasedPeopleDetectionApp ()
{
  // TODO Auto-generated destructor stub
}
#endif /* OPEN_PTRACK_DETECTION_GROUND_BASED_PEOPLE_DETECTION_APP_HPP_ */
