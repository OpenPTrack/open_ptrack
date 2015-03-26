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

#include <open_ptrack/opt_calibration/trajectory_registration.h>

namespace open_ptrack
{
  namespace opt_calibration
  {

    TrajectoryRegistration::TrajectoryRegistration ()
    {

    }

    TrajectoryRegistration::~TrajectoryRegistration ()
    {

    }

    void
    TrajectoryRegistration::setTimeBinSize (double time_bin_size)
    {
      time_bin_size_ = time_bin_size;
    }

    void
    TrajectoryRegistration::setICPMaxCorrespondenceDistance (double icp_max_correspondence_distance)
    {
      icp_max_correspondence_distance_ = icp_max_correspondence_distance;
    }

    void
    TrajectoryRegistration::setTimestamps (std::vector<std::vector<double> > timestamp_vector)
    {
      timestamp_vector_ = timestamp_vector;
    }

    void
    TrajectoryRegistration::setColormap (std::map<std::string, int> color_map)
    {
      color_map_ = color_map;
    }

    void
    TrajectoryRegistration::setNIterations (unsigned int N_iter)
    {
      N_iter_ = N_iter;
    }

    void
    TrajectoryRegistration::computeTimeRange ()
    {
      start_time_ = std::numeric_limits<float>::max();
      end_time_ = 0.0;

      for (unsigned int i = 0; i < timestamp_vector_.size(); i++)
      {
        start_time_ = std::min(start_time_, timestamp_vector_[i][0]);
        end_time_ = std::max(end_time_, timestamp_vector_[i][timestamp_vector_[i].size()-1]);
      }
      //  std::cout << "start_time: " << start_time_ << std::endl;
      //  std::cout << "end_time: " << end_time_ << std::endl;
    }

    void
    TrajectoryRegistration::createTimeBasedClouds (std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_vector, double time_scale_factor,
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cloud_with_time_vector)
    {
      pcl::PointXYZRGB point;
      for (unsigned int i = 0; i < cloud_vector.size(); i++)
      {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        if (cloud_vector[i]->points.size() > 0)
        {
          point.r = cloud_vector[i]->points[0].r;
          point.g = cloud_vector[i]->points[0].g;
          point.b = cloud_vector[i]->points[0].b;
          for (unsigned int j = 0; j < cloud_vector[i]->points.size(); j++)
          {
            point.x = cloud_vector[i]->points[j].x;
            point.y = cloud_vector[i]->points[j].y;
            point.z = timestamp_vector_[i][j] / time_scale_factor;

            current_cloud->push_back (point);
          }
        }
        cloud_with_time_vector.push_back (current_cloud);
      }
    }

    void
    TrajectoryRegistration::alignAllCloudsToOne (std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr >& input_cloud_vector,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr& target_cloud, double max_correspondence_distance,
        double ransac_outlier_rejection_threshold, std::map<std::string, Eigen::Matrix4d>& registration_matrices,
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr >& registered_cloud_vector)
    {
      pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
      icp.setMaxCorrespondenceDistance(max_correspondence_distance);
      icp.setUseReciprocalCorrespondences(true);
      icp.setMaximumIterations(1000);
      icp.setRANSACOutlierRejectionThreshold(ransac_outlier_rejection_threshold);
      for(std::map<std::string, int>::iterator colormap_iterator = color_map_.begin(); colormap_iterator != color_map_.end(); colormap_iterator++)
      {
        std::cout << "Aligning " << colormap_iterator->first << "..." << std::endl;
        icp.setInputTarget(target_cloud);
        icp.setInputSource (input_cloud_vector[colormap_iterator->second]);
        icp.align (*registered_cloud_vector[colormap_iterator->second]);
        std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
        registration_matrices[colormap_iterator->first] = icp.getFinalTransformation().cast<double>() * registration_matrices[colormap_iterator->first];
        std::cout << registration_matrices[colormap_iterator->first] << std::endl << std::endl;
      }
    }

    void
    TrajectoryRegistration::refineCalibration (std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cloud_vector,
        std::map<std::string, Eigen::Matrix4d>& registration_matrices, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& average_cloud)
    {
      // Organize points into bins:
      int n_bins = int ((end_time_ - start_time_) / time_bin_size_);
      std::vector<pcl::PointCloud<pcl::PointXYZ> > clouds_voxelized;
      pcl::PointCloud<pcl::PointXYZ> empty_cloud;
      clouds_voxelized.resize(n_bins+1, empty_cloud);

      for (unsigned int i = 0; i < cloud_vector.size(); i++)
      {
        for (unsigned int j = 0; j < cloud_vector[i]->points.size(); j++)
        {
          int current_bin = int((timestamp_vector_[i][j] - start_time_) / time_bin_size_);
          clouds_voxelized[current_bin].push_back(pcl::PointXYZ(cloud_vector[i]->points[j].x, cloud_vector[i]->points[j].y, cloud_vector[i]->points[j].z));
        }
      }

      // Compute average point cloud:
      pcl::PointXYZRGB colored_centroid;
      colored_centroid.r = 255;
      colored_centroid.g = 255;
      colored_centroid.b = 255;
      average_cloud->points.clear();
      for (unsigned int i = 0; i < clouds_voxelized.size(); i++)
      {
        if (clouds_voxelized[i].size() > 1)
        {
          Eigen::Vector4f centroid;
          pcl::compute3DCentroid (clouds_voxelized[i], centroid);
          colored_centroid.x = centroid(0);
          colored_centroid.y = centroid(1);
          colored_centroid.z = centroid(2);
          average_cloud->points.push_back(colored_centroid);
        }
      }

      // Compute ICP registration of all clouds to the average cloud:
      std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > registered_cloud_vector;
      for (unsigned int i = 0; i < cloud_vector.size(); i++)
      {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        registered_cloud_vector.push_back(empty_cloud_ptr);
      }
      alignAllCloudsToOne (cloud_vector, average_cloud, icp_max_correspondence_distance_, icp_max_correspondence_distance_,
          registration_matrices, registered_cloud_vector);

      // Fill cloud_vector with registered clouds:
      for (unsigned int i = 0; i < cloud_vector.size(); i++)
      {
        *cloud_vector[i] = *registered_cloud_vector[i];
      }

//      // Visualization:
//      pcl::visualization::PCLVisualizer detection_registered_viewer("After registration");
//      for(std::map<std::string, int>::iterator colormap_iterator = color_map_.begin(); colormap_iterator != color_map_.end(); colormap_iterator++)
//      {
//        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb((registered_cloud_vector[colormap_iterator->second]));
//        detection_registered_viewer.addPointCloud<pcl::PointXYZRGB> (registered_cloud_vector[colormap_iterator->second], rgb, colormap_iterator->first);
//        detection_registered_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, colormap_iterator->first);
//        detection_registered_viewer.spinOnce();
//      }
//
//      detection_registered_viewer.addPointCloud<pcl::PointXYZRGB> (average_cloud, "average_cloud");
//      detection_registered_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "average_cloud");
//      detection_registered_viewer.spinOnce();
//
//      detection_registered_viewer.spin();
    }

    void
    TrajectoryRegistration::projectAverageCloudToTrackingPlane (pcl::PointCloud<pcl::PointXYZRGB>::Ptr average_cloud,
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_vector, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& average_cloud_projected)
    {
      // Create cloud containing all detections from all sensors:
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr sum_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      for (unsigned int i = 0; i < cloud_vector.size(); i++)
      {
        *sum_cloud = *sum_cloud + *cloud_vector[i];
      }

      // Compute plane containing registered detections:
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      pcl::SACSegmentation<pcl::PointXYZRGB> seg;
      seg.setOptimizeCoefficients (true);
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (0.2);
      seg.setInputCloud (sum_cloud);
      seg.segment (*inliers, *coefficients);
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      extract.setInputCloud (sum_cloud);
      extract.setIndices (inliers);
      extract.setNegative (false);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr inliers_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      extract.filter (*inliers_cloud);

      // Compute projection of the average cloud to the detection plane:
      pcl::PointXYZRGB point;
      point.r = 255;
      point.g = 255;
      point.b = 255;
      for (unsigned int i = 0; i < average_cloud->points.size(); i++)
      {
        point.x = average_cloud->points[i].x;
        point.y = average_cloud->points[i].y;
        point.z = -(coefficients->values[0]*point.x + coefficients->values[1]*point.y + coefficients->values[3])/coefficients->values[2];  // z = -(ax+by+d)/c
        average_cloud_projected->points.push_back(point);
      }
    }

    void
    TrajectoryRegistration::visualizeClouds (std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > cloud_vector,
        std::string viewer_name, bool spin_flag)
    {
      pcl::visualization::PCLVisualizer viewer (viewer_name);
      viewer.setCameraPosition(4.85038, 0.777564, 23.63, 0.0101831, 0.998897, -0.0458376);
      for(std::map<std::string, int>::iterator colormap_iterator = color_map_.begin(); colormap_iterator != color_map_.end(); colormap_iterator++)
      {
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_vector[colormap_iterator->second]);
        viewer.addPointCloud<pcl::PointXYZRGB> (cloud_vector[colormap_iterator->second], rgb, colormap_iterator->first);
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, colormap_iterator->first);
        viewer.spinOnce();
      }
      if (spin_flag)
        viewer.spin();
    }

    void
    TrajectoryRegistration::visualizeFinalRegistration (std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr >& cloud_vector,
        std::map<std::string, Eigen::Matrix4d>& registration_matrices, pcl::visualization::PCLVisualizer& viewer_name)
    {
      std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > registered_cloud_vector;

      // Input-output transformation test and visualization:
      for(std::map<std::string, int>::iterator colormap_iterator = color_map_.begin(); colormap_iterator != color_map_.end(); colormap_iterator++)
      {
        // Registration matrix:
        std::map<std::string, Eigen::Matrix4d>::iterator registration_matrices_iterator = registration_matrices.find(colormap_iterator->first);
        if (registration_matrices_iterator == registration_matrices.end())
        {
          std::cout << "ERROR! Frame_id not found!" << std::endl;
        }
        else
        {
//          std::cout << registration_matrices_iterator->first << std::endl;
          Eigen::Matrix4d registration_matrix = registration_matrices_iterator->second;

          pcl::PointCloud<pcl::PointXYZRGB>::Ptr registered_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
          pcl::transformPointCloud (*cloud_vector[colormap_iterator->second], *registered_cloud, registration_matrix);
          registered_cloud_vector.push_back (registered_cloud);
        }
      }

      int v1(0);
      viewer_name.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
      for(std::map<std::string, int>::iterator colormap_iterator = color_map_.begin(); colormap_iterator != color_map_.end(); colormap_iterator++)
      {
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb((cloud_vector[colormap_iterator->second]));
        viewer_name.addPointCloud<pcl::PointXYZRGB> (cloud_vector[colormap_iterator->second], rgb, colormap_iterator->first, v1);
        viewer_name.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, colormap_iterator->first, v1);
        viewer_name.spinOnce();
      }
      viewer_name.addText ("BEFORE CALIBRATION REFINEMENT", 10, 10, "v1 text", v1);

      int v2(0);
      viewer_name.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
      for(std::map<std::string, int>::iterator colormap_iterator = color_map_.begin(); colormap_iterator != color_map_.end(); colormap_iterator++)
      {
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb((registered_cloud_vector[colormap_iterator->second]));
        viewer_name.addPointCloud<pcl::PointXYZRGB> (registered_cloud_vector[colormap_iterator->second], rgb, colormap_iterator->first + "_registered", v2);
        viewer_name.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, colormap_iterator->first + "_registered", v2);
        viewer_name.spinOnce();
      }
      viewer_name.addText ("AFTER CALIBRATION REFINEMENT", 10, 10, "v2 text", v2);

      viewer_name.setCameraPosition(-2.40825,-0.54434,30.8287, 0.00352108,0.999943,0.0100698, v1);
      viewer_name.setCameraPosition(-2.40825,-0.54434,30.8287, 0.00352108,0.999943,0.0100698, v2);
    }

    void
    TrajectoryRegistration::compute (std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr >& cloud_vector, double time_scale_factor,
                std::map<std::string, Eigen::Matrix4d>& registration_matrices,
                std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr >& registered_cloud_vector)
    {
      // Compute time range:
      computeTimeRange ();

      // Create point clouds with time as Z dimension:
      std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_with_time_vector;
      createTimeBasedClouds (cloud_vector, time_scale_factor, cloud_with_time_vector);

//      // Visualization of detection trajectories with time extension:
//      visualizeClouds (cloud_with_time_vector, "Detection viewer", true);

      // Initialize registration matrices:
      for(std::map<std::string, int>::iterator colormap_iterator = color_map_.begin(); colormap_iterator != color_map_.end(); colormap_iterator++)
      {
        registration_matrices.insert (std::pair<std::string, Eigen::Matrix4d> (colormap_iterator->first, Eigen::Matrix4d::Identity()));
      }

      // Refinement procedure:
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr average_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      double temp_max_correspondence_distance = icp_max_correspondence_distance_;
      for (unsigned int i = 0; i < N_iter_; i++)
      {
        std::cout << "******************************" << std::endl;
        std::cout << "*** ALIGNMENT: ITERATION " << i+1 << " ***" << std::endl;
        std::cout << "******************************" << std::endl;

        refineCalibration (cloud_with_time_vector, registration_matrices, average_cloud);
        icp_max_correspondence_distance_ = icp_max_correspondence_distance_ / 2;
      }
      icp_max_correspondence_distance_ = temp_max_correspondence_distance;

      // Transform original XYZ point clouds with obtained registration matrices:
      for(std::map<std::string, int>::iterator colormap_iterator = color_map_.begin(); colormap_iterator != color_map_.end(); colormap_iterator++)
      {
        // Registration matrix:
        std::map<std::string, Eigen::Matrix4d>::iterator registration_matrices_iterator = registration_matrices.find(colormap_iterator->first);
        Eigen::Matrix4d registration_matrix = registration_matrices_iterator->second;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr registered_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::transformPointCloud (*cloud_vector[colormap_iterator->second], *registered_cloud, registration_matrix);
        *registered_cloud_vector[colormap_iterator->second] = *registered_cloud;
      }

      // Project average_cloud to detection plane:
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr average_cloud_projected(new pcl::PointCloud<pcl::PointXYZRGB>);
      projectAverageCloudToTrackingPlane (average_cloud, cloud_vector, average_cloud_projected);

//      // Visualization of clouds in registered_cloud_vector:
//      visualizeClouds (registered_cloud_vector, "XYZ after registration with time dimension", true);
//      //  detection_registered_viewer.addPointCloud<pcl::PointXYZRGB> (average_cloud_projected, "average_cloud_projected");
//      //  detection_registered_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "average_cloud_projected");
//      //  detection_registered_viewer.spinOnce();

      // Fill intermerdiate_cloud_vector with registered clouds:
      std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > intermediate_cloud_vector;
      for (unsigned int i = 0; i < cloud_vector.size(); i++)
      {
        intermediate_cloud_vector.push_back(pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>));
      }
      for (unsigned int i = 0; i < intermediate_cloud_vector.size(); i++)
      {
        *intermediate_cloud_vector[i] = *registered_cloud_vector[i];
      }

      // Register clouds in intermediate_cloud_vector to average_cloud_projected:
      std::cout << "***********************" << std::endl;
      std::cout << "*** FINAL ALIGNMENT ***" << std::endl;
      std::cout << "***********************" << std::endl;
      alignAllCloudsToOne (intermediate_cloud_vector, average_cloud_projected, 0.3, icp_max_correspondence_distance_,
          registration_matrices, registered_cloud_vector);
    }

  } /* namespace opt_calibration */
} /* namespace open_ptrack */
