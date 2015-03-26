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

#ifndef OPEN_PTRACK_OPT_CALIBRATION_TRAJECTORY_REGISTRATION_H_
#define OPEN_PTRACK_OPT_CALIBRATION_TRAJECTORY_REGISTRATION_H_

#include <iostream>
#include <fstream>
#include <string.h>
#include <Eigen/Eigen>
#include <limits>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/centroid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

namespace open_ptrack
{
  namespace opt_calibration
  {
    class TrajectoryRegistration;

    /** \brief TrajectoryRegistration provides functions for registering point clouds containing people trajectory */
    class TrajectoryRegistration
    {

      public:

        /** \brief Constructor. */
        TrajectoryRegistration ();

        /** \brief Destructor. */
        virtual ~TrajectoryRegistration ();

        /**
         * \brief Set the bin size for the time variable.
         *
         * \param[in] time_bin_size Bin size for the time variable.
         */
        void
        setTimeBinSize (double time_bin_size);

        /**
         * \brief Set the max correspondence distance for ICP.
         *
         * \param[in] icp_max_correspondence_distance Max correspondence distance for ICP.
         */
        void
        setICPMaxCorrespondenceDistance (double icp_max_correspondence_distance);

        /**
         * \brief Set the vector containing detection timestamps.
         *
         * \param[in] timestamp_vector The vector containing detection timestamps.
         */
        void
        setTimestamps (std::vector<std::vector<double> > timestamp_vector);

        /**
         * \brief Set the map between camera frame_id and color.
         *
         * \param[in] color_map Map between camera frame_id and color.
         */
        void
        setColormap (std::map<std::string, int> color_map);

        /**
         * \brief Set the number of iterations of the calibration refinement procedure.
         *
         * \param[in] N_iter Number of iterations of the calibration refinement procedure.
         */
        void
        setNIterations (unsigned int N_iter);

        /**
         * \brief Compute time range between first and last data.
         *
         * \param[in] timestamp_vector Vector containing the timestamps of the original clouds.
         * \param[in] start_time Trajectory start time.
         * \param[out] end_time Trajectory end time.
         */
        void
        computeTimeRange ();

        /**
         * \brief Create point clouds substituting the z coordinate with the time variable.
         *
         * \param[in] cloud_vector Vector containing the original point clouds.
         * \param[in] timestamp_vector Vector containing the timestamps of the original clouds.
         * \param[in] time_scale_factor Scale factor used to weight time in z coordinate.
         * \param[out] cloud_with_time_vector Vector containing output point clouds.
         */
        void
        createTimeBasedClouds (std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_vector,
            double time_scale_factor, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cloud_with_time_vector);

        /**
         * \brief Align all clouds to a reference one.
         *
         * \param[in] input_cloud_vector Vector containing the point clouds to align.
         * \param[in] target_cloud The target point cloud to use as a reference.
         * \param[in] max_correspondence_distance ICP maximum correspondence distance.
         * \param[in] ransac_outlier_rejection_threshold ICP RANSAC outlier rejection threshold.
         * \param[in/out] registration_matrices Vector containing the registration matrices.
         * \param[out] registered_cloud_vector Vector containing the output (registered) point clouds.
         */
        void
        alignAllCloudsToOne (std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr >& input_cloud_vector,
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr& target_cloud, double max_correspondence_distance,
            double ransac_outlier_rejection_threshold, std::map<std::string, Eigen::Matrix4d>& registration_matrices,
            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr >& registered_cloud_vector);

        /**
         * \brief Refine camera extrinsic calibration by aligning people detection trajectories.
         *
         * \param[in] cloud_vector Vector containing input point clouds.
         * \param[in/out] registration_matrices Vector containing the registration matrices.
         * \param[out] average_cloud Average point cloud computed to align every cloud to this one.
         */
        void
        refineCalibration (std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cloud_vector,
            std::map<std::string, Eigen::Matrix4d>& registration_matrices, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& average_cloud);

        /**
         * \brief  Project average_cloud to detection plane.
         *
         * \param[in] average_cloud Average point cloud computed to align every cloud to this one.
         * \param[in] cloud_vector Vector containing input point clouds.
         * \param[out] average_cloud_projected Average point cloud projected to the detection plane.
         */
        void
        projectAverageCloudToTrackingPlane (pcl::PointCloud<pcl::PointXYZRGB>::Ptr average_cloud,
            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_vector, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& average_cloud_projected);

        /**
         * \brief Visualize the clouds contained in cloud_vector.
         *
         * \param[in] cloud_vector Vector containing the points clouds to visualize.
         * \param[in] color_map Map between the clouds name and an integer index.
         * \param[in] viewer_name Viewer name.
         * \param[in] spin_flag If true, viewer.spin() is called after visualization.
         */
        void
        visualizeClouds (std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > cloud_vector,
            std::string viewer_name, bool spin_flag);

        /**
         * \brief Visualize input and registered clouds in two separate viewports.
         *
         * \param[in] cloud_vector Vector containing the points clouds before registration.
         * \param[in] registration_matrices Map containing the final transformations obtained with the refinement.
         * \param[in] viewer_name Viewer name.
         */
        void
        visualizeFinalRegistration (std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr >& cloud_vector,
            std::map<std::string, Eigen::Matrix4d>& registration_matrices, pcl::visualization::PCLVisualizer& viewer_name);

        /**
         * \brief Perform all computations of the trajectory registration method.
         * \param[in] cloud_vector Vector containing input point clouds.
         * \param[in] time_scale_factor Scale factor for the time dimension.
         * \param[in/out] registration_matrices Vector containing the registration matrices.
         * \param[out] registered_cloud_vector Vector containing the output / registered point clouds.
         */
        void
        compute (std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr >& cloud_vector, double time_scale_factor,
            std::map<std::string, Eigen::Matrix4d>& registration_matrices,
            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr >& registered_cloud_vector);

      private:

        /** \brief Start time for considering detections */
        double start_time_;

        /** \brief End time for considering detections */
        double end_time_;

        /** \brief Bin size for the time variable */
        double time_bin_size_;

        /** \brief Number of iterations of the calibration refinement procedure */
        unsigned int N_iter_;

        /** \brief Max correspondence distance for ICP */
        double icp_max_correspondence_distance_;
        
        /** \brief Vector containing detection timestamps */
        std::vector<std::vector<double> > timestamp_vector_;

        /** \brief Map between camera frame_id and color */
        std::map<std::string, int> color_map_;

    };
  } /* namespace opt_calibration */
} /* namespace open_ptrack */
#endif /* OPEN_PTRACK_OPT_CALIBRATION_TRAJECTORY_REGISTRATION_H_ */
