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
 * Author: Matteo Munaro [matteo.munaro@dei.unipd.it], Nicola Ristè
 *
 */

#ifndef OPEN_PTRACK_DETECTION_GROUND_SEGMENTATION_H_
#define OPEN_PTRACK_DETECTION_GROUND_SEGMENTATION_H_

#include <Eigen/Eigen>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/filters/extract_indices.h>
#include <tf/transform_listener.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<iostream>
#include<fstream>

namespace open_ptrack
{
  namespace detection
  {
    template <typename PointT> class GroundplaneEstimation;

    template <typename PointT>

    /** \brief GroundplaneEstimation estimates the ground plane equation from a 3D point cloud */
    class GroundplaneEstimation
    {

      public:

        typedef pcl::PointCloud<PointT> PointCloud;
        typedef boost::shared_ptr<PointCloud> PointCloudPtr;
        typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

        /** \brief Constructor. */
        GroundplaneEstimation (int ground_estimation_mode);

        /** \brief Destructor. */
        virtual ~GroundplaneEstimation ();

        /**
         * \brief Set the pointer to the input cloud.
         *
         * \param[in] cloud A pointer to the input cloud.
         */
        void
        setInputCloud (PointCloudPtr& cloud);

        /**
         * \brief Return true if the cloud has ratio of NaN over total number of points greater than "max_ratio".
         *
         * \param[in] cloud The input cloud.
         * \param[in] max_ratio The ratio of invalid points over which a cloud is considered as not valid.
         *
         * \return true if the cloud has ratio of NaN over total number of points greater than "max_ratio".
         */
        bool
        tooManyNaN (PointCloudConstPtr cloud, float max_ratio);

        /**
         * \brief Return true if the percentage of points with confidence below the confidence_threshold is greater than max_ratio.
         *
         * \param[in] confidence_image Image with confidence values for every pixel.
         * \param[in] confidence_threshold Threshold on the confidence to consider a point as valid.
         * \param[in] max_ratio The ratio of invalid points over which a cloud is considered as not valid.
         *
         * \return true if the cloud has ratio of NaN over total number of points greater than "max_ratio".
         */
        bool
        tooManyLowConfidencePoints (cv::Mat& confidence_image, int confidence_threshold, double max_ratio);

        /**
         * \brief Compute the ground plane coefficients from the transform between two reference frames.
         *
         * \param[in] camera_frame Camera frame id.
         * \param[in] world_frame Ground frame id.
         *
         * \return Vector of ground plane coefficients.
         */
        Eigen::VectorXf
        computeFromTF (std::string camera_frame, std::string ground_frame);

        /**
         * \brief Compute the ground plane coefficients from the transform between two reference frames.
         *
         * \param[in] worldToCamTransform ROS transform between world frame and camera frame.
         *
         * \return Vector of ground plane coefficients.
         */
        Eigen::VectorXf
        computeFromTF (tf::Transform worldToCamTransform);

        /**
         * \brief Read the world to camera transform from file.
         *
         * \param[in] filename Filename listing camera poses for each camera.
         * \param[in] camera_name Name of the camera for which the pose should be read.
         *
         * \return The world to camera transform.
         */
        tf::Transform
        readTFFromFile (std::string filename, std::string camera_name);

        /**
         * \brief Compute the ground plane coefficients.
         *
         * \return Vector of ground plane coefficients.
         */
        Eigen::VectorXf
        compute ();

        /**
         * \brief Compute the ground plane coefficients with the procedure used in multi-camera systems.
         * \param[in] ground_from_extrinsic_calibration If true, exploit extrinsic calibration for estimatin the ground plane equation.
         * \param[in] read_ground_from_file Flag stating if the ground should be read from file, if present.
         * \param[in] pointcloud_topic Topic containing the point cloud.
         * \param[in] sampling_factor Scale factor used to downsample the point cloud.
         *
         * \return Vector of ground plane coefficients.
         */
        Eigen::VectorXf
        computeMulticamera (bool ground_from_extrinsic_calibration, bool read_ground_from_file, std::string pointcloud_topic,
            int sampling_factor, float voxel_size);

        /**
         * \brief Refine ground coefficients by iterating ground plane detection on the input cloud
         *
         * \param[in] cloud Input cloud.
         * \param[in] num_iter Number of iterations.
         * \param[in] inliers_threshold Distance threshold for selecting inliers.
         * \param[in/out] Ground coefficients.
         *
         * return true if ground coefficients have been updated, false otherwise.
         */
        bool
        refineGround (int num_iter, float voxel_size, float inliers_threshold, Eigen::VectorXf& ground_coeffs_calib);

      private:

        /**
         * \brief Callback listening to point clicking on PCL visualizer.
         *
         */
        static void
        pp_callback (const pcl::visualization::PointPickingEvent& event, void* args);

        /**
         * \brief States which planar region is lower.
         *
         * \param[in] region1 First planar region.
         * \param[in] region2 Second planar region.
         *
         * \return true if region2 is lower than region1.
         */
        static bool
        planeHeightComparator (pcl::PlanarRegion<PointT> region1, pcl::PlanarRegion<PointT> region2);

        /**
         * \brief Color the planar regions with different colors or the index-th planar region in red.
         *
         * \param[in] regions Vector of planar regions.
         * \param[in] index If set and > 0, specify the index of the region to be colored in red. If not set, all regions are colored with different colors.
         *
         * \return The colored point cloud.
         */
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr
        colorRegions (std::vector<pcl::PlanarRegion<PointT>,
            Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions, int index = -1);

      protected:
        /** \brief Flag stating if ground should be estimated manually (0), semi-automatically (1) or automatically with validation (2) or fully automatically (3) */
        int ground_estimation_mode_;

        /** \brief pointer to the input cloud */
        PointCloudPtr cloud_;

        /** \brief structure used to pass arguments to the callback function */
        struct callback_args{
            pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d;
            pcl::visualization::PCLVisualizer* viewerPtr;
        };

        /** \brief structure used to pass arguments to the callback function */
        struct callback_args_color{
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr clicked_points_3d;
            pcl::visualization::PCLVisualizer* viewerPtr;
        };
    };
  } /* namespace detection */
} /* namespace open_ptrack */
#include <open_ptrack/detection/impl/ground_segmentation.hpp>
#endif /* OPEN_PTRACK_DETECTION_GROUND_SEGMENTATION_H_ */
