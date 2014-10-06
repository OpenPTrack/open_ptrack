/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011-2012, Matteo Munaro [matteo.munaro@dei.unipd.it], Filippo Basso [filippo.basso@dei.unipd.it]
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
 * Author: Matteo Munaro [matteo.munaro@dei.unipd.it], Filippo Basso [filippo.basso@dei.unipd.it]
 *
 */

#ifndef OPEN_PTRACK_TRACKING_TRACK_H_
#define OPEN_PTRACK_TRACKING_TRACK_H_

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <cmath>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <open_ptrack/opt_utils/conversions.h>
#include <open_ptrack/tracking/kalman_filter.h>
#include <open_ptrack/bayes/bayesFlt.hpp>
#include <open_ptrack/detection/detection_source.h>
#include <opt_msgs/Track.h>

namespace open_ptrack
{
  namespace tracking
  {
    /** \brief Track represents information about a track (or target) */
    class Track
    {
      public:

        /** \brief A track has Status NEW if it has been recently created, otherwise it has NORMAL Status */
        enum Status
        {
          NEW,
          NORMAL
        };

        /** \brief Visibility states if the track is currently visible by the sensor or partially occluded or totally occluded */
        enum Visibility
        {
          VISIBLE = 0,      // No occlusion
          OCCLUDED = 1,     // Partially occlusion
          NOT_VISIBLE = 2   // Total occlusion
        };

      protected:

        /** \brief Dimension of a circular buffer which keep tracks of filter parameters along time */
        int MAX_SIZE;

        /** \brief Track ID */
        const int id_;

        /** \brief Track frame id (frame id of the last detection associated to the track */
        const std::string frame_id_;

        /** \brief Inverse of the frame rate */
        const double period_;

        /** \brief If true, the track is validated, meaning that it has been associated with a certain number of high confidence detections */
        bool validated_;

        /** \brief Kalman filter associated to the track */
        open_ptrack::tracking::KalmanFilter* filter_;

        /** \brief Temporary copy of the Kalman filter associated to the track (used for recovery filter information when a track is re-found) */
        open_ptrack::tracking::KalmanFilter* tmp_filter_;

        /** \brief First time a detection is associated to the track */
        ros::Time first_time_detected_;

        /** \brief Last time a detection is associated to the track */
        ros::Time last_time_detected_;

        /** \brief Last time a detection with high detection confidence is associated to the track */
        ros::Time last_time_detected_with_high_confidence_;

        /** \brief Last time a prediction has been performed for the track */
        ros::Time last_time_predicted_;

        /** \brief Index in the circular buffer corresponding to the last time a prediction has been performed */
        int last_time_predicted_index_;

        /** Variables used for computing the detection/track Mahalanobis distance */
        std::vector<MahalanobisParameters2d> mahalanobis_map2d_;

        /** Variables used for computing the detection/track Mahalanobis distance */
        std::vector<MahalanobisParameters4d> mahalanobis_map4d_;

        /** \brief Track Status*/
        Status status_;

        /** \brief Track Visibility */
        Visibility visibility_;

        /** \brief Number of high confidence detections associated to the track */
        int updates_with_enough_confidence_;

        /** \brief Track centroid z coordinate */
        double z_;

        /** \brief Track height */
        double height_;

        /** \brief Track distance from the camera */
        double distance_;

        /** \brief Confidence of the last detection associated to the track */
        double last_detector_confidence_;

        /** \brief Color associated to the track */
        Eigen::Vector3f color_;

        /** \brief DetectionSource which provided the last detection associated to the track */
        open_ptrack::detection::DetectionSource* detection_source_;

        /** \brief If true, both position and velocity are considered in computing detection<->track Mahalanobis distance */
        bool velocity_in_motion_term_;

        /** \brief Count the number of consecutive updates with low confidence detections */
        int low_confidence_consecutive_frames_;

      public:

        /** \brief Constructor. */
        Track(
            int id,
            std::string frame_id,
            double position_variance,
            double acceleration_variance,
            double period,
            bool velocity_in_motion_term);

        /** \brief Destructor. */
        virtual ~Track();

        /** \brief Track initialization with an old track. */
        void
        init(const Track& old_track);

        /**
         * \brief Track initialization.
         *
         * \param[in] x Track centroid x coordinate
         * \param[in] y Track centroid y coordinate
         * \param[in] z Track centroid z coordinate
         * \param[in] height Track height
         * \param[in] distance Track distance from the sensor
         * \param[in] detection_source DetectionSource which provided the last detection associated to the track
         */
        void
        init(
            double x,
            double y,
            double z,
            double height,
            double distance,
            open_ptrack::detection::DetectionSource* detection_source);

        /**
         * \brief Update track with new detection information.
         *
         * \param[in] x Detection centroid x coordinate
         * \param[in] y Detection centroid y coordinate
         * \param[in] z Detection centroid z coordinate
         * \param[in] height Detection height
         * \param[in] distance Detection distance from the sensor
         * \param[in] confidence Detection confidence
         * \param[in] min_confidence Minimum confidence for track initialization
         * \param[in] min_confidence_detections Minimum confidence for detection
         * \param[in] detection_source DetectionSource which provided the detection
         */
        void
        update(
            double x,
            double y,
            double z,
            double height,
            double distance,
            double confidence,
            double min_confidence,
            double min_confidence_detections,
            open_ptrack::detection::DetectionSource* detection_source,
            bool first_update = false);

        /**
         * \brief Compute Mahalanobis distance between detection with position (x,y) and track.
         *
         * \param[in] x Detection centroid x coordinate.
         * \param[in] y Detection centroid y coordinate.
         * \param[in] when Time instant.
         *
         * \return the Mahalanobis distance.
         */
        double
        getMahalanobisDistance(double x, double y, const ros::Time& when);

        /* Validate a track */
        void
        validate();

        /**
         * \brief Get track validation flag
         *
         * \return true if the track has been validated, false otherwise.
         */
        bool
        isValidated();

        /**
         * \brief Get track ID
         *
         * \return track ID
         */
        int
        getId();

        /**
         * \brief Set track status to s
         *
         * \param[in] s status
         */
        void
        setStatus(Status s);

        /**
         * \brief Get track status
         *
         * \return track status
         */
        Status
        getStatus();

        /**
         * \brief Set track Visibility.
         *
         * \param[in] v Visibility status.
         */
        void
        setVisibility(Visibility v);

        /**
         * \brief Get track Visibility.
         *
         * \return track Visibility.
         */
        Visibility
        getVisibility();

        /**
         * \brief Get time passed from first detection-track association.
         *
         * \return time passed from first detection-track association.
         */
        float
        getSecFromFirstDetection(ros::Time current_time);

        /**
         * \brief Get time passed from last detection-track association.
         *
         * \return time passed from last detection-track association.
         */
        float
        getSecFromLastDetection(ros::Time current_time);

        /**
         * \brief Get time passed from last detection-track association with a high confidence detection.
         *
         * \return time passed from last detection-track association with a high confidence detection.
         */
        float
        getSecFromLastHighConfidenceDetection(ros::Time current_time);

        /**
         * \brief Get the number of consecutive updates with low confidence detections.
         *
         * \return the number of consecutive updates with low confidence detections.
         */
        float
        getLowConfidenceConsecutiveFrames();

        /**
         * \brief Get the number of updates with enough confidence detections.
         *
         * \return the number of updates with enough confidence detections.
         */
        int
        getUpdatesWithEnoughConfidence();

        /**
         * \brief Draw track bounding box in the image.
         *
         * \param[in] vertical States if the camera is vertically oriented (true) or not (false).
         */
        void
        draw(bool vertical);

        /**
         * \brief Create RViz visualization marker with the track position.
         *
         * \param[in/out] msg Array containing markers of every track.
         */
        void
        createMarker(visualization_msgs::MarkerArray::Ptr& msg);

        /**
         * \brief Get a XYZRGB point from a point cloud.
         *
         * \param[in/out] p Point containing position information and to be filled with color.
         *
         * \return true if track is visible, false if not visible.
         */
        bool
        getPointXYZRGB(pcl::PointXYZRGB& p);

        /**
         * \brief Create track ROS message.
         *
         * \param[in/out] track_msg Track ROS message.
         * \param[in] vertical States if the camera is vertically oriented (true) or not (false).
         */
        void
        toMsg(opt_msgs::Track& track_msg, bool vertical);

        /**
         * \brief Get the DetectionSource corresponding to the last associated detection.
         *
         * \return the DetectionSource corresponding to the last associated detection.
         */
        open_ptrack::detection::DetectionSource*
        getDetectionSource();

        /**
         * \brief Set flag stating if people velocity should be used in motion term for data association
         *
         * \param[in] velocity_in_motion_term If true, people velocity is also used in motion term for data association
         * \param[in] acceleration_variance Acceleration variance (for Kalman Filter)
         * \param[in] position_variance Position variance (for Kalman Filter)
         */
        void
        setVelocityInMotionTerm (bool velocity_in_motion_term, double acceleration_variance, double position_variance);

        /**
         * \brief Set acceleration variance (for Kalman Filter)
         *
         * \param[in] acceleration_variance Acceleration variance (for Kalman Filter)
         */
        void
        setAccelerationVariance (double acceleration_variance);

        /**
         * \brief Set position variance (for Kalman Filter)
         *
         * \param[in] position_variance Position variance (for Kalman Filter)
         */
        void
        setPositionVariance (double position_variance);
    };

  } /* namespace tracking */
} /* namespace open_ptrack */
#endif /* OPEN_PTRACK_TRACKING_TRACK_H_ */
