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

#ifndef OPEN_PTRACK_TRACKING_TRACKER_H_
#define OPEN_PTRACK_TRACKING_TRACKER_H_

#include <open_ptrack/detection/detection.h>
#include <open_ptrack/tracking/track.h>
#include <open_ptrack/tracking/munkres.h>
#include <opt_msgs/TrackArray.h>
#include <opt_msgs/IDArray.h>
#include <visualization_msgs/MarkerArray.h>

namespace open_ptrack
{
  namespace tracking
  {
    /** \brief Tracker performs tracking-by-detection */
    class Tracker
    {
      protected:
        /** \brief List of all active tracks */
        std::list<open_ptrack::tracking::Track*> tracks_;

        /** \brief List of lost tracks */
        std::list<open_ptrack::tracking::Track*> lost_tracks_;

        /** \brief List of tracks with Status = NEW */
        std::list<open_ptrack::tracking::Track*> new_tracks_;

        /** \brief List of current detections */
        std::vector<open_ptrack::detection::Detection> detections_;

        /** \brief List of current detections not associated to any track */
        std::list<open_ptrack::detection::Detection> unassociated_detections_;

        /** \brief Track ID counter */
        int tracks_counter_;

        /** \brief World reference frame used for tracking */
        std::string world_frame_id_;

        /** \brief Minimum confidence for track initialization */
        double min_confidence_;

        /** \brief Minimum confidence of detections sent to tracking */
        const double min_confidence_detections_;

        /** \brief Minimum number of detection<->track associations needed for validating a track */
        int detections_to_validate_;

        /** \brief Time after which a not visible track becomes old */
        double sec_before_old_;

        /** \brief Time after which a visible track obtain NORMAL status */
        double sec_remain_new_;

        /** \brief Time within which a track should be validated (otherwise it is discarded) */
        double sec_before_fake_;

        /** \brief Gate distance for joint likelihood in data association */
        double gate_distance_;

        /** \brief Flag stating if people detection confidence should be used in data association (true) or not (false) */
        bool detector_likelihood_;

        /** \brief Weights for the single terms of the joint likelihood */
        std::vector<double> likelihood_weights_;

        /** \brief If true, people velocity is also used in motion term for data association */
        bool velocity_in_motion_term_;

        /** \brief Minimum time period between two detections messages */
        const double period_;

        /** \brief Position variance (for Kalman Filter) */
        double position_variance_;

        /** \brief Acceleration variance (for Kalman Filter) */
        double acceleration_variance_;

        /** \brief Flag enabling debug mode */
        const bool debug_mode_;

        /** \brief Detections<->tracks distance matrix for data association */
        cv::Mat_<double> distance_matrix_;

        /** \brief Detections<->tracks cost matrix to be used to solve the Global Nearest Neighbor problem */
        cv::Mat_<double> cost_matrix_;

        /** \brief if true, the sensor is considered to be vertically placed (portrait mode) */
        bool vertical_;

        /** \brief Create detections<->tracks distance matrix for data association */
        void
        createDistanceMatrix();

        /** \brief Create detections<->tracks cost matrix to be used to solve the Global Nearest Neighbor problem */
        void
        createCostMatrix();

        /** \brief Update tracks associated to a detection in the current frame */
        void
        updateDetectedTracks();

        /** \brief Fill list containing unassociated detections */
        void
        fillUnassociatedDetections();

        /** \brief Create new tracks with high confidence unassociated detections */
        void
        createNewTracks();

        /** \brief Create a new track with detection information */
        int
        createNewTrack(open_ptrack::detection::Detection& detection);

        /** \brief Update lost tracks */
        void
        updateLostTracks();

      public:
        /** \brief Constructor */
        Tracker(double gate_distance, bool detector_likelihood, std::vector<double> likelihood_weights, bool velocity_in_motion_term,
            double min_confidence, double min_confidence_detections, double sec_before_old, double sec_before_fake,
            double sec_remain_new, int detections_to_validate, double period, double position_variance,
            double acceleration_variance, std::string world_frame_id, bool debug_mode, bool vertical);

        /** \brief Destructor */
        virtual ~Tracker();

        /**
         * \brief Initialization when a new set of detections arrive.
         *
         * \param[in] detections Vector of current detections.
         *
         */
        void
        newFrame(const std::vector<open_ptrack::detection::Detection>& detections);

        /**
         * \brief Update the list of tracks according to the current set of detections.
         */
        void
        updateTracks();

//        /**
//         * \brief Draw the tracks into the RGB image given by its sensor.
//         */
//        void
//        drawRgb();

        /**
         * \brief Fills the MarkerArray message with a marker for each visible track (in correspondance
         * of its centroid) and its number.
         *
         * \param[in] msg The MarkerArray message to fill.
         */
        void
        toMarkerArray(visualization_msgs::MarkerArray::Ptr& msg);

        /**
         * \brief Writes the state of each track into a TrackArray message.
         *
         * \param[in] msg The TrackArray message to fill.
         */
        void
        toMsg(opt_msgs::TrackArray::Ptr& msg);

        /**
         * \brief Writes the state of tracks with a given frame id into a TrackArray message.
         *
         * \param[in] msg The TrackArray message to fill.
         * \param[in] source_frame_id Frame id of tracks that have to be written to msg.
         */
        void
        toMsg(opt_msgs::TrackArray::Ptr& msg, std::string& source_frame_id);

        /**
         * \brief Writes the ID of each alive track into an IDArray message.
         *
         * \param[in] msg The IDArray message to fill.
         */
        void
        getAliveIDs (opt_msgs::IDArray::Ptr& msg);

        /**
         * \brief Appends the location of each track to a point cloud starting from starting_index (using
         * a circular array)
         *
         * \param[in] pointcloud The point cloud where to append the locations.
         * \param[in] starting_index The starting index of the array.
         * \param[in] max_size The maximum size of the point cloud (when reached the points overwrite the initial ones)
         *
         * \return the new starting_index.
         */
        size_t
        appendToPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointcloud,
            size_t starting_index, size_t max_size);

        /**
         * \brief Set minimum confidence for track initialization
         *
         * \param[in] min_confidence Minimum confidence for track initialization
         */
        void
        setMinConfidenceForTrackInitialization (double min_confidence);

        /**
         * \brief Set time after which a not visible track becomes old
         *
         * \param[in] sec_before_old Time after which a not visible track becomes old
         */
        void
        setSecBeforeOld (double sec_before_old);

        /**
         * \brief Set time within which a track should be validated (otherwise it is discarded)
         *
         * \param[in] sec_before_fake Time within which a track should be validated (otherwise it is discarded)
         */
        void
        setSecBeforeFake (double sec_before_fake);

        /**
         * \brief Set time after which a visible track obtain NORMAL status
         *
         * \param[in] sec_remain_new Time after which a visible track obtain NORMAL status
         */
        void
        setSecRemainNew (double sec_remain_new);

        /**
         * \brief Set minimum number of detection<->track associations needed for validating a track
         *
         * \param[in] detections_to_validate Minimum number of detection<->track associations needed for validating a track
         */
        void
        setDetectionsToValidate (int detections_to_validate);

        /**
         * \brief Set flag stating if people detection confidence should be used in data association (true) or not (false)
         *
         * \param[in] detector_likelihood Flag stating if people detection confidence should be used in data association (true) or not (false)
         */
        void
        setDetectorLikelihood (bool detector_likelihood);

        /**
         * \brief Set likelihood weights for data association
         *
         * \param[in] detector_weight Weight for detector likelihood
         * \param[in] motion_weight Weight for motion likelihood
         */
        void
        setLikelihoodWeights (double detector_weight, double motion_weight);

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

        /**
         * \brief Set gate distance for joint likelihood in data association
         *
         * \param[in] gate_distance Gate distance for joint likelihood in data association.
         */
        void
        setGateDistance (double gate_distance);
    };

  } /* namespace tracking */
} /* namespace open_ptrack */

#endif /* OPEN_PTRACK_TRACKING_TRACKER_H_ */
