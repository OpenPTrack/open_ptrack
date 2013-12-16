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
#include <visualization_msgs/MarkerArray.h>

namespace open_ptrack
{
  namespace tracking
  {

    class Tracker
    {
      protected:

        const double gate_distance_;
        const bool detector_likelihood_;
        const std::vector<double> likelihood_weights_;
        const bool velocity_in_motion_term_;
        const double min_confidence_;
        const double min_confidence_detections_;
        const int detections_to_validate_;

        const double sec_before_old_;
        const double sec_before_fake_;
        const double sec_remain_new_;

        const double period_;
        const double position_variance_;
        const double acceleration_variance_;

        const bool debug_mode_;

        std::list<open_ptrack::tracking::Track*> tracks_;
        std::list<open_ptrack::tracking::Track*> lost_tracks_;
        std::list<open_ptrack::tracking::Track*> new_tracks_;
        std::list<open_ptrack::detection::Detection> unassociated_detections_;
        std::vector<open_ptrack::detection::Detection> detections_;

        int tracks_counter_;

        std::string world_frame_id_;

        cv::Mat_<double> distance_matrix_;
        cv::Mat_<double> cost_matrix_;

        ros::Time last_time_;

        bool vertical_;

        int createNewTrack(open_ptrack::detection::Detection& detection);
        void createDistanceMatrix();
        void createCostMatrix();
        void updateDetectedTracks();
        void fillUnassociatedDetections();
        void updateLostTracks();
        void createNewTracks();

      public:
        Tracker(double gate_distance, bool detector_likelihood, std::vector<double> likelihood_weights, bool velocity_in_motion_term,
            double min_confidence, double min_confidence_detections, double sec_before_old, double sec_before_fake,
            double sec_remain_new, int detections_to_validate, double period, double position_variance,
            double acceleration_variance, std::string world_frame_id, bool debug_mode, bool vertical);
        virtual ~Tracker();

        /**
         * Initializes a new frame starting from the vector containing the detections.
         * @param detections the vector containing the detections to analyze.
         */
        void newFrame(const std::vector<open_ptrack::detection::Detection>& detections);

        /**
         * Update the set of tracks after the new frame is created.
         */
        void updateTracks();

        /**
         * Draws the tracks into the RGB image given by its sensor.
         * @see DetectionSource
         */
        void drawRgb();

        /**
         * Fills the MarkerArray message given with a marker for each visible cluster (in correspondance
         * of its centroid) and its number.
         * @param msg the MarkerArray message to fill.
         */
        void toMarkerArray(visualization_msgs::MarkerArray::Ptr& msg);

        /**
         * Writes the state of each track into a TrackingResult message.
         * @param msg the TrackingResult message to fill.
         */
        void toMsg(opt_msgs::TrackArray::Ptr& msg);

        /**
         * Writes the state of each track into a TrackingResult message.
         * @param msg the TrackingResult message to fill.
         */
        void toMsg(opt_msgs::TrackArray::Ptr& msg, std::string& source_frame_id);

        /**
         * Appends the location of each track to the given point cloud starting from starting_index (using
         * a circular array)
         * @param pointcloud the point cloud where to append the locations.
         * @param starting_index the starting index of the array.
         * @param max_size the maximum size of the point cloud (when reached the points overwrite the initial ones)
         */
        size_t appendToPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointcloud,
            size_t starting_index, size_t max_size);
    };

  } /* namespace tracking */
} /* namespace open_ptrack */

#endif /* OPEN_PTRACK_TRACKING_TRACKER_H_ */
