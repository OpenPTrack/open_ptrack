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
    class Track
    {
      public:

        enum Status
        {
          NEW,
          NORMAL,
          TRANSFERRED
        };

        enum Visibility
        {
          VISIBLE = 0,
          OCCLUDED = 1,
          NOT_VISIBLE = 2
        };

      protected:

        int MAX_SIZE; //TODO create a parameter!

        const int id_;
        const std::string frame_id_;
        const double period_;

        bool validated_;

        open_ptrack::tracking::KalmanFilter* filter_;
        ros::Time first_time_detected_;
        ros::Time last_time_detected_;
        ros::Time last_time_detected_with_high_confidence_;

        open_ptrack::tracking::KalmanFilter* tmp_filter_;
        ros::Time last_time_predicted_;
        int last_time_predicted_index_;
        std::vector<MahalanobisParameters2d> mahalanobis_map2d_;
        std::vector<MahalanobisParameters4d> mahalanobis_map4d_;

        Status status_;
        Visibility visibility_;
        int updates_with_enough_confidence_;

        double z_;
        double height_;
        double distance_;
        double log_likelihood_;
        double last_detector_confidence_;

        Eigen::Vector3f color_;
        open_ptrack::detection::DetectionSource* detection_source_;

        bool velocity_in_motion_term_;
        int low_confidence_consecutive_frames_;		// count the number of consecutive updates with low confidence detections

      public:

        Track(
            int id,
            std::string frame_id,
            double position_variance,		//TODO parametes referred to the Kalman filter:
            double acceleration_variance,	//	   should not stay here!!!
            double period,
            bool velocity_in_motion_term);

        virtual ~Track();

        void init(const Track& old_track);

        void init(
            double x,
            double y,
            double z,
            double height,
            double distance,
            open_ptrack::detection::DetectionSource* detection_source);

       void predict(
            double& x,
            double& y,
            double& height,
            double& vx,
            double& vy,
            ros::Time& when);

        void update(
            double x,
            double y,
            double z,
            double height,
            double distance,
            double log_likelihood,
            double confidence,
            double min_confidence,
            double min_confidence_detections,
            open_ptrack::detection::DetectionSource* detection_source,
            bool first_update = false);

        double getMahalanobisDistance(double x, double y, const ros::Time& when);
        void validate();
        bool isValidated();
        int getId();

        void setStatus(Status s);
        Status getStatus();
        void setVisibility(Visibility v);
        Visibility getVisibility();

        float getSecFromFirstDetection(ros::Time current_time);
        float getSecFromLastDetection(ros::Time current_time);
        float getSecFromLastHighConfidenceDetection(ros::Time current_time);
        float getLowConfidenceConsecutiveFrames();
        int getUpdatesWithEnoughConfidence();

        void draw(bool vertical);
        void createMarker(visualization_msgs::MarkerArray::Ptr& msg);
        bool getPointXYZRGB(pcl::PointXYZRGB& p);
        void toMsg(opt_msgs::Track& track_msg, bool vertical);
        open_ptrack::detection::DetectionSource* getDetectionSource();
    };

  } /* namespace tracking */
} /* namespace open_ptrack */
#endif /* OPEN_PTRACK_TRACKING_TRACK_H_ */
