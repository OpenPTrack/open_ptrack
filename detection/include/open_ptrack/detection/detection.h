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

#ifndef OPEN_PTRACK_DETECTION_DETECTION_H_
#define OPEN_PTRACK_DETECTION_DETECTION_H_

#include <Eigen/Eigen>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opt_msgs/Detection.h>
#include <open_ptrack/detection/detection_source.h>

namespace open_ptrack
{
  namespace detection
  {
    /** \brief Detection represents information about a people detection */
    class Detection
    {

      protected:
        /** \brief  */
        opt_msgs::Detection detection_msg_;		///The Detection3D message
        open_ptrack::detection::DetectionSource* source_;		///The source of the detection
        Eigen::Vector3d world_centroid_;				///The centroid point in world coordinates
        Eigen::Vector3d world_bottom_;				///The bottom point in world coordinates
        Eigen::Vector3d world_top_;					///The top point in world coordinates

        /**
         * Converts a BoundingBox2D message into a cv::Rect.
         * @return the converted cv::Rect.
         * @param bb the BoundingBox2D to convert.
         */
        cv::Rect BoundingBox2D2cvRect(const opt_msgs::BoundingBox2D& bb);

        /**
         * Converts a BoundingBox2D message into a cv::Rect.
         * @param bb the BoundingBox2D to convert.
         * @param rect the converted cv::Rect as a returning param.
         */
        void BoundingBox2D2cvRect(const opt_msgs::BoundingBox2D& bb, cv::Rect& rect);

      public:

        /**
         *
         */
        Detection(opt_msgs::Detection detection, open_ptrack::detection::DetectionSource* source);

        /**
         * Default destructor.
         */
        virtual ~Detection();

        //const people_tracking::Detection3D& getDetection();

        /**
         * Returns the pointer to the DetectionSource which generated the detection.
         */
        open_ptrack::detection::DetectionSource* getSource();

        /**
         * Returns the centroid location in world coordinates.
         */
        Eigen::Vector3d& getWorldCentroid();

        /**
         * Returns the top point location in world coordinates.
         */
        Eigen::Vector3d& getWorldTop();

        /**
         * Returns the bottom point location in world coordinates.
         */
        Eigen::Vector3d& getWorldBottom();

        /**
         * Returns the height of the object detected.
         */
        double getHeight();

        /**
         * Returns the confidence given by the HOG classifier.
         */
        double getConfidence();

        /**
         * Returns the distance of the object from the sensor.
         * @return the distance of the object from the sensor.
         */
        double getDistance();

        /**
         * Returns whether the object is occluded or not.
         * @return whether the object is occluded or not.
         */
        bool isOccluded();

        /**
         * Returns the bounding box of the detected object in pixel coordinates.
         * @return the bounding box of the detected object in pixel coordinates.
         */
        cv::Rect getBox2D();

        /**
         * Returns the cv::Mat of the detected object.
         * @return the cv::Mat of the detected object.
         */
        cv::Mat& getImage();
    };
  } /* namespace detection */
} /* namespace open_ptrack */

#endif /* OPEN_PTRACK_DETECTION_DETECTION_H_ */
