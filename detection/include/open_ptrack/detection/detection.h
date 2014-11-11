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
        /** \brief ROS message containing detection information */
        opt_msgs::Detection detection_msg_;

        /** \brief Detection source which produced the detection */
        open_ptrack::detection::DetectionSource* source_;

        /** \brief Detection centroid in world reference frame */
        Eigen::Vector3d world_centroid_;

        /** \brief Detection bottom point in world reference frame */
        Eigen::Vector3d world_bottom_;

        /** \brief Detection top point in world reference frame */
        Eigen::Vector3d world_top_;

        /**
         * \brief Converts a BoundingBox2D message into a cv::Rect.
         *
         * \param[in] bb The BoundingBox2D to convert.
         *
         * \return the converted cv::Rect.
         */
        cv::Rect
        BoundingBox2D2cvRect(const opt_msgs::BoundingBox2D& bb);

        /**
         * \brief Converts a BoundingBox2D message into a cv::Rect.
         *
         * \param[in] bb The BoundingBox2D to convert.
         * \param[in] rect The converted cv::Rect.
         */
        void
        BoundingBox2D2cvRect(const opt_msgs::BoundingBox2D& bb, cv::Rect& rect);

      public:

        /** \brief Constructor. */
        Detection(opt_msgs::Detection detection, open_ptrack::detection::DetectionSource* source);

        /** \brief Destructor. */
        virtual ~Detection();

        /**
         * \brief Returns a pointer to the DetectionSource which generated the detection.
         *
         * \return a pointer to the DetectionSource which generated the detection.
         */
        open_ptrack::detection::DetectionSource*
        getSource();

        /**
         * \brief Returns the detection centroid in world reference frame.
         *
         * \return the detection centroid in world reference frame.
         */
        Eigen::Vector3d&
        getWorldCentroid();

        /**
         * \brief Returns the detection top point in world reference frame.
         *
         * \return the detection top point in world reference frame.
         */
        Eigen::Vector3d&
        getWorldTop();

        /**
         * \brief Returns the detection bottom point in world reference frame.
         *
         * \return the detection bottom point in world reference frame.
         */
        Eigen::Vector3d&
        getWorldBottom();

        /**
         * \brief Returns the detection height from the ground plane.
         *
         * \return the detection height from the ground plane.
         */
        double
        getHeight();

        /**
         * \brief Returns the confidence of the people detector associated to the detection.
         *
         * \return the confidence of the people detector associated to the detection.
         */
        double
        getConfidence();

        /**
         * \brief Returns the distance of the detection from the sensor.
         *
         * \return the distance of the detection from the sensor.
         */
        double
        getDistance();

        /**
         * \brief Returns if the detection corresponds to an occluded person or not.
         *
         * \return true if the detection corresponds to an occluded person, false otherwise.
         */
        bool
        isOccluded();

        /**
         * \brief Returns the bounding box of the detection in pixel coordinates.
         *
         * \return the bounding box of the detection in pixel coordinates.
         */
        cv::Rect
        getBox2D();

        /**
         * \brief The image where the detection has been found.
         *
         * \return the image where the detection has been found.
         */
        cv::Mat&
        getImage();

        /**
         * \brief Set the confidence of the people detector associated to the detection.
         *
         * \param[in] confidence Confidence of the people detector associated to the detection.
         */
        void
        setConfidence(double confidence);

        /**
         * \brief Set the detection centroid in world reference frame.
         *
         * \param[in] centroid The detection centroid in world reference frame.
         */
        void
        setWorldCentroid(const Eigen::Vector3d& centroid);
    };
  } /* namespace detection */
} /* namespace open_ptrack */

#endif /* OPEN_PTRACK_DETECTION_DETECTION_H_ */
