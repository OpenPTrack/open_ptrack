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

#ifndef OPEN_PTRACK_DETECTION_DETECTION_SOURCE_H_
#define OPEN_PTRACK_DETECTION_DETECTION_SOURCE_H_

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <open_ptrack/opt_utils/conversions.h>
#include <tf/tf.h>

namespace open_ptrack
{
  namespace detection
  {
    /** \brief DetectionSource represents information about a source of people detections */
    class DetectionSource
    {
      protected:
        /** \brief last image associated to the detection source */
        cv::Mat image_;

        /** \brief transform between camera and world reference frames */
        tf::StampedTransform transform_;

        /** \brief transform between world and camera reference frames */
        tf::StampedTransform inverse_transform_;

        /** \brief intrinsic parameters of the camera associated to the detection source */
        Eigen::Matrix3d intrinsic_matrix_;

        /** \brief last time detections arrived from the detection source */
        ros::Time time_;

        /** \brief time passed between last two detection messages */
        ros::Duration duration_;

        /** \brief frame id associated to the detection source */
        std::string frame_id_;

      public:
        /** \brief Constructor. */
        DetectionSource(cv::Mat image, tf::StampedTransform transform, tf::StampedTransform inverse_transform,
            Eigen::Matrix3d intrinsic_matrix, ros::Time time, std::string frame_id);

        /** \brief Destructor. */
        virtual ~DetectionSource();

        /**
         * \brief Update detection source information with last received message.
         *
         * \param[in] image Image.
         * \param[in] transform Transform between camera and world reference frames.
         * \param[in] inverse_transform Transform between world and camera reference frames.
         * \param[in] intrinsic_matrix Intrinsic parameters of the camera associated to the detection source.
         * \param[in] time ROS time.
         * \param[in] frame_id Frame id.
         */
        void
        update(cv::Mat image, tf::StampedTransform transform, tf::StampedTransform inverse_transform,
            Eigen::Matrix3d intrinsic_matrix, ros::Time time, std::string frame_id);

        /**
         * \brief Apply camera to world transformation to the input vector.
         *
         * \param[in] v 3D vector.
         *
         * \return Transformed vector.
         */
        Eigen::Vector3d
        transform(const Eigen::Vector3d& v);

        /**
         * \brief Apply camera to world transformation to the input vector.
         *
         * \param[in] v 3D vector.
         *
         * \return Transformed vector.
         */
        Eigen::Vector3d
        transform(const geometry_msgs::Vector3& v);

        /**
         * \brief Apply world to camera transformation to the input vector.
         *
         * \param[in] v 3D vector.
         *
         * \return Transformed vector.
         */
        Eigen::Vector3d
        inverseTransform(const Eigen::Vector3d& v);

        /**
         * \brief Apply world to camera transformation to the input vector.
         *
         * \param[in] v 3D vector.
         *
         * \return Transformed vector.
         */
        Eigen::Vector3d
        inverseTransform(const geometry_msgs::Vector3& v);

        /**
         * \brief Project 3D point in world frame to pixel position in image associated to the detection source.
         *
         * \param[in] v 3D point in world frame.
         *
         * \return image point in homogeneous coordinates [x y 1].
         */
        Eigen::Vector3d
        transformToCam(const Eigen::Vector3d& v);

        /**
         * \brief Get last image associated to the detection source.
         *
         * \return the image as OpenCV matrix.
         */
        cv::Mat&
        getImage();

        /**
         * \brief Get last time detections from this detection source arrived.
         *
         * \return time as ros::Time.
         */
        ros::Time
        getTime();

        /**
         * \brief Get time passed between last two detection messages.
         *
         * \return
         */
        ros::Duration
        getDuration();

        /**
         * \brief Get frame id associated to the detection source.
         *
         * \return the frame id as a string.
         */
        std::string
        getFrameId();

        /**
         * \brief Set the image associated to the detection source with the value of the input image.
         *
         * \param[in] image The input image.
         */
        void
        setImage(cv::Mat& image);
    };

  } /* namespace detection */
} /* namespace open_ptrack */

#endif /* OPEN_PTRACK_DETECTION_DETECTION_SOURCE_H_ */
