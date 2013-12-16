/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011-, Matteo Munaro [matteo.munaro@dei.unipd.it], Filippo Basso
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
 */

#ifndef OPEN_PTRACK_DETECTION_DETECTION_SOURCE_H_
#define OPEN_PTRACK_DETECTION_DETECTION_SOURCE_H_

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

#include <open_ptrack/detection/conversions.h>
#include <tf/tf.h>

namespace open_ptrack
{
  namespace detection
  {

    class DetectionSource
    {
      protected:
        cv::Mat image_;
        tf::StampedTransform transform_;
        tf::StampedTransform inverse_transform_;
        Eigen::Matrix3d intrinsic_matrix_;
        ros::Time time_;
        ros::Duration duration_;
        std::string frame_id_;

      public:
        DetectionSource(cv::Mat image, tf::StampedTransform transform, tf::StampedTransform inverse_transform,
            Eigen::Matrix3d intrinsic_matrix, ros::Time time, std::string frame_id);

        virtual ~DetectionSource();

        void update(cv::Mat image, tf::StampedTransform transform, tf::StampedTransform inverse_transform,
            Eigen::Matrix3d intrinsic_matrix, ros::Time time, std::string frame_id);

        Eigen::Vector3d transform(const Eigen::Vector3d& v);
        Eigen::Vector3d transform(const geometry_msgs::Vector3& v);
        Eigen::Vector3d inverseTransform(const Eigen::Vector3d& v);
        Eigen::Vector3d inverseTransform(const geometry_msgs::Vector3& v);
        Eigen::Vector3d world2cam(const Eigen::Vector3d& v);
        Eigen::Vector3d transformToCam(const Eigen::Vector3d& v);
        cv::Mat& getImage();
        ros::Time getTime();
        ros::Duration getDuration();
        std::string getFrameId();
        void setImage(cv::Mat& image);
    };

  } /* namespace detection */
} /* namespace open_ptrack */

#endif /* OPEN_PTRACK_DETECTION_DETECTION_SOURCE_H_ */
