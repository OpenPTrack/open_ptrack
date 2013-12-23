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

#include <open_ptrack/detection/detection_source.h>

namespace open_ptrack
{
  namespace detection
  {

    DetectionSource::DetectionSource(cv::Mat image, tf::StampedTransform transform,
        tf::StampedTransform inverse_transform, Eigen::Matrix3d intrinsic_matrix, ros::Time time, std::string frame_id) :
	    image_(image), transform_(transform), inverse_transform_(inverse_transform),
	    intrinsic_matrix_(intrinsic_matrix), time_(time), duration_(0), frame_id_(frame_id)
    {

    }

    DetectionSource::~DetectionSource()
    {

    }

    void
    DetectionSource::update(cv::Mat image, tf::StampedTransform transform,
        tf::StampedTransform inverse_transform, Eigen::Matrix3d intrinsic_matrix, ros::Time time, std::string frame_id)
    {
      image_ = image;
      transform_ = transform;
      inverse_transform_ = inverse_transform;
      intrinsic_matrix_ = intrinsic_matrix;
      duration_ = time - time_;
      time_ = time;
      frame_id_ = frame_id;
    }

    Eigen::Vector3d
    DetectionSource::transform(const Eigen::Vector3d& v)
    {
      Eigen::Vector3d ret;
      tf::Vector3 t(v(0), v(1), v(2));
      t = transform_(t);
      ret(0) = t.getX();
      ret(1) = t.getY();
      ret(2) = t.getZ();
      return ret;
    }

    Eigen::Vector3d
    DetectionSource::transform(const geometry_msgs::Vector3& v)
    {
      Eigen::Vector3d ret;
      tf::Vector3 t;
      tf::vector3MsgToTF(v, t);
      t = transform_(t);
      ret(0) = t.getX();
      ret(1) = t.getY();
      ret(2) = t.getZ();
      return ret;
    }

    Eigen::Vector3d
    DetectionSource::inverseTransform(const Eigen::Vector3d& v)
    {
      Eigen::Vector3d ret;
      tf::Vector3 t(v(0), v(1), v(2));
      //ROS_INFO("C: %f %f %f", t.getX(), t.getY(), t.getZ());
      t = inverse_transform_(t);
      ret(0) = t.getX();
      ret(1) = t.getY();
      ret(2) = t.getZ();
      return ret;
    }

    Eigen::Vector3d
    DetectionSource::inverseTransform(const geometry_msgs::Vector3& v)
    {
      Eigen::Vector3d ret;
      tf::Vector3 t;
      tf::vector3MsgToTF(v, t);
      t = inverse_transform_(t);
      ret(0) = t.getX();
      ret(1) = t.getY();
      ret(2) = t.getZ();
      return ret;
    }

    Eigen::Vector3d
    DetectionSource::transformToCam(const Eigen::Vector3d& v)
    {
      return open_ptrack::opt_utils::Conversions::world2cam(inverseTransform(v), intrinsic_matrix_);
    }

    cv::Mat&
    DetectionSource::getImage()
    {
      return image_;
    }

    ros::Time
    DetectionSource::getTime()
    {
      return time_;
    }

    ros::Duration
    DetectionSource::getDuration()
    {
      return duration_;
    }

    std::string
    DetectionSource::getFrameId()
    {
      return frame_id_;
    }

    void
    DetectionSource::setImage(cv::Mat& image)
    {
      image_ = image;
    }
  } /* namespace detection */
} /* namespace open_ptrack */
