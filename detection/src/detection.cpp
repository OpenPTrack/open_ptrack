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

#include <open_ptrack/detection/detection.h>

namespace open_ptrack
{
  namespace detection
  {

    Detection::Detection(opt_msgs::Detection detection, open_ptrack::detection::DetectionSource* source) :
		    detection_msg_(detection), source_(source)
    {
      // Transform centroid, top and bottom points from camera frame to world frame:
      Eigen::Vector3d v;
      v(0) = detection.centroid.x;
      v(1) = detection.centroid.y;
      v(2) = detection.centroid.z;
      world_centroid_ = source->transform(v);

      v(0) = detection.top.x;
      v(1) = detection.top.y;
      v(2) = detection.top.z;
      world_top_ = source->transform(v);

      v(0) = detection.bottom.x;
      v(1) = detection.bottom.y;
      v(2) = detection.bottom.z;
      world_bottom_ = source->transform(v);
    }

    Detection::~Detection()
    {

    }

    open_ptrack::detection::DetectionSource*
    Detection::getSource()
    {
      return source_;
    }

    Eigen::Vector3d&
    Detection::getWorldCentroid()
    {
      return world_centroid_;
    }

    Eigen::Vector3d&
    Detection::getWorldTop()
    {
      return world_top_;
    }

    Eigen::Vector3d&
    Detection::getWorldBottom()
    {
      return world_bottom_;
    }

    double
    Detection::getHeight()
    {
      return detection_msg_.height;
    }

    double
    Detection::getConfidence()
    {
      return detection_msg_.confidence;
    }

    double
    Detection::getDistance()
    {
      return detection_msg_.distance;
    }

    bool
    Detection::isOccluded()
    {
      return detection_msg_.occluded;
    }

    cv::Rect
    Detection::getBox2D()
    {
      return BoundingBox2D2cvRect(detection_msg_.box_2D);
    }

    cv::Mat&
    Detection::getImage()
    {
      return source_->getImage();
    }

    void
    Detection::setConfidence(double confidence)
    {
      detection_msg_.confidence = confidence;
    }

    void
    Detection::setWorldCentroid(const Eigen::Vector3d& centroid)
    {
      world_centroid_ = centroid;
    }

    /************************ protected methods ************************/

    cv::Rect
    Detection::BoundingBox2D2cvRect(const opt_msgs::BoundingBox2D& bb)
    {
      cv::Rect rect;
      rect.x = bb.x;
      rect.y = bb.y;
      rect.width = bb.width;
      rect.height = bb.height;
      return rect;
    }

    void
    Detection::BoundingBox2D2cvRect(const opt_msgs::BoundingBox2D& bb, cv::Rect& rect)
    {
      rect.x = bb.x;
      rect.y = bb.y;
      rect.width = bb.width;
      rect.height = bb.height;
    }

  } /* namespace detection */
} /* namespace open_ptrack */
