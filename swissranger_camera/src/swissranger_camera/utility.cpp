/*
 *  Copyright (c) 2013-2014, Filippo Basso <bassofil@dei.unipd.it>
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *     3. Neither the name of the copyright holder(s) nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <opencv2/imgproc/imgproc.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <swissranger_camera/utility.h>

namespace sr
{

void Utility::getOffsets_(int & x_offset,
                          int & y_offset,
                          int & z_offset,
                          int & intensity_offset,
                          int & confidence_offset) const
{
  x_offset = -1;
  y_offset = -1;
  z_offset = -1;
  intensity_offset = -1;
  confidence_offset = -1;
  for (size_t i = 0; i < swiss_ranger_msg_->fields.size(); ++i)
  {
    if (swiss_ranger_msg_->fields[i].name == "x")
      x_offset = swiss_ranger_msg_->fields[i].offset;
    else if (swiss_ranger_msg_->fields[i].name == "y")
      y_offset = swiss_ranger_msg_->fields[i].offset;
    else if (swiss_ranger_msg_->fields[i].name == "z")
      z_offset = swiss_ranger_msg_->fields[i].offset;
    else if (swiss_ranger_msg_->fields[i].name == "intensity")
      intensity_offset = swiss_ranger_msg_->fields[i].offset;
    else if (swiss_ranger_msg_->fields[i].name == "confidence")
      confidence_offset = swiss_ranger_msg_->fields[i].offset;
  }
  if (x_offset < 0)
    throw std::runtime_error("No \"x\" field found in the cloud.");
  if (y_offset < 0)
    throw std::runtime_error("No \"y\" field found in the cloud.");
  if (z_offset < 0)
    throw std::runtime_error("No \"z\" field found in the cloud.");
  if (intensity_offset < 0)
    throw std::runtime_error("No \"intensity\" field found in the cloud.");
  if (confidence_offset < 0)
    throw std::runtime_error("No \"confidence\" field found in the cloud.");
}

//void Utility::split(unsigned int fields)
//{
//  split_fields_ = fields;
//  if (not (split_fields_ & ALL))
//    return;

//  const size_t height = swiss_ranger_msg_->height;
//  const size_t width = swiss_ranger_msg_->width;
//  const size_t step = swiss_ranger_msg_->point_step;

//  int x_offset = -1;
//  int y_offset = -1;
//  int z_offset = -1;
//  int intensity_offset = -1;
//  int confidence_offset = -1;
//  for (size_t i = 0; i < swiss_ranger_msg_->fields.size(); ++i)
//  {
//    if (swiss_ranger_msg_->fields[i].name == "x")
//      x_offset = swiss_ranger_msg_->fields[i].offset;
//    else if (swiss_ranger_msg_->fields[i].name == "y")
//      y_offset = swiss_ranger_msg_->fields[i].offset;
//    else if (swiss_ranger_msg_->fields[i].name == "z")
//      z_offset = swiss_ranger_msg_->fields[i].offset;
//    else if (swiss_ranger_msg_->fields[i].name == "intensity")
//      intensity_offset = swiss_ranger_msg_->fields[i].offset;
//    else if (swiss_ranger_msg_->fields[i].name == "confidence")
//      confidence_offset = swiss_ranger_msg_->fields[i].offset;
//  }
//  if (x_offset < 0)
//    throw std::runtime_error("No \"x\" field found in the cloud.");
//  if (y_offset < 0)
//    throw std::runtime_error("No \"y\" field found in the cloud.");
//  if (z_offset < 0)
//    throw std::runtime_error("No \"z\" field found in the cloud.");
//  if (intensity_offset < 0)
//    throw std::runtime_error("No \"intensity\" field found in the cloud.");
//  if (confidence_offset < 0)
//    throw std::runtime_error("No \"confidence\" field found in the cloud.");

//  if (split_fields_ & CLOUD)
//  {
//    cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
//    cloud_->header = pcl_conversions::toPCL(swiss_ranger_msg_->header);
//    cloud_->points.resize(height * width);
//    cloud_->height = height;
//    cloud_->width = width;
//    cloud_->is_dense = true;
//  }

//  if (split_fields_ & INTENSITY)
//  {
//    if (intensity_type_ == INTENSITY_FLOAT)
//      intensity_image_ = cv::Mat(height, width, CV_32FC1);
//    else if (intensity_type_ == INTENSITY_8BIT)
//      intensity_image_ = cv::Mat(height, width, CV_8UC1);
//    else //(intensity_type_ == INTENSITY_16BIT)
//      intensity_image_ = cv::Mat(height, width, CV_16UC1);
//  }

//  if (split_fields_ & CONFIDENCE)
//  {
//    if (confidence_type_ == CONFIDENCE_FLOAT)
//      confidence_image_ = cv::Mat(height, width, CV_32FC1);
//    else if (confidence_type_ == CONFIDENCE_8BIT)
//      confidence_image_ = cv::Mat(height, width, CV_8UC1);
//    else //(confidence_type_ == CONFIDENCE_16BIT)
//      confidence_image_ = cv::Mat2b(height, width, CV_16UC1);
//  }


//  for (size_t i = 0; i < height; ++i)
//  {
//    for (size_t j = 0; j < width; ++j)
//    {
//      size_t index = (j + i * width) * step;
//      const float * x_ptr = reinterpret_cast<const float *>(&swiss_ranger_msg_->data[index + x_offset]);
//      const float * y_ptr = reinterpret_cast<const float *>(&swiss_ranger_msg_->data[index + y_offset]);
//      const float * z_ptr = reinterpret_cast<const float *>(&swiss_ranger_msg_->data[index + z_offset]);
//      const float * intensity_ptr = reinterpret_cast<const float *>(&swiss_ranger_msg_->data[index + intensity_offset]);
//      const float * confidence_ptr = reinterpret_cast<const float *>(&swiss_ranger_msg_->data[index + confidence_offset]);

//      if (split_fields_ & CLOUD)
//      {
//        pcl::PointXYZ & point = (*cloud_)(j, i);
//        if (*confidence_ptr >= confidence_threshold_)
//        {
//          point.x = *x_ptr;
//          point.y = *y_ptr;
//          point.z = *z_ptr;
//        }
//        else
//        {
//          point.x = bad_point_;
//          point.y = bad_point_;
//          point.z = bad_point_;
//          cloud_->is_dense = false;
//        }
//      }
//      if (split_fields_ & INTENSITY)
//      {
//        if (intensity_type_ == INTENSITY_FLOAT)
//          intensity_image_.at<float>(i, j) = *intensity_ptr;
//        else if (intensity_type_ == INTENSITY_8BIT)
//          intensity_image_.at<uint8_t>(i, j) = static_cast<uint8_t>(*intensity_ptr / 257.0f);
//        else //(intensity_type_ == INTENSITY_16BIT)
//          intensity_image_.at<uint16_t>(i, j) = static_cast<uint16_t>(*intensity_ptr);
//      }
//      if (split_fields_ & CONFIDENCE)
//      {
//        if (*confidence_ptr >= confidence_threshold_)
//        {
//          if (confidence_type_ == CONFIDENCE_FLOAT)
//            confidence_image_.at<float>(i, j) = *intensity_ptr;
//          else if (confidence_type_ == CONFIDENCE_8BIT)
//            confidence_image_.at<uint8_t>(i, j) = static_cast<uint8_t>(*intensity_ptr / 257.0f);
//          else //(confidence_type_ == CONFIDENCE_16BIT)
//            confidence_image_.at<uint16_t>(i, j) = static_cast<uint16_t>(*intensity_ptr);
//        }
//        else
//        {
//          if (confidence_type_ == CONFIDENCE_FLOAT)
//            confidence_image_.at<float>(i, j) = 0.0f;
//          else if (confidence_type_ == CONFIDENCE_8BIT)
//            confidence_image_.at<uint8_t>(i, j) = static_cast<uint8_t>(0u);
//          else //(confidence_type_ == CONFIDENCE_16BIT)
//            confidence_image_.at<uint16_t>(i, j) = static_cast<uint16_t>(0u);
//        }
//      }
//    }
//  }

//  if ((split_fields_ & INTENSITY) and normalize_intensity_)
//  {
//    double min_val, max_val;
//    cv::Point min_loc, max_loc;
//    cv::minMaxLoc(intensity_image_, &min_val, &max_val, &min_loc, &max_loc, confidence_image_);

//    // Rescale image intensity:
//    for (size_t i = 0; i < height; ++i)
//    {
//      for (size_t j = 0; j < width; ++j)
//      {
//        size_t index = (j + i * width) * step;
//        const float * intensity_ptr = reinterpret_cast<const float *>(&swiss_ranger_msg_->data[index + intensity_offset]);
//        if (intensity_type_ == INTENSITY_FLOAT)
//          intensity_image_.at<float>(i, j) = (*intensity_ptr - min_val) / (max_val - min_val);
//        else if (intensity_type_ == INTENSITY_8BIT)
//          intensity_image_.at<uint8_t>(i, j) = static_cast<uint8_t>(*intensity_ptr / 257.0f);
//        else //(intensity_type_ == INTENSITY_16BIT)
//          intensity_image_.at<uint16_t>(i, j) = static_cast<uint16_t>(*intensity_ptr);
//        intensity_image_.at<unsigned char>(i,j) = (intensity_ptr[0] / 257 - min_val) * 255 / (max_val - min_val);
//      }
//    }
//  }

//}

} /* namespace calibration */
