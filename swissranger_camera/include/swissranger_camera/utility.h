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

#ifndef SWISSRANGER_CAMERA_UTILITY_H_
#define SWISSRANGER_CAMERA_UTILITY_H_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl_conversions/pcl_conversions.h>


#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

namespace sr
{

class Utility
{

public:

  typedef boost::shared_ptr<Utility> Ptr;
  typedef boost::shared_ptr<const Utility> ConstPtr;

  enum { MAX_CONFIDENCE = 65535u };
  enum { MAX_INTENSITY = 65535u };

  enum
  {
    CLOUD = 1u,
    INTENSITY = 2u,
    CONFIDENCE = 4u,
    ALL = 7u
  };

  enum IntensityType
  {
    INTENSITY_FLOAT,
    INTENSITY_8BIT,
    INTENSITY_16BIT
  };

  enum ConfidenceType
  {
    CONFIDENCE_FLOAT,
    CONFIDENCE_8BIT,
    CONFIDENCE_16BIT
  };

  Utility()
    : confidence_threshold_(MAX_CONFIDENCE * 0.95f),
      split_fields_(0u),
      bad_point_(std::numeric_limits<float>::quiet_NaN()),
      intensity_type_(INTENSITY_FLOAT),
      confidence_type_(CONFIDENCE_FLOAT),
      normalize_intensity_(false)
  {
    // Do nothing
  }

  inline void setInputCloud(const sensor_msgs::PointCloud2::ConstPtr & swiss_ranger_msg)
  {
    swiss_ranger_msg_ = swiss_ranger_msg;
    split_fields_ = 0u;
  }

  inline void setConfidenceThreshold(float threshold)
  {
    confidence_threshold_ = MAX_CONFIDENCE * threshold;
  }

  inline void setIntensityType(IntensityType type)
  {
    intensity_type_ = type;
  }

  inline void setConfidenceType(ConfidenceType type)
  {
    confidence_type_ = type;
  }

  inline void setNormalizeIntensity(bool normalize)
  {
    normalize_intensity_ = normalize;
  }

  inline pcl::PointCloud<pcl::PointXYZ>::Ptr cloud() const
  {
    assert(split_fields_ | CLOUD);
    return cloud_;
  }

  inline cv::Mat intensityImage() const
  {
    assert(split_fields_ | INTENSITY);
    return intensity_image_;
  }

  inline cv::Mat confidenceImage() const
  {
    assert(split_fields_ | CONFIDENCE);
    return confidence_image_;
  }

  void split(unsigned int fields = ALL)
  {
    if (intensity_type_ == INTENSITY_FLOAT)
      split2_<float>(fields);
    else if (intensity_type_ == INTENSITY_8BIT)
      split2_<uint8_t>(fields);
    else //(intensity_type_ == INTENSITY_16BIT)
      split2_<uint16_t>(fields);
  }

private:

  template <typename IntensityT_>
    void split2_(unsigned int fields)
    {
      if (confidence_type_ == CONFIDENCE_FLOAT)
        split_<IntensityT_, float>(fields);
      else if (confidence_type_ == CONFIDENCE_8BIT)
        split_<IntensityT_, uint8_t>(fields);
      else //(confidence_type_ == CONFIDENCE_16BIT)
        split_<IntensityT_, uint16_t>(fields);
    }

  void getOffsets_(int & x_offset,
                   int & y_offset,
                   int & z_offset,
                   int & intensity_offset,
                   int & confidence_offset) const;

  template <typename T_>
    struct Convert
    {
      static inline T_ fromFloat(const float & f)
      {
        return static_cast<T_>(f);
      }
    };

//  template <typename T_>
//    struct Normalize
//    {
//      static inline T_ fromFloat(const float & f)
//      {
//        return static_cast<T_>(f);
//      }
//    };

  template <typename IntensityT_, typename ConfidenceT_>
    void split_(unsigned int fields = ALL);

  sensor_msgs::PointCloud2::ConstPtr swiss_ranger_msg_;
  float confidence_threshold_;
  const float bad_point_;

  IntensityType intensity_type_;
  ConfidenceType confidence_type_;
  bool normalize_intensity_;

  unsigned int split_fields_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  cv::Mat intensity_image_;
  cv::Mat confidence_image_;

};

template <>
  struct Utility::Convert<uint8_t>
  {
    static inline uint8_t fromFloat(const float & f)
    {
      return static_cast<uint8_t>(f / 257);
    }
  };

//template <>
//  struct Utility::Normalize<uint8_t>
//  {
//    static inline uint8_t fromFloat(const float & f)
//    {
//      return static_cast<uint8_t>(f * 255);
//    }
//  };

//template <>
//  struct Utility::Normalize<uint16_t>
//  {
//    static inline uint16_t fromFloat(const float & f)
//    {
//      return static_cast<uint16_t>(f * 65535);
//    }
//  };

template <typename IntensityT_, typename ConfidenceT_>
  void Utility::split_(unsigned int fields)
  {
    split_fields_ = fields;
    if (not (split_fields_ & ALL))
      return;

    const size_t height = swiss_ranger_msg_->height;
    const size_t width = swiss_ranger_msg_->width;
    const size_t step = swiss_ranger_msg_->point_step;

    int x_offset, y_offset, z_offset, intensity_offset, confidence_offset;
    getOffsets_(x_offset, y_offset, z_offset, intensity_offset, confidence_offset);

    if (split_fields_ & CLOUD)
    {
      cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
      cloud_->header = pcl_conversions::toPCL(swiss_ranger_msg_->header);
      cloud_->points.resize(height * width);
      cloud_->height = height;
      cloud_->width = width;
      cloud_->is_dense = true;
    }

    if (split_fields_ & INTENSITY)
      intensity_image_ = cv::Mat_<IntensityT_>(height, width);

    if (split_fields_ & CONFIDENCE)
      confidence_image_ = cv::Mat_<ConfidenceT_>(height, width);

    for (size_t i = 0; i < height; ++i)
    {
      for (size_t j = 0; j < width; ++j)
      {
        size_t index = (j + i * width) * step;
        const float * x_ptr = reinterpret_cast<const float *>(&swiss_ranger_msg_->data[index + x_offset]);
        const float * y_ptr = reinterpret_cast<const float *>(&swiss_ranger_msg_->data[index + y_offset]);
        const float * z_ptr = reinterpret_cast<const float *>(&swiss_ranger_msg_->data[index + z_offset]);
        const float * intensity_ptr = reinterpret_cast<const float *>(&swiss_ranger_msg_->data[index + intensity_offset]);
        const float * confidence_ptr = reinterpret_cast<const float *>(&swiss_ranger_msg_->data[index + confidence_offset]);

        if (split_fields_ & CLOUD)
        {
          pcl::PointXYZ & point = (*cloud_)(j, i);
          if (*confidence_ptr >= confidence_threshold_)
          {
            point.x = *x_ptr;
            point.y = *y_ptr;
            point.z = *z_ptr;
          }
          else
          {
            point.x = bad_point_;
            point.y = bad_point_;
            point.z = bad_point_;
            cloud_->is_dense = false;
          }
        }
        if (split_fields_ & INTENSITY)
        {
          intensity_image_.at<IntensityT_>(i, j) = Convert<IntensityT_>::fromFloat(*intensity_ptr);
//          if (intensity_type_ == INTENSITY_8BIT)
//            intensity_image_.at<IntensityT_>(i, j) = static_cast<IntensityT_>(*intensity_ptr / 257.0f);
//          else
//            intensity_image_.at<IntensityT_>(i, j) = static_cast<IntensityT_>(*intensity_ptr);
        }
        if (split_fields_ & CONFIDENCE)
        {
//          if (*confidence_ptr >= confidence_threshold_)
//          {
            confidence_image_.at<ConfidenceT_>(i, j) = Convert<ConfidenceT_>::fromFloat(*confidence_ptr);
//          }
//          else
//          {
//            confidence_image_.at<ConfidenceT_>(i, j) = static_cast<ConfidenceT_>(0);
//          }
        }
      }
    }

    if ((split_fields_ & INTENSITY) and normalize_intensity_ and (intensity_type_ == INTENSITY_8BIT))
    {
//      double min_val, max_val;
//      cv::Point min_loc, max_loc;
//      cv::minMaxLoc(intensity_image_, &min_val, &max_val, &min_loc, &max_loc, confidence_image_);

//      // Rescale image intensity:
//      for (size_t i = 0; i < height; ++i)
//      {
//        for (size_t j = 0; j < width; ++j)
//        {
//          IntensityT_ & val = intensity_image_.at<IntensityT_>(i, j);
//          val = Normalize<IntensityT_>::fromFloat((val - min_val) / (max_val - min_val));
////          if (intensity_type_ == INTENSITY_FLOAT)
////            val = static_cast<IntensityT_>((val - min_val) / (max_val - min_val));
////          else if (intensity_type_ == INTENSITY_8BIT)
////            val = static_cast<IntensityT_>((val - min_val) / (max_val - min_val) * 255);
////          else //(intensity_type_ == INTENSITY_16BIT)
////            val = static_cast<IntensityT_>((val - min_val) / (max_val - min_val) * MAX_INTENSITY);
//        }
//      }

      cv::equalizeHist(intensity_image_, intensity_image_);

    }
    else if ((split_fields_ & INTENSITY) and (intensity_type_ == INTENSITY_FLOAT))
    {
      for (size_t i = 0; i < height; ++i)
      {
        for (size_t j = 0; j < width; ++j)
        {
          IntensityT_ & val = intensity_image_.at<IntensityT_>(i, j);
          val = static_cast<IntensityT_>(val / MAX_INTENSITY);
        }
      }
    }

    if ((split_fields_ & CONFIDENCE) and (confidence_type_ == CONFIDENCE_FLOAT))
    {
      for (size_t i = 0; i < height; ++i)
      {
        for (size_t j = 0; j < width; ++j)
        {
          ConfidenceT_ & val = confidence_image_.at<ConfidenceT_>(i, j);
          val = static_cast<ConfidenceT_>(val / MAX_CONFIDENCE);
        }
      }
    }

  }


} /* namespace sr */

#endif /* SWISSRANGER_CAMERA_UTILITY_H_ */
