/*
 *  Copyright (c) 2013- Filippo Basso, Riccardo Levorato, Matteo Munaro
 *  Copyright (c) 2014-, Open Perception, Inc.
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
 *
 *  Author: Filippo Basso [bassofil@dei.unipd.it]
 *          Riccardo Levorato [levorato@dei.unipd.it]
 *          Matteo Munaro [matteo.munaro@dei.unipd.it]
 */

#ifndef OPEN_PTRACK_OPT_CALIBRATION_OPT_CHECKERBOARD_EXTRACTION_H_
#define OPEN_PTRACK_OPT_CALIBRATION_OPT_CHECKERBOARD_EXTRACTION_H_

#include <open_ptrack/opt_calibration/ros_device.h>

namespace calibration
{
class PlanarObject;
class Checkerboard;

template <typename ObjectT_>
  class PinholeView;

template <typename ObjectT_>
  class DepthViewPCL;

}

namespace open_ptrack
{
namespace opt_calibration
{

namespace cb = calibration;

class OPTCheckerboardExtraction
{
public:

  enum Status
  {
    EXTRACTED_NONE,
    EXTRACTED_COLOR,
    EXTRACTED_COLOR_AND_DEPTH,
  };

  OPTCheckerboardExtraction()
    : depth_transform_(Eigen::Affine3d::Identity())
  {
    // Do nothing
  }

  inline void setImage(const cv::Mat & image)
  {
    image_ = image;
  }

  inline void setCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud)
  {
    cloud_ = cloud;
  }

  inline void setColorSensor(const boost::shared_ptr<const cb::PinholeSensor> & sensor)
  {
    color_sensor_= sensor;
  }

  inline void setDepthSensor(const boost::shared_ptr<const cb::DepthSensor> & sensor)
  {
    depth_sensor_= sensor;
  }

  inline void setCheckerboard(const boost::shared_ptr<const cb::Checkerboard> & checkerboard)
  {
    checkerboard_ = checkerboard;
  }

  inline void setDepthTransform(const Eigen::Affine3d & depth_transform)
  {
    depth_transform_ = depth_transform;
  }

  Status perform(boost::shared_ptr<cb::PinholeView<cb::Checkerboard> > & color_view,
                 boost::shared_ptr<cb::Checkerboard> & extracted_checkerboard) const;

  Status perform(boost::shared_ptr<cb::PinholeView<cb::Checkerboard> > & color_view,
                 boost::shared_ptr<cb::DepthViewPCL<cb::PlanarObject> > & depth_view,
                 boost::shared_ptr<cb::Checkerboard> & extracted_checkerboard,
                 boost::shared_ptr<cb::PlanarObject> & extracted_plane) const;

private:

  cv::Mat image_;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_;

  boost::shared_ptr<const cb::PinholeSensor> color_sensor_;
  boost::shared_ptr<const cb::DepthSensor> depth_sensor_;
  Eigen::Affine3d depth_transform_;

  boost::shared_ptr<const cb::Checkerboard> checkerboard_;

};

} /* namespace opt_calibration */
} /* namespace open_ptrack */

#endif /* OPEN_PTRACK_OPT_CALIBRATION_OPT_CHECKERBOARD_EXTRACTION_H_ */
