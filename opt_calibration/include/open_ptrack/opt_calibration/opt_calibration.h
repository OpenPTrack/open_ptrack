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

#ifndef OPEN_PTRACK_OPT_CALIBRATION_OPT_CALIBRATION_H_
#define OPEN_PTRACK_OPT_CALIBRATION_OPT_CALIBRATION_H_

#include <ros/ros.h>
#include <ros/package.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <calibration_common/calibration_common.h>
#include <kinect/depth/depth.h>

#include <camera_info_manager/camera_info_manager.h>

#include <sstream>

using namespace camera_info_manager;
using namespace calibration;

namespace open_ptrack
{
namespace opt_calibration
{

typedef PolynomialMatrixProjectedModel<Polynomial<Scalar, 2, 0> > MatrixModel;
typedef TwoStepsModel<Scalar, MatrixModel, MatrixModel> UndistortionModel;
typedef DepthUndistortionImpl<UndistortionModel, DepthPCL> UndistortionPCL;

struct SensorNode
{
  typedef boost::shared_ptr<SensorNode> Ptr;
  typedef boost::shared_ptr<const SensorNode> ConstPtr;

  enum SensorType
  {
    KINECT_DEPTH,
    PINHOLE_RGB
  };

  static const int MAX_LEVEL;
  static const double MAX_DISTANCE;
  static const double MAX_ERROR;

  SensorNode(const Sensor::Ptr & sensor,
             SensorType type,
             size_t id)
    : sensor_(sensor),
      type_(type),
      level_(MAX_LEVEL),
      distance_(MAX_DISTANCE),
      min_error_(MAX_ERROR),
      id_(id)
  {
    // Do nothing
  }

  Sensor::Ptr sensor_;
  SensorType type_;

  int level_;
  double distance_;
  double min_error_;

  size_t id_;

  SensorNode::Ptr connected_sensor_;
  Pose connected_pose_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

class OPTCalibration
{
public:

  typedef boost::shared_ptr<OPTCalibration> Ptr;
  typedef boost::shared_ptr<const OPTCalibration> ConstPtr;

  OPTCalibration(const ros::NodeHandle & node_handle);

  inline void setCheckerboard(const Checkerboard::Ptr & checkerboard)
  {
    checkerboard_ = checkerboard;
  }

  inline void addSensor(const PinholeSensor::Ptr & sensor)
  {
    SensorNode::Ptr sensor_node = boost::make_shared<SensorNode>(sensor, SensorNode::PINHOLE_RGB, sensor_map_.size());
    sensor_map_[sensor] = sensor_node;
    sensor_vec_.push_back(sensor_node);
  }

  inline void nextAcquisition()
  {
    view_vec_.push_back(ViewMap());
  }

  bool addData(const PinholeSensor::Ptr & sensor,
               const cv::Mat & image)
  {
    PinholeView<Checkerboard>::Ptr color_view;
    Checkerboard::Ptr checkerboard;

    if (findCheckerboard(image, sensor, color_view, checkerboard))
    {
      addData(sensor, color_view, checkerboard, checkerboard->center());
      return true;
    }

    return false;
  }

  inline void addData(const Sensor::Ptr & sensor,
                      const View::Ptr & view,
                      const PlanarObject::Ptr & object,
                      const Point3 & center)
  {
    SensorNode::Ptr & sensor_node = sensor_map_.at(sensor);
    ViewMap & view_map = view_vec_.back();
    view_map[sensor_node] = boost::make_shared<CheckerboardView>(view, object, center);
  }

  void perform();
  void publish();
  void optimize();
  const Pose & getLastCheckerboardPose() const;

private:

  bool findCheckerboard(const cv::Mat & image,
                        const PinholeSensor::Ptr & sensor,
                        PinholeView<Checkerboard>::Ptr & color_view,
                        Checkerboard::Ptr & checkerboard);

  ros::NodeHandle node_handle_;

  std::map<Sensor::ConstPtr, SensorNode::Ptr> sensor_map_;
  std::vector<SensorNode::Ptr> sensor_vec_;
  Checkerboard::Ptr checkerboard_;

  AutomaticCheckerboardFinder finder_;

  BaseObject::Ptr world_;

  tf::TransformBroadcaster tf_pub_;
  ros::Publisher marker_pub_;

  bool world_set_;

  bool calibration_with_serials_;

  struct CheckerboardView
  {
    typedef boost::shared_ptr<CheckerboardView> Ptr;
    typedef boost::shared_ptr<const CheckerboardView> ConstPtr;

    CheckerboardView() {}

    CheckerboardView(const View::Ptr & view,
                     const PlanarObject::Ptr & object,
                     const Point3 & center)
      : view_(view),
        object_(object),
        center_(center)
    {
    }

    View::Ptr view_;
    PlanarObject::Ptr object_;
    Point3 center_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  typedef std::map<SensorNode::Ptr, CheckerboardView::Ptr> ViewMap;
  std::vector<ViewMap> view_vec_;

  std::vector<Checkerboard::Ptr> checkerboard_vec_;

  bool initialization_;
  int last_optimization_;

};

} /* namespace opt_calibration */
} /* namespace open_ptrack */

#endif /* OPEN_PTRACK_OPT_CALIBRATION_OPT_CALIBRATION_H_ */
