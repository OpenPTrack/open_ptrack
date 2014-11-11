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

#include <sstream>

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

#include <open_ptrack/opt_calibration/opt_checkerboard_extraction.h>


namespace open_ptrack
{
namespace opt_calibration
{

namespace cb = calibration;

class TreeNode
{

public:

  typedef boost::shared_ptr<TreeNode> Ptr;
  typedef boost::shared_ptr<const TreeNode> ConstPtr;

  enum Type
  {
    INTENSITY,
    DEPTH,
  };

  static const int MAX_LEVEL;
  static const double MAX_ERROR;

  TreeNode(const cb::Sensor::Ptr & sensor,
           size_t id,
           Type type)
    : sensor_(sensor),
      type_(type),
      id_(id),
      level_(MAX_LEVEL),
      min_error_(MAX_ERROR),
      estimate_pose_(true)
  {
    // Do nothing
  }

  const cb::Sensor::Ptr & sensor() const
  {
    return sensor_;
  }

  void setSensor(const cb::Sensor::Ptr & sensor)
  {
    sensor_ = sensor;
  }

  int level() const
  {
    return level_;
  }

  void setLevel(int level)
  {
    level_ = level;
  }

  double minError() const
  {
    return min_error_;
  }

  void setMinError(double error)
  {
    min_error_ = error;
  }

  size_t id() const
  {
    return id_;
  }

  void setEstimatePose(bool estimate_pose)
  {
    estimate_pose_ = estimate_pose;
  }

  bool estimatePose() const
  {
    return estimate_pose_;
  }

  Type type() const
  {
    return type_;
  }

private:

  cb::Sensor::Ptr sensor_;
  Type type_;
  size_t id_;

  int level_;
  double min_error_;

  bool estimate_pose_;

};

class OPTCalibration
{
public:

  typedef boost::shared_ptr<OPTCalibration> Ptr;
  typedef boost::shared_ptr<const OPTCalibration> ConstPtr;

  struct CheckerboardView
  {
    typedef boost::shared_ptr<CheckerboardView> Ptr;
    typedef boost::shared_ptr<const CheckerboardView> ConstPtr;

    CheckerboardView() {}

    CheckerboardView(const cb::View::Ptr & view,
                     const cb::PlanarObject::Ptr & object,
                     const cb::Point3 & center,
                     bool is_floor)
      : view(view),
        object(object),
        center(center),
        is_floor(is_floor)
    {
    }

    cb::View::Ptr view;
    cb::PlanarObject::Ptr object;
    cb::Point3 center;
    bool is_floor;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  OPTCalibration(const ros::NodeHandle & node_handle);

  inline void setCheckerboard(const cb::Checkerboard::Ptr & checkerboard)
  {
    checkerboard_ = checkerboard;
  }

  void addSensor(const cb::PinholeSensor::Ptr & sensor,
                 bool estimate_pose = true);

  void addSensor(const cb::DepthSensor::Ptr & sensor,
                 bool estimate_pose = true);

  inline void nextAcquisition()
  {
    view_map_vec_.push_back(ViewMap());
  }

  bool analyzeData(const cb::PinholeSensor::Ptr & color_sensor,
                   const cb::DepthSensor::Ptr & depth_sensor,
                   const cv::Mat & image,
                   const cb::PCLCloud3::Ptr & cloud,
                   CheckerboardView::Ptr & color_cb_view,
                   CheckerboardView::Ptr & depth_cb_view);

  bool analyzeData(const cb::PinholeSensor::Ptr & color_sensor,
                   const cv::Mat & image,
                   CheckerboardView::Ptr & color_cb_view);

  inline void addData(const cb::Sensor::Ptr & sensor,
                      const CheckerboardView::Ptr & cb_view)
  {
    TreeNode::Ptr & node = node_map_.at(sensor);
    ViewMap & view_map = view_map_vec_.back();
    view_map[node] = cb_view;
    if (floorAcquisition())
      floor_estimated_ = false;
  }

  inline void startFloorAcquisition()
  {
    assert(not floor_acquisition_);
    floor_acquisition_ = true;
  }

  inline void stopFloorAcquisition()
  {
    assert(floor_acquisition_);
    floor_acquisition_ = false;
  }

  inline bool floorAcquisition() const
  {
    return floor_acquisition_;
  }

  void perform();
  void publish();
  void optimize();
  const cb::Pose & getLastCheckerboardPose() const;

private:

  bool estimateFloor();
  void convertToWorldFrame();


  ros::NodeHandle node_handle_;
  tf::TransformBroadcaster tf_pub_;
  ros::Publisher marker_pub_;

  std::map<cb::Sensor::ConstPtr, TreeNode::Ptr> node_map_;
  std::vector<TreeNode::Ptr> node_vec_;
  std::vector<cb::Sensor::Ptr> sensor_vec_;

  cb::Checkerboard::Ptr checkerboard_;
  cb::BaseObject::Ptr world_;

  typedef std::map<TreeNode::Ptr, CheckerboardView::Ptr> ViewMap;
  std::vector<ViewMap> view_map_vec_;

  std::vector<cb::Checkerboard::Ptr> checkerboard_vec_;
  std::vector<bool> is_floor_vec_;

  bool tree_initialized_;
  bool initialization_;
  int last_optimization_;

  bool floor_acquisition_;
  bool floor_estimated_;
  cb::Plane floor_;

};

} /* namespace opt_calibration */
} /* namespace open_ptrack */

#endif /* OPEN_PTRACK_OPT_CALIBRATION_OPT_CALIBRATION_H_ */
