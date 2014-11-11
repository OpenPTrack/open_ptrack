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

#include <fstream>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

#include <ceres/ceres.h>

#include <open_ptrack/opt_calibration/opt_calibration.h>

#define OPTIMIZATION_COUNT 25

namespace open_ptrack
{
namespace opt_calibration
{

const int TreeNode::MAX_LEVEL = 10000;
const double TreeNode::MAX_ERROR = 10000.0;

OPTCalibration::OPTCalibration(const ros::NodeHandle & node_handle)
  : node_handle_(node_handle),
    world_(boost::make_shared<cb::BaseObject>("/world")),
    tree_initialized_(false),
    initialization_(true),
    last_optimization_(OPTIMIZATION_COUNT),
    floor_acquisition_(false),
    floor_estimated_(false)
{
  marker_pub_ = node_handle_.advertise<visualization_msgs::Marker>("markers", 0);
}

void OPTCalibration::addSensor(const cb::PinholeSensor::Ptr & sensor,
                               bool estimate_pose)
{
  sensor_vec_.push_back(sensor);
  TreeNode::Ptr node = boost::make_shared<TreeNode>(sensor, sensor_vec_.size() - 1, TreeNode::INTENSITY);
  node_map_[sensor] = node;
  node_vec_.push_back(node);
  node->setEstimatePose(estimate_pose);
}

void OPTCalibration::addSensor(const cb::DepthSensor::Ptr & sensor,
                               bool estimate_pose)
{
  assert((estimate_pose and sensor->parent()) or (not estimate_pose));
  sensor_vec_.push_back(sensor);
  TreeNode::Ptr node = boost::make_shared<TreeNode>(sensor, sensor_vec_.size() - 1, TreeNode::DEPTH);
  node_map_[sensor] = node;
  node_vec_.push_back(node);
  node->setEstimatePose(estimate_pose);
}

bool OPTCalibration::analyzeData(const cb::PinholeSensor::Ptr & color_sensor,
                                 const cb::DepthSensor::Ptr & depth_sensor,
                                 const cv::Mat & image,
                                 const cb::PCLCloud3::Ptr & cloud,
                                 CheckerboardView::Ptr & color_cb_view,
                                 CheckerboardView::Ptr & depth_cb_view)
{
  OPTCheckerboardExtraction ex;
  ex.setImage(image);
  ex.setCloud(cloud);
  ex.setColorSensor(color_sensor);
  ex.setDepthSensor(depth_sensor);
  ex.setDepthTransform(depth_sensor->pose());
  ex.setCheckerboard(checkerboard_);

  cb::PinholeView<cb::Checkerboard>::Ptr color_view;
  cb::DepthViewPCL<cb::PlanarObject>::Ptr depth_view;
  cb::Checkerboard::Ptr extracted_checkerboard;
  cb::PlanarObject::Ptr extracted_plane;
  if (ex.perform(color_view, depth_view, extracted_checkerboard, extracted_plane))
  {
    visualization_msgs::Marker checkerboard_marker;
    checkerboard_marker.ns = "checkerboard";
    checkerboard_marker.id = node_map_[color_sensor]->id();
    extracted_checkerboard->toMarker(checkerboard_marker);
    marker_pub_.publish(checkerboard_marker);

    visualization_msgs::Marker plane_marker;
    plane_marker.ns = "plane";
    plane_marker.id = node_map_[color_sensor]->id();
    extracted_plane->toMarker(plane_marker);
    marker_pub_.publish(plane_marker);

    geometry_msgs::TransformStamped transform_msg;
    extracted_checkerboard->toTF(transform_msg);
    tf_pub_.sendTransform(transform_msg);
    extracted_plane->toTF(transform_msg);
    tf_pub_.sendTransform(transform_msg);

    color_cb_view = boost::make_shared<CheckerboardView>(color_view,
                                                         extracted_checkerboard,
                                                         extracted_checkerboard->center(),
                                                         floorAcquisition());

    depth_cb_view = boost::make_shared<CheckerboardView>(depth_view,
                                                         extracted_plane,
                                                         depth_view->centroid(),
                                                         floorAcquisition());

//    analyzeData(color_sensor, color_view, extracted_checkerboard, extracted_checkerboard->center());
//    analyzeData(depth_sensor, depth_view, extracted_plane, depth_view->centroid());

    return true;
  }
  return false;
}

bool OPTCalibration::analyzeData(const cb::PinholeSensor::Ptr & color_sensor,
                                 const cv::Mat & image,
                                 CheckerboardView::Ptr & color_cb_view)
{
  OPTCheckerboardExtraction ex;
  ex.setImage(image);
  ex.setColorSensor(color_sensor);
  ex.setCheckerboard(checkerboard_);

  cb::PinholeView<cb::Checkerboard>::Ptr color_view;
  cb::Checkerboard::Ptr extracted_checkerboard;
  if (ex.perform(color_view, extracted_checkerboard))
  {
    visualization_msgs::Marker checkerboard_marker;
    checkerboard_marker.ns = "checkerboard";
    checkerboard_marker.id = node_map_[color_sensor]->id();
    extracted_checkerboard->toMarker(checkerboard_marker);
    marker_pub_.publish(checkerboard_marker);

    geometry_msgs::TransformStamped transform_msg;
    extracted_checkerboard->toTF(transform_msg);
    tf_pub_.sendTransform(transform_msg);

    color_cb_view = boost::make_shared<CheckerboardView>(color_view,
                                                         extracted_checkerboard,
                                                         extracted_checkerboard->center(),
                                                         floorAcquisition());
//    analyzeData(color_sensor, color_view, extracted_checkerboard, extracted_checkerboard->center());

    return true;
  }
  return false;
}

void OPTCalibration::perform()
{
  const ViewMap & view_map = view_map_vec_.back();

  if (initialization_)
  {
    if (not tree_initialized_ and not view_map.empty()) // Set /world
    {
      ViewMap::const_iterator it = view_map.begin();

      while (it != view_map.end() and it->first->type() == TreeNode::DEPTH)
        ++it;

      if (it == view_map.end())
      {
        view_map_vec_.resize(view_map_vec_.size() - 1);
        return;
      }

      TreeNode::Ptr tree_node = it->first;
      tree_node->sensor()->setParent(world_);
      tree_node->setLevel(0);

      ROS_INFO_STREAM(tree_node->sensor()->frameId() << " added to the tree.");
      tree_initialized_ = true;
    }

    if (view_map.empty())
    {
      view_map_vec_.resize(view_map_vec_.size() - 1); // Remove data
    }
    else if (view_map.size() >= 2) // At least 2 cameras
    {
      int min_level = TreeNode::MAX_LEVEL;
      TreeNode::Ptr min_sensor_node;
      for (ViewMap::const_iterator it = view_map.begin(); it != view_map.end(); ++it)
      {
        TreeNode::Ptr tree_node = it->first;
        if (tree_node->level() < min_level)
        {
          min_level = tree_node->level();
          min_sensor_node = tree_node;
        }
      }

      if (min_level < TreeNode::MAX_LEVEL) // At least one already in tree
      {
        for (ViewMap::const_iterator it = view_map.begin(); it != view_map.end(); ++it)
        {
          TreeNode::Ptr tree_node = it->first;

          if (tree_node != min_sensor_node)
          {
            const CheckerboardView::Ptr & view = it->second;
            cb::PlanarObject::Ptr planar_object = view->object;
            cb::Point3 center = view->center;

            if (tree_node->type() == TreeNode::INTENSITY)
            {
              cb::Scalar error = std::abs(planar_object->plane().normal().dot(cb::Vector3::UnitZ())) * center.squaredNorm();

              if (tree_node->level() > min_level and error < tree_node->minError())
              {
                CheckerboardView::Ptr min_checkerboard_view = view_map.at(min_sensor_node);
                cb::PlanarObject::Ptr min_planar_object = min_checkerboard_view->object;

                if (tree_node->level() == TreeNode::MAX_LEVEL)
                  ROS_INFO_STREAM(tree_node->sensor()->frameId() << " added to the tree.");

                tree_node->setMinError(error);
                tree_node->setLevel(min_level + 1);

                tree_node->sensor()->setParent(min_sensor_node->sensor());
                tree_node->sensor()->setPose(min_planar_object->pose() * planar_object->pose().inverse());
              }
            }
          }
        }       
      }

      initialization_ = (view_map_vec_.size() < OPTIMIZATION_COUNT);
      for (int i = 0; not initialization_ and i < node_vec_.size(); ++i)
      {
        TreeNode::Ptr & tree_node = node_vec_[i];
        if (tree_node->type() == TreeNode::INTENSITY and tree_node->level() == TreeNode::MAX_LEVEL)
          initialization_ = true;
      }
      
      if (not initialization_)
        ROS_INFO("All cameras added to the tree. Now calibrate the global reference frame and save!");

    }
  }
  else
  {
    if (view_map.empty())
    {
      view_map_vec_.resize(view_map_vec_.size() - 1); // Remove data
    }
    else if (view_map.size() >= 2) // At least 2 cameras
    {
      if (last_optimization_ == 0)
      {
        optimize();
        last_optimization_ = OPTIMIZATION_COUNT;
      }
      else
        last_optimization_--;
    }
  }
}

void OPTCalibration::publish()
{
  for (size_t i = 0; i < node_vec_.size(); ++i)
  {
    TreeNode::Ptr sensor_node = node_vec_[i];
    geometry_msgs::TransformStamped transform_msg;
    if (sensor_node->sensor()->toTF(transform_msg))
      tf_pub_.sendTransform(transform_msg);
  }

  for (size_t i = 0; i < checkerboard_vec_.size(); ++i)
  {
    cb::Checkerboard::Ptr checkerboard = checkerboard_vec_[i];
    visualization_msgs::Marker marker;
    checkerboard->toMarker(marker);
    marker.ns = "optimized checkerboard";
    marker.id = i;
    marker_pub_.publish(marker);
  }

  if (floor_estimated_)
  {
    cb::Vector3 floor_origin = -floor_.normal() * floor_.offset();
    cb::Vector3 floor_x = (floor_.projection(floor_origin + cb::Vector3::UnitX()) - floor_origin).normalized();

    cb::Transform floor_pose;
    floor_pose.linear().col(2) = floor_.normal();
    floor_pose.linear().col(0) = floor_x;
    floor_pose.linear().col(1) = floor_pose.linear().col(2).cross(floor_pose.linear().col(0));
    floor_pose.translation() = floor_origin;

    cb::PlanarObject plane("/floor");
    plane.transform(floor_pose);
    plane.setParent(world_);

    visualization_msgs::Marker marker;
    plane.toMarker(marker);
    marker.ns = "floor";
    marker.id = 0;
    marker.scale.x = 10;
    marker.scale.y = 10;
    marker_pub_.publish(marker);

    geometry_msgs::TransformStamped transform_msg;
    if (plane.toTF(transform_msg))
      tf_pub_.sendTransform(transform_msg);
  }

}

bool OPTCalibration::estimateFloor()
{
  cb::Cloud3 cloud(cb::Size2(checkerboard_->corners().elements(), is_floor_vec_.size()));
  size_t size = 0;
  for (size_t i = 0; i < is_floor_vec_.size(); ++i)
  {
    if (is_floor_vec_[i])
    {
      const cb::Checkerboard & checkerboard = *checkerboard_vec_[i];
      for (size_t j = 0; j < checkerboard_->corners().elements(); ++j)
        cloud(j, size) = checkerboard[j];
      ++size;
    }
  }
  if (size < 1)
    return false;

  cloud.resize(cb::Size2(checkerboard_->corners().elements(), size));
  floor_ = cb::PlaneFit<cb::Scalar>::fit(cloud);
  if (floor_.offset() < 0)
    floor_.coeffs() = -floor_.coeffs();
  return true;

}

void OPTCalibration::convertToWorldFrame()
{
  for (size_t i = 0; i < node_vec_.size(); ++i)
  {
    const TreeNode & sensor_node = *node_vec_[i];
    cb::Pose pose = sensor_node.sensor()->pose();
    cb::BaseObject::ConstPtr parent = sensor_node.sensor()->parent();

    while (parent->parent())
    {
      pose = parent->pose() * pose;
      parent = parent->parent();
      sensor_node.sensor()->setParent(parent);
      sensor_node.sensor()->setPose(pose);
    }

    //ROS_INFO_STREAM(std::endl << sensor_node.sensor()->pose().matrix());
  }

  for (size_t i = checkerboard_vec_.size(); i < view_map_vec_.size(); ++i)
  {
    ViewMap & view_map = view_map_vec_[i];

    ViewMap::iterator it = view_map.begin();
    bool ok = false;
    while (not ok and it != view_map.end())
    {
      if (it->first->type() == TreeNode::DEPTH)
        ++it;
      else
        ok = true;
    }

    const CheckerboardView::Ptr & cb_view = it->second;
    const TreeNode::Ptr & sensor_node = it->first;

    cb::Checkerboard::Ptr cb = boost::make_shared<cb::Checkerboard>(*boost::static_pointer_cast<cb::Checkerboard>(cb_view->object));

    cb::BaseObject::ConstPtr parent = sensor_node->sensor();
    while (parent->parent())
    {
      cb->transform(parent->pose());
      parent = parent->parent();
    }
    cb->setParent(parent);

    checkerboard_vec_.push_back(cb);
    is_floor_vec_.push_back(cb_view->is_floor);

  }
}

class RootPinholeError
{
public:

  RootPinholeError(const cb::PinholeCameraModel::ConstPtr & camera_model,
                   const cb::Checkerboard::ConstPtr & checkerboard,
                   const cb::Cloud2 & image_corners)
    : camera_model_(camera_model),
      checkerboard_(checkerboard),
      image_corners_(image_corners)
  {
  }

  template <typename T>
    bool operator ()(const T * const checkerboard_pose,
                     T * residuals) const
    {
      typename cb::Types<T>::Vector3 checkerboard_r_vec(checkerboard_pose[0], checkerboard_pose[1], checkerboard_pose[2]);
      typename cb::Types<T>::AngleAxis checkerboard_r(checkerboard_r_vec.norm(), checkerboard_r_vec.normalized());
      typename cb::Types<T>::Translation3 checkerboard_t(checkerboard_pose[3], checkerboard_pose[4], checkerboard_pose[5]);

      typename cb::Types<T>::Transform checkerboard_pose_eigen = checkerboard_t * checkerboard_r;

      typename cb::Types<T>::Cloud3 cb_corners(cb::Size2(checkerboard_->cols(), checkerboard_->rows()));
      cb_corners.container() = checkerboard_pose_eigen * checkerboard_->corners().container().cast<T>();

      typename cb::Types<T>::Cloud2 reprojected_corners = camera_model_->project3dToPixel<T>(cb_corners);

      for (cb::Size1 i = 0; i < cb_corners.elements(); ++i)
        residuals[i] = T((reprojected_corners[i] - image_corners_[i].cast<T>()).norm() / 0.5);

      return true;
    }

private:

  const cb::PinholeCameraModel::ConstPtr camera_model_;
  const cb::Checkerboard::ConstPtr checkerboard_;
  const cb::Cloud2 image_corners_;

};

class RootPinholeFloorError
{
public:

  RootPinholeFloorError(const cb::PinholeCameraModel::ConstPtr & camera_model,
                        const cb::Checkerboard::ConstPtr & checkerboard,
                        const cb::Cloud2 & image_corners)
    : camera_model_(camera_model),
      checkerboard_(checkerboard),
      image_corners_(image_corners)
  {
  }

  template <typename T>
    bool operator ()(const T * const checkerboard_pose,
                     const T * const plane_origin,
                     T * residuals) const
    {
      typename cb::Types<T>::Translation2 checkerboard_t(checkerboard_pose[0], checkerboard_pose[1]);
      typename cb::Types<T>::Rotation2 checkerboard_r(checkerboard_pose[2]);
      typename cb::Types<T>::Transform2 checkerboard_pose_eigen = checkerboard_t * checkerboard_r;

      typename cb::Types<T>::Vector3 plane_origin_eigen(plane_origin[0], plane_origin[1], plane_origin[2]);
      typename cb::Types<T>::Plane plane(-plane_origin_eigen.normalized(), plane_origin_eigen.norm());

      typename cb::Types<T>::Transform plane_pose;
      plane_pose.linear().col(2) = plane.normal();
      plane_pose.linear().col(0) = (plane.projection(plane_origin_eigen + cb::Types<T>::Vector3::UnitX()) - plane_origin_eigen).normalized();
      plane_pose.linear().col(1) = plane_pose.linear().col(2).cross(plane_pose.linear().col(0));
      plane_pose.translation() = plane_origin_eigen;

      typename cb::Types<T>::Cloud3 cb_corners(cb::Size2(checkerboard_->cols(), checkerboard_->rows()), cb::Types<T>::Point3::Zero());
      cb_corners.container().template topRows<2>() = checkerboard_pose_eigen * checkerboard_->corners().container().template topRows<2>().cast<T>();
      cb_corners.container() = plane_pose * cb_corners.container();

      typename cb::Types<T>::Cloud2 reprojected_corners = camera_model_->project3dToPixel<T>(cb_corners);

      for (cb::Size1 i = 0; i < cb_corners.elements(); ++i)
        residuals[i] = T((reprojected_corners[i] - image_corners_[i].cast<T>()).norm() / 0.5);

      return true;
    }

private:

  const cb::PinholeCameraModel::ConstPtr camera_model_;
  const cb::Checkerboard::ConstPtr checkerboard_;
  const cb::Cloud2 image_corners_;

};

class PinholeError
{
public:

  PinholeError(const cb::PinholeCameraModel::ConstPtr & camera_model,
               const cb::Checkerboard::ConstPtr & checkerboard,
               const cb::Cloud2 & image_corners)
    : camera_model_(camera_model),
      checkerboard_(checkerboard),
      image_corners_(image_corners)
  {
  }

  template <typename T>
    bool operator ()(const T * const sensor_pose,
                     const T * const checkerboard_pose,
                     T * residuals) const
    {
      typename cb::Types<T>::Vector3 sensor_r_vec(sensor_pose[0], sensor_pose[1], sensor_pose[2]);
      typename cb::Types<T>::AngleAxis sensor_r(sensor_r_vec.norm(), sensor_r_vec.normalized());
      typename cb::Types<T>::Translation3 sensor_t(sensor_pose[3], sensor_pose[4], sensor_pose[5]);

      typename cb::Types<T>::Transform sensor_pose_eigen = cb::Types<T>::Transform::Identity() * sensor_t;

      if (sensor_r_vec.norm() != T(0))
        sensor_pose_eigen = sensor_t * sensor_r;

      typename cb::Types<T>::Vector3 checkerboard_r_vec(checkerboard_pose[0], checkerboard_pose[1], checkerboard_pose[2]);
      typename cb::Types<T>::AngleAxis checkerboard_r(checkerboard_r_vec.norm(), checkerboard_r_vec.normalized());
      typename cb::Types<T>::Translation3 checkerboard_t(checkerboard_pose[3], checkerboard_pose[4], checkerboard_pose[5]);

      typename cb::Types<T>::Transform checkerboard_pose_eigen = checkerboard_t * checkerboard_r;

      typename cb::Types<T>::Cloud3 cb_corners(cb::Size2(checkerboard_->cols(), checkerboard_->rows()));
      cb_corners.container() = sensor_pose_eigen.inverse() * checkerboard_pose_eigen
                               * checkerboard_->corners().container().cast<T>();

      typename cb::Types<T>::Cloud2 reprojected_corners = camera_model_->project3dToPixel<T>(cb_corners);

      for (cb::Size1 i = 0; i < cb_corners.elements(); ++i)
        residuals[i] = T((reprojected_corners[i] - image_corners_[i].cast<T>()).norm() / 0.5);

      return true;
    }

private:

  const cb::PinholeCameraModel::ConstPtr camera_model_;
  const cb::Checkerboard::ConstPtr checkerboard_;
  const cb::Cloud2 image_corners_;

};

class PinholeFloorError
{
public:

  PinholeFloorError(const cb::PinholeCameraModel::ConstPtr & camera_model,
                    const cb::Checkerboard::ConstPtr & checkerboard,
                    const cb::Cloud2 & image_corners)
    : camera_model_(camera_model),
      checkerboard_(checkerboard),
      image_corners_(image_corners)
  {
  }

  template <typename T>
    bool operator ()(const T * const sensor_pose,
                     const T * const checkerboard_pose,
                     const T * const plane_origin,
                     T * residuals) const
    {
      typename cb::Types<T>::Vector3 sensor_r_vec(sensor_pose[0], sensor_pose[1], sensor_pose[2]);
      typename cb::Types<T>::AngleAxis sensor_r(sensor_r_vec.norm(), sensor_r_vec.normalized());
      typename cb::Types<T>::Translation3 sensor_t(sensor_pose[3], sensor_pose[4], sensor_pose[5]);

      typename cb::Types<T>::Transform sensor_pose_eigen = cb::Types<T>::Transform::Identity() * sensor_t;

      if (sensor_r_vec.norm() != T(0))
        sensor_pose_eigen = sensor_t * sensor_r;

      typename cb::Types<T>::Translation2 checkerboard_t(checkerboard_pose[0], checkerboard_pose[1]);
      typename cb::Types<T>::Rotation2 checkerboard_r(checkerboard_pose[0]);
      typename cb::Types<T>::Transform2 checkerboard_pose_eigen = checkerboard_t * checkerboard_r;

      typename cb::Types<T>::Vector3 plane_origin_eigen(plane_origin[0], plane_origin[1], plane_origin[2]);
      typename cb::Types<T>::Plane plane(-plane_origin_eigen.normalized(), plane_origin_eigen.norm());

      typename cb::Types<T>::Transform plane_pose;
      plane_pose.linear().col(2) = plane.normal();
      plane_pose.linear().col(0) = (plane.projection(plane_origin_eigen + cb::Types<T>::Vector3::UnitX()) - plane_origin_eigen).normalized();
      plane_pose.linear().col(1) = plane_pose.linear().col(2).cross(plane_pose.linear().col(0));
      plane_pose.translation() = plane_origin_eigen;

      typename cb::Types<T>::Cloud3 cb_corners(cb::Size2(checkerboard_->cols(), checkerboard_->rows()), cb::Types<T>::Point3::Zero());
      cb_corners.container().template topRows<2>() = checkerboard_pose_eigen * checkerboard_->corners().container().template topRows<2>().cast<T>();
      cb_corners.container() = sensor_pose_eigen.inverse() * plane_pose * cb_corners.container();

      typename cb::Types<T>::Cloud2 reprojected_corners = camera_model_->project3dToPixel<T>(cb_corners);

      for (cb::Size1 i = 0; i < cb_corners.elements(); ++i)
        residuals[i] = T((reprojected_corners[i] - image_corners_[i].cast<T>()).norm() / 0.5);

      return true;
    }

private:

  const cb::PinholeCameraModel::ConstPtr camera_model_;
  const cb::Checkerboard::ConstPtr checkerboard_;
  const cb::Cloud2 image_corners_;

};

class DepthError
{
public:

  DepthError(const cb::Checkerboard::ConstPtr & checkerboard,
             const cb::Plane & depth_plane,
             const cb::Cloud3 & points,
             const cb::Polynomial<cb::Scalar, 2> & depth_error_function)
    : checkerboard_(checkerboard),
      depth_plane_(depth_plane),
      points_(points),
      depth_error_function_(depth_error_function)
  {
  }

  template <typename T>
    bool operator ()(const T * const sensor_pose,
                     const T * const checkerboard_pose,
                     T * residuals) const
    {
      typename cb::Types<T>::Vector3 sensor_r_vec(sensor_pose[0], sensor_pose[1], sensor_pose[2]);
      typename cb::Types<T>::AngleAxis sensor_r(sensor_r_vec.norm(), sensor_r_vec.normalized());
      typename cb::Types<T>::Translation3 sensor_t(sensor_pose[3], sensor_pose[4], sensor_pose[5]);

      typename cb::Types<T>::Transform sensor_pose_eigen = cb::Types<T>::Transform::Identity() * sensor_t;

      if (sensor_r_vec.norm() > T(0.0001))
        sensor_pose_eigen = sensor_t * sensor_r;

      typename cb::Types<T>::Vector3 checkerboard_r_vec(checkerboard_pose[0], checkerboard_pose[1], checkerboard_pose[2]);
      typename cb::Types<T>::AngleAxis checkerboard_r(checkerboard_r_vec.norm(), checkerboard_r_vec.normalized());
      typename cb::Types<T>::Translation3 checkerboard_t(checkerboard_pose[3], checkerboard_pose[4], checkerboard_pose[5]);

      typename cb::Types<T>::Transform checkerboard_pose_eigen = checkerboard_t * checkerboard_r;
      typename cb::Types<T>::Cloud3 cb_corners(cb::Size2(checkerboard_->cols(), checkerboard_->rows()));

      cb_corners.container() = sensor_pose_eigen.inverse() * checkerboard_pose_eigen
                            * checkerboard_->corners().container().cast<T>();

      typename cb::Types<T>::Plane depth_plane(depth_plane_.normal().cast<T>(), T(depth_plane_.offset()));
      cb::Polynomial<T, 2> depth_error_function(depth_error_function_.coefficients().cast<T>());
      for (cb::Size1 i = 0; i < cb_corners.elements(); ++i)
      {
        typename cb::Types<T>::Line line(cb::Types<T>::Point3::Zero(), cb_corners[i]);
        residuals[i] = T((line.intersectionPoint(depth_plane) - cb_corners[i]).norm()
          / ceres::poly_eval(depth_error_function.coefficients(), cb_corners[i].z()));
      }

//      typename Types<T>::Plane cb_plane = Types<T>::Plane::Through(cb_corners(0, 0), cb_corners(0, 1), cb_corners(1, 0));
//
//      typename Types<T>::Cloud3 plane_points(points_.xSize(), points_.ySize());
//      plane_points.matrix() = points_.matrix().cast<T>();
//
//      Polynomial<T, 2> depth_error_function(depth_error_function_.coefficients().cast<T>());
//      for (size_t i = 0; i < plane_points.size(); ++i)
//      {
//        typename Types<T>::Line line(Types<T>::Point3::Zero(), plane_points[i]);
//        residuals[i] = T((line.intersectionPoint(cb_plane) - plane_points[i]).norm()
//          / ceres::poly_eval(depth_error_function.coefficients(), plane_points[i].z()) / plane_points[i].z());
//      }

      return true;
    }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

  const cb::Checkerboard::ConstPtr checkerboard_;
  const cb::Plane depth_plane_;
  const cb::Cloud3 points_;
  const cb::Polynomial<cb::Scalar, 2> depth_error_function_;

};

void OPTCalibration::optimize()
{
  convertToWorldFrame();

  ROS_INFO("Optimizing...");

  Eigen::Matrix<cb::Scalar, Eigen::Dynamic, 6, Eigen::DontAlign | Eigen::RowMajor> cb_data(view_map_vec_.size(), 6);
  Eigen::Matrix<cb::Scalar, Eigen::Dynamic, 6, Eigen::DontAlign | Eigen::RowMajor> sensor_data(node_vec_.size(), 6);
  Eigen::Matrix<cb::Scalar, 3, 1, Eigen::DontAlign> floor_data;

  for (size_t i = 0; i < node_vec_.size(); ++i)
  {
    const TreeNode & sensor_node = *node_vec_[i];
    cb::AngleAxis rotation = cb::AngleAxis(sensor_node.sensor()->pose().linear());
    sensor_data.row(i).head<3>() = rotation.angle() * rotation.axis();
    sensor_data.row(i).tail<3>() = sensor_node.sensor()->pose().translation();
  }

  for (size_t i = 0; i < checkerboard_vec_.size(); ++i)
  {
    const cb::Checkerboard & checkerboard = *checkerboard_vec_[i];
    cb::AngleAxis rotation = cb::AngleAxis(checkerboard.pose().linear());
    cb_data.row(i).head<3>() = rotation.angle() * rotation.axis();
    cb_data.row(i).tail<3>() = checkerboard.pose().translation();

  }

//  ROS_INFO("Before optimization:");
//  for (size_t i = 0; i < sensor_vec_.size(); ++i)
//    ROS_INFO_STREAM("(" << node_vec_[i]->sensor()->parent()->frameId() << ") "
//                        << node_vec_[i]->sensor()->frameId() << ": " << sensor_data.row(i));

  if (not floor_estimated_)
  {
    floor_estimated_ = estimateFloor();
//    if (floor_estimated_)
//      ROS_INFO_STREAM("F  " << floor_.normal().transpose() << " " << floor_.offset());
  }

  cb::Vector3 floor_origin;
  cb::Vector3 floor_x;
  cb::Transform floor_pose;
  if (floor_estimated_)
  {
    floor_origin = -floor_.normal() * floor_.offset();
    floor_x = (floor_.projection(floor_origin + cb::Vector3::UnitX()) - floor_origin).normalized();

    floor_pose.linear().col(2) = floor_.normal();
    floor_pose.linear().col(0) = floor_x;
    floor_pose.linear().col(1) = floor_pose.linear().col(2).cross(floor_pose.linear().col(0));
    floor_pose.translation() = floor_origin;

    floor_data = floor_origin;
  }

  for (size_t i = 0; i < checkerboard_vec_.size(); ++i)
  {
    if (floor_estimated_ and is_floor_vec_[i])
    {
      const cb::Checkerboard & checkerboard = *checkerboard_vec_[i];
      cb::Vector3 origin = floor_.projection(checkerboard[0]); // TODO use line-of-sight projection
      cb::Vector3 x = (floor_.projection(checkerboard(checkerboard.cols() - 1, 0)) - origin).normalized(); // TODO use line-of-sight projection
      cb::Scalar theta = std::atan2((x - floor_x * floor_x.dot(x)).norm(), floor_x.dot(x));
//      ROS_INFO_STREAM("O  " << origin.transpose());// << " ** " <<  x.transpose() << " -- " << theta);

      cb::Vector3 origin_floor = floor_pose.inverse() * origin;
//      ROS_INFO_STREAM("O 2D  " << origin_floor.transpose());

      cb_data.row(i).head<2>() = origin_floor.head<2>();
      cb_data.row(i)[2] = theta;
    }
  }

  ceres::Problem problem;

  for (size_t i = 0; i < view_map_vec_.size(); ++i)
  {
    ViewMap & view_map = view_map_vec_[i];
    if (view_map.size() == 1) // Nothing to optimize
      continue;

    for (ViewMap::iterator it = view_map.begin(); it != view_map.end(); ++it)
    {
      CheckerboardView::Ptr cb_view = it->second;
      const TreeNode::Ptr & tree_node = it->first;

      if (tree_node->type() == TreeNode::INTENSITY)
      {
        cb::PinholeSensor::Ptr sensor = boost::static_pointer_cast<cb::PinholeSensor>(tree_node->sensor());
        cb::PinholeView<cb::Checkerboard>::Ptr view = boost::static_pointer_cast<cb::PinholeView<cb::Checkerboard> >(cb_view->view);
        if (tree_node->level() > 0 and not cb_view->is_floor)
        {
          PinholeError * error = new PinholeError(sensor->cameraModel(), checkerboard_, view->points());
          typedef ceres::AutoDiffCostFunction<PinholeError, ceres::DYNAMIC, 6, 6> PinholeErrorFunction;

          ceres::CostFunction * cost_function = new PinholeErrorFunction(error, checkerboard_->size());
          problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.5), sensor_data.row(tree_node->id()).data(), cb_data.row(i).data());
        }
        else if (tree_node->level() > 0 and cb_view->is_floor)
        {
          PinholeFloorError * error = new PinholeFloorError(sensor->cameraModel(), checkerboard_, view->points());
          typedef ceres::AutoDiffCostFunction<PinholeFloorError, ceres::DYNAMIC, 6, 3, 3> PinholeFloorErrorFunction;

          ceres::CostFunction * cost_function = new PinholeFloorErrorFunction(error, checkerboard_->size());
          problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.5), sensor_data.row(tree_node->id()).data(), cb_data.row(i).data(), floor_data.data());
        }
        else if (not cb_view->is_floor)
        {
          RootPinholeError * error = new RootPinholeError(sensor->cameraModel(), checkerboard_, view->points());
          typedef ceres::AutoDiffCostFunction<RootPinholeError, ceres::DYNAMIC, 6> RootPinholeErrorFunction;

          ceres::CostFunction * cost_function = new RootPinholeErrorFunction(error, checkerboard_->size());
          problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.5), cb_data.row(i).data());
        }
        else // (cb_view->is_floor)
        {
          RootPinholeFloorError * error = new RootPinholeFloorError(sensor->cameraModel(), checkerboard_, view->points());
          typedef ceres::AutoDiffCostFunction<RootPinholeFloorError, ceres::DYNAMIC, 3, 3> RootPinholeFloorErrorFunction;

          ceres::CostFunction * cost_function = new RootPinholeFloorErrorFunction(error, checkerboard_->size());
          problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.5), cb_data.row(i).data(), floor_data.data());
        }
      }
      else //(tree_node->type() == TreeNode::DEPTH)
      {
        cb::DepthSensor::Ptr sensor = boost::static_pointer_cast<cb::DepthSensor>(tree_node->sensor());
        cb::DepthViewPCL<cb::PlanarObject>::Ptr view = boost::static_pointer_cast<cb::DepthViewPCL<cb::PlanarObject> >(cb_view->view);
        if (tree_node->level() > 0)
        {
          DepthError * error = new DepthError(checkerboard_,
                                              cb_view->object->plane(),
                                              cb::PCLConversion<cb::Scalar>::toPointMatrix(*view->data(), view->points()),
                                              sensor->depthErrorFunction());

          typedef ceres::AutoDiffCostFunction<DepthError, ceres::DYNAMIC, 6, 6> DepthErrorFunction;

          ceres::CostFunction * cost_function = new DepthErrorFunction(error, checkerboard_->size());
          //ceres::CostFunction * cost_function = new DepthErrorFunction(error, view->points().size());
          problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.5), sensor_data.row(tree_node->id()).data(), cb_data.row(i).data());

        }
        else
        {
          //ROS_FATAL("FATAL");
        }
      }
    }
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.max_num_iterations = 200;
//  options.minimizer_progress_to_stdout = true;
  options.num_threads = 8;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

//  ROS_INFO("After optimization:");
  for (size_t i = 0; i < node_vec_.size(); ++i)
  {
//    ROS_INFO_STREAM("(" << node_vec_[i]->sensor()->parent()->frameId() << ") "
//                        << node_vec_[i]->sensor()->frameId() << ": " << sensor_data.row(i));
    TreeNode::Ptr & sensor_node = node_vec_[i];
    if (sensor_data.row(i).head<3>().norm() != 0)
    {
      cb::AngleAxis rotation(sensor_data.row(i).head<3>().norm(), sensor_data.row(i).head<3>().normalized());
      cb::Translation3 translation(sensor_data.row(i).tail<3>());
      sensor_node->sensor()->setPose(translation * rotation);
    }
    else
    {
      cb::Translation3 translation(sensor_data.row(i).tail<3>());
      sensor_node->sensor()->setPose(cb::Pose::Identity() * translation);
    }
  }

//  if (floor_estimated_)
//    ROS_INFO_STREAM("F* " << -floor_data.normalized().transpose() << " " << floor_data.norm());

  cb::Plane plane(-floor_data.normalized(), floor_data.norm());
  cb::Transform plane_pose;
  plane_pose.linear().col(2) = plane.normal();
  plane_pose.linear().col(0) = (plane.projection(floor_data + cb::Vector3::UnitX()) - floor_data).normalized();
  plane_pose.linear().col(1) = plane_pose.linear().col(2).cross(plane_pose.linear().col(0));
  plane_pose.translation() = floor_data;

  for (size_t i = 0; i < checkerboard_vec_.size(); ++i)
  {
    cb::Checkerboard::Ptr & checkerboard = checkerboard_vec_[i];

    if (floor_estimated_ and is_floor_vec_[i])
    {
      cb::Translation2 checkerboard_t_2d(cb_data.row(i).head<2>());
      cb::Rotation2 checkerboard_r_2d(cb_data.row(i)[2]);
      cb::Transform2 checkerboard_pose_2d = checkerboard_t_2d * checkerboard_r_2d;
      cb::Transform checkerboard_pose = cb::Transform::Identity();

      checkerboard_pose.linear().topLeftCorner<2, 2>() = checkerboard_pose_2d.linear();
      checkerboard_pose.translation().head<2>() = checkerboard_pose_2d.translation();
//      ROS_INFO_STREAM("O 2D* " << (checkerboard_pose * cb::Point3::Zero()).transpose());

//      ROS_INFO_STREAM("O* " << (plane_pose * checkerboard_pose * cb::Point3::Zero()).transpose());// << "\n------------------\n" << checkerboard->pose().matrix());
      checkerboard->transform(plane_pose * checkerboard_pose * checkerboard->pose().inverse());
    }
    else
    {
      if (cb_data.row(i).head<3>().norm() > 0.001)
      {
        cb::AngleAxis rotation(cb_data.row(i).head<3>().norm(), cb_data.row(i).head<3>().normalized());
        cb::Translation3 translation(cb_data.row(i).tail<3>());
        checkerboard->transform(translation * rotation * checkerboard->pose().inverse());
      }
      else
      {
        cb::Translation3 translation(cb_data.row(i).tail<3>());
        checkerboard->transform(cb::Pose::Identity() * translation * checkerboard->pose().inverse());
      }
    }
  }
}

const cb::Pose & OPTCalibration::getLastCheckerboardPose() const
{
  return checkerboard_vec_.back()->pose();
}

} /* namespace opt_calibration */
} /* namespace open_ptrack */
