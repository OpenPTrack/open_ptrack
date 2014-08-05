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

#define OPTIMIZATION_COUNT 20

namespace open_ptrack
{
namespace opt_calibration
{

const int SensorNode::MAX_LEVEL = 10000;
const double SensorNode::MAX_DISTANCE = 10000.0;
const double SensorNode::MAX_ERROR = 10000.0;

OPTCalibration::OPTCalibration(const ros::NodeHandle & node_handle,
                               bool calibration_with_serials)
  : node_handle_(node_handle),
    world_(boost::make_shared<BaseObject>("/world")),
    world_set_(false),
    initialization_(true),
    last_optimization_(0),
    calibration_with_serials_(calibration_with_serials)
{
  marker_pub_ = node_handle_.advertise<visualization_msgs::Marker>("markers", 0);
}

bool OPTCalibration::findCheckerboard(const cv::Mat & image,
                                      const PinholeSensor::Ptr & sensor,
                                      PinholeView<Checkerboard>::Ptr & color_view,
                                      Checkerboard::Ptr & checkerboard)
{
  const SensorNode::Ptr & sensor_node = sensor_map_[sensor];

  Cloud2 corners(checkerboard_->rows(), checkerboard_->cols());
  finder_.setImage(image);
  if (not finder_.find(*checkerboard_, corners))
    return false;

  std::stringstream ss;
  ss << "cb_" << sensor->frameId();

  color_view = boost::make_shared<PinholeView<Checkerboard> >();
  color_view->setId(ss.str());
  color_view->setObject(checkerboard_);
  color_view->setPoints(corners);
  color_view->setSensor(sensor);
  color_view->setId(ss.str());

  checkerboard = boost::make_shared<Checkerboard>(*color_view);

  visualization_msgs::Marker checkerboard_marker;
  checkerboard_marker.ns = "checkerboard";
  checkerboard_marker.id = sensor_node->id_;

  checkerboard->toMarker(checkerboard_marker);
  marker_pub_.publish(checkerboard_marker);

  geometry_msgs::TransformStamped transform_msg;
  checkerboard->toTF(transform_msg);
  tf_pub_.sendTransform(transform_msg);

  return true;

}

void OPTCalibration::perform()
{
  const ViewMap & view_map = view_vec_.back();

  if (initialization_)
  {
    if (not world_set_ and not view_map.empty()) // Set /world
    {
      ViewMap::const_iterator it = view_map.begin();

      SensorNode::Ptr sensor_node = it->first;
      sensor_node->sensor_->setParent(world_);
      sensor_node->level_ = 0;

      ROS_INFO_STREAM(sensor_node->sensor_->frameId() << " added to the tree.");
      world_set_ = true;
    }

    if (view_map.empty())
    {
      view_vec_.resize(view_vec_.size() - 1); // Remove data
    }
    else if(view_map.size() == 1)
    {
      single_view_vec_.push_back(view_map);
      view_vec_.resize(view_vec_.size() - 1);
    }
    else if (view_map.size() >= 2) // At least 2 cameras
    {
      int min_level = SensorNode::MAX_LEVEL;
      SensorNode::Ptr min_sensor_node;
      for (ViewMap::const_iterator it = view_map.begin(); it != view_map.end(); ++it)
      {
        SensorNode::Ptr sensor_node = it->first;
        if (sensor_node->level_ < min_level)
        {
          min_level = sensor_node->level_;
          min_sensor_node = sensor_node;
        }
      }

      if (min_level < SensorNode::MAX_LEVEL) // At least one already in tree
      {
        for (ViewMap::const_iterator it = view_map.begin(); it != view_map.end(); ++it)
        {
          SensorNode::Ptr sensor_node = it->first;

          if (sensor_node != min_sensor_node)
          {
            const CheckerboardView::Ptr & view = it->second;
            PlanarObject::Ptr planar_object = view->object_;
            Point3 center = view->center_;

            double camera_error = std::abs(planar_object->plane().normal().dot(Vector3::UnitZ())) * center.squaredNorm();

            if (sensor_node->level_ > min_level and camera_error < sensor_node->min_error_)
            {
              CheckerboardView::Ptr min_checkerboard_view = view_map.at(min_sensor_node);
              PlanarObject::Ptr min_planar_object = min_checkerboard_view->object_;
              Point3 min_center = min_checkerboard_view->center_;

              if (sensor_node->level_ == SensorNode::MAX_LEVEL)
                ROS_INFO_STREAM(sensor_node->sensor_->frameId() << " added to the tree.");

              double distance = (min_center - center).norm();

              sensor_node->min_error_ = camera_error;
              sensor_node->distance_ = distance;
              sensor_node->level_ = min_level + 1;

              sensor_node->sensor_->setParent(min_sensor_node->sensor_);
              sensor_node->sensor_->setPose(min_planar_object->pose() * planar_object->pose().inverse());

            }
          }
        }       
      }

      initialization_ = (view_vec_.size() < OPTIMIZATION_COUNT);
      for (int i = 0; not initialization_ and i < sensor_vec_.size(); ++i)
      {
        SensorNode::Ptr & sensor_node = sensor_vec_[i];
        if (sensor_node->level_ == SensorNode::MAX_LEVEL)
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
      view_vec_.resize(view_vec_.size() - 1); // Remove data
    }
    else if(view_map.size() == 1)
    {
      single_view_vec_.push_back(view_map);
      view_vec_.resize(view_vec_.size() - 1);
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

  publish();

}

void OPTCalibration::publish()
{

  for (size_t i = 0; i < sensor_vec_.size(); ++i)
  {
    SensorNode::Ptr sensor_node = sensor_vec_[i];
    geometry_msgs::TransformStamped transform_msg;
    if (sensor_node->sensor_->toTF(transform_msg))
      tf_pub_.sendTransform(transform_msg);
  }

//  for (size_t i = 0; i < checkerboard_vec_.size(); ++i)
//  {
//    Checkerboard::Ptr cb = checkerboard_vec_[i];
//    visualization_msgs::Marker marker;
//    cb->toMarker(marker);
//    marker.ns = "optimized checkerboard";
//    marker.id = i;
//    marker_pub_.publish(marker);
//  }

}

class RootPinholeError
{
public:

  RootPinholeError(const PinholeCameraModel::ConstPtr & camera_model,
                   const Checkerboard::ConstPtr & checkerboard,
                   const Cloud2 & image_corners)
    : camera_model_(camera_model),
      checkerboard_(checkerboard),
      image_corners_(image_corners)
  {
  }

  template <typename T>
    bool operator ()(const T * const checkerboard_pose,
                     T * residuals) const
    {
      typename Types<T>::Vector3 checkerboard_r_vec(checkerboard_pose[0], checkerboard_pose[1], checkerboard_pose[2]);
      typename Types<T>::AngleAxis checkerboard_r(checkerboard_r_vec.norm(), checkerboard_r_vec.normalized());
      typename Types<T>::Translation3 checkerboard_t(checkerboard_pose[3], checkerboard_pose[4], checkerboard_pose[5]);

      typename Types<T>::Transform checkerboard_pose_eigen = checkerboard_t * checkerboard_r;

      typename Types<T>::Cloud3 cb_corners(checkerboard_->cols(), checkerboard_->rows());
      cb_corners.container() = checkerboard_pose_eigen * checkerboard_->corners().container().cast<T>();

      typename Types<T>::Cloud2 reprojected_corners = camera_model_->project3dToPixel<T>(cb_corners);

      for (size_t i = 0; i < cb_corners.size(); ++i)
        residuals[i] = T((reprojected_corners[i] - image_corners_[i].cast<T>()).norm() / 0.5);

      return true;
    }

private:

  const PinholeCameraModel::ConstPtr camera_model_;
  const Checkerboard::ConstPtr checkerboard_;
  const Cloud2 image_corners_;

};

class PinholeError
{
public:

  PinholeError(const PinholeCameraModel::ConstPtr & camera_model,
               const Checkerboard::ConstPtr & checkerboard,
               const Cloud2 & image_corners)
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
      typename Types<T>::Vector3 sensor_r_vec(sensor_pose[0], sensor_pose[1], sensor_pose[2]);
      typename Types<T>::AngleAxis sensor_r(sensor_r_vec.norm(), sensor_r_vec.normalized());
      typename Types<T>::Translation3 sensor_t(sensor_pose[3], sensor_pose[4], sensor_pose[5]);

      typename Types<T>::Transform sensor_pose_eigen = Types<T>::Transform::Identity() * sensor_t;

      if (sensor_r_vec.norm() != T(0))
        sensor_pose_eigen = sensor_t * sensor_r;

      typename Types<T>::Vector3 checkerboard_r_vec(checkerboard_pose[0], checkerboard_pose[1], checkerboard_pose[2]);
      typename Types<T>::AngleAxis checkerboard_r(checkerboard_r_vec.norm(), checkerboard_r_vec.normalized());
      typename Types<T>::Translation3 checkerboard_t(checkerboard_pose[3], checkerboard_pose[4], checkerboard_pose[5]);

      typename Types<T>::Transform checkerboard_pose_eigen = checkerboard_t * checkerboard_r;

      typename Types<T>::Cloud3 cb_corners(checkerboard_->cols(), checkerboard_->rows());
      cb_corners.container() = sensor_pose_eigen.inverse() * checkerboard_pose_eigen
                            * checkerboard_->corners().container().cast<T>();

      typename Types<T>::Cloud2 reprojected_corners = camera_model_->project3dToPixel<T>(cb_corners);

      for (size_t i = 0; i < cb_corners.size(); ++i)
        residuals[i] = T((reprojected_corners[i] - image_corners_[i].cast<T>()).norm() / 0.5);

      return true;
    }

private:

  const PinholeCameraModel::ConstPtr camera_model_;
  const Checkerboard::ConstPtr checkerboard_;
  const Cloud2 image_corners_;

};

void OPTCalibration::optimize()
{
  ROS_INFO("Optimizing...");

  ceres::Problem problem;
  Eigen::Matrix<Scalar, Eigen::Dynamic, 6, Eigen::DontAlign | Eigen::RowMajor> cb_data(view_vec_.size(), 6);
  Eigen::Matrix<Scalar, Eigen::Dynamic, 6, Eigen::DontAlign | Eigen::RowMajor> sensor_data(sensor_vec_.size(), 6);

  for (size_t i = 0; i < sensor_vec_.size(); ++i)
  {
    const SensorNode & sensor_node = *sensor_vec_[i];
    Pose pose = sensor_node.sensor_->pose();
    BaseObject::ConstPtr parent = sensor_node.sensor_->parent();

    while (parent->parent())
    {
      pose = parent->pose() * pose;
      parent = parent->parent();
      sensor_node.sensor_->setParent(parent);
      sensor_node.sensor_->setPose(pose);
    }

    AngleAxis rotation = AngleAxis(pose.linear());
    sensor_data.row(i).head<3>() = rotation.angle() * rotation.axis();
    sensor_data.row(i).tail<3>() = pose.translation();
  }

//  ROS_INFO("Before optimization:");
//  for (size_t i = 0; i < sensor_vec_.size(); ++i)
//    ROS_INFO_STREAM("(" << sensor_vec_[i]->sensor_->parent()->frameId() << ") "
//                        << sensor_vec_[i]->sensor_->frameId() << ": " << sensor_data.row(i));

  for (size_t i = checkerboard_vec_.size(); i < view_vec_.size(); ++i)
  {
    ViewMap & view_map = view_vec_[i];
    ViewMap::iterator it = view_map.begin();
    bool ok = false;
    while (not ok and it != view_map.end())
    {
      if (it->first->type_ != SensorNode::PINHOLE_RGB)
        it++;
      else
        ok = true;
    }

    const CheckerboardView::Ptr & cb_view = it->second;
    const SensorNode::Ptr & sensor_node = it->first;

    Checkerboard::Ptr cb = boost::make_shared<Checkerboard>(*boost::static_pointer_cast<Checkerboard>(cb_view->object_));

    BaseObject::ConstPtr parent = sensor_node->sensor_;
    while (parent->parent())
    {
      cb->transform(parent->pose());
      parent = parent->parent();
    }
    cb->setParent(parent);

    checkerboard_vec_.push_back(cb);

  }

  for (size_t i = 0; i < view_vec_.size(); ++i)
  {
    ViewMap & data_map = view_vec_[i];

    Pose pose = checkerboard_vec_[i]->pose();

    AngleAxis rotation = AngleAxis(pose.linear());
    cb_data.row(i).head<3>() = rotation.angle() * rotation.axis();
    cb_data.row(i).tail<3>() = pose.translation();

    for (ViewMap::iterator it = data_map.begin(); it != data_map.end(); ++it)
    {
      CheckerboardView::Ptr cb_view = it->second;
      const SensorNode::Ptr & sensor_node = it->first;

      if (sensor_node->type_ == SensorNode::PINHOLE_RGB)
      {
        PinholeSensor::Ptr sensor = boost::static_pointer_cast<PinholeSensor>(sensor_node->sensor_);
        PinholeView<Checkerboard>::Ptr view = boost::static_pointer_cast<PinholeView<Checkerboard> >(cb_view->view_);
        if (sensor_node->level_ > 0)
        {
          PinholeError * error = new PinholeError(sensor->cameraModel(), checkerboard_, view->points());
          typedef ceres::AutoDiffCostFunction<PinholeError, ceres::DYNAMIC, 6, 6> PinholeErrorFunction;

          ceres::CostFunction * cost_function = new PinholeErrorFunction(error, checkerboard_->size());
          problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.5), sensor_data.row(sensor_node->id_).data(), cb_data.row(i).data());

        }
        else
        {
          RootPinholeError * error = new RootPinholeError(sensor->cameraModel(), checkerboard_, view->points());
          typedef ceres::AutoDiffCostFunction<RootPinholeError, ceres::DYNAMIC, 6> RootPinholeErrorFunction;

          ceres::CostFunction * cost_function = new RootPinholeErrorFunction(error, checkerboard_->size());
          problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.5), cb_data.row(i).data());
        }
      }
      else //(sensor_node->type_ == SensorNode::KINECT_DEPTH)
      {
        ROS_FATAL("SensorNode::KINECT_DEPTH");
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
  for (size_t i = 0; i < sensor_vec_.size(); ++i)
  {
//    ROS_INFO_STREAM("(" << sensor_vec_[i]->sensor_->parent()->frameId() << ") "
//                        << sensor_vec_[i]->sensor_->frameId() << ": " << sensor_data.row(i));
    SensorNode::Ptr & sensor_node = sensor_vec_[i];
    if (sensor_data.row(i).head<3>().norm() != 0)
    {
      AngleAxis rotation(sensor_data.row(i).head<3>().norm(), sensor_data.row(i).head<3>().normalized());
      Translation3 translation(sensor_data.row(i).tail<3>());
      sensor_node->sensor_->setPose(translation * rotation);
    }
    else
    {
      Translation3 translation(sensor_data.row(i).tail<3>());
      sensor_node->sensor_->setPose(Pose::Identity() * translation);
    }
  }

  for (size_t i = 0; i < checkerboard_vec_.size(); ++i)
  {
    Checkerboard::Ptr & cb = checkerboard_vec_[i];
    if (cb_data.row(i).head<3>().norm() != 0)
    {
      AngleAxis rotation(cb_data.row(i).head<3>().norm(), cb_data.row(i).head<3>().normalized());
      Translation3 translation(cb_data.row(i).tail<3>());
      cb->setPose(translation * rotation);
    }
    else
    {
      Translation3 translation(cb_data.row(i).tail<3>());
      cb->setPose(Pose::Identity() * translation);
    }
  }

}

void OPTCalibration::save()
{
  // save tf between camera and world coordinate system ( chessboard ) to launch file
  std::string file_name = ros::package::getPath("opt_calibration") + "/launch/multicamera_calibration_results.launch";
  std::ofstream launch_file;
  launch_file.open(file_name.c_str());

  std::stringstream optical_frame_string, link_string;
  optical_frame_string << "_rgb_optical_frame";
  link_string << "_link";

  if (launch_file.is_open())
  {
    launch_file << "<launch>" << std::endl << std::endl;

    // Write number of cameras:
    launch_file << "  <!-- Network parameters -->" << std::endl
                << "  <arg name=\"num_cameras\" value=\"" << sensor_vec_.size() << "\" />" << std::endl;

    // Write camera ID and names:
    for (size_t i = 0; i < sensor_vec_.size(); ++i)
    {
      SensorNode::Ptr & sensor_node = sensor_vec_[i];
      launch_file << "  <arg name=\"camera" << i << "_id\" value=\"" << sensor_node->sensor_->frameId() << "\" />" << std::endl;
      launch_file << "  <arg name=\"camera" << i << "_name\" value=\"$(arg camera" << i << "_id)\" />" << std::endl;
    }
    launch_file << std::endl;

    ViewMap & view_map = single_view_vec_.back();
    const CheckerboardView::Ptr & cb_view = view_map.begin()->second;
    const SensorNode::Ptr & sensor_node = view_map.begin()->first;

    Checkerboard::Ptr cb = boost::make_shared<Checkerboard>(*boost::static_pointer_cast<Checkerboard>(cb_view->object_));

    BaseObject::ConstPtr parent = sensor_node->sensor_;
    while (parent->parent())
    {
      cb->transform(parent->pose());
      parent = parent->parent();
    }
    cb->setParent(parent);


    AngleAxis rotation(M_PI, Vector3(1.0, 1.0, 0.0).normalized());
    Pose new_world_pose = cb->pose().inverse();

    // Write TF transforms between cameras and world frame:
    launch_file << "  <!-- Transforms tree -->" << std::endl;
    for (size_t i = 0; i < sensor_vec_.size(); ++i)
    {
      SensorNode::Ptr & sensor_node = sensor_vec_[i];

      Pose new_pose = rotation * new_world_pose * sensor_node->sensor_->pose();

      launch_file << "  <node pkg=\"tf\" type=\"static_transform_publisher\" name=\""
                  << sensor_node->sensor_->frameId().substr(1) << "_broadcaster\" args=\""
                  << new_pose.translation().transpose() << " "
                  << Quaternion(new_pose.linear()).coeffs().transpose() << " "
                  << world_->frameId() << " " << sensor_node->sensor_->frameId() << " 100\" />" << std::endl << std::endl;

      if (std::strcmp(sensor_node->sensor_->frameId().substr(1, 2).c_str(), "SR")) // if Kinect
      {
        // Write transform between camera_link and camera_rgb_optical_frame to file:
        launch_file << "  <node pkg=\"tf\" type=\"static_transform_publisher\" name=\""
                    << sensor_node->sensor_->frameId().substr(1) << "_broadcaster2\" args=\" -0.045 0 0 1.57 -1.57 0 "
                    << sensor_node->sensor_->frameId() << " "
                    << sensor_node->sensor_->frameId() + link_string.str() << " 100\" />" << std::endl << std::endl;
      }

    }

    launch_file << "</launch>" << std::endl;
  }
  launch_file.close();

  ROS_INFO_STREAM(file_name << " created!");

//  // Save launch files to be used to perform people detection with every sensor:
//  for (size_t i = 0; i < sensor_vec_.size(); i++)
//  {
//    SensorNode::Ptr & sensor_node = sensor_vec_[i];

//    std::string serial_number = sensor_node->sensor_->frameId().substr(1, sensor_node->sensor_->frameId().length() - 1);
//    std::string file_name = ros::package::getPath("detection") + "/launch/detection_node_" + serial_number + ".launch";
//    std::ofstream launch_file;
//    launch_file.open(file_name.c_str());
//    if (launch_file.is_open())
//    {
//      launch_file << "<launch>" << std::endl << std::endl;
//      launch_file << "  <!-- Camera ID -->" << std::endl
//                  << "  <arg name=\"camera_id\" value=\"" << serial_number << "\" />" << std::endl << std::endl
//                  << "  <!-- Detection node -->" << std::endl;

//      if (std::strcmp(sensor_node->sensor_->frameId().substr(1, 2).c_str(), "SR")) // if Kinect
//      {
//        if (calibration_with_serials_)
//          launch_file << "  <include file=\"$(find detection)/launch/detector_serial.launch\">" << std::endl;
//        else
//          launch_file << "  <include file=\"$(find detection)/launch/detector_with_name.launch\">" << std::endl;
//      }
//      else // if SwissRanger
//      {
//        std::string device_ip = sensor_vec_[i]->sensor_->frameId();
//        std::replace(device_ip.begin(), device_ip.end(), '_', '.');
//        device_ip = device_ip.substr(4, device_ip.length() - 4);
//        launch_file << "  <include file=\"$(find detection)/launch/detector_with_name_sr.launch\">" << std::endl
//                    << "    <arg name=\"device_ip\" value=\"" << device_ip << "\" />" << std::endl;
//      }

//      launch_file << "    <arg name=\"camera_id\" value=\"$(arg camera_id)\" />" << std::endl
//                  << "    <arg name=\"ground_from_calibration\" value=\"true\" />" << std::endl
//                  << "  </include>" << std::endl;

//      launch_file << "</launch>" << std::endl;
//    }
//    launch_file.close();
//    ROS_INFO_STREAM(file_name << " created!");
//  }

  // Run node which saves camera poses to text file:
  int r = system("roslaunch opt_calibration calibration_saver.launch");
}

} /* namespace opt_calibration */
} /* namespace open_ptrack */
