/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013 Riccardo Levorato, Filippo Basso
 * Copyright (c) 2014-, Open Perception, Inc.
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
 * Author: Matteo Munaro [matteo.munaro@dei.unipd.it], Riccardo Levorato [levorato@dei.unipd.it], Filippo Basso [bassofil@dei.unipd.it]
 *
 */

#include <open_ptrack/opt_calibration/multicamera_calibration.h>
#include <fstream>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
//#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

#include <ceres/ceres.h>

#define OPTIMIZATION_COUNT 10;

namespace open_ptrack
{
  namespace opt_calibration
  {

    MultiCameraCalibration::MultiCameraCalibration(ros::NodeHandle & node_handle)
    : node_handle_(node_handle),
      image_transport_(node_handle),
      world_(boost::make_shared<BaseObject>()),
      world_set_(false),
      stop_(false),
      initialization_(true)
    {
      world_->setFrameId("/world");

      action_sub_ = node_handle_.subscribe("action", 1, &MultiCameraCalibration::actionCallback, this);

      node_handle_.param("num_cameras", num_cameras_, 0);
      node_handle_.param("base_camera", base_camera_frame_id_, std::string("./"));
      node_handle_.param("calibration_with_serials", calibration_with_serials_, false);
      calibrated_cameras_ = 0;

      double cell_width, cell_height;
      int rows, cols;

      node_handle_.param("cell_width", cell_width, 1.0);
      node_handle_.param("cell_height", cell_height, 2.0);
      node_handle_.param("rows", rows, 3);
      node_handle_.param("cols", cols, 4);

      checkerboard_ = boost::make_shared<Checkerboard>(rows, cols, cell_width, cell_height);
      checkerboard_->setFrameId("/checkerboard");

      for (int id = 0; id < num_cameras_; ++id)
      {
        Camera camera(id);

        std::stringstream ss;
        ss << "camera_" << id << "/image";
        camera.image_sub_ = image_transport_.subscribe(ss.str(), 1,
            boost::bind(&MultiCameraCalibration::imageCallback, this, _1, id));

        ss.str("");
        ss << "camera_" << id << "/camera_info";
        camera.camera_info_sub_ = node_handle_.subscribe<sensor_msgs::CameraInfo>(
            ss.str(), 1, boost::bind(&MultiCameraCalibration::cameraInfoCallback, this, _1, id));

        std::string name;
        ss.str("");
        ss << "camera_" << id << "/name";
        if (node_handle_.getParam(ss.str(), name))
          camera.name_ = name;

        camera_vector_.push_back(camera);
      }

      marker_pub_ = node_handle_.advertise<visualization_msgs::Marker>("markers", 0);

    }

    bool
    MultiCameraCalibration::initialize()
    {
      return true;
    }

    void
    MultiCameraCalibration::imageCallback(const sensor_msgs::Image::ConstPtr & msg,
        int id)
    {
      camera_vector_[id].image_msg_ = msg;
    }

    void
    MultiCameraCalibration::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr & msg,
        int id)
    {
      if (not camera_vector_[id].sensor_)
      {
        //    std::stringstream ss;
        //    ss << "/camera_" << id;

        PinholeCameraModel::ConstPtr cm(new PinholeCameraModel(*msg));
        camera_vector_[id].sensor_ = boost::make_shared<PinholeSensor>();
        camera_vector_[id].sensor_->setCameraModel(cm);
        //    camera_vector_[id].sensor_->setFrameId(ss.str());
        camera_vector_[id].sensor_->setFrameId(camera_vector_[id].name_);
      }
    }

    void
    MultiCameraCalibration::actionCallback(const std_msgs::String::ConstPtr & msg)
    {
      if (msg->data == "save")
      {
        for (int id = 0; id < num_cameras_; ++id)
        {
          if (camera_vector_[id].level_ == MAX_LEVEL)
          {
            ROS_WARN("Not all camera poses estimated!!! File not saved!!!");
            return;
          }
        }
        saveTF();
      }
      if (msg->data == "saveExtrinsicCalibration"){
        for (int id = 0; id < num_cameras_; ++id){
          if (camera_vector_[id].level_ == MAX_LEVEL){
            ROS_WARN("Not all camera poses estimated!!! File not saved!!!");
            return;
          } else {
            ROS_INFO("Saving...");
          }
        }
        ROS_INFO("Optimizing...");
        optimize();
        saveExtrinsicCalibration();
      }
      else if (msg->data == "stop")
      {
        stop_ = true;
        ROS_INFO("Optimizing...");
        optimize();
        //    ROS_INFO("Saving...");
        //    saveTF();
        //    saveCameraAndFrames();
      }
    }

    bool
    MultiCameraCalibration::findCheckerboard(cv::Mat & image,
        int id,
        typename PinholeView<Checkerboard>::Ptr & color_view)
    {
      Types::Point2Matrix corners(checkerboard_->rows(), checkerboard_->cols());
      finder_.setImage(image);
      if (finder_.find(*checkerboard_, corners))
      {
        std::stringstream ss; ss << id;

        color_view = boost::make_shared<PinholeView<Checkerboard> >();
        color_view->setId(ss.str());
        color_view->setObject(checkerboard_);
        color_view->setPoints(corners);
        color_view->setSensor(camera_vector_[id].sensor_);

        Checkerboard checkerboard(*color_view);

        visualization_msgs::Marker checkerboard_marker;
        checkerboard_marker.ns = "checkerboard";
        checkerboard_marker.id = id;

        checkerboard.toMarker(checkerboard_marker);
        marker_pub_.publish(checkerboard_marker);

        geometry_msgs::TransformStamped transform_msg;
        checkerboard.toTF(transform_msg);
        tf_pub_.sendTransform(transform_msg);

        return true;
      }
      return false;
    }

    void
    MultiCameraCalibration::spin()
    {
      ros::Rate rate(5.0);
      bool finished = false;    
      int last_optimization_ = 0;
      while (ros::ok())
      {
        ros::spinOnce();

        DataMap view_map;

        if (initialization_)
        {
          for (int id = 0; id < num_cameras_; ++id)
          {
            const Camera & camera = camera_vector_[id];

            if (not camera.image_msg_)
              continue;
            try
            {
              cv_bridge::CvImage::Ptr image_ptr = cv_bridge::toCvCopy(camera.image_msg_,
                  sensor_msgs::image_encodings::BGR8);
              PinholeView<Checkerboard>::Ptr color_view;

              if (findCheckerboard(image_ptr->image, id, color_view))
                view_map[id] = color_view;

              geometry_msgs::TransformStamped transform_msg;
              if (camera.sensor_->toTF(transform_msg))
                tf_pub_.sendTransform(transform_msg);

            }
            catch (cv_bridge::Exception & ex)
            {
              ROS_ERROR("cv_bridge exception: %s", ex.what());
              return;
            }
          }

          if (not world_set_ and view_map.size() > 0) // Set /world
          {
            for (int id = 0; id < num_cameras_; ++id)
            {
              Camera & camera = camera_vector_[id];
              if (view_map.find(id) != view_map.end())
              {
                if (camera.level_ == MAX_LEVEL)
                {
                  std::cout << camera.name_ << " calibrated!"<< std::endl;
                  calibrated_cameras_++;
                }

                camera.sensor_->setParent(world_);
                camera.level_ = 0;
                world_set_ = true;
                break;
              }
            }
          }

          if (view_map.size() > 1) // At least 2 cameras
          {
            data_vec_.push_back(view_map);

            int min_level = MAX_LEVEL;
            int min_id;
            for (int id = 0; id < num_cameras_; ++id)
            {
              const Camera & camera = camera_vector_[id];
              if (view_map.find(id) != view_map.end())
              {
                if (camera.level_ < min_level)
                {
                  min_level = camera.level_;
                  min_id = id;
                }
              }
            }

            if (min_level < MAX_LEVEL) // At least one already in tree
            {
              for (int id = 0; id < num_cameras_; ++id)
              {
                Camera & camera = camera_vector_[id];
                const Camera & min_camera = camera_vector_[min_id];

                if (id != min_id and view_map.find(id) != view_map.end())
                {

                  Checkerboard checkerboard(*view_map[id]);
                  double camera_error = std::abs(checkerboard.plane().normal().dot(Types::Vector3::UnitZ()))
                  * checkerboard.center().squaredNorm();

                  if (camera.level_ > min_level and camera_error < camera.min_error_)
                  {
                    Checkerboard min_checkerboard(*view_map[min_id]);

                    double distance = (min_checkerboard.center() - checkerboard.center()).norm();

                    if (camera.level_ == MAX_LEVEL)
                    {
                      calibrated_cameras_++;
                      std::cout << camera.name_ << " calibrated!"<< std::endl;
                    }

                    camera.sensor_->setParent(min_camera.sensor_);
                    camera.sensor_->setPose(min_checkerboard.pose() * checkerboard.pose().inverse());

                    camera.min_error_ = camera_error;
                    camera.distance_ = distance;
                    camera.level_ = min_level + 1;
                  }
                }
              }
            }

            // If all cameras calibrated, write to save results:
            if ((calibrated_cameras_ >= num_cameras_) & !finished)
            {
              std::cout << "All cameras calibrated. Now calibrate the global reference frame and save!" << std::endl;
              finished = true;
            }
          }

          initialization_ = (data_vec_.size() < 20);
          for (int id = 0; id < num_cameras_; ++id)
          {
            const Camera & camera = camera_vector_[id];
            if (camera.level_ == MAX_LEVEL)
            {
              initialization_ = true;
              break;
            }
          }
        }
        else
        {
          for (int id = 0; id < num_cameras_; ++id)
          {
            const Camera & camera = camera_vector_[id];

            if (not camera.image_msg_)
              continue;
            try
            {
              cv_bridge::CvImage::Ptr image_ptr = cv_bridge::toCvCopy(camera.image_msg_,
                  sensor_msgs::image_encodings::BGR8);
              PinholeView<Checkerboard>::Ptr color_view;

              if (findCheckerboard(image_ptr->image, id, color_view))
                view_map[id] = color_view;

              geometry_msgs::TransformStamped transform_msg;
              if (camera.sensor_->toTF(transform_msg))
                tf_pub_.sendTransform(transform_msg);

            }
            catch (cv_bridge::Exception & ex)
            {
              ROS_ERROR("cv_bridge exception: %s", ex.what());
              return;
            }
          }

          if (view_map.size() > 1) // At least 2 cameras
          {
            data_vec_.push_back(view_map);
            if (last_optimization_ == 0)
            {
              ROS_INFO("Optimizing...");
              optimize();
              last_optimization_ = OPTIMIZATION_COUNT;
            }
            else
              last_optimization_--;
          }

        }
        rate.sleep();
      }
      while (ros::ok())
      {
        ros::spinOnce();

        for (int id = 0; id < num_cameras_; ++id)
        {
          const Camera & camera = camera_vector_[id];

          if (not camera.image_msg_)
            continue;
          try
          {
            cv_bridge::CvImage::Ptr image_ptr = cv_bridge::toCvCopy(camera.image_msg_, sensor_msgs::image_encodings::BGR8);
            PinholeView<Checkerboard>::Ptr color_view;

            findCheckerboard(image_ptr->image, id, color_view);

            geometry_msgs::TransformStamped transform_msg;
            if (camera.sensor_->toTF(transform_msg))
              tf_pub_.sendTransform(transform_msg);

          }
          catch (cv_bridge::Exception & ex)
          {
            ROS_ERROR("cv_bridge exception: %s", ex.what());
            return;
          }
        }

        rate.sleep();
      }

    }

    class CheckerboardError
    {
      public:

        CheckerboardError(const PinholeCameraModel::ConstPtr & camera_model,
            const Checkerboard::ConstPtr & checkerboard,
            const Types::Point2Matrix & image_corners)
      : camera_model_(camera_model),
        checkerboard_(checkerboard),
        image_corners_(image_corners)
      {
      }

        template <typename T>
        bool operator ()(const T * const checkerboard_pose,
            T * residuals) const
        {

          typename Types_<T>::Vector3 checkerboard_r_vec(checkerboard_pose[0], checkerboard_pose[1], checkerboard_pose[2]);
          typename Types_<T>::AngleAxis checkerboard_r(checkerboard_r_vec.norm(), checkerboard_r_vec.normalized());
          typename Types_<T>::Translation3 checkerboard_t(checkerboard_pose[3], checkerboard_pose[4], checkerboard_pose[5]);

          typename Types_<T>::Transform checkerboard_pose_eigen = checkerboard_t * checkerboard_r;

          typename Types_<T>::Point3Matrix cb_corners(checkerboard_->cols(), checkerboard_->rows());
          cb_corners.matrix() = checkerboard_pose_eigen * checkerboard_->corners().matrix().cast<T>();

          typename Types_<T>::Point2Matrix reprojected_corners = camera_model_->project3dToPixel<T>(cb_corners);

          for (size_t i = 0; i < cb_corners.size(); ++i)
            residuals[i] = T((reprojected_corners[i] - image_corners_[i].cast<T>()).norm());

          return true;
        }

      private:

        const PinholeCameraModel::ConstPtr & camera_model_;
        const Checkerboard::ConstPtr & checkerboard_;
        const Types::Point2Matrix & image_corners_;

    };

    class GlobalError
    {
      public:

        GlobalError(const PinholeCameraModel::ConstPtr & camera_model,
            const Checkerboard::ConstPtr & checkerboard,
            const Types::Point2Matrix & image_corners)
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
          typename Types_<T>::Vector3 sensor_r_vec(sensor_pose[0], sensor_pose[1], sensor_pose[2]);
          typename Types_<T>::AngleAxis sensor_r(sensor_r_vec.norm(), sensor_r_vec.normalized());
          typename Types_<T>::Translation3 sensor_t(sensor_pose[3], sensor_pose[4], sensor_pose[5]);

          typename Types_<T>::Transform sensor_pose_eigen = Types_<T>::Transform::Identity() * sensor_t;

          if (sensor_r_vec.norm() != T(0))
            sensor_pose_eigen = sensor_t * sensor_r;

          typename Types_<T>::Vector3 checkerboard_r_vec(checkerboard_pose[0], checkerboard_pose[1], checkerboard_pose[2]);
          typename Types_<T>::AngleAxis checkerboard_r(checkerboard_r_vec.norm(), checkerboard_r_vec.normalized());
          typename Types_<T>::Translation3 checkerboard_t(checkerboard_pose[3], checkerboard_pose[4], checkerboard_pose[5]);

          typename Types_<T>::Transform checkerboard_pose_eigen = checkerboard_t * checkerboard_r;

          typename Types_<T>::Point3Matrix cb_corners(checkerboard_->cols(), checkerboard_->rows());
          cb_corners.matrix() = sensor_pose_eigen.inverse() * checkerboard_pose_eigen
              * checkerboard_->corners().matrix().cast<T>();

          typename Types_<T>::Point2Matrix reprojected_corners = camera_model_->project3dToPixel<T>(cb_corners);

          for (size_t i = 0; i < cb_corners.size(); ++i)
            residuals[i] = T((reprojected_corners[i] - image_corners_[i].cast<T>()).norm());

          return true;
        }

      private:

        const PinholeCameraModel::ConstPtr & camera_model_;
        const Checkerboard::ConstPtr & checkerboard_;
        const Types::Point2Matrix & image_corners_;

    };

    void MultiCameraCalibration::optimize()
    {

      ceres::Problem problem;
      Eigen::Matrix<Types::Scalar, Eigen::Dynamic, 6, Eigen::DontAlign | Eigen::RowMajor> cb_data(data_vec_.size(), 6);
      Eigen::Matrix<Types::Scalar, Eigen::Dynamic, 6, Eigen::DontAlign | Eigen::RowMajor> camera_data(camera_vector_.size(),
          6);
      for (size_t i = 0; i < camera_vector_.size(); ++i)
      {
        const Camera & camera = camera_vector_[i];
        Types::Pose pose = camera.sensor_->pose();
        BaseObject::ConstPtr parent = camera.sensor_->parent();
        while (parent)
        {
          camera.sensor_->setParent(parent);
          pose = parent->pose() * pose; //TODO controllare
          parent = parent->parent();
        }

        Types::AngleAxis rotation = Types::AngleAxis(pose.linear());
        camera_data.row(i).head<3>() = rotation.angle() * rotation.axis();
        camera_data.row(i).tail<3>() = pose.translation();
      }

      for (size_t i = 0; i < data_vec_.size(); ++i)
      {
        DataMap & data_map = data_vec_[i];
        DataMap::iterator it = data_map.begin();
        const int id = it->first;
        const PinholeView<Checkerboard>::Ptr & view = it->second;
        const Camera & camera = camera_vector_[id];
        Types::Pose pose = camera.sensor_->cameraModel()->estimatePose(view->points(), view->object()->points());
        BaseObject::ConstPtr parent = camera.sensor_;
        while (parent)
        {
          pose = parent->pose() * pose;
          parent = parent->parent();
        }

        Types::AngleAxis rotation = Types::AngleAxis(pose.linear());
        cb_data.row(i).head<3>() = rotation.angle() * rotation.axis();
        cb_data.row(i).tail<3>() = pose.translation();

        for (DataMap::iterator it = data_map.begin(); it != data_map.end(); ++it)
        {
          const int id = it->first;
          PinholeView<Checkerboard>::Ptr & view = it->second;
          const Camera & camera = camera_vector_[id];

          if (camera.level_ > 0)
          {
            GlobalError * error = new GlobalError(camera.sensor_->cameraModel(), view->object(), view->points());

            typedef ceres::AutoDiffCostFunction<GlobalError, ceres::DYNAMIC, 6, 6> GlobalErrorFunction;

            ceres::CostFunction * cost_function = new GlobalErrorFunction(error, checkerboard_->size());
            problem.AddResidualBlock(cost_function, NULL, camera_data.row(id).data(), cb_data.row(i).data());
          }
          else
          {
            CheckerboardError * error = new CheckerboardError(camera.sensor_->cameraModel(),
                view->object(),
                view->points());

            typedef ceres::AutoDiffCostFunction<CheckerboardError, ceres::DYNAMIC, 6> CheckerboardErrorFunction;

            ceres::CostFunction * cost_function = new CheckerboardErrorFunction(error, checkerboard_->size());
            problem.AddResidualBlock(cost_function, NULL, cb_data.row(i).data());
          }

        }
      }

      ceres::Solver::Options options;
      options.linear_solver_type = ceres::SPARSE_SCHUR;
      options.max_num_iterations = 50;
      //  options.minimizer_progress_to_stdout = true;
      options.num_threads = 8;

      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);
      for (size_t i = 0; i < camera_vector_.size(); ++i)
      {
        std::cout << i << ": " << camera_data.row(i) << std::endl;
        Camera & camera = camera_vector_[i];
        if (camera_data.row(i).head<3>().norm() != 0)
        {
          Types::AngleAxis rotation(camera_data.row(i).head<3>().norm(), camera_data.row(i).head<3>().normalized());
          Types::Translation3 translation(camera_data.row(i).tail<3>());
          camera.sensor_->setPose(translation * rotation);
        }
        else
        {
          Types::Translation3 translation(camera_data.row(i).tail<3>());
          camera.sensor_->setPose(Types::Pose::Identity() * translation);
        }
      }
    }


    void
    MultiCameraCalibration::saveWorldToCam()
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

        tf::StampedTransform transform;
        tf::Transform transform_final;
        //        tf::StampedTransform link_transform;

        // Write number of cameras:
        launch_file << "  <!-- Network parameters -->" << std::endl
            << "  <arg name=\"num_cameras\" value=\"" << num_cameras_ << "\" />" << std::endl;

        // Write camera ID and names:
        for (int id = 0; id < num_cameras_; ++id)
        {
          launch_file << "  <arg name=\"camera" << id << "_id\" value=\"" << camera_vector_[id].sensor_->frameId() << "\" />" << std::endl;
          launch_file << "  <arg name=\"camera" << id << "_name\" default=\"$(arg camera" << id << "_id)\" />" << std::endl;
        }
        launch_file << std::endl;
        //        launch_file << "  <arg name=\"period\" default=\"10\" />" << std::endl << std::endl;

        // Write TF transforms between cameras and world frame:
        launch_file << "  <!-- World<->camera transforms -->" << std::endl;
        for (int id = 0; id < num_cameras_; ++id)
        {
          std::stringstream ss;
          ss << id;

          // Read transform between camera optical frame and checkerboard:
          tfListener.lookupTransform("checkerboard_"+ss.str(), camera_vector_[id].sensor_->frameId()+(optical_frame_string.str()), ros::Time(0), transform);

          // Apply rotation of 90° in X and -90° in Z in order to move from optical frame to camera link frame:
          tf::Matrix3x3 temp_rotation;
          temp_rotation.setRPY(M_PI, 0, -M_PI/2);
          tf::Vector3 temp_origin = tf::Vector3(0.0,0.0,0.0);
          tf::Transform flip_z = tf::Transform(temp_rotation, temp_origin);
          transform_final = flip_z * tf::Transform(transform);

          // Extract rotation angles from camera pose:
          double yaw, pitch, roll;
          tf::Matrix3x3(transform_final.getRotation()).getRPY(roll, pitch, yaw);

          // Write transform between camera and world frame to file:
          launch_file << "  <node pkg=\"tf\" type=\"static_transform_publisher\" name=\""
              << camera_vector_[id].sensor_->frameId().substr(1) << "_broadcaster\" args=\""
              << transform_final.getOrigin().x() << " " << transform_final.getOrigin().y() << " " << transform_final.getOrigin().z() << " "
              << yaw << " " << pitch << " " << roll << " " << world_->frameId() << " "
              << camera_vector_[id].sensor_->frameId() << " 100\" />\n\n";

          //          // Extract transform between camera_link and camera_rgb_optical_frame:
          //          tfListener.lookupTransform(camera_vector_[id].sensor_->frameId()+(optical_frame_string.str()), camera_vector_[id].sensor_->frameId()+(link_string.str()), ros::Time(0), link_transform);

          // Write transform between camera_link and camera_rgb_optical_frame to file:
          launch_file << "  <node pkg=\"tf\" type=\"static_transform_publisher\" name=\""
              << camera_vector_[id].sensor_->frameId().substr(1) << "_broadcaster2\" args=\" -0.045 0 0 "
              //<< link_transform.getRotation().x() << " " << link_transform.getRotation().y() << " " << link_transform.getRotation().z() << " "
              << "1.57 -1.57 0 "
              << camera_vector_[id].sensor_->frameId() << " "
              << camera_vector_[id].sensor_->frameId()+link_string.str() << " 100\" />\n\n";

        }

        launch_file << "</launch>" << std::endl;
      }
      launch_file.close();
      ROS_INFO_STREAM(file_name << " created!");
    }

    void
    MultiCameraCalibration::saveExtrinsicCalibration()
    {
      // save tf between camera and world coordinate system ( chessboard ) to launch file
      std::string file_name = ros::package::getPath("opt_calibration") + "/launch/multicamera_calibration_results.launch";
      std::ofstream launch_file;
      launch_file.open(file_name.c_str());

      std::stringstream optical_frame_string, link_string;
      optical_frame_string << "_rgb_optical_frame";
      link_string << "_link";

      tf::Transform world_to_base;
      if (launch_file.is_open())
      {
        launch_file << "<launch>" << std::endl << std::endl;

        tf::StampedTransform transform;
        tf::Transform transform_final;
        //        tf::StampedTransform link_transform;

        // Write number of cameras:
        launch_file << "  <!-- Network parameters -->" << std::endl
            << "  <arg name=\"num_cameras\" value=\"" << num_cameras_ << "\" />" << std::endl;

        // Write camera ID and names:
        for (int id = 0; id < num_cameras_; ++id)
        {
          launch_file << "  <arg name=\"camera" << id << "_id\" value=\"" << camera_vector_[id].sensor_->frameId() << "\" />" << std::endl;
          launch_file << "  <arg name=\"camera" << id << "_name\" default=\"$(arg camera" << id << "_id)\" />" << std::endl;
        }
        launch_file << std::endl;
        //        launch_file << "  <arg name=\"period\" default=\"10\" />" << std::endl << std::endl;

        // Write TF transforms between cameras and world frame:
        launch_file << "  <!-- Transforms tree -->" << std::endl;
        for (int id = 0; id < num_cameras_; ++id)
        {
          // If this camera is not the base camera:
          if (strcmp(camera_vector_[id].sensor_->frameId().c_str(), ("/" + base_camera_frame_id_).c_str()) > 0)
          {
            const Types::Pose & pose = camera_vector_[id].sensor_->pose();
            Types::Quaternion q(pose.linear());
            launch_file << "  <node pkg=\"tf\" type=\"static_transform_publisher\" name=\""
                << camera_vector_[id].sensor_->frameId().substr(1) << "_broadcaster\" args=\""
                << pose.translation().transpose() << " " << q.coeffs().transpose() << " ";

            if (strcmp(camera_vector_[id].sensor_->parent()->frameId().c_str(), world_->frameId().c_str()) == 0)
            { // If the parent is the world, refer to the base_camera instead:
              launch_file << "/" << base_camera_frame_id_;
            }
            else
            { // otherwise, refer to the parent:
              launch_file << camera_vector_[id].sensor_->parent()->frameId();
            }

            launch_file << " " << camera_vector_[id].sensor_->frameId()
                << " 100\" />\n\n";
          }

          if (std::strcmp(camera_vector_[id].name_.substr(1,2).c_str(), "SR"))    // if Kinect
          {
            // Write transform between camera_link and camera_rgb_optical_frame to file:
            launch_file << "  <node pkg=\"tf\" type=\"static_transform_publisher\" name=\""
                << camera_vector_[id].sensor_->frameId().substr(1) << "_broadcaster2\" args=\" -0.045 0 0 "
                //<< link_transform.getRotation().x() << " " << link_transform.getRotation().y() << " " << link_transform.getRotation().z() << " "
                << "1.57 -1.57 0 "
                << camera_vector_[id].sensor_->frameId() << " "
                << camera_vector_[id].sensor_->frameId()+link_string.str() << " 100\" />\n\n";
          }

          // If the camera is the base camera used to calibrate the network with the ground, write transform to world:
          if (strcmp(camera_vector_[id].sensor_->frameId().c_str(), ("/" + base_camera_frame_id_).c_str()) == 0)
          {
            std::stringstream ss;
            ss << id;

            // Read transform between camera optical frame and checkerboard:
            tfListener.lookupTransform("checkerboard_"+ss.str(), camera_vector_[id].sensor_->frameId()+(optical_frame_string.str()), ros::Time(0), transform);

            // Apply rotation of 90° in X and -90° in Z in order to move from optical frame to camera link frame:
            tf::Matrix3x3 temp_rotation;
            temp_rotation.setRPY(M_PI, 0, -M_PI/2);
            tf::Vector3 temp_origin = tf::Vector3(0.0,0.0,0.0);
            tf::Transform flip_z = tf::Transform(temp_rotation, temp_origin);
            transform_final = flip_z * tf::Transform(transform);

            // Extract rotation angles from camera pose:
            double yaw, pitch, roll;
            tf::Matrix3x3(transform_final.getRotation()).getRPY(roll, pitch, yaw);

            // Write transform between camera and world frame to file:
            launch_file << "  <node pkg=\"tf\" type=\"static_transform_publisher\" name=\""
                << "ground_broadcaster\" args=\""
                << transform_final.getOrigin().x() << " " << transform_final.getOrigin().y() << " " << transform_final.getOrigin().z() << " "
                << yaw << " " << pitch << " " << roll << " "
                << world_->frameId() << " " << camera_vector_[id].sensor_->frameId()
                << " 100\" />\n\n";

            world_to_base = transform_final;

            //            // Send world to base_camera transform over the network:
            //            static tf::TransformBroadcaster base_tf_publisher;
            //            base_tf_publisher.sendTransform(tf::StampedTransform(transform_final, ros::Time::now(), "/world", camera_vector_[id].sensor_->frameId()));

          }
        }

        launch_file << "</launch>" << std::endl;
      }
      launch_file.close();
      ROS_INFO_STREAM(file_name << " created!");

      // Save launch files to be used to perform people detection with every sensor:
      for (unsigned int i = 0; i < num_cameras_; i++)
      {
        std::string serial_number = camera_vector_[i].sensor_->frameId().substr(1, camera_vector_[i].sensor_->frameId().length()-1);
        std::string file_name = ros::package::getPath("detection") + "/launch/detection_node_" + serial_number + ".launch";
        std::ofstream launch_file;
        launch_file.open(file_name.c_str());
        if (launch_file.is_open())
        {
          launch_file << "<launch>" << std::endl << std::endl;

          launch_file << "  <!-- Load calibration results -->" << std::endl
              << "  <include file=\"$(find opt_calibration)/launch/multicamera_calibration_results.launch\"/>" << std::endl << std::endl;

          launch_file << "  <!-- Camera ID -->" << std::endl
              << "  <arg name=\"camera_id\" value=\"" << serial_number << "\" />" << std::endl << std::endl
              << "  <!-- Detection node -->" << std::endl;

          if (std::strcmp(camera_vector_[i].name_.substr(1,2).c_str(), "SR"))    // if Kinect
          {
            if (calibration_with_serials_)
              launch_file << "  <include file=\"$(find detection)/launch/detector_serial.launch\">" << std::endl;
            else
              launch_file << "  <include file=\"$(find detection)/launch/detector_with_name.launch\">" << std::endl;
          }
          else                                                                   // if SwissRanger
          {
            std::string device_ip = camera_vector_[i].name_;
            replace(device_ip.begin(), device_ip.end(), '_', '.');
            device_ip = device_ip.substr(4, device_ip.length()-4);
            launch_file << "  <include file=\"$(find detection)/launch/detector_with_name_sr.launch\">" << std::endl
                << "    <arg name=\"device_ip\" value=\"" << device_ip << "\" />" << std::endl;
          }

          launch_file << "    <arg name=\"camera_id\" value=\"$(arg camera_id)\" />" << std::endl
              << "    <arg name=\"ground_from_calibration\" value=\"true\" />" << std::endl
              << "  </include>" << std::endl;

          launch_file << "</launch>" << std::endl;
        }
        launch_file.close();
        ROS_INFO_STREAM(file_name << " created!");
      }

      //      // Save file with all cameras position with respect to the world frame:
      //      file_name = ros::package::getPath("detection") + "/launch/camera_poses.txt";
      //      std::ofstream poses_file;
      //      poses_file.open(file_name.c_str());
      //      for (unsigned int i = 0; i < num_cameras_; i++)
      //      {
      //        // Read transform between world and camera reference frame:
      //        tf::StampedTransform worldToCamStampedTransform;
      //        tf::Transform worldToCamTransform;
      //        tf::Vector3 origin;
      //        tf::Quaternion rotation;
      //        try
      //        {
      //          tfListener.waitForTransform(camera_vector_[i].sensor_->frameId(), "/world", ros::Time(0), ros::Duration(3.0), ros::Duration(0.01));
      //          tfListener.lookupTransform(camera_vector_[i].sensor_->frameId(), "/world", ros::Time(0), worldToCamStampedTransform);
      //          worldToCamTransform =  tf::Transform(worldToCamStampedTransform) * world_to_base;
      ////          worldToCamTransform =  tf::Transform(worldToCamStampedTransform) * world_to_base.inverse();
      ////          worldToCamTransform =  tf::Transform(worldToCamStampedTransform.inverse()) * world_to_base ;
      //          origin = worldToCamTransform.getOrigin();
      //          rotation = worldToCamTransform.getRotation();
      //        }
      //        catch (tf::TransformException ex)
      //        {
      //          ROS_ERROR("%s",ex.what());
      //        }
      //        poses_file << camera_vector_[i].sensor_->frameId() << "->" << "/world" << ":"
      //                   << origin.x() << "," << origin.y() << "," << origin.z() << ","
      //                   << rotation.getX() << "," << rotation.getY() << "," << rotation.getZ() << "," << rotation.getW() << std::endl;
      //      }
      //      poses_file.close();
      //      ROS_INFO_STREAM(file_name << " created!");
    }

    void
    MultiCameraCalibration::saveTF()
    {
      //save tf to launch file
      std::string file_name = ros::package::getPath("opt_calibration") + "/launch/frames.launch";
      std::ofstream launch_file;
      launch_file.open(file_name.c_str());
      if (launch_file.is_open())
      {
        launch_file << "<launch>" << std::endl << std::endl;

        for (int id = 0; id < num_cameras_; ++id)
        {
          const Types::Pose & pose = camera_vector_[id].sensor_->pose();
          Types::Quaternion q(pose.linear());
          launch_file << "<node pkg=\"tf\" type=\"static_transform_publisher\" name=\""
              << camera_vector_[id].sensor_->frameId().substr(1) << "_broadcaster\" args=\""
              << pose.translation().transpose() << " " << q.coeffs().transpose() << " "
              << camera_vector_[id].sensor_->parent()->frameId() << " " << camera_vector_[id].sensor_->frameId()
              << " 100\" />\n\n";
        }
        launch_file << "</launch>" << std::endl;
      }
      launch_file.close();
      ROS_INFO_STREAM(file_name << " created!");
    }

  } /* namespace opt_calibration */
} /* namespace open_ptrack */

int
main(int argc, char ** argv)
{
  ros::init(argc, argv, "opt_calibration");
  ros::NodeHandle node_handle("~");

  try
  {
    open_ptrack::opt_calibration::MultiCameraCalibration calib_node(node_handle);
    if (not calib_node.initialize())
      return 0;
    calib_node.spin();
  }
  catch (std::runtime_error & error)
  {
    ROS_FATAL("Calibration error: %s", error.what());
    return 1;
  }

  return 0;
}
