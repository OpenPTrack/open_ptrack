/*
 *  Copyright (c) 2015- Open Perception, Inc.
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
 *  Author: Matteo Munaro [matteo.munaro@dei.unipd.it]
 *          Filippo Basso [bassofil@dei.unipd.it]
 */

#include <fstream>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

#include <open_ptrack/opt_calibration/opt_define_reference_frame.h>

struct callback_args{
  // structure used to pass arguments to the callback function
  cv::Point clicked_point;
  int button;
};

void
mouseCallback(int event, int x, int y, int flags, void* args)
{
  struct callback_args* data = (struct callback_args *)args;

  if  ( event == cv::EVENT_LBUTTONDOWN )
  {
    data->clicked_point.x = x;
    data->clicked_point.y = y;
    data->button = 0;
//    std::cout << "Left button of the mouse is clicked - position (" << data->clicked_point.x << ", " << data->clicked_point.y << ")" << std::endl;
  }
  else if  ( event == cv::EVENT_RBUTTONDOWN )
  {
    data->clicked_point.x = -1;
    data->clicked_point.y = -1;
    data->button = 2;
//    std::cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
  }
  else if  ( event == cv::EVENT_MBUTTONDOWN )
  {
    data->clicked_point.x = -1;
    data->clicked_point.y = -1;
    data->button = 1;
//    std::cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
  }
}

namespace open_ptrack
{
namespace opt_calibration
{

  OPTDefineReferenceFrame::OPTDefineReferenceFrame(const ros::NodeHandle & node_handle)
  : node_handle_(node_handle),
    image_transport_(node_handle)
{
  // Read frame_id of all sensors and read corresponding pose:
  XmlRpc::XmlRpcValue network;
  node_handle_.getParam("network", network);
  int k = 0;
  for (unsigned i = 0; i < network.size(); i++)
  {
    for (unsigned j = 0; j < network[i]["sensors"].size(); j++)
    {
      std::string frame_id = network[i]["sensors"][j]["id"];
      camera_frame_ids_.push_back(frame_id);

      std::string pose_s = "/poses/" + frame_id;
      std::string inverse_pose_s = "/inverse_poses/" + frame_id;
      if (not node_handle_.hasParam(pose_s))
        ROS_FATAL_STREAM("No \"" << pose_s << "\" parameter found!! Has \"conf/camera_poses.yaml\" been loaded with \"rosparam load ...\"?");

      // Read pose:
      Translation3 t;
      node_handle_.getParam(pose_s + "/translation/x", t.x());
      node_handle_.getParam(pose_s + "/translation/y", t.y());
      node_handle_.getParam(pose_s + "/translation/z", t.z());

      Quaternion q;
      node_handle_.getParam(pose_s + "/rotation/x", q.x());
      node_handle_.getParam(pose_s + "/rotation/y", q.y());
      node_handle_.getParam(pose_s + "/rotation/z", q.z());
      node_handle_.getParam(pose_s + "/rotation/w", q.w());

      camera_poses_.push_back(Pose::Identity());
      camera_poses_[k].linear() = q.toRotationMatrix();
      camera_poses_[k].translation() = t.vector();

      // Read inverse pose
      node_handle_.getParam(inverse_pose_s + "/translation/x", t.x());
      node_handle_.getParam(inverse_pose_s + "/translation/y", t.y());
      node_handle_.getParam(inverse_pose_s + "/translation/z", t.z());

      node_handle_.getParam(inverse_pose_s + "/rotation/x", q.x());
      node_handle_.getParam(inverse_pose_s + "/rotation/y", q.y());
      node_handle_.getParam(inverse_pose_s + "/rotation/z", q.z());
      node_handle_.getParam(inverse_pose_s + "/rotation/w", q.w());

      inverse_camera_poses_.push_back(Pose::Identity());
      inverse_camera_poses_[k].linear() = q.toRotationMatrix();
      inverse_camera_poses_[k].translation() = t.vector();

      k++;
    }
  }

  node_handle_.param("num_sensors", num_sensors_, 0);

  for (int i = 0; i < num_sensors_; ++i)
  {
    std::stringstream ss;

    std::string type_s;
    ss.str("");
    ss << "sensor_" << i << "/type";
    if (not node_handle_.getParam(ss.str(), type_s))
      ROS_FATAL_STREAM("No \"" << ss.str() << "\" parameter found!!");

    ss.str("");
    ss << "/sensor_" << i;
    std::string frame_id = ss.str();

    ss.str("");
    ss << "sensor_" << i << "/name";
    node_handle_.param(ss.str(), frame_id, frame_id);

    ROSDevice::Ptr ros_device;

    if (type_s == "pinhole_rgb")
    {
      PinholeRGBDevice::Ptr device = boost::make_shared<PinholeRGBDevice>(frame_id);
      pinhole_vec_.push_back(device);
      ros_device = device;
      ROS_INFO_STREAM(device->frameId() << " added.");
    }
    else if (type_s == "kinect1")
    {
      KinectDevice::Ptr device = boost::make_shared<KinectDevice>(frame_id, frame_id + "_depth");
      kinect_vec_.push_back(device);
      ros_device = device;
    }
    else if (type_s == "sr4500")
    {
      SwissRangerDevice::Ptr device = boost::make_shared<SwissRangerDevice>(frame_id);
      swiss_ranger_vec_.push_back(device);
      ros_device = device;
    }
    else
    {
      ROS_FATAL_STREAM("\"" << ss.str() << "\" parameter value not valid. Please use \"pinhole_rgb\", \"kinect\" or \"swiss_ranger\".");
    }

    ss.str("");
    ss << "sensor_" << i;
    ros_device->createSubscribers(node_handle_, image_transport_, ss.str());

  }
}

bool OPTDefineReferenceFrame::initialize()
{
  bool all_messages_received = false;
  ros::Rate rate(1.0);
  while (ros::ok() and not all_messages_received)
  {
    ros::spinOnce();
    all_messages_received = true;
    for (size_t i = 0; all_messages_received and i < pinhole_vec_.size(); ++i)
      all_messages_received = pinhole_vec_[i]->hasNewMessages();
    for (size_t i = 0; all_messages_received and i < kinect_vec_.size(); ++i)
      all_messages_received = kinect_vec_[i]->hasNewMessages();
    for (size_t i = 0; all_messages_received and i < swiss_ranger_vec_.size(); ++i)
      all_messages_received = swiss_ranger_vec_[i]->hasNewMessages();

    if (not all_messages_received)
      ROS_WARN_THROTTLE(5, "Not all messages received. Waiting...");
    rate.sleep();
  }

  ROS_INFO("All sensors connected.");

  calibration_ = boost::make_shared<OPTCalibration>(node_handle_);

  for (size_t i = 0; i < pinhole_vec_.size(); ++i)
  {
    const PinholeRGBDevice::Ptr & device = pinhole_vec_[i];
    calibration_->addSensor(device->sensor(), true);
    sensor_vec_.push_back(device->sensor());
    images_acquired_map_[device->frameId()] = 0;
  }

  for (size_t i = 0; i < kinect_vec_.size(); ++i) // TODO Add flags
  {
    const KinectDevice::Ptr & device = kinect_vec_[i];
    calibration_->addSensor(device->colorSensor(), true);
    calibration_->addSensor(device->depthSensor(), true);
    sensor_vec_.push_back(device->colorSensor());
    sensor_vec_.push_back(device->depthSensor());
    images_acquired_map_[device->colorFrameId()] = 0;
    images_acquired_map_[device->depthFrameId()] = 0;
  }

  for (size_t i = 0; i < swiss_ranger_vec_.size(); ++i) // TODO Add flags
  {
    const SwissRangerDevice::Ptr & device = swiss_ranger_vec_[i];
    calibration_->addSensor(device->intensitySensor(), true);
    calibration_->addSensor(device->depthSensor(), true);
    sensor_vec_.push_back(device->intensitySensor());
    sensor_vec_.push_back(device->depthSensor());
    images_acquired_map_[device->frameId()] = 0;
  }

  return true;
}

bool OPTDefineReferenceFrame::compute_and_save(const std::vector<Eigen::Vector2d>& reference_points_2d,
                                               const std::vector<Eigen::Vector3d>& reference_points_3d,
                                               std::vector<Pose> reference_camera_poses,
                                               std::vector<sensor_msgs::CameraInfo::ConstPtr> reference_camera_info)
{
  // Define ground plane:
  Eigen::Hyperplane<double, 3> ground_plane(Eigen::Vector3d(0.0, 0.0, 1.0), 0.0);

  // Find world coordinates of the first reference point:
  std::vector<Eigen::Vector3d> original_points_3d;
  for (unsigned int i = 0; i < reference_points_3d.size(); i++)
  {
    // Define points of the selected camera ray in camera coordinates (3D line passing through two points):
    Eigen::Vector3d camera_center(0.0, 0.0, 0.0);
    Eigen::Vector3d selected_point(reference_points_2d[i](0) - reference_camera_info[i]->K[2],
                                   reference_points_2d[i](1) - reference_camera_info[i]->K[5],
                                   (reference_camera_info[i]->K[0] + reference_camera_info[i]->K[4]) / 2);

    // Transform points to world coordinates:
    camera_center = reference_camera_poses[i] * camera_center;
    selected_point = reference_camera_poses[i] * selected_point;

    // Compute equation of the camera ray passing through the two points:
    Eigen::ParametrizedLine<double,3> camera_ray = Eigen::ParametrizedLine<double,3>::Through(camera_center, selected_point);

    // Intersect camera ray and ground plane:
    double intersection = camera_ray.intersection(ground_plane);
    original_points_3d.push_back(intersection*((selected_point-camera_center).normalized()) + camera_center);

//    std::cout << "Original coordinates: " << original_points_3d[i].matrix().transpose() << std::endl;
//    std::cout << "New coordinates: " << reference_points_3d[i].matrix().transpose() << std::endl;

  }

  // Find rotation:
  Eigen::Vector3d original_direction = original_points_3d[1] - original_points_3d[0];
  original_direction.normalize();
  Eigen::Vector3d reference_direction = reference_points_3d[1] - reference_points_3d[0];
  reference_direction.normalize();
//  std::cout << "original_direction: " << original_direction.matrix().transpose() << std::endl;
//  std::cout << "reference_direction: " << reference_direction.matrix().transpose() << std::endl;
  double correction_angle = std::acos(original_direction.dot (reference_direction));
  if ((original_direction(1) - reference_direction(1)) > 0)
    correction_angle = - correction_angle;

  std::cout << "Rotation around Z axis: " << correction_angle * 180 / M_PI << " degrees" << std::endl;

  // Apply rotation to original points:
  Pose rotation_transform(Pose::Identity());
  rotation_transform.rotate(Eigen::AngleAxis<double>(correction_angle, Eigen::Vector3d::UnitZ()));
  std::vector<Eigen::Vector3d> original_points_3d_rotated;
  for (unsigned int i = 0; i < original_points_3d.size(); i++)
  {
    original_points_3d_rotated.push_back(rotation_transform*original_points_3d[i]);
  }

  // Find translation:
  Eigen::Vector3d translation = reference_points_3d[0] - original_points_3d_rotated[0];

  std::cout << "Translation: " << translation.matrix().transpose() << std::endl;

  // Compute transformation to new reference frame:
  Pose translation_transform(Pose::Identity());
  translation_transform.translate(translation);
  Pose correction_transform(Pose::Identity());
  correction_transform = translation_transform*rotation_transform*correction_transform;

  std::vector<Eigen::Vector3d> original_rototranslated;
  original_rototranslated.push_back(correction_transform*original_points_3d[0]);
  original_rototranslated.push_back(correction_transform*original_points_3d[1]);

//  std::cout << original_rototranslated[0] << std::endl;
//  std::cout << original_rototranslated[1] << std::endl;

//  // Drawing:
//  cv::Mat arrows(cv::Size(1000, 1000), CV_8UC3);
//  arrows = cv::Mat::ones(1000, 1000, CV_8UC3);
//  cv::circle(arrows, cv::Point((unsigned int)(1000*original_rototranslated[0](0)), (unsigned int)(1000*original_rototranslated[0](1))),
//      7, cv::Scalar(255,0,0), -1);
//  cv::line(arrows, cv::Point((unsigned int)(1000*original_rototranslated[0](0)), (unsigned int)(1000*original_rototranslated[0](1))),
//      cv::Point((unsigned int)(1000*original_rototranslated[1](0)), (unsigned int)(1000*original_rototranslated[1](1))),
//      cv::Scalar(255,0,0), 5);
//  cv::circle(arrows, cv::Point((unsigned int)(1000*original_points_3d[0](0)), (unsigned int)(1000*original_points_3d[0](1))),
//        6, cv::Scalar(0,255,0), -1);
//  cv::line(arrows, cv::Point((unsigned int)(1000*original_points_3d[0](0)), (unsigned int)(1000*original_points_3d[0](1))),
//      cv::Point((unsigned int)(1000*original_points_3d[1](0)), (unsigned int)(1000*original_points_3d[1](1))),
//      cv::Scalar(0,255,0), 4);
//  cv::circle(arrows, cv::Point((unsigned int)(1000*reference_points_3d[0](0)), (unsigned int)(1000*reference_points_3d[0](1))),
//        5, cv::Scalar(0,0,255), -1);
//  cv::line(arrows, cv::Point((unsigned int)(1000*reference_points_3d[0](0)), (unsigned int)(1000*reference_points_3d[0](1))),
//      cv::Point((unsigned int)(1000*reference_points_3d[1](0)), (unsigned int)(1000*reference_points_3d[1](1))),
//      cv::Scalar(0,0,255), 3);
//
//  cv::putText(arrows, "original_rototranslated", cv::Point(10,800), 0, 0.5, cv::Scalar(255,0,0), 2);
//  cv::putText(arrows, "original_points_3d", cv::Point(10,830), 0, 0.5, cv::Scalar(0,255,0), 2);
//  cv::putText(arrows, "reference_points_3d", cv::Point(10,860), 0, 0.5, cv::Scalar(0,0,255), 2);
//
//  cv::imshow("Arrows", arrows);
//  cv::waitKey(0);

  // Save results:
  save(correction_transform);
}

bool OPTDefineReferenceFrame::save(Pose correction_transform)
{
  // Save tfs between sensors and world coordinate system (last checherboard) to file
  std::string file_name = ros::package::getPath("opt_calibration") + "/conf/camera_poses.yaml";
  std::ofstream file;
  file.open(file_name.c_str());

  if (file.is_open())
  {
    ros::Time time = ros::Time::now();
    file << "# Auto generated file." << std::endl;
    file << "calibration_id: " << time.sec << std::endl << std::endl;

    // Write TF transforms between cameras and world frame in the user-defined reference frame:
    file << "# Poses w.r.t. the \"world\" reference frame" << std::endl;
    file << "poses:" << std::endl;
    for (size_t i = 0; i < camera_poses_.size(); ++i)
    {
      Pose pose = correction_transform * camera_poses_[i];

      file << "  " << camera_frame_ids_[i] << ":" << std::endl;

      file << "    translation:" << std::endl
           << "      x: " << pose.translation().x() << std::endl
           << "      y: " << pose.translation().y() << std::endl
           << "      z: " << pose.translation().z() << std::endl;

      Quaternion rotation(pose.rotation());
      file << "    rotation:" << std::endl
           << "      x: " << rotation.x() << std::endl
           << "      y: " << rotation.y() << std::endl
           << "      z: " << rotation.z() << std::endl
           << "      w: " << rotation.w() << std::endl;

    }

    file << std::endl << "# Inverse poses" << std::endl;
    file << "inverse_poses:" << std::endl;
    for (size_t i = 0; i < camera_poses_.size(); ++i)
    {
      Pose pose = correction_transform * camera_poses_[i];
      pose = pose.inverse();

      file << "  " << camera_frame_ids_[i] << ":" << std::endl;

      file << "    translation:" << std::endl
           << "      x: " << pose.translation().x() << std::endl
           << "      y: " << pose.translation().y() << std::endl
           << "      z: " << pose.translation().z() << std::endl;

      Quaternion rotation(pose.rotation());
      file << "    rotation:" << std::endl
           << "      x: " << rotation.x() << std::endl
           << "      y: " << rotation.y() << std::endl
           << "      z: " << rotation.z() << std::endl
           << "      w: " << rotation.w() << std::endl;

    }
  }
  file.close();

  ROS_INFO_STREAM(file_name << " created!");

  return true;

}

void OPTDefineReferenceFrame::spin()
{
  ros::Rate rate(5.0);
  struct callback_args cb_args;

  std::vector<Eigen::Vector3d> reference_points_3d;
  std::vector<Eigen::Vector2d> reference_points_2d;
  reference_points_3d.resize(2, Eigen::Vector3d(0.0, 0.0, 0.0));
  reference_points_2d.resize(2, Eigen::Vector2d(0.0, 0.0));

  std::vector<sensor_msgs::CameraInfo::ConstPtr> reference_camera_info;
  reference_camera_info.resize(2, sensor_msgs::CameraInfo::ConstPtr());
  std::vector<Pose> reference_camera_poses;
  reference_camera_poses.resize(2, Pose::Identity());

  int count = 0;
  std::cout << std::endl << "Provide coordinates of two points in the desired reference frame." << std::endl << std::endl;

  while (ros::ok())
  {
    try
    {
      for (size_t i = 0; i < pinhole_vec_.size(); ++i)
      {
        const PinholeRGBDevice::Ptr & device = pinhole_vec_[i];

        ROS_DEBUG_STREAM("[" << device->frameId() << "] analysing image generated at: " << device->lastMessages().image_msg->header.stamp);

        //set the callback function for any mouse event
        cv::Point clicked_point(1, -1);
        cb_args.clicked_point = clicked_point;
        cb_args.button = -1;
        while (cb_args.button <= 0)
        {
          ros::spinOnce();
          calibration_->nextAcquisition();

          // Get image:
          device->convertLastMessages();
          PinholeRGBDevice::Data::Ptr data = device->lastData();
          cv::putText(data->image, device->frameId(), cv::Point(10,20), 0, 0.5, cv::Scalar::all(255), 2);
          cv::imshow("LEFT-CLICK: select reference point; RIGHT-CLICK: next image", data->image);
          cv::waitKey(1);

          // Set callback:
          cv::setMouseCallback("LEFT-CLICK: select reference point; RIGHT-CLICK: next image", mouseCallback, (void*)(&cb_args));

          // Left-click: select point
          if (cb_args.button == 0)
          {
            reference_points_2d[count] = Eigen::Vector2d(cb_args.clicked_point.x, cb_args.clicked_point.y);
            std::cout << "Selected point: (" << cb_args.clicked_point.x << ", " << cb_args.clicked_point.y << ")" << std::endl;

            cv::circle(data->image, cv::Point(cb_args.clicked_point.x,cb_args.clicked_point.y), 5, cv::Scalar(0,0,255), -1);
            cv::imshow("LEFT-CLICK: select reference point; RIGHT-CLICK: next image", data->image);
            cv::waitKey(1);

            // Ask for 3D coordinates:
            std::cout << "Please enter X coordinate of the selected point in the desired reference frame: ";
            std::cin >> reference_points_3d[count](0);

            std::cout << "Please enter Y coordinate of the selected point in the desired reference frame: ";
            std::cin >> reference_points_3d[count](1);

            // Add camera info:
            reference_camera_info[count] = device->lastMessages().camera_info_msg;

            // Search for camera pose with the right frame_id:
            std::string query_frame_id = device->frameId().substr(1, device->frameId().length()-1);
            for (unsigned int num = 0; num < camera_frame_ids_.size(); num++)
            {
              if (strcmp(query_frame_id.c_str(), camera_frame_ids_[num].c_str()) == 0)
              {
                reference_camera_poses[count] = camera_poses_[num];
              }
            }

            count = ++count % reference_points_2d.size();
            if (count == 0)
              std::cout << "Enough points selected. Middle click on the image for computing and saving the new reference frame." << std::endl << std::endl;

            cb_args.button = -1;
          }
        }

        if (cb_args.button == 1)
        { // Middle click: reference points inserted, compute reference frame:
          compute_and_save (reference_points_2d, reference_points_3d, reference_camera_poses, reference_camera_info);
          break;
        }
        else
        {
          // Right click: change image
        }

      }
    }
    catch (cv_bridge::Exception & ex)
    {
      ROS_ERROR_STREAM("cv_bridge exception: " << ex.what());
      return;
    }
    catch (std::runtime_error & ex)
    {
      ROS_ERROR_STREAM("exception: " << ex.what());
      return;
    }

    if (cb_args.button == 1)
    { // Middle click: reference points inserted, compute reference frame
      break;
    }

    rate.sleep();
  }
}

} /* namespace opt_calibration */
} /* namespace open_ptrack */

int main(int argc,
         char ** argv)
{
  ros::init(argc, argv, "opt_define_reference_frame");
  ros::NodeHandle node_handle("~");

  try
  {
    open_ptrack::opt_calibration::OPTDefineReferenceFrame calib_node(node_handle);
    if (not calib_node.initialize())
      return 1;
    calib_node.spin();
  }
  catch (std::runtime_error & error)
  {
    ROS_FATAL("Calibration error: %s", error.what());
    return 2;
  }

  return 0;
}
