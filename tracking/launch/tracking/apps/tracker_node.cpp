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

#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <list>
#include <sstream>
#include <fstream>
#include <string.h>

#include <open_ptrack/opt_utils/conversions.h>
#include <open_ptrack/detection/detection.h>
#include <open_ptrack/detection/detection_source.h>
#include <open_ptrack/tracking/tracker.h>
#include <opt_msgs/Detection.h>
#include <opt_msgs/DetectionArray.h>
#include <opt_msgs/TrackArray.h>
#include <opt_msgs/IDArray.h>
//#include <open_ptrack/opt_utils/ImageConverter.h>

// Dynamic reconfigure:
#include <dynamic_reconfigure/server.h>
#include <tracking/TrackerConfig.h>

typedef tracking::TrackerConfig Config;
typedef dynamic_reconfigure::Server<Config> ReconfigureServer;

// Global variables:
std::map<std::string, open_ptrack::detection::DetectionSource*> detection_sources_map;
tf::TransformListener* tf_listener;
std::string world_frame_id;
bool output_history_pointcloud;
int output_history_size;
int detection_history_size;
bool output_markers;
bool output_image_rgb;
bool output_tracking_results;
bool output_detection_results;  // Enables/disables the publishing of detection positions to be visualized in RViz
bool vertical;
ros::Publisher results_pub;
ros::Publisher marker_pub_tmp;
ros::Publisher marker_pub;
ros::Publisher pointcloud_pub;
ros::Publisher detection_marker_pub;
ros::Publisher detection_trajectory_pub;
ros::Publisher alive_ids_pub;
size_t starting_index;
size_t detection_insert_index;
tf::Transform camera_frame_to_world_transform;
tf::Transform world_to_camera_frame_transform;
bool extrinsic_calibration;
double period;
open_ptrack::tracking::Tracker* tracker;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr history_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr detection_history_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
bool swissranger;
double min_confidence;
double min_confidence_sr;
double min_confidence_detections;
double min_confidence_detections_sr;
std::vector<cv::Vec3f> camera_colors;     // vector containing colors to use to identify cameras in the network
std::map<std::string, int> color_map;     // map between camera frame_id and color
// Chi square distribution
std::map<double, double> chi_map;
bool velocity_in_motion_term;
double acceleration_variance;
double position_variance_weight;
double voxel_size;
double gate_distance;
bool calibration_refinement;
std::map<std::string, Eigen::Matrix4d> registration_matrices;
double max_detection_delay;
ros::Time latest_time;

std::map<std::string, ros::Time> last_received_detection_;
ros::Duration max_time_between_detections_;

std::map<std::string, std::pair<double, int> > number_messages_delay_map_;

/**
 * \brief Create marker to be visualized in RViz
 *
 * \param[in] id The marker ID.
 * \param[in] frame_id The marker reference frame.
 * \param[in] position The marker position.
 * \param[in] color The marker color.
 */
visualization_msgs::Marker
createMarker (int id, std::string frame_id, ros::Time stamp, Eigen::Vector3d position, cv::Vec3f color)
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = world_frame_id;
  marker.header.stamp = stamp;
  marker.ns = frame_id;
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = position(0);
  marker.pose.position.y = position(1);
  marker.pose.position.z = position(2);
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.r = color(0);
  marker.color.g = color(1);
  marker.color.b = color(2);
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(0.2);

  return marker;
}

void
plotCameraLegend (std::map<std::string, int> curr_color_map)
{
  // Compose camera legend:
  cv::Mat legend_image = cv::Mat::zeros(500, 500, CV_8UC3);
  for(std::map<std::string, int>::iterator colormap_iterator = curr_color_map.begin(); colormap_iterator != curr_color_map.end(); colormap_iterator++)
  {
    int color_index = colormap_iterator->second;
    cv::Vec3f color = camera_colors[color_index];
    int y_coord = color_index * legend_image.rows / (curr_color_map.size()+1) + 0.5 * legend_image.rows / (curr_color_map.size()+1);
    cv::line(legend_image, cv::Point(0,y_coord), cv::Point(100,y_coord), cv::Scalar(255*color(2), 255*color(1), 255*color(0)), 8);
    cv::putText(legend_image, colormap_iterator->first, cv::Point(110,y_coord), 1, 1, cv::Scalar(255, 255, 255), 1);
  }

  // Display the cv image
  cv::imshow("Camera legend", legend_image);
  cv::waitKey(1);
}

Eigen::Matrix4d
readMatrixFromFile (std::string filename)
{
  Eigen::Matrix4d matrix;
  std::string line;
  std::ifstream myfile (filename.c_str());
  if (myfile.is_open())
  {
    int k = 0;
    std::string number;
    while (myfile >> number)
    {
      matrix(int(k/4), int(k%4)) = std::atof(number.c_str());
      k++;
    }
    myfile.close();
  }

  std::cout << matrix << std::endl;

  return matrix;
}

/**
 * \brief Read the DetectionArray message and use the detections for creating/updating/deleting tracks
 *
 * \param[in] msg the DetectionArray message.
 */
void
detection_cb(const opt_msgs::DetectionArray::ConstPtr& msg)
{
  // Read message header information:
  std::string frame_id = msg->header.frame_id;
  ros::Time frame_time = msg->header.stamp;

  std::string frame_id_tmp = frame_id;
  int pos = frame_id_tmp.find("_rgb_optical_frame");
  if (pos != std::string::npos)
    frame_id_tmp.replace(pos, std::string("_rgb_optical_frame").size(), "");
  pos = frame_id_tmp.find("_depth_optical_frame");
  if (pos != std::string::npos)
  frame_id_tmp.replace(pos, std::string("_depth_optical_frame").size(), "");
  last_received_detection_[frame_id_tmp] = frame_time;

  // Compute delay of detection message, if any:
  double time_delay = 0.0;
  if (frame_time > latest_time)
  {
    latest_time = frame_time;
    time_delay = 0.0;
  }
  else
  {
    time_delay = (latest_time - frame_time).toSec();
  }

  tf::StampedTransform transform;
  tf::StampedTransform inverse_transform;
  //	cv_bridge::CvImage::Ptr cvPtr;

  try
  {
    // Read transforms between camera frame and world frame:
    if (!extrinsic_calibration)
    {
      static tf::TransformBroadcaster world_to_camera_tf_publisher;
//      world_to_camera_tf_publisher.sendTransform(tf::StampedTransform(camera_frame_to_world_transform, ros::Time::now(), world_frame_id, frame_id));
      world_to_camera_tf_publisher.sendTransform(tf::StampedTransform(world_to_camera_frame_transform, ros::Time::now(), frame_id, world_frame_id));
    }

    //Calculate direct and inverse transforms between camera and world frame:
    tf_listener->lookupTransform(world_frame_id, frame_id, ros::Time(0), transform);
    tf_listener->lookupTransform(frame_id, world_frame_id, ros::Time(0), inverse_transform);

    //		cvPtr = cv_bridge::toCvCopy(msg->image, sensor_msgs::image_encodings::BGR8);

    // Read camera intrinsic parameters:
    Eigen::Matrix3d intrinsic_matrix;
    for(int i = 0; i < 3; i++)
      for(int j = 0; j < 3; j++)
        intrinsic_matrix(i, j) = msg->intrinsic_matrix[i * 3 + j];

    // Add a new DetectionSource or update an existing one:
    if(detection_sources_map.find(frame_id) == detection_sources_map.end())
    {
      detection_sources_map[frame_id] = new open_ptrack::detection::DetectionSource(cv::Mat(0, 0, CV_8UC3),
          transform, inverse_transform, intrinsic_matrix, frame_time, frame_id);
    }
    else
    {
      detection_sources_map[frame_id]->update(cv::Mat(0, 0, CV_8UC3), transform, inverse_transform,
          intrinsic_matrix, frame_time, frame_id);
      double d = detection_sources_map[frame_id]->getDuration().toSec() / period;
      int lostFrames = int(round(d)) - 1;
    }
    open_ptrack::detection::DetectionSource* source = detection_sources_map[frame_id];

    // Create a Detection object for every detection in the detection message:
    std::vector<open_ptrack::detection::Detection> detections_vector;
    for(std::vector<opt_msgs::Detection>::const_iterator it = msg->detections.begin();
        it != msg->detections.end(); it++)
    {
      detections_vector.push_back(open_ptrack::detection::Detection(*it, source));
    }

    // Convert HOG+SVM confidences to HAAR+ADABOOST-like people detection confidences:
    if (not std::strcmp(msg->confidence_type.c_str(), "hog+svm"))
    {
      for(unsigned int i = 0; i < detections_vector.size(); i++)
      {
//        double new_confidence = detections_vector[i].getConfidence();
//        new_confidence = (new_confidence - min_confidence_detections_sr) / (min_confidence_sr - min_confidence_detections_sr) *
//                         (min_confidence - min_confidence_detections) + min_confidence_detections;
//        detections_vector[i].setConfidence(new_confidence+2);

        double new_confidence = detections_vector[i].getConfidence();
        new_confidence = (new_confidence - (-3)) / 3 * 4 + 2;
        detections_vector[i].setConfidence(new_confidence);

        //std::cout << detections_vector[i].getConfidence() << std::endl;
      }
    }

    // Detection correction by means of calibration refinement:
    if (calibration_refinement)
    {
      if (strcmp(frame_id.substr(0,1).c_str(), "/") == 0)
      {
        frame_id = frame_id.substr(1, frame_id.size() - 1);
      }

      Eigen::Matrix4d registration_matrix;
      std::map<std::string, Eigen::Matrix4d>::iterator registration_matrices_iterator = registration_matrices.find(frame_id);
      if (registration_matrices_iterator != registration_matrices.end())
      { // camera already present
        registration_matrix = registration_matrices_iterator->second;
      }
      else
      { // camera not present
        std::cout << "Reading refinement matrix of " << frame_id << " from file." << std::endl;
        std::string refinement_filename = ros::package::getPath("opt_calibration") + "/conf/registration_" + frame_id + ".txt";
        std::ifstream f(refinement_filename.c_str());
        if (f.good()) // if the file exists
        {
          f.close();
          registration_matrix = readMatrixFromFile (refinement_filename);
          registration_matrices.insert(std::pair<std::string, Eigen::Matrix4d> (frame_id, registration_matrix));
        }
        else  // if the file does not exist
        {
          // insert the identity matrix
          std::cout << "Refinement file not found! Not doing refinement for this sensor." << std::endl;
          registration_matrices.insert(std::pair<std::string, Eigen::Matrix4d> (frame_id, Eigen::Matrix4d::Identity()));
        }
      }

      if(detections_vector.size() > 0)
      {
        // Apply detection refinement:
        for(unsigned int i = 0; i < detections_vector.size(); i++)
        {
          Eigen::Vector3d old_centroid = detections_vector[i].getWorldCentroid();

//          std::cout << frame_id << std::endl;
//          std::cout << registration_matrix << std::endl;
//          std::cout << "old_centroid: " << old_centroid.transpose() << std::endl;
          Eigen::Vector4d old_centroid_homogeneous(old_centroid(0), old_centroid(1), old_centroid(2), 1.0);
          Eigen::Vector4d refined_centroid = registration_matrix * old_centroid_homogeneous;
          detections_vector[i].setWorldCentroid(Eigen::Vector3d(refined_centroid(0), refined_centroid(1), refined_centroid(2)));

          Eigen::Vector3d refined_centroid2 = detections_vector[i].getWorldCentroid();
//          std::cout << "refined_centroid2: " << refined_centroid2.transpose() << std::endl;
//          std::cout << "difference: " << (refined_centroid2 - old_centroid).transpose() << std::endl << std::endl;
        }
      }
    }

    // If at least one detection has been received:
    if((detections_vector.size() > 0) && (time_delay < max_detection_delay))
    {
      // Perform detection-track association:
      tracker->newFrame(detections_vector);
      tracker->updateTracks();

      // Create a TrackingResult message with the output of the tracking process
      if(output_tracking_results)
      {
        opt_msgs::TrackArray::Ptr tracking_results_msg(new opt_msgs::TrackArray);
        tracking_results_msg->header.stamp = ros::Time::now();//frame_time;
        tracking_results_msg->header.frame_id = world_frame_id;
        tracker->toMsg(tracking_results_msg);
        // Publish tracking message:
        results_pub.publish(tracking_results_msg);
      }

//      //Show the tracking process' results as an image
//      if(output_image_rgb)
//      {
//        tracker->drawRgb();
//        for(std::map<std::string, open_ptrack::detection::DetectionSource*>::iterator
//            it = detection_sources_map.begin(); it != detection_sources_map.end(); it++)
//        {
//          cv::Mat image_to_show = it->second->getImage();
//          if (not vertical)
//          {
//            //cv::imshow("TRACKER " + it->first, image_to_show);
//            cv::imshow("TRACKER ", image_to_show);		// TODO: use the above row if using multiple cameras
//          }
//          else
//          {
//            cv::flip(image_to_show.t(), image_to_show, -1);
//            cv::flip(image_to_show, image_to_show, 1);
//            //cv::imshow("TRACKER " + it->first, image_to_show);
//            cv::imshow("TRACKER ", image_to_show);		// TODO: use the above row if using multiple cameras
//          }
//          cv::waitKey(2);
//        }
//      }

      // Publish IDs of active tracks:
      opt_msgs::IDArray::Ptr alive_ids_msg(new opt_msgs::IDArray);
      alive_ids_msg->header.stamp = ros::Time::now();
      alive_ids_msg->header.frame_id = world_frame_id;
      tracker->getAliveIDs (alive_ids_msg);
      alive_ids_pub.publish (alive_ids_msg);

      // Show the pose of each tracked object with a 3D marker (to be visualized with ROS RViz)
      if(output_markers)
      {
        visualization_msgs::MarkerArray::Ptr marker_msg(new visualization_msgs::MarkerArray);
        tracker->toMarkerArray(marker_msg);
        marker_pub.publish(marker_msg);
      }

      // Show the history of the movements in 3D (3D trajectory) of each tracked object as a PointCloud (which can be visualized in RViz)
      if(output_history_pointcloud)
      {
        history_pointcloud->header.stamp = frame_time.toNSec() / 1e3;  // Convert from ns to us
        history_pointcloud->header.frame_id = world_frame_id;
        starting_index = tracker->appendToPointCloud(history_pointcloud, starting_index,
            output_history_size);
        pointcloud_pub.publish(history_pointcloud);
      }

      // Create message for showing detection positions in RViz:
      if (output_detection_results)
      {
        visualization_msgs::MarkerArray::Ptr marker_msg(new visualization_msgs::MarkerArray);
        detection_history_pointcloud->header.stamp = frame_time.toNSec() / 1e3;  // Convert from ns to us
        detection_history_pointcloud->header.frame_id = world_frame_id;
        std::string frame_id = detections_vector[0].getSource()->getFrameId();

        // Define color:
        int color_index;
        std::map<std::string, int>::iterator colormap_iterator = color_map.find(frame_id);
        if (colormap_iterator != color_map.end())
        { // camera already present
          color_index = colormap_iterator->second;
        }
        else
        { // camera not present
          color_index = color_map.size();
          color_map.insert(std::pair<std::string, int> (frame_id, color_index));

          // Plot legend with camera names and colors:
          plotCameraLegend (color_map);
        }
        for (unsigned int i = 0; i < detections_vector.size(); i++)
        {
          // Create marker and add it to message:
          Eigen::Vector3d centroid = detections_vector[i].getWorldCentroid();
          visualization_msgs::Marker marker = createMarker (i, frame_id, frame_time, centroid, camera_colors[color_index]);
          marker_msg->markers.push_back(marker);

          // Point cloud:
          pcl::PointXYZRGB point;
          point.x = marker.pose.position.x;
          point.y = marker.pose.position.y;
          point.z = marker.pose.position.z;
          point.r = marker.color.r * 255.0f;
          point.g = marker.color.g * 255.0f;
          point.b = marker.color.b * 255.0f;
          detection_insert_index = (detection_insert_index + 1) % detection_history_size;
          detection_history_pointcloud->points[detection_insert_index] = point;
        }
        detection_marker_pub.publish(marker_msg); // publish marker message
        detection_trajectory_pub.publish(detection_history_pointcloud); // publish trajectory message
      }
    }
    else // if no detections have been received or detection_delay is above max_detection_delay
    {
      if(output_tracking_results)
      { // Publish an empty tracking message
        opt_msgs::TrackArray::Ptr tracking_results_msg(new opt_msgs::TrackArray);
        tracking_results_msg->header.stamp = frame_time;
        tracking_results_msg->header.frame_id = world_frame_id;
        results_pub.publish(tracking_results_msg);
      }
      if((detections_vector.size() > 0) && (time_delay >= max_detection_delay))
      {
        if (number_messages_delay_map_.find(msg->header.frame_id) == number_messages_delay_map_.end())
          number_messages_delay_map_[msg->header.frame_id] = std::pair<double, int>(0.0, 0);

        number_messages_delay_map_[msg->header.frame_id].first += time_delay;
        number_messages_delay_map_[msg->header.frame_id].second++;

        if (number_messages_delay_map_[msg->header.frame_id].second == 100)
        {
          double avg = number_messages_delay_map_[msg->header.frame_id].first / number_messages_delay_map_[msg->header.frame_id].second;
          ROS_WARN_STREAM("[" << msg->header.frame_id << "] received 100 detections with average delay " << avg << " > " << max_detection_delay);
          number_messages_delay_map_[msg->header.frame_id] = std::pair<double, int>(0.0, 0);
        }
      }
    }
  }
//  catch(cv_bridge::Exception& ex)
//  {
//    ROS_ERROR("cv_bridge exception: %s", ex.what());
//  }
  catch(tf::TransformException& ex)
  {
    ROS_ERROR("transform exception: %s", ex.what());
  }
}

void
generateColors(int colors_number, std::vector<cv::Vec3f>& colors)
{
  for (unsigned int i = 0; i < colors_number; i++)
  {
    colors.push_back(cv::Vec3f(
        float(rand() % 256) / 255,
        float(rand() % 256) / 255,
        float(rand() % 256) / 255));
  }
}

void
fillChiMap(std::map<double, double>& chi_map, bool velocity_in_motion_term)
{
  if (velocity_in_motion_term)		// chi square values with state dimension = 4
  {
    chi_map[0.5] = 3.357;
    chi_map[0.75] = 5.385;
    chi_map[0.8] = 5.989;
    chi_map[0.9] = 7.779;
    chi_map[0.95] = 9.488;
    chi_map[0.98] = 11.668;
    chi_map[0.99] = 13.277;
    chi_map[0.995] = 14.860;
    chi_map[0.998] = 16.924;
    chi_map[0.999] = 18.467;
  }
  else							              // chi square values with state dimension = 2
  {
    chi_map[0.5] = 1.386;
    chi_map[0.75] = 2.773;
    chi_map[0.8] = 3.219;
    chi_map[0.9] = 4.605;
    chi_map[0.95] = 5.991;
    chi_map[0.98] = 7.824;
    chi_map[0.99] = 9.210;
    chi_map[0.995] = 10.597;
    chi_map[0.998] = 12.429;
    chi_map[0.999] = 13.816;
  }
}

void
configCb(Config &config, uint32_t level)
{
  tracker->setMinConfidenceForTrackInitialization (config.min_confidence_initialization);
  max_detection_delay = config.max_detection_delay;
  calibration_refinement = config.calibration_refinement;
  tracker->setSecBeforeOld (config.sec_before_old);
  tracker->setSecBeforeFake (config.sec_before_fake);
  tracker->setSecRemainNew (config.sec_remain_new);
  tracker->setDetectionsToValidate (config.detections_to_validate);
  tracker->setDetectorLikelihood (config.detector_likelihood);
  tracker->setLikelihoodWeights (config.detector_weight*chi_map[0.999]/18.467, config.motion_weight);

//  if (config.velocity_in_motion_term != velocity_in_motion_term)
//  {
//    // Take chi square values with regards to the state dimension:
//    fillChiMap(chi_map, config.velocity_in_motion_term);
//
//    double position_variance = config.position_variance_weight*std::pow(2 * voxel_size, 2) / 12.0;
//    tracker->setVelocityInMotionTerm (config.velocity_in_motion_term, config.acceleration_variance, position_variance);
//  }
//  else
//  {
    if (config.acceleration_variance != acceleration_variance)
    {
      tracker->setAccelerationVariance (config.acceleration_variance);
    }

    if (config.position_variance_weight != position_variance_weight)
    {
      double position_variance = config.position_variance_weight*std::pow(2 * voxel_size, 2) / 12.0;
      tracker->setPositionVariance (position_variance);
    }
//  }

  gate_distance = chi_map.find(config.gate_distance_probability) != chi_map.end() ? chi_map[config.gate_distance_probability] : chi_map[0.999];
  tracker->setGateDistance (config.gate_distance_probability);
}

int
main(int argc, char** argv)
{

  ros::init(argc, argv, "tracker");
  ros::NodeHandle nh("~");

  // Subscribers/Publishers:
  ros::Subscriber input_sub = nh.subscribe("input", 5, detection_cb);
  marker_pub_tmp = nh.advertise<visualization_msgs::Marker>("/tracker/markers", 1);
  marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/tracker/markers_array", 1);
  pointcloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGBA> >("/tracker/history", 1);
  results_pub = nh.advertise<opt_msgs::TrackArray>("/tracker/tracks", 100);
  detection_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/detector/markers_array", 1);
  detection_trajectory_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGBA> >("/detector/history", 1);
  alive_ids_pub = nh.advertise<opt_msgs::IDArray>("/tracker/alive_ids", 1);

  // Dynamic reconfigure
  boost::recursive_mutex config_mutex_;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  tf_listener = new tf::TransformListener();

  // Read tracking parameters:
  nh.param("world_frame_id", world_frame_id, std::string("/odom"));

  nh.param("orientation/vertical", vertical, false);
  nh.param("extrinsic_calibration", extrinsic_calibration, false);

  nh.param("voxel_size", voxel_size, 0.06);

  double rate;
  nh.param("rate", rate, 30.0);

//  double min_confidence;
  nh.param("min_confidence_initialization", min_confidence, -2.5); //0.0);

  double chi_value;
  nh.param("gate_distance_probability", chi_value, 0.9);

  nh.param("acceleration_variance", acceleration_variance, 1.0);

  nh.param("position_variance_weight", position_variance_weight, 1.0);

  bool detector_likelihood;
  nh.param("detector_likelihood", detector_likelihood, false);

  nh.param("velocity_in_motion_term", velocity_in_motion_term, false);

  double detector_weight;
  nh.param("detector_weight", detector_weight, -1.0);

  double motion_weight;
  nh.param("motion_weight", motion_weight, 0.5);

  double sec_before_old;
  nh.param("sec_before_old", sec_before_old, 3.6);

  double sec_before_fake;
  nh.param("sec_before_fake", sec_before_fake, 2.4);

  double sec_remain_new;
  nh.param("sec_remain_new", sec_remain_new, 1.2);

  int detections_to_validate;
  nh.param("detections_to_validate", detections_to_validate, 5);

  double haar_disp_ada_min_confidence, ground_based_people_detection_min_confidence;
  nh.param("haar_disp_ada_min_confidence", haar_disp_ada_min_confidence, -2.5); //0.0);
  nh.param("ground_based_people_detection_min_confidence", ground_based_people_detection_min_confidence, -2.5); //0.0);

  nh.param("swissranger", swissranger, false);

  nh.param("ground_based_people_detection_min_confidence_sr", min_confidence_detections_sr, -1.5);
  nh.param("min_confidence_initialization_sr", min_confidence_sr, -1.1);

  nh.param("history_pointcloud", output_history_pointcloud, false);
  nh.param("history_size", output_history_size, 1000);
  nh.param("markers", output_markers, true);
  nh.param("image_rgb", output_image_rgb, true);
  nh.param("tracking_results", output_tracking_results, true);

  nh.param("detection_debug", output_detection_results, true);
  nh.param("detection_history_size", detection_history_size, 1000);

  bool debug_mode;
  nh.param("debug_active", debug_mode, false);

  nh.param("calibration_refinement", calibration_refinement, false);
  nh.param("max_detection_delay", max_detection_delay, 3.0);

  double max_time_between_detections_d;
  nh.param("max_time_between_detections", max_time_between_detections_d, 10.0);
  max_time_between_detections_ = ros::Duration(max_time_between_detections_d);

  // Read number of sensors in the network:
  int num_cameras = 1;
  if (extrinsic_calibration)
  {
    num_cameras = 0;
    XmlRpc::XmlRpcValue network;
    nh.getParam("network", network);
    for (unsigned i = 0; i < network.size(); i++)
    {
      num_cameras += network[i]["sensors"].size();
      for (unsigned j = 0; j < network[i]["sensors"].size(); j++)
      {
        std::string frame_id = network[i]["sensors"][j]["id"];
        last_received_detection_["/" + frame_id] = ros::Time(0);
      }
    }
  }

  // Set min_confidence_detections variable based on sensor type:
  if (swissranger)
    min_confidence_detections = ground_based_people_detection_min_confidence;
  else
    min_confidence_detections = haar_disp_ada_min_confidence;

  // Take chi square values with regards to the state dimension:
  fillChiMap(chi_map, velocity_in_motion_term);

  // Compute additional parameters:
  period = 1.0 / rate;
  gate_distance = chi_map.find(chi_value) != chi_map.end() ? chi_map[chi_value] : chi_map[0.999];

  double position_variance;
//  position_variance = 3*std::pow(2 * voxel_size, 2) / 12.0; // DEFAULT
  position_variance = position_variance_weight*std::pow(2 * voxel_size, 2) / 12.0;
  std::vector<double> likelihood_weights;
  likelihood_weights.push_back(detector_weight*chi_map[0.999]/18.467);
  likelihood_weights.push_back(motion_weight);

  // Generate colors used to identify different cameras:
  generateColors(num_cameras, camera_colors);

  // Initialize point cloud containing detections trajectory:
  pcl::PointXYZRGB nan_point;
  nan_point.x = std::numeric_limits<float>::quiet_NaN();
  nan_point.y = std::numeric_limits<float>::quiet_NaN();
  nan_point.z = std::numeric_limits<float>::quiet_NaN();
  detection_history_pointcloud->points.resize(detection_history_size, nan_point);

  ros::Rate hz(num_cameras*rate);

//  cv::namedWindow("TRACKER ", CV_WINDOW_NORMAL);

  // Initialize an instance of the Tracker object:
  tracker = new open_ptrack::tracking::Tracker(
      gate_distance,
      detector_likelihood,
      likelihood_weights,
      velocity_in_motion_term,
      min_confidence,
      min_confidence_detections,
      sec_before_old,
      sec_before_fake,
      sec_remain_new,
      detections_to_validate,
      period,
      position_variance,
      acceleration_variance,
      world_frame_id,
      debug_mode,
      vertical);

  starting_index = 0;

  // Set up dynamic reconfiguration
  ReconfigureServer::CallbackType f = boost::bind(&configCb, _1, _2);
  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, nh));
  reconfigure_server_->setCallback(f);

  // If extrinsic calibration is not available:
  if (!extrinsic_calibration)
  { // Set fixed transformation from rgb frame and base_link
    tf::Vector3 fixed_translation(0, 0, 0);                  // camera_rgb_optical_frame -> world
    tf::Quaternion fixed_rotation(-0.5, 0.5, -0.5, -0.5);	// camera_rgb_optical_frame -> world
    tf::Vector3 inv_fixed_translation(0.0, 0.0, 0);			// world -> camera_rgb_optical_frame
    tf::Quaternion inv_fixed_rotation(-0.5, 0.5, -0.5, 0.5);	// world -> camera_rgb_optical_frame
    world_to_camera_frame_transform = tf::Transform(fixed_rotation, fixed_translation);
    camera_frame_to_world_transform = tf::Transform(inv_fixed_rotation, inv_fixed_translation);
  }

  // Spin and execute callbacks:
//  ros::spin();

  std::map<std::string, ros::Time> last_message;
  for (std::map<std::string, ros::Time>::const_iterator it = last_received_detection_.begin(); it != last_received_detection_.end(); ++it)
    last_message[it->first] = ros::Time::now();

  ros::Time last_camera_legend_update = ros::Time::now();  // last time when the camera legend has been updated

  while (ros::ok())
  {
    ros::spinOnce();
    ros::Time now = ros::Time::now();
    for (std::map<std::string, ros::Time>::const_iterator it = last_received_detection_.begin(); it != last_received_detection_.end(); ++it)
    {
      ros::Duration duration(now - it->second);
      if (duration > max_time_between_detections_)
      {
        if (it->second > ros::Time(0) and now - last_message[it->first] > max_time_between_detections_)
        {
          ROS_WARN_STREAM("[" << it->first << "] last detection was " << duration.toSec() << " seconds ago");
          last_message[it->first] = now;
        }
        else if (now - last_message[it->first] > max_time_between_detections_)
        {
          ROS_WARN_STREAM("[" << it->first << "] still waiting for detection messages...");
          last_message[it->first] = now;
        }
      }

      // Update camera legend every second:
      if ((now - last_camera_legend_update) > ros::Duration(1.0))    // if more than one second passed since last update
      { // update OpenCV image with a waitKey:
        cv::waitKey(1);
        last_camera_legend_update = now;
      }
    }
    hz.sleep();
  }

  return 0;
}
