/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013-, Matteo Munaro [matteo.munaro@dei.unipd.it]
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
 * ground_based_people_detector_node.cpp
 * Created on: Jul 07, 2013
 * Author: Matteo Munaro
 *
 * ROS node which performs people detection assuming that people stand/walk on a ground plane.
 * As a first step, the ground is manually initialized, then people detection is performed with the GroundBasedPeopleDetectionApp class,
 * which implements the people detection algorithm described here:
 * M. Munaro, F. Basso and E. Menegatti,
 * Tracking people within groups with RGB-D data,
 * In Proceedings of the International Conference on Intelligent Robots and Systems (IROS) 2012, Vilamoura (Portugal), 2012.
 */

// ROS includes:
#include <ros/ros.h>
#include <ros/package.h>

// PCL includes:
#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

// Open PTrack includes:
#include <open_ptrack/detection/ground_segmentation.h>
#include <open_ptrack/detection/ground_based_people_detection_app.h>
#include <open_ptrack/opt_utils/conversions.h>

//Publish Messages
#include <opt_msgs/RoiRect.h>
#include <opt_msgs/Rois.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opt_msgs/Detection.h>
#include <opt_msgs/DetectionArray.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//Time Synchronizer
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

// Dynamic reconfigure:
#include <dynamic_reconfigure/server.h>
#include <detection/GroundBasedPeopleDetectorConfig.h>

using namespace opt_msgs;
using namespace sensor_msgs;
using namespace message_filters::sync_policies;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef detection::GroundBasedPeopleDetectorConfig Config;
typedef dynamic_reconfigure::Server<Config> ReconfigureServer;

bool new_cloud_available_flag = false;
bool camera_info_available_flag = false;
PointCloudT::Ptr cloud(new PointCloudT);

bool intrinsics_already_set = false;
Eigen::Matrix3f intrinsics_matrix;
bool debug;
bool update_background = false;

cv::Mat confidence_image, intensity_image;

// Min confidence for people detection:
double min_confidence;
// People detection object
open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT> people_detector;
// Flag stating if classifiers based on RGB image should be used or not
bool use_rgb;
// Threshold on image luminance. If luminance is over this threshold, classifiers on RGB image are also used
int minimum_luminance;
// If true, sensor tilt angle wrt ground plane is compensated to improve people detection
bool sensor_tilt_compensation;
// Voxel size for downsampling the cloud
double voxel_size;
// If true, do not update the ground plane at every frame
bool lock_ground;
// Frames to use for updating the background
int max_background_frames;
// Main loop rate:
double rate_value;
// Voxel resolution of the octree used to represent the background
double background_octree_resolution;
// Background cloud
PointCloudT::Ptr background_cloud;
// If true, background subtraction is performed
bool background_subtraction;
// Threshold on SwissRanger depth confidence
int sr_conf_threshold;
// Threshold on the ratio of valid points needed for ground estimation
double valid_points_threshold;

enum { COLS = 176, ROWS = 144 };

void
camera_info_cb (const CameraInfo::ConstPtr & msg)
{
  intrinsics_matrix << msg->P[0], 0, msg->P[2], 0, msg->P[5], msg->P[6], 0, 0, 1;
  people_detector.setIntrinsics(intrinsics_matrix);
  camera_info_available_flag = true;
}

void
cloud_cb (const PointCloud2ConstPtr& callback_cloud)
{
  // Point Cloud Fields:
  //  name: x  //  offset: 0  //  datatype: 7  //  count: 1
  //  name: y  //  offset: 4  //  datatype: 7  //  count: 1
  //  name: z  //  offset: 8  //  datatype: 7  //  count: 1
  //  name: intensity  //  offset: 12  //  datatype: 7  //  count: 1
  //  name: confidence //  offset: 16  //  datatype: 7  //  count: 1

  // Create intensity and confidence images:
  int intensity_offset = 12;
  int confidence_offset = 16;

  float point_step = callback_cloud->point_step;
  for (unsigned int i = 0; i < ROWS; i++)
  {
    for (unsigned int j = 0; j < COLS; j++)
    {
      float * intensity_ptr = (float*)&callback_cloud->data.at(intensity_offset + (j + i*COLS)*point_step);
      float * confidence_ptr = (float*)&callback_cloud->data.at(confidence_offset + (j + i*COLS)*point_step);
      intensity_image.at<unsigned char>(i,j) = (unsigned char)(intensity_ptr[0] / 257);
      confidence_image.at<unsigned char>(i,j) = (unsigned char)(confidence_ptr[0] / 257);
    }
  }

  // Find min and max of intensity image:
  double minVal, maxVal;
  cv::Point minLoc, maxLoc;
  cv::minMaxLoc( intensity_image, &minVal, &maxVal, &minLoc, &maxLoc, confidence_image);

  // Create point cloud XYZRGB:
  cloud->header = pcl_conversions::toPCL(callback_cloud->header);
  for (unsigned int i = 0; i < (callback_cloud->data.size() / point_step); i++)
  {
    float * current_point_ptr = (float*)&callback_cloud->data.at(i*point_step);
    cloud->points[i].x = current_point_ptr[0];
    cloud->points[i].y = current_point_ptr[1];
    cloud->points[i].z = current_point_ptr[2];
    float * intensity_ptr = (float*)&callback_cloud->data.at(intensity_offset + i*point_step);
    unsigned char intensity = ((intensity_ptr[0])/257 - minVal) * 255 / (maxVal - minVal);
    cloud->points[i].r = intensity;
    cloud->points[i].g = intensity;
    cloud->points[i].b = intensity;
    intensity_image.at<unsigned char>(i) = intensity;
  }

  new_cloud_available_flag = true;
}

//void
//swissrangerCb (const ImageConstPtr& image_msg, const PointCloudT::ConstPtr& callback_cloud)
//{
//  cv_bridge::CvImagePtr cv_ptr  = cv_bridge::toCvCopy(image_msg, image_encodings::MONO8);
//  confidence_image  = cv_ptr->image;
//
//  *cloud = *callback_cloud;
//  new_cloud_available_flag = true;
//}

//void
//swissrangerCb (const ImageConstPtr& intensity_msg, const ImageConstPtr& confidence_msg, const PointCloudT::ConstPtr& callback_cloud)
//{
//  cv_bridge::CvImagePtr intensity_cv_ptr  = cv_bridge::toCvCopy(intensity_msg, image_encodings::MONO8);
//  intensity_image  = intensity_cv_ptr->image;
//
//  cv_bridge::CvImagePtr confidence_cv_ptr  = cv_bridge::toCvCopy(confidence_msg, image_encodings::MONO8);
//  confidence_image  = confidence_cv_ptr->image;
//
//  *cloud = *callback_cloud;
//  new_cloud_available_flag = true;
//}

void removeLowConfidencePoints(cv::Mat& confidence_image, int threshold, PointCloudT::Ptr& cloud)
{
  for (int i=0;i<cloud->height;i++)
  {
    for (int j=0;j<cloud->width;j++)
    {
      if (confidence_image.at<unsigned char>(i,j) < threshold)
      {
        cloud->at(j,i).x = std::numeric_limits<float>::quiet_NaN();
        cloud->at(j,i).y = std::numeric_limits<float>::quiet_NaN();
        cloud->at(j,i).z = std::numeric_limits<float>::quiet_NaN();
       
        confidence_image.at<unsigned char>(i,j) = 0;    // just for visualization
      }
      else
        confidence_image.at<unsigned char>(i,j) = 255;  // just for visualization
    }
  }
  cloud->is_dense = false;
}

void
updateBackgroundCallback (const std_msgs::String::ConstPtr & msg)
{
  if (msg->data == "update")
  {
    update_background = true;
  }
}

void
computeBackgroundCloud (int frames, float voxel_size, int sr_conf_threshold, std::string frame_id, ros::Rate rate, PointCloudT::Ptr& background_cloud)
{
  std::cout << "Background acquisition..." << std::flush;

  // Create background cloud:
  PointCloudT::Ptr cloud_to_process(new PointCloudT);
  background_cloud->header = cloud->header;
  for (unsigned int i = 0; i < frames; i++)
  {
    *cloud_to_process = *cloud;

    // Remove low confidence points:
    removeLowConfidencePoints(confidence_image, sr_conf_threshold, cloud_to_process);

    // Voxel grid filtering:
    PointCloudT::Ptr cloud_filtered(new PointCloudT);
    pcl::VoxelGrid<PointT> voxel_grid_filter_object;
    voxel_grid_filter_object.setInputCloud(cloud_to_process);
    voxel_grid_filter_object.setLeafSize (voxel_size, voxel_size, voxel_size);
    voxel_grid_filter_object.filter (*cloud_filtered);

    *background_cloud += *cloud_filtered;
    ros::spinOnce();
    rate.sleep();
  }

  // Voxel grid filtering:
  PointCloudT::Ptr cloud_filtered(new PointCloudT);
  pcl::VoxelGrid<PointT> voxel_grid_filter_object;
  voxel_grid_filter_object.setInputCloud(background_cloud);
  voxel_grid_filter_object.setLeafSize (voxel_size, voxel_size, voxel_size);
  voxel_grid_filter_object.filter (*cloud_filtered);

  background_cloud = cloud_filtered;

  // Background saving:
  pcl::io::savePCDFileASCII ("/tmp/background_" + frame_id.substr(1, frame_id.length()-1) + ".pcd", *background_cloud);

  std::cout << "done." << std::endl << std::endl;
}

void
configCb(Config &config, uint32_t level)
{
  valid_points_threshold = config.valid_points_threshold;

  min_confidence = config.ground_based_people_detection_min_confidence;

  people_detector.setHeightLimits (config.minimum_person_height, config.maximum_person_height);

  people_detector.setMaxDistance (config.max_distance);

  people_detector.setSamplingFactor (config.sampling_factor);

  use_rgb = config.use_rgb;
  people_detector.setUseRGB (config.use_rgb);

  minimum_luminance = config.minimum_luminance;

  sensor_tilt_compensation = config.sensor_tilt_compensation;
  people_detector.setSensorTiltCompensation (config.sensor_tilt_compensation);

  people_detector.setMinimumDistanceBetweenHeads (config.heads_minimum_distance);

  voxel_size = config.voxel_size;
  people_detector.setVoxelSize (config.voxel_size);

  lock_ground = config.lock_ground;

  max_background_frames = int(config.background_seconds * rate_value);

  if (config.background_resolution != background_octree_resolution)
  {
    background_octree_resolution = config.background_resolution;
    if (background_subtraction)
      people_detector.setBackground(background_subtraction, background_octree_resolution, background_cloud);
  }

  if (config.background_subtraction != background_subtraction)
  {
    if (config.background_subtraction)
    {
      update_background = true;
    }
    else
    {
      background_subtraction = false;
      people_detector.setBackground(false, background_octree_resolution, background_cloud);
    }
  }

  sr_conf_threshold = config.sr_conf_threshold;
}

bool
fileExists(const char *fileName)
{
    ifstream infile(fileName);
    return infile.good();
}

int
main (int argc, char** argv)
{
	ros::init(argc, argv, "ground_based_people_detector");
	ros::NodeHandle nh("~");

  // Dynamic reconfigure
  boost::recursive_mutex config_mutex_;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

	// Read some parameters from launch file:
	int ground_estimation_mode;
	nh.param("ground_estimation_mode", ground_estimation_mode, 0);
	std::string svm_filename;
	nh.param("classifier_file", svm_filename, std::string("./"));
	nh.param("use_rgb", use_rgb, false);
	nh.param("ground_based_people_detection_min_confidence", min_confidence, -1.5);
	double min_height;
	nh.param("minimum_person_height", min_height, 1.3);
	double max_height;
	nh.param("maximum_person_height", max_height, 2.3);
	double max_distance;
	nh.param("max_distance", max_distance, 50.0);
	int sampling_factor;
	nh.param("sampling_factor", sampling_factor, 1);
	std::string pointcloud_topic;
	nh.param("pointcloud_topic", pointcloud_topic, std::string("/SwissRanger/pointcloud2_raw"));
	std::string confidence_topic;
	nh.param("confidence_topic", confidence_topic, std::string("/SwissRanger/confidence/image_raw"));
	std::string intensity_topic;
	nh.param("intensity_topic", intensity_topic, std::string("/SwissRanger/intensity/image_raw"));
	std::string output_topic;
	nh.param("output_topic", output_topic, std::string("/ground_based_people_detector/detections"));
	std::string camera_info_topic;
	nh.param("camera_info_topic", camera_info_topic, std::string("/camera/rgb/camera_info"));
	nh.param("rate", rate_value, 30.0);
	nh.param("sr_conf_threshold", sr_conf_threshold, 200);
	bool ground_from_extrinsic_calibration;
	nh.param("ground_from_extrinsic_calibration", ground_from_extrinsic_calibration, false);
	nh.param("lock_ground", lock_ground, false);
	nh.param("sensor_tilt_compensation", sensor_tilt_compensation, false);
	nh.param("valid_points_threshold", valid_points_threshold, 0.3);
	nh.param("background_subtraction", background_subtraction, false);
	nh.param("background_resolution", background_octree_resolution, 0.3);
	double background_seconds;
	nh.param("background_seconds", background_seconds, 3.0);
	std::string update_background_topic;  // Topic where the background update message is published/read
	nh.param("update_background_topic", update_background_topic, std::string("/background_update"));
	double heads_minimum_distance; // Minimum distance between two persons' head
	nh.param("heads_minimum_distance", heads_minimum_distance, 0.3);
	nh.param("voxel_size", voxel_size, 0.06);
  bool read_ground_from_file;    // Flag stating if the ground should be read from file, if present
  nh.param("read_ground_from_file", read_ground_from_file, false);

//	Eigen::Matrix3f intrinsics_matrix;
	//horizontal field of view = 2 atan(0.5 width / focallength)
	//vertical field of view = 2 atan(0.5 height / focallength)
//	intrinsics_matrix << 128.0, 0.0, 88.0, 0.0, 128.0, 72.0, 0.0, 0.0, 1.0; // camera intrinsics
//	intrinsics_matrix << 148.906628757021, 0, 87.307592764537, 0, 148.634545599915, 70.8860920144388, 0, 0, 1;

	// Initialize transforms to be used to correct sensor tilt to identity matrix:
	Eigen::Affine3f transform, anti_transform;
	transform = transform.Identity();
	anti_transform = transform.inverse();

	// Subscribers:
	ros::Subscriber sub = nh.subscribe(pointcloud_topic, 1, cloud_cb);
  ros::Subscriber camera_info_sub = nh.subscribe(camera_info_topic, 1, camera_info_cb);
	ros::Subscriber update_background_sub = nh.subscribe(update_background_topic, 1, updateBackgroundCallback);

	// Publishers:
	ros::Publisher detection_pub;
	detection_pub= nh.advertise<DetectionArray>(output_topic, 3);
	ros::Publisher image_pub;
	image_pub = nh.advertise<Image>("/swissranger/intensity/image",3);

	Rois output_rois_;
	open_ptrack::opt_utils::Conversions converter;

	// Initialize confidence and intensity images
	intensity_image = cv::Mat(144, 176, CV_8U, cv::Scalar(255));
	confidence_image = cv::Mat(144, 176, CV_8U, cv::Scalar(0));

	// Initialize point cloud:
	pcl::PointXYZRGB p(0,0,0);
	cloud->points.resize(intensity_image.cols * intensity_image.rows, p);
	cloud->width = COLS;
	cloud->height = ROWS;
	cloud->is_dense = true;

	debug = false;
	if (debug)
	  cv::namedWindow("Valid points", CV_WINDOW_NORMAL);

	ros::Rate rate(rate_value);
	while(ros::ok() && !new_cloud_available_flag)
	{
		ros::spinOnce();
		rate.sleep();
	}

	// Create classifier for people detection:
	open_ptrack::detection::PersonClassifier<pcl::RGB> person_classifier;
	person_classifier.loadSVMFromFile(svm_filename);   // load trained SVM

	// People detection app initialization:
	people_detector.setVoxelSize(voxel_size);                        // set the voxel size
	people_detector.setMaxDistance(max_distance);                    // set maximum distance of people from the sensor
	//  people_detector.setIntrinsics(intrinsics_matrix);                // set RGB camera intrinsic parameters
	people_detector.setClassifier(person_classifier);                // set person classifier
	people_detector.setHeightLimits(min_height, max_height);         // set person classifier
	people_detector.setSamplingFactor(sampling_factor);              // set sampling factor
	people_detector.setUseRGB(use_rgb);                                // set if RGB should be used or not
	people_detector.setSensorTiltCompensation(sensor_tilt_compensation);             // enable point cloud rotation correction
	people_detector.setMinimumDistanceBetweenHeads (heads_minimum_distance);  // set minimum distance between persons' head

	// Set up dynamic reconfiguration
	ReconfigureServer::CallbackType f = boost::bind(&configCb, _1, _2);
	reconfigure_server_.reset(new ReconfigureServer(config_mutex_, nh));
	reconfigure_server_->setCallback(f);

	// Loop until a valid point cloud is found
	open_ptrack::detection::GroundplaneEstimation<PointT> ground_estimator(ground_estimation_mode);
	bool first_valid_frame = false;
	while (!first_valid_frame)
	{
	  // Execute callbacks:
	  ros::spinOnce();
	  rate.sleep();

	  if (debug)
	  {
	    cv::imshow("Valid points", confidence_image);
	    cv::waitKey(1);
	  }

	  if (!ground_estimator.tooManyLowConfidencePoints(confidence_image, sr_conf_threshold, 1.0 - valid_points_threshold))
	  { // A point cloud is valid if the ratio #NaN / #valid points is lower than a threshold
	    first_valid_frame = true;
	    std::cout << "Valid frame found!" << std::endl;
	  }
	  else
	  {
	    std::cout << "No valid frame. Move the camera to a better position..." << std::endl;
	  }
	}
	std::cout << std::endl;

	// Initialization for background subtraction:
	PointCloudT::Ptr background_cloud (new PointCloudT);
	std::string frame_id = cloud->header.frame_id;
	int max_background_frames = int(background_seconds * rate_value);
	if (background_subtraction)
	{
	  std::cout << "Background subtraction enabled." << std::endl;

	  // Try to load the background from file:
	  if (pcl::io::loadPCDFile<PointT> ("/tmp/background_" + frame_id.substr(1, frame_id.length()-1) + ".pcd", *background_cloud) == -1)
	  {
	    // File not found, then background acquisition:
	    computeBackgroundCloud (max_background_frames, voxel_size, sr_conf_threshold, frame_id, rate, background_cloud);
	  }
	  else
	  {
	    std::cout << "Background read from file." << std::endl << std::endl;
	  }

	  people_detector.setBackground(background_subtraction, background_octree_resolution, background_cloud);
	}

	// Remove low confidence points:
	PointCloudT::Ptr cloud_to_process(new PointCloudT);
	*cloud_to_process = *cloud;
	removeLowConfidencePoints(confidence_image, sr_conf_threshold, cloud_to_process);

	// Ground estimation:
	std::cout << "Ground plane initialization starting..." << std::endl;
	ground_estimator.setInputCloud(cloud_to_process);
	Eigen::VectorXf ground_coeffs = ground_estimator.computeMulticamera(ground_from_extrinsic_calibration, read_ground_from_file,
	    pointcloud_topic, sampling_factor, voxel_size);

//	// Initialize new viewer:
//	pcl::visualization::PCLVisualizer viewer("PCL Viewer");          // viewer initialization
//	viewer.setCameraPosition(0,0,-2,0,-1,0,0);

	// Main loop:
	while(ros::ok())
	{
    if (new_cloud_available_flag and camera_info_available_flag)
		{
			new_cloud_available_flag = false;

			// Convert PCL cloud header to ROS header:
			std_msgs::Header cloud_header = pcl_conversions::fromPCL(cloud_to_process->header);

			// If requested, update background:
			if (update_background)
			{
			  if (not background_subtraction)
			  {
			    std::cout << "Background subtraction enabled." << std::endl;
			    background_subtraction = true;
			  }
			  computeBackgroundCloud (max_background_frames, voxel_size, sr_conf_threshold, frame_id, rate, background_cloud);
			  people_detector.setBackground (background_subtraction, background_octree_resolution, background_cloud);

			  update_background = false;
			}

			// Current cloud to process:
			*cloud_to_process = *cloud;

			// Remove low confidence points:
			removeLowConfidencePoints(confidence_image, sr_conf_threshold, cloud_to_process);

			// Perform people detection on the new cloud:
			std::vector<pcl::people::PersonCluster<PointT> > clusters;   // vector containing persons clusters
			people_detector.setInputCloud(cloud_to_process);
			people_detector.setGround(ground_coeffs);                    // set floor coefficients
      people_detector.compute(clusters);                           // perform people detection

      // If not lock_ground, update ground coefficients:
      if (not lock_ground)
        ground_coeffs = people_detector.getGround();                 // get updated floor coefficients

			if (sensor_tilt_compensation)
			  people_detector.getTiltCompensationTransforms(transform, anti_transform);

//			// Draw cloud and people bounding boxes in the viewer:
//			PointCloudT::Ptr no_ground_cloud(new PointCloudT);
//			no_ground_cloud = people_detector.getNoGroundCloud();
//			viewer.removeAllPointClouds();
//			viewer.removeAllShapes();
//			viewer.addPointCloud<PointT> (no_ground_cloud, "input_cloud");
//			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "input_cloud");

			/// Write detection message:
			DetectionArray::Ptr detection_array_msg(new DetectionArray);
			// Set camera-specific fields:
			detection_array_msg->header = cloud_header;
			for(int i = 0; i < 3; i++)
				for(int j = 0; j < 3; j++)
					detection_array_msg->intrinsic_matrix.push_back(intrinsics_matrix(i, j));
			detection_array_msg->header.frame_id = cloud_header.frame_id;
			detection_array_msg->confidence_type = std::string("hog+svm");
			detection_array_msg->image_type = std::string("ir");
			// Add all valid detections:
			unsigned int k = 0;
			for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
			{
			  if((!use_rgb) | (it->getPersonConfidence() > min_confidence))            // if intensity is used, keep only people with confidence above a threshold
			  {
				  // Create detection message:
					Detection detection_msg;
					converter.Vector3fToVector3(anti_transform * it->getMin(), detection_msg.box_3D.p1);
					converter.Vector3fToVector3(anti_transform * it->getMax(), detection_msg.box_3D.p2);

					float head_centroid_compensation = 0.05;

					// theoretical person centroid:
					Eigen::Vector3f centroid3d = anti_transform * it->getTCenter();
					Eigen::Vector3f centroid2d = converter.world2cam(centroid3d, intrinsics_matrix);
					// theoretical person top point:
					Eigen::Vector3f top3d = anti_transform * it->getTTop();
					Eigen::Vector3f top2d = converter.world2cam(top3d, intrinsics_matrix);
					// theoretical person bottom point:
					Eigen::Vector3f bottom3d = anti_transform * it->getTBottom();
					Eigen::Vector3f bottom2d = converter.world2cam(bottom3d, intrinsics_matrix);
          float enlarge_factor = 1.1;
          float pixel_xc = centroid2d(0);
					float pixel_yc = centroid2d(1);
					float pixel_height = (bottom2d(1) - top2d(1)) * enlarge_factor;
					float pixel_width = pixel_height / 2;
					detection_msg.box_2D.x = int(centroid2d(0) - pixel_width/2.0);
					detection_msg.box_2D.y = int(centroid2d(1) - pixel_height/2.0);
					detection_msg.box_2D.width = int(pixel_width);
					detection_msg.box_2D.height = int(pixel_height);
					detection_msg.height = it->getHeight();
					detection_msg.confidence = it->getPersonConfidence();
					detection_msg.distance = it->getDistance();
					converter.Vector3fToVector3((1+head_centroid_compensation/centroid3d.norm())*centroid3d, detection_msg.centroid);
					converter.Vector3fToVector3((1+head_centroid_compensation/top3d.norm())*top3d, detection_msg.top);
					converter.Vector3fToVector3((1+head_centroid_compensation/bottom3d.norm())*bottom3d, detection_msg.bottom);

					// Add message:
					detection_array_msg->detections.push_back(detection_msg);

//					// draw theoretical person bounding box in the PCL viewer:
//					it->drawTBoundingBox(viewer, k);
//					k++;
				}
			}
			detection_pub.publish(detection_array_msg);		 // publish message

			// Send intensity image:
			cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
			cv_ptr->encoding = "mono8";
			cv_ptr->image = intensity_image;
      cv_ptr->header = detection_array_msg->header;
			cv_ptr->header.frame_id = "/swissranger";
			image_pub.publish(cv_ptr->toImageMsg());

//			std::cout << k << " people found" << std::endl;
//			viewer.spinOnce();

			if (debug)
			{
			  cv::imshow("Valid points", confidence_image);
			  cv::waitKey(1);
			}
		}

		// Execute callbacks:
		ros::spinOnce();
		rate.sleep();
	}

	// Delete background file from disk:
	std::string filename = "/tmp/background_" + frame_id.substr(1, frame_id.length()-1) + ".pcd";
	if (fileExists (filename.c_str()))
	{
	  remove( filename.c_str() );
	}

	return 0;
}

