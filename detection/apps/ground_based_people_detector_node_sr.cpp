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

// PCL includes:
#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl_conversions/pcl_conversions.h>

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

//Time Synchronizer
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace opt_msgs;
using namespace sensor_msgs;
using namespace message_filters::sync_policies;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool new_cloud_available_flag = false;
PointCloudT::Ptr cloud(new PointCloudT);

bool intrinsics_already_set = false;
Eigen::Matrix3f intrinsics_matrix;

cv::Mat confidence_image, intensity_image;

enum { COLS = 176, ROWS = 144 };

void
cloud_cb (const PointCloudConstPtr& callback_cloud)
{
  // Create intensity and confidence images:
  for (unsigned int i = 0; i < ROWS; i++)
  {
    for (unsigned int j = 0; j < COLS; j++)
    {
      intensity_image.at<unsigned char>(i,j) = callback_cloud->channels[0].values[j + i*COLS]/255;
      confidence_image.at<unsigned char>(i,j) = callback_cloud->channels[1].values[j + i*COLS]/255;
    }
  }

  // Find min and max of intensity image:
  double minVal, maxVal;
  cv::Point minLoc, maxLoc;
  cv::minMaxLoc( intensity_image, &minVal, &maxVal, &minLoc, &maxLoc );

  // Create point cloud XYZRGB:
  cloud->header = pcl_conversions::toPCL(callback_cloud->header);
  cloud->points.clear();
  pcl::PointXYZRGB p;
  cloud->points.resize(callback_cloud->points.size(), p);
  cloud->width = COLS;
  cloud->height = ROWS;
  cloud->is_dense = true;
  for (unsigned int i = 0; i < callback_cloud->points.size(); i++)
  {
    cloud->points[i].x = callback_cloud->points[i].x;
    cloud->points[i].y = callback_cloud->points[i].y;
    cloud->points[i].z = callback_cloud->points[i].z;
    unsigned char intensity = ((callback_cloud->channels[0].values[i])/255 - minVal) * 255 / (maxVal - minVal);
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

int
main (int argc, char** argv)
{
	ros::init(argc, argv, "ground_based_people_detector");
	ros::NodeHandle nh("~");

	// Read some parameters from launch file:
	int ground_estimation_mode;
	nh.param("ground_estimation_mode", ground_estimation_mode, 0);
	std::string svm_filename;
	nh.param("classifier_file", svm_filename, std::string("./"));
	bool use_rgb;
	nh.param("use_rgb", use_rgb, false);
	double min_confidence;
	nh.param("ground_based_people_detection_min_confidence", min_confidence, -1.5);
	double min_height;
	nh.param("minimum_person_height", min_height, 1.3);
	double max_height;
	nh.param("maximum_person_height", max_height, 2.3);
	int sampling_factor;
	nh.param("sampling_factor", sampling_factor, 1);
	std::string pointcloud_topic;
	nh.param("pointcloud_topic", pointcloud_topic, std::string("/camera/depth_registered/points"));
	std::string confidence_topic;
	nh.param("confidence_topic", confidence_topic, std::string("/swissranger/confidence/image_raw"));
	std::string intensity_topic;
	nh.param("intensity_topic", intensity_topic, std::string("/swissranger/intensity/image_raw"));
	std::string output_topic;
	nh.param("output_topic", output_topic, std::string("/ground_based_people_detector/detections"));
	std::string camera_info_topic;
	nh.param("camera_info_topic", camera_info_topic, std::string("/camera/rgb/camera_info"));
	double rate_value;
	nh.param("rate", rate_value, 30.0);
	int sr_conf_threshold;
	nh.param("sr_conf_threshold", sr_conf_threshold, 200);

	// Fixed parameters:
	float voxel_size = 0.06;
//	Eigen::Matrix3f intrinsics_matrix;
	//horizontal field of view = 2 atan(0.5 width / focallength)
	//vertical field of view = 2 atan(0.5 height / focallength)
//	intrinsics_matrix << 128.0, 0.0, 88.0, 0.0, 128.0, 72.0, 0.0, 0.0, 1.0; // camera intrinsics
	intrinsics_matrix << 148.906628757021, 0, 87.307592764537, 0, 148.634545599915, 70.8860920144388, 0, 0, 1;

	// Subscribers:
	ros::Subscriber sub = nh.subscribe(pointcloud_topic, 1, cloud_cb);

	// Publishers:
	ros::Publisher detection_pub;
	detection_pub= nh.advertise<DetectionArray>(output_topic, 3);
	ros::Publisher image_pub;
	image_pub = nh.advertise<Image>("/swissranger/intensity/image",3);

	Rois output_rois_;
	open_ptrack::opt_utils::Conversions converter;

	// Initialize confidence and intensity images
	intensity_image = cv::Mat(144, 176, CV_8U, cv::Scalar(255));
	confidence_image = cv::Mat(144, 176, CV_8U, cv::Scalar(255));
	cv::namedWindow("Confidence map", CV_WINDOW_NORMAL);

	ros::Rate rate(rate_value);
	while(ros::ok() && !new_cloud_available_flag)
	{
		ros::spinOnce();
		rate.sleep();
	}

	// Loop until a valid point cloud is found
	open_ptrack::detection::GroundplaneEstimation<PointT> ground_estimator(ground_estimation_mode);
	bool first_valid_frame = false;
	while (!first_valid_frame)
	{
	  if (!ground_estimator.tooManyNaN(cloud, 0.7))
	  { // A point cloud is valid if the ratio #NaN / #valid points is lower than a threshold
	    first_valid_frame = true;
	  }
	  // Execute callbacks:
	  ros::spinOnce();
	  rate.sleep();
	}

	// Remove low confidence points:
	removeLowConfidencePoints(confidence_image, sr_conf_threshold, cloud);

	// Ground estimation:
	ground_estimator.setInputCloud(cloud);
	Eigen::VectorXf ground_coeffs = ground_estimator.compute();

	// Create classifier for people detection:
	open_ptrack::detection::PersonClassifier<pcl::RGB> person_classifier;
	person_classifier.loadSVMFromFile(svm_filename);   // load trained SVM

	// People detection app initialization:
	open_ptrack::detection::GroundBasedPeopleDetectionApp<PointT> people_detector;    // people detection object
	people_detector.setVoxelSize(voxel_size);                        // set the voxel size
	people_detector.setIntrinsics(intrinsics_matrix);                // set RGB camera intrinsic parameters
	people_detector.setClassifier(person_classifier);                // set person classifier
	people_detector.setHeightLimits(min_height, max_height);         // set person classifier
	people_detector.setSamplingFactor(sampling_factor);              // set sampling factor
	people_detector.setUseRGB(use_rgb);                                // set if RGB should be used or not

//	// Initialize new viewer:
//	pcl::visualization::PCLVisualizer viewer("PCL Viewer");          // viewer initialization
//	viewer.setCameraPosition(0,0,-2,0,-1,0,0);

	// Main loop:
	while(ros::ok())
	{
		if (new_cloud_available_flag)
		{
			new_cloud_available_flag = false;

			// Convert PCL cloud header to ROS header:
			std_msgs::Header cloud_header = pcl_conversions::fromPCL(cloud->header);

			// Remove low confidence points:
			removeLowConfidencePoints(confidence_image, sr_conf_threshold, cloud);

			// Perform people detection on the new cloud:
			std::vector<pcl::people::PersonCluster<PointT> > clusters;   // vector containing persons clusters
			people_detector.setInputCloud(cloud);
			people_detector.setGround(ground_coeffs);                    // set floor coefficients
			people_detector.compute(clusters);                           // perform people detection

			ground_coeffs = people_detector.getGround();                 // get updated floor coefficients

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
			detection_array_msg->header.frame_id = "/camera_rgb_optical_frame";
			// Add all valid detections:
			unsigned int k = 0;
			for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
			{
			  if((!use_rgb) | (it->getPersonConfidence() > min_confidence))            // if intensity is used, keep only people with confidence above a threshold
			  {
				  // Create detection message:
					Detection detection_msg;
					converter.Vector3fToVector3(it->getMin(), detection_msg.box_3D.p1);
					converter.Vector3fToVector3(it->getMax(), detection_msg.box_3D.p2);

					float head_centroid_compensation = 0.05;

					// theoretical person centroid:
					Eigen::Vector3f centroid3d = it->getTCenter();
					Eigen::Vector3f centroid2d = converter.world2cam(centroid3d, intrinsics_matrix);
					// theoretical person top point:
					Eigen::Vector3f top3d = it->getTTop();
					Eigen::Vector3f top2d = converter.world2cam(top3d, intrinsics_matrix);
					// theoretical person bottom point:
					Eigen::Vector3f bottom3d = it->getTBottom();
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

			cv::imshow("Confidence map", confidence_image);
			cv::waitKey(1);
		}

		// Execute callbacks:
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}

