/*
 * Software License Agreement (BSD License)
 *
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
 * Author: Matteo Munaro [matteo.munaro@dei.unipd.it], Nicola Ristè
 *
 */

#include <open_ptrack/detection/ground_segmentation.h>

template <typename PointT>
open_ptrack::detection::GroundplaneEstimation<PointT>::GroundplaneEstimation (int ground_estimation_mode)
{
  ground_estimation_mode_ = ground_estimation_mode;

  if ((ground_estimation_mode > 3) || (ground_estimation_mode < 0))
  {
    ground_estimation_mode_ = 0;
    std::cout << "ERROR: invalid mode for groundplane segmentation. Manual mode is selected." << std::endl;
  }
}

template <typename PointT>
open_ptrack::detection::GroundplaneEstimation<PointT>::~GroundplaneEstimation()
{

}

template <typename PointT> void
open_ptrack::detection::GroundplaneEstimation<PointT>::setInputCloud (PointCloudPtr& cloud)
{
  cloud_ = cloud;
}

template <typename PointT> bool
open_ptrack::detection::GroundplaneEstimation<PointT>::tooManyNaN(PointCloudConstPtr cloud, float max_ratio)
{
  int nan_counter = 0;
  for(unsigned int i = 0; i < cloud->size(); i++)
  {
    // If the point has a non-valid coordinate:
    if(isnan(cloud->points[i].x) || isnan(cloud->points[i].y) || isnan(cloud->points[i].z) )
    { // update the counter:
      nan_counter++;
    }
  }

  // If the nan ratio is over max_ratio:
  if( (float) nan_counter/cloud->size() > max_ratio )
    return true;    // too many NaNs, frame invalid
  else
    return false;
}

template <typename PointT> bool
open_ptrack::detection::GroundplaneEstimation<PointT>::tooManyLowConfidencePoints (cv::Mat& confidence_image, int confidence_threshold, double max_ratio)
{
  int invalid_counter = 0;
  for(unsigned int i = 0; i < confidence_image.rows; i++)
  {
    for(unsigned int j = 0; j < confidence_image.cols; j++)
    {
      // If the point has a non-valid confidence:
      if(confidence_image.at<unsigned char>(i,j) < confidence_threshold)
      { // update the counter:
        invalid_counter++;
      }
    }
  }

  // If the invalid ratio is over max_ratio:
  if( (float (invalid_counter))/(confidence_image.rows * confidence_image.cols) > max_ratio )
    return true;    // too many invalid points, frame invalid
  else
    return false;
}

template <typename PointT> Eigen::VectorXf
open_ptrack::detection::GroundplaneEstimation<PointT>::computeFromTF (std::string camera_frame, std::string ground_frame)
{
  // Select 3 points in world reference frame:
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_points (new pcl::PointCloud<pcl::PointXYZ>);
  ground_points->points.push_back(pcl::PointXYZ(0.0, 0.0, 0.0));
  ground_points->points.push_back(pcl::PointXYZ(1.0, 0.0, 0.0));
  ground_points->points.push_back(pcl::PointXYZ(0.0, 1.0, 0.0));

  // Read transform between world and camera reference frame:
  tf::TransformListener tfListener;
  tf::StampedTransform worldToCamTransform;
  try
  {
    tfListener.waitForTransform(camera_frame, ground_frame, ros::Time(0), ros::Duration(3.0), ros::Duration(0.01));
    tfListener.lookupTransform(camera_frame, ground_frame, ros::Time(0), worldToCamTransform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
  }

  // Transform points to camera reference frame:
  for (unsigned int i = 0; i < ground_points->points.size(); i++)
  {
    tf::Vector3 current_point(ground_points->points[i].x, ground_points->points[i].y, ground_points->points[i].z);
    current_point = worldToCamTransform(current_point);
    ground_points->points[i].x = current_point.x();
    ground_points->points[i].y = current_point.y();
    ground_points->points[i].z = current_point.z();
  }

  // Compute ground equation:
  std::vector<int> ground_points_indices;
  for (unsigned int i = 0; i < ground_points->points.size(); i++)
    ground_points_indices.push_back(i);
  pcl::SampleConsensusModelPlane<pcl::PointXYZ> model_plane(ground_points);
  Eigen::VectorXf ground_coeffs;
  model_plane.computeModelCoefficients(ground_points_indices,ground_coeffs);
  std::cout << "Ground plane coefficients obtained from calibration: " << ground_coeffs(0) << ", " << ground_coeffs(1) << ", " << ground_coeffs(2) <<
      ", " << ground_coeffs(3) << "." << std::endl;

  return ground_coeffs;
}

template <typename PointT> Eigen::VectorXf
open_ptrack::detection::GroundplaneEstimation<PointT>::computeFromTF (tf::Transform worldToCamTransform)
{
  // Select 3 points in world reference frame:
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_points (new pcl::PointCloud<pcl::PointXYZ>);
  ground_points->points.push_back(pcl::PointXYZ(0.0, 0.0, 0.0));
  ground_points->points.push_back(pcl::PointXYZ(1.0, 0.0, 0.0));
  ground_points->points.push_back(pcl::PointXYZ(0.0, 1.0, 0.0));

  // Transform points to camera reference frame:
  for (unsigned int i = 0; i < ground_points->points.size(); i++)
  {
    tf::Vector3 current_point(ground_points->points[i].x, ground_points->points[i].y, ground_points->points[i].z);
    current_point = worldToCamTransform(current_point);
    ground_points->points[i].x = current_point.x();
    ground_points->points[i].y = current_point.y();
    ground_points->points[i].z = current_point.z();
  }

  // Compute ground equation:
  std::vector<int> ground_points_indices;
  for (unsigned int i = 0; i < ground_points->points.size(); i++)
    ground_points_indices.push_back(i);
  pcl::SampleConsensusModelPlane<pcl::PointXYZ> model_plane(ground_points);
  Eigen::VectorXf ground_coeffs;
  model_plane.computeModelCoefficients(ground_points_indices,ground_coeffs);
  std::cout << "Ground plane coefficients obtained from calibration: " << ground_coeffs(0) << ", " << ground_coeffs(1) << ", " << ground_coeffs(2) <<
      ", " << ground_coeffs(3) << "." << std::endl;

  return ground_coeffs;
}

template <typename PointT> tf::Transform
open_ptrack::detection::GroundplaneEstimation<PointT>::readTFFromFile (std::string filename, std::string camera_name)
{
  tf::Transform worldToCamTransform;

  ifstream poses_file;
  poses_file.open(filename.c_str());
  std::string line;
  std::string pose_string;
  while(getline(poses_file, line))
  {
    int pos = line.find(camera_name, 0);
    if (pos != std::string::npos)
    {
      pose_string = line.substr(camera_name.size() + 2, line.size() - camera_name.size() - 2);
    }
  }
  poses_file.close();

  // Create transform:
  std::vector<double> pose;
  pose.resize(7, 0.0);
  sscanf(pose_string.c_str(), "%lf %lf %lf %lf %lf %lf %lf", &pose[0], &pose[1], &pose[2], &pose[3], &pose[4], &pose[5], &pose[6]);
  worldToCamTransform.setOrigin(tf::Vector3(pose[0], pose[1], pose[2]));
  worldToCamTransform.setRotation(tf::Quaternion(pose[3], pose[4], pose[5], pose[6]));

  return worldToCamTransform;
}

template <typename PointT> Eigen::VectorXf
open_ptrack::detection::GroundplaneEstimation<PointT>::compute ()
{
  Eigen::VectorXf ground_coeffs;
  ground_coeffs.resize(4);

  // Manual mode:
  if (ground_estimation_mode_ == 0)
  {
    std::cout << "Manual mode for ground plane estimation." << std::endl;

    // Initialize viewer:
    pcl::visualization::PCLVisualizer viewer("Pick 3 points");

    // Create XYZ cloud:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB xyzrgb_point;
    cloud_xyzrgb->points.resize(cloud_->width * cloud_->height, xyzrgb_point);
    cloud_xyzrgb->width = cloud_->width;
    cloud_xyzrgb->height = cloud_->height;
    cloud_xyzrgb->is_dense = false;
    for (int i=0;i<cloud_->height;i++)
    {
      for (int j=0;j<cloud_->width;j++)
      {
        cloud_xyzrgb->at(j,i).x = cloud_->at(j,i).x;
        cloud_xyzrgb->at(j,i).y = cloud_->at(j,i).y;
        cloud_xyzrgb->at(j,i).z = cloud_->at(j,i).z;
      }
    }

//#if (XYZRGB_CLOUDS)
//    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_);
//    viewer.addPointCloud<pcl::PointXYZRGB> (cloud_, rgb, "input_cloud");
//#else
//    viewer.addPointCloud<pcl::PointXYZ> (cloud_, "input_cloud");
//#endif

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb(cloud_xyzrgb, 255, 255, 255);
    viewer.addPointCloud<pcl::PointXYZRGB> (cloud_xyzrgb, rgb, "input_cloud");
    viewer.setCameraPosition(0,0,-2,0,-1,0,0);

    // Add point picking callback to viewer:
    struct callback_args_color cb_args;

//#if (XYZRGB_CLOUDS)
//    PointCloudPtr clicked_points_3d (new PointCloud);
//#else
//    pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d (new pcl::PointCloud<pcl::PointXYZ>);
//#endif

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clicked_points_3d (new pcl::PointCloud<pcl::PointXYZRGB>);
    cb_args.clicked_points_3d = clicked_points_3d;
    cb_args.viewerPtr = &viewer;
    viewer.registerPointPickingCallback (GroundplaneEstimation::pp_callback, (void*)&cb_args);

    // Spin until 'Q' is pressed:
    viewer.spin();
    viewer.setSize(1,1);  // resize viewer in order to make it disappear
    viewer.spinOnce();
    viewer.close();       // close method does not work
    std::cout << "done." << std::endl;

    // Keep only the last three clicked points:
    while(clicked_points_3d->points.size()>3)
    {
      clicked_points_3d->points.erase(clicked_points_3d->points.begin());
    }

    // Ground plane estimation:
    std::vector<int> clicked_points_indices;
    for (unsigned int i = 0; i < clicked_points_3d->points.size(); i++)
      clicked_points_indices.push_back(i);
//    pcl::SampleConsensusModelPlane<PointT> model_plane(clicked_points_3d);
    pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> model_plane(clicked_points_3d);
    model_plane.computeModelCoefficients(clicked_points_indices,ground_coeffs);
    std::cout << "Ground plane coefficients: " << ground_coeffs(0) << ", " << ground_coeffs(1) << ", " << ground_coeffs(2) <<
        ", " << ground_coeffs(3) << "." << std::endl;
  }

  // Semi-automatic mode:
  if (ground_estimation_mode_ == 1)
  {
    std::cout << "Semi-automatic mode for ground plane estimation." << std::endl;

    // Normals computation:
    pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
    ne.setMaxDepthChangeFactor (0.03f);
    ne.setNormalSmoothingSize (20.0f);
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
    ne.setInputCloud (cloud_);
    ne.compute (*normal_cloud);

    // Multi plane segmentation initialization:
    std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
    pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
    mps.setMinInliers (500);
    mps.setAngularThreshold (2.0 * M_PI / 180);
    mps.setDistanceThreshold (0.2);
    mps.setInputNormals (normal_cloud);
    mps.setInputCloud (cloud_);
    mps.segmentAndRefine (regions);

    std::cout << "Found " << regions.size() << " planar regions." << std::endl;

    // Color planar regions with different colors:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    colored_cloud = colorRegions(regions);
    if (regions.size()>0)
    {
      // Viewer initialization:
      pcl::visualization::PCLVisualizer viewer("PCL Viewer");
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(colored_cloud);
      viewer.addPointCloud<pcl::PointXYZRGB> (colored_cloud, rgb, "input_cloud");
      viewer.setCameraPosition(0,0,-2,0,-1,0,0);

      // Add point picking callback to viewer:
      struct callback_args_color cb_args;
      typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr clicked_points_3d (new pcl::PointCloud<pcl::PointXYZRGB>);
      cb_args.clicked_points_3d = clicked_points_3d;
      cb_args.viewerPtr = &viewer;
      viewer.registerPointPickingCallback (GroundplaneEstimation::pp_callback, (void*)&cb_args);
      std::cout << "Shift+click on a floor point, then press 'Q'..." << std::endl;

      // Spin until 'Q' is pressed:
      viewer.spin();
      viewer.setSize(1,1);  // resize viewer in order to make it disappear
      viewer.spinOnce();
      viewer.close();       // close method does not work
      std::cout << "done." << std::endl;

      // Find plane closest to clicked point:
      unsigned int index = 0;
      float min_distance = FLT_MAX;
      float distance;

      float X = cb_args.clicked_points_3d->points[clicked_points_3d->points.size() - 1].x;
      float Y = cb_args.clicked_points_3d->points[clicked_points_3d->points.size() - 1].y;
      float Z = cb_args.clicked_points_3d->points[clicked_points_3d->points.size() - 1].z;

      for(unsigned int i = 0; i < regions.size(); i++)
      {
        float a = regions[i].getCoefficients()[0];
        float b = regions[i].getCoefficients()[1];
        float c = regions[i].getCoefficients()[2];
        float d = regions[i].getCoefficients()[3];

        distance = (float) (fabs((a*X + b*Y + c*Z + d)))/(sqrtf(a*a+b*b+c*c));

        if(distance < min_distance)
        {
          min_distance = distance;
          index = i;
        }
      }

      ground_coeffs[0] = regions[index].getCoefficients()[0];
      ground_coeffs[1] = regions[index].getCoefficients()[1];
      ground_coeffs[2] = regions[index].getCoefficients()[2];
      ground_coeffs[3] = regions[index].getCoefficients()[3];

      std::cout << "Ground plane coefficients: " << regions[index].getCoefficients()[0] << ", " << regions[index].getCoefficients()[1] << ", " <<
          regions[index].getCoefficients()[2] << ", " << regions[index].getCoefficients()[3] << "." << std::endl;
    }
  }

  // Automatic mode:
  if ((ground_estimation_mode_ == 2) || (ground_estimation_mode_ == 3))
  {
    std::cout << "Automatic mode for ground plane estimation." << std::endl;

    // Normals computation:

//    pcl::NormalEstimation<PointT, pcl::Normal> ne;
////    ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
////    ne.setMaxDepthChangeFactor (0.03f);
////    ne.setNormalSmoothingSize (20.0f);
//    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
//    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
//    ne.setSearchMethod (tree);
//    ne.setRadiusSearch (0.2);
//    ne.setInputCloud (cloud_);
//    ne.compute (*normal_cloud);

    pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
    ne.setMaxDepthChangeFactor (0.03f);
    ne.setNormalSmoothingSize (20.0f);
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
    ne.setInputCloud (cloud_);
    ne.compute (*normal_cloud);

//    std::cout << "Normals estimated!" << std::endl;
//
//    // Multi plane segmentation initialization:
//    std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
//    pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
//    mps.setMinInliers (500);
//    mps.setAngularThreshold (2.0 * M_PI / 180);
//    mps.setDistanceThreshold (0.2);
//    mps.setInputNormals (normal_cloud);
//    mps.setInputCloud (cloud_);
//    mps.segmentAndRefine (regions);

    // Multi plane segmentation initialization:
    std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
    pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
    mps.setMinInliers (500);
    mps.setAngularThreshold (2.0 * M_PI / 180);
    mps.setDistanceThreshold (0.2);
    mps.setInputNormals (normal_cloud);
    mps.setInputCloud (cloud_);
    mps.segmentAndRefine (regions);

//    std::cout << "Found " << regions.size() << " planar regions." << std::endl;

    // Removing planes not compatible with camera roll ~= 0:
    unsigned int i = 0;
    while(i < regions.size())
    { // Check on the normal to the plane:
      if(fabs(regions[i].getCoefficients()[1]) < 0.70)
      {
        regions.erase(regions.begin()+i);
      }
      else
        ++i;
    }

    // Order planar regions according to height (y coordinate):
    std::sort(regions.begin(), regions.end(), GroundplaneEstimation::planeHeightComparator);

    // Color selected planar region in red:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    colored_cloud = colorRegions(regions, 0);

    // If at least a valid plane remained:
    if (regions.size()>0)
    {
      ground_coeffs[0] = regions[0].getCoefficients()[0];
      ground_coeffs[1] = regions[0].getCoefficients()[1];
      ground_coeffs[2] = regions[0].getCoefficients()[2];
      ground_coeffs[3] = regions[0].getCoefficients()[3];

      std::cout << "Ground plane coefficients: " << regions[0].getCoefficients()[0] << ", " << regions[0].getCoefficients()[1] << ", " <<
          regions[0].getCoefficients()[2] << ", " << regions[0].getCoefficients()[3] << "." << std::endl;

      // Result visualization:
      if (ground_estimation_mode_ == 2)
      {
        // Viewer initialization:
        pcl::visualization::PCLVisualizer viewer("PCL Viewer");
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(colored_cloud);
        viewer.addPointCloud<pcl::PointXYZRGB> (colored_cloud, rgb, "input_cloud");
        viewer.setCameraPosition(0,0,-2,0,-1,0,0);

        // Spin until 'Q' is pressed:
        viewer.spin();
        viewer.setSize(1,1);  // resize viewer in order to make it disappear
        viewer.spinOnce();
        viewer.close();       // close method does not work
      }
    }
    else
    {
      std::cout << "ERROR: no valid ground plane found!" << std::endl;
    }
  }

  return ground_coeffs;
}

template <typename PointT> Eigen::VectorXf
open_ptrack::detection::GroundplaneEstimation<PointT>::computeMulticamera (bool ground_from_extrinsic_calibration, bool read_ground_from_file,
    std::string pointcloud_topic, int sampling_factor, float voxel_size)
{
  Eigen::VectorXf ground_coeffs;
  ground_coeffs = compute();

  std::string frame_id = cloud_->header.frame_id;
  if (strcmp(frame_id.substr(0,1).c_str(), "/") == 0)
  {
    frame_id = frame_id.substr(1, frame_id.length()-1);
  }

  // If manual ground plane selection, save the result to file:
  if (ground_estimation_mode_ == 0)
  {
    std::ofstream ground_file;
    ground_file.open ((ros::package::getPath("detection") + "/conf/ground_" + frame_id + ".txt").c_str());
    ground_file << ground_coeffs;
    ground_file.close();

    std::cout << "Ground plane saved to " << ros::package::getPath("detection") + "/conf/ground_" + frame_id + ".txt" << std::endl;
  }

  bool ground_estimated = false;
  if (read_ground_from_file)
  {
    if (ground_estimation_mode_ > 0) // automatic or semi-automatic mode
    {
      // Read ground from file, if the file exists:
      std::string ground_filename = ros::package::getPath("detection") + "/conf/ground_" + frame_id + ".txt";
      ifstream infile(ground_filename.c_str());
      if (infile.good())
      {
        std::ifstream ground_file (ground_filename.c_str());
        std::string line;
        for (unsigned int row_ind = 0; row_ind < 4; row_ind++)
        {
          getline (ground_file, line);
          ground_coeffs(row_ind) = std::atof(line.c_str());
        }
        ground_file.close();

        ground_estimated = true;
        std::cout << "Chosen ground plane read from file." << std::endl;
        std::cout << "Ground plane coefficients: " << ground_coeffs(0) << " " << ground_coeffs(1) << " " << ground_coeffs(2) << " " << ground_coeffs(3) << "." << std::endl;
      }
      else
      {
        std::cout << "Ground plane file not found!" << std::endl;
      }
    }
    else //keeps the ground plane just estimated manually.
    {
      ground_estimated = true;
      std::cout << "Ground plane manually estimated from point cloud." << std::endl;
    }
  }

  if (not ground_estimated & ground_from_extrinsic_calibration)
  { // Ground plane equation derived from extrinsic calibration:
    int pos = pointcloud_topic.find("/", 1);
    std::string camera_name = pointcloud_topic.substr(1, pos-1);

    // Read worldToCam transform from file:
    std::string filename = ros::package::getPath("detection") + "/launch/camera_poses.txt";
    tf::Transform worldToCamTransform = readTFFromFile (filename, camera_name);

    // Compute ground coeffs from world to camera transform:
    Eigen::VectorXf ground_coeffs_calib = computeFromTF (worldToCamTransform);

    // If ground could not be well estimated from point cloud data, use calibration data:
    // (if error in ground plane estimation from point cloud OR if d coefficient estimated from point cloud
    // is too different from d coefficient obtained from calibration)
    bool updated = false; // states if ground plane coefficients are updated according to the point cloud or not
    if ((ground_coeffs.sum() == 0.0) | (std::fabs(float(ground_coeffs_calib(3) - ground_coeffs(3))) > 0.2))
    {
      updated = refineGround (10, voxel_size, 300 * 0.06 / voxel_size / std::pow (static_cast<double> (sampling_factor), 2), ground_coeffs_calib);

      ground_coeffs = ground_coeffs_calib;

      if (updated)
        std::cout << "Chosen ground plane estimate obtained from calibration and refined with point cloud." << std::endl;
      else
        std::cout << "Chosen ground plane estimate obtained from calibration." << std::endl;
    }
  }

  std::cout << std::endl;

  return ground_coeffs;
}

template <typename PointT> bool
open_ptrack::detection::GroundplaneEstimation<PointT>::refineGround (int num_iter, float voxel_size, float inliers_threshold, Eigen::VectorXf& ground_coeffs_calib)
{
//  PointCloudT::Ptr no_ground_cloud(new PointCloudT);
  bool updated = false;
  for (unsigned int l = 0; l < num_iter; l++)
  {
    pcl::IndicesPtr inliers(new std::vector<int>);
    boost::shared_ptr<pcl::SampleConsensusModelPlane<PointT> > ground_model(new pcl::SampleConsensusModelPlane<PointT>(cloud_));
    ground_model->selectWithinDistance(ground_coeffs_calib, voxel_size, *inliers);

    if (inliers->size () >= inliers_threshold)
    {
      ground_model->optimizeModelCoefficients (*inliers, ground_coeffs_calib, ground_coeffs_calib);
      updated = true;
    }
    else
    {
      return updated;
    }

//    no_ground_cloud->points.clear();
//    pcl::ExtractIndices<PointT> extract;
//    extract.setInputCloud(cloud_);
//    extract.setIndices(inliers);
//    extract.setNegative(true);
//    extract.filter(*no_ground_cloud);
  }

  return updated;
}

template <typename PointT> void
open_ptrack::detection::GroundplaneEstimation<PointT>::pp_callback (const pcl::visualization::PointPickingEvent& event, void* args)
{
  struct callback_args_color* data = (struct callback_args_color *)args;
  if (event.getPointIndex () == -1)
    return;
  pcl::PointXYZRGB current_point;
  event.getPoint(current_point.x, current_point.y, current_point.z);
  data->clicked_points_3d->points.push_back(current_point);
  // Draw clicked points in red:
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red (data->clicked_points_3d, 255, 0, 0);
  data->viewerPtr->removePointCloud("clicked_points");
  data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
  data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
  std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}

template <typename PointT> bool
open_ptrack::detection::GroundplaneEstimation<PointT>::planeHeightComparator (pcl::PlanarRegion<PointT> region1, pcl::PlanarRegion<PointT> region2)
{
  return region1.getCentroid()[1] > region2.getCentroid()[1];
}

template <typename PointT> pcl::PointCloud<pcl::PointXYZRGB>::Ptr
open_ptrack::detection::GroundplaneEstimation<PointT>::colorRegions (std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions, int index)
{
  // Color different planes with different colors:
  float voxel_size = 0.06;

  // Initialize colored point cloud with points from input point cloud:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointXYZRGB white_point;
  white_point.r = 255;
  white_point.g = 255;
  white_point.b = 255;
  colored_cloud->points.resize(cloud_->width * cloud_->height, white_point);
  colored_cloud->width = cloud_->width;
  colored_cloud->height = cloud_->height;
  colored_cloud->is_dense = false;
  for (int i=0;i<cloud_->height;i++)
  {
    for (int j=0;j<cloud_->width;j++)
    {
      colored_cloud->at(j,i).x = cloud_->at(j,i).x;
      colored_cloud->at(j,i).y = cloud_->at(j,i).y;
      colored_cloud->at(j,i).z = cloud_->at(j,i).z;
    }
  }

  for (size_t i = 0; i < regions.size (); i++)
  {
    Eigen::Vector3f centroid = regions[i].getCentroid ();
    Eigen::Vector4f model = regions[i].getCoefficients ();

    pcl::IndicesPtr inliers(new std::vector<int>);
    boost::shared_ptr<pcl::SampleConsensusModelPlane<PointT> > ground_model(new pcl::SampleConsensusModelPlane<PointT>(cloud_));
    ground_model->selectWithinDistance(model, voxel_size, *inliers);

    int r = (rand() % 256);
    int g = (rand() % 256);
    int b = (rand() % 256);

    for(unsigned int j = 0; j < inliers->size(); j++)
    {
      colored_cloud->points.at(inliers->at(j)).r = r;
      colored_cloud->points.at(inliers->at(j)).g = g;
      colored_cloud->points.at(inliers->at(j)).b = b;
    }
  }

  // If index is passed, color index-th region in red:
  if (index >= 0 && !regions.empty())
  {
    Eigen::Vector3f centroid = regions[index].getCentroid ();
    Eigen::Vector4f model = regions[index].getCoefficients ();

    pcl::IndicesPtr inliers(new std::vector<int>);
    boost::shared_ptr<pcl::SampleConsensusModelPlane<PointT> > ground_model(new pcl::SampleConsensusModelPlane<PointT>(cloud_));
    ground_model->selectWithinDistance(model, voxel_size, *inliers);

    for(unsigned int j = 0; j < inliers->size(); j++)
    {
      colored_cloud->points.at(inliers->at(j)).r = 255;
      colored_cloud->points.at(inliers->at(j)).g = 0;
      colored_cloud->points.at(inliers->at(j)).b = 0;
    }
  }

  return colored_cloud;
}
