/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2016-, Matteo Munaro
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
 * Author: Matteo Munaro [matteo.munaro@dei.unipd.it]
 *
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <opt_msgs/TrackArray.h>
#include <Eigen/Dense>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

// Used to display OPENCV images
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>

#include <dynamic_reconfigure/server.h>
#include <opt_utils/HeatmapBuilderConfig.h>

namespace open_ptrack
{
namespace opt_utils
{

typedef ::opt_utils::HeatmapBuilderConfig Config;
typedef ::dynamic_reconfigure::Server<Config> ReconfigureServer;

class HeatmapBuilder
{

public:

    HeatmapBuilder(ros::NodeHandle & node_handle)
    : node_handle_(node_handle),
      reconfigure_server_(new ReconfigureServer(config_mutex_, node_handle_)),
      rate_(ros::Rate(30.0))
  {
    double rate_d;

    node_handle_.param("rate", rate_d, 30.0);
    node_handle_.param("map_real_size", map_real_size_, 20.0);
    node_handle_.param("map_pixel_size", map_pixel_size_, 400);

    tracking_grid_ = cv::Mat::zeros(100, 100, CV_32S);

    heatmap_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >();
    heatmap_cloud_->header.frame_id = "world";
    heatmap_cloud_->width = map_pixel_size_;
    heatmap_cloud_->height = map_pixel_size_;
    heatmap_cloud_->points.resize(heatmap_cloud_->width * heatmap_cloud_->height, pcl::PointXYZRGB(0, 0, 0));
    for (int i=0; i<heatmap_cloud_->height; i++)
    {
      for (int j=0; j<heatmap_cloud_->width; j++)
      {
        heatmap_cloud_->at(j,i).x = j * map_real_size_ / heatmap_cloud_->width - map_real_size_ / 2;
        heatmap_cloud_->at(j,i).y = i * map_real_size_ / heatmap_cloud_->height - map_real_size_ / 2;
        heatmap_cloud_->at(j,i).z = 0.0;
      }
    }

    tracking_sub_ = node_handle_.subscribe<opt_msgs::TrackArray>("input", 100, &HeatmapBuilder::trackingCallback, this);
    heatmap_pub_ = node_handle_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("heatmap", 1);

    rate_ = ros::Rate(rate_d);

    reset_heatmap_ = false;

    ReconfigureServer::CallbackType callback = boost::bind(&HeatmapBuilder::configCallback, this, _1, _2);
    reconfigure_server_->setCallback(callback);

  }

  void configCallback(Config & config, uint32_t level)
  {
    config_mutex_.lock();

    // rate:
    rate_ = ros::Rate(config.rate);

    // reset_heatmap:
    if ((reset_heatmap_ == false) && (config.reset_heatmap == true))
    {
      tracking_grid_ = cv::Mat::zeros(tracking_grid_.rows, tracking_grid_.cols, CV_32S);
      reset_heatmap_ = true;
    }
    else if (config.reset_heatmap == false)
      reset_heatmap_ = false;

    // max_pixel_size:
    if (config.map_pixel_size != map_pixel_size_)
    {
      map_pixel_size_ = std::min(config.map_pixel_size, 4*tracking_grid_.rows);

      //heatmap_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >();
      heatmap_cloud_->header.frame_id = "world";
      heatmap_cloud_->points.clear();
      heatmap_cloud_->width = map_pixel_size_;
      heatmap_cloud_->height = map_pixel_size_;
      heatmap_cloud_->points.resize(heatmap_cloud_->width * heatmap_cloud_->height, pcl::PointXYZRGB(0, 0, 0));
      for (int i=0; i<heatmap_cloud_->height; i++)
      {
        for (int j=0; j<heatmap_cloud_->width; j++)
        {
          heatmap_cloud_->at(j,i).x = j * map_real_size_ / heatmap_cloud_->width - map_real_size_ / 2;
          heatmap_cloud_->at(j,i).y = i * map_real_size_ / heatmap_cloud_->height - map_real_size_ / 2;
          heatmap_cloud_->at(j,i).z = 0.0;
        }
      }
    }

    // map_real_size:
    if (config.map_real_size != map_real_size_)
    {
      map_real_size_ = config.map_real_size;

      for (int i=0; i<heatmap_cloud_->height; i++)
      {
        for (int j=0; j<heatmap_cloud_->width; j++)
        {
          heatmap_cloud_->at(j,i).x = j * map_real_size_ / heatmap_cloud_->width - map_real_size_ / 2;
          heatmap_cloud_->at(j,i).y = i * map_real_size_ / heatmap_cloud_->height - map_real_size_ / 2;
          heatmap_cloud_->at(j,i).z = 0.0;
        }
      }

      // Reset tracking_grid:
      tracking_grid_ = cv::Mat::zeros(tracking_grid_.rows, tracking_grid_.cols, CV_32S);
    }

    config_mutex_.unlock();
  }

  void trackingCallback(const opt_msgs::TrackArray::ConstPtr & tracking_msg)
  {
    // Read tracking data and update tracking_grid_:
    for (size_t i = 0; i < tracking_msg->tracks.size(); ++i)
    {
      opt_msgs::Track msg_track = tracking_msg->tracks[i];

      int x_grid = (tracking_grid_.cols * msg_track.x / map_real_size_) + tracking_grid_.cols / 2;
      int y_grid = (tracking_grid_.rows * msg_track.y / map_real_size_) + tracking_grid_.rows / 2;

      if ((x_grid >= 0) && (y_grid >= 0) && (x_grid < tracking_grid_.cols) && (y_grid < tracking_grid_.rows))
      {
        tracking_grid_.at<int>(y_grid, x_grid) = tracking_grid_.at<int>(y_grid, x_grid) + 1;
      }
    }

    // Rescaling between 0 and 255:
    double maxN, minN;
    cv::minMaxLoc(tracking_grid_, &minN, &maxN);
    cv::Mat tracking_grid_rescaled = 255 * ((tracking_grid_ - minN) / float(maxN - minN));
    cv::Mat heatmap_gray, heatmap_color;
    tracking_grid_rescaled.convertTo(heatmap_gray, CV_8U);

    // Blur image:
    unsigned int kernel_size = 60.0 / map_real_size_;
    if ((kernel_size % 2) == 0) // force the kernel to be an odd number
      kernel_size++;
    cv::GaussianBlur( heatmap_gray, heatmap_gray, cv::Size( kernel_size, kernel_size ), kernel_size, kernel_size );
    cv::resize(heatmap_gray, heatmap_gray, cv::Size(2*heatmap_gray.rows, 2*heatmap_gray.cols));
    cv::GaussianBlur( heatmap_gray, heatmap_gray, cv::Size( kernel_size, kernel_size ), kernel_size, kernel_size );
    cv::resize(heatmap_gray, heatmap_gray, cv::Size(2*heatmap_gray.rows, 2*heatmap_gray.cols));

    // Rescaling between 0 and 255:
    cv::minMaxLoc(heatmap_gray, &minN, &maxN);
    tracking_grid_rescaled = 255 * ((heatmap_gray - minN) / float(maxN - minN));
    tracking_grid_rescaled.convertTo(heatmap_gray, CV_8U);

    // Convert to colors (JET colormap):
    cv::applyColorMap(heatmap_gray, heatmap_color, cv::COLORMAP_JET);

    // Report colors to the point cloud heatmap:
    float scale_factor =  heatmap_color.cols / float(heatmap_cloud_->width);
    for (int i=0; i<heatmap_cloud_->height; i++)
    {
      for (int j=0; j<heatmap_cloud_->width; j++)
      {
        heatmap_cloud_->at(j,i).r = heatmap_color.at<cv::Vec3b>((unsigned int) (i*scale_factor), (unsigned int) (j*scale_factor))[2];
        heatmap_cloud_->at(j,i).g = heatmap_color.at<cv::Vec3b>((unsigned int) (i*scale_factor), (unsigned int) (j*scale_factor))[1];
        heatmap_cloud_->at(j,i).b = heatmap_color.at<cv::Vec3b>((unsigned int) (i*scale_factor), (unsigned int) (j*scale_factor))[0];
      }
    }
    heatmap_cloud_->header.stamp = tracking_msg->header.stamp.toNSec() / 1000;

//    cv::namedWindow("Heatmap image", cv::WINDOW_NORMAL);
//    cv::imshow("Heatmap image", heatmap_color);
//    cv::waitKey(1);

    if (maxN > std::pow(2, 31)) // to avoid overflow
      tracking_grid_ = tracking_grid_ / 2;
  }

  void spin()
  {
    while (ros::ok())
    {
      ros::spinOnce();

      config_mutex_.lock();
      heatmap_pub_.publish(heatmap_cloud_);
      config_mutex_.unlock();

      rate_.sleep();
    }
  }

private:

  ros::NodeHandle node_handle_;

  boost::recursive_mutex config_mutex_;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  ros::Subscriber tracking_sub_;
  ros::Publisher heatmap_pub_;

  // Grid containing tracks position:
  cv::Mat tracking_grid_;

  // Heatmap represented as point cloud:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr heatmap_cloud_;

  ros::Rate rate_;

  // Max size of the heatmap (in meters and pixels):
  double map_real_size_;
  int map_pixel_size_;

  // Flag stating if the heatmap should be reset:
  bool reset_heatmap_;
};

} // namespace opt_utils
} // namespace open_ptrack

using open_ptrack::opt_utils::HeatmapBuilder;

int main(int argc, char **argv)
{
  // Initialization:
  ros::init(argc, argv, "heatmap_builder");
  ros::NodeHandle nh("~");

  HeatmapBuilder heatmap_builder(nh);
  heatmap_builder.spin();

  return 0;
}
