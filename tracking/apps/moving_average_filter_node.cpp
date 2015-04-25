/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015-, Open Perception, Inc.
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
 * Author: Filippo Basso [bassofil@dei.unipd.it]
 *         Matteo Munaro [matteo.munaro@dei.unipd.it]
 *
 */

#include <ros/ros.h>
#include <opt_msgs/TrackArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>

#include <open_ptrack/tracking/track.h>

#include <dynamic_reconfigure/server.h>
#include <tracking/MovingAverageSmootherConfig.h>

namespace open_ptrack
{
namespace tracking
{

typedef ::tracking::MovingAverageSmootherConfig Config;
typedef ::dynamic_reconfigure::Server<Config> ReconfigureServer;


struct TrackPositionNode
{
  typedef boost::shared_ptr<TrackPositionNode> Ptr;

  TrackPositionNode(const Eigen::Array2d & position, const ros::Time & time) : position_(position), time_(time) {}

  const Eigen::Array2d position_;
  const ros::Time time_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class TrackFilter
{
public:

  typedef boost::shared_ptr<TrackFilter> Ptr;

  TrackFilter(int window_size) : window_size_(window_size), last_positions_sum_(Eigen::Array2d::Zero()) {}

  Eigen::Array2d movingAveragePosition()
  {
    return last_positions_sum_ / last_positions_.size();
  }

  void addPosition(const Eigen::Array2d & position, const ros::Time & time)
  {
    TrackPositionNode::Ptr node = boost::make_shared<TrackPositionNode>(position, time);
    if (last_positions_.size() < window_size_)
    {
      last_positions_.push_front(node);
      last_positions_sum_ += node->position_;
    }
    else
    {
      const TrackPositionNode::Ptr & remove_node = last_positions_.back();
      last_positions_sum_ -= remove_node->position_;
      last_positions_.pop_back();
      last_positions_.push_front(node);
      last_positions_sum_ += node->position_;
    }
  }

  void setWindowSize(int new_size)
  {
    if (last_positions_.size() > new_size)
    {
      while (last_positions_.size() > new_size)
      {
        const TrackPositionNode::Ptr & remove_node = last_positions_.back();
        last_positions_sum_ -= remove_node->position_;
        last_positions_.pop_back();
      }
    }
    window_size_ = new_size;
  }

  void removeOldPositions(const ros::Time & min_allowed_time)
  {
    bool removed = false;
    while (not removed)
    {
      const TrackPositionNode::Ptr & test_node = last_positions_.back();
      if (test_node->time_ < min_allowed_time and last_positions_.size() > 1)
      {
        last_positions_sum_ -= test_node->position_;
        last_positions_.pop_back();
      }
      else
      {
        removed = true;
      }
    }
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:

  std::list<TrackPositionNode::Ptr> last_positions_;
  int window_size_;

  Eigen::Array2d last_positions_sum_;

};

struct TrackInfo
{
  TrackFilter::Ptr track_;
  opt_msgs::Track::Ptr last_msg_;
  ros::Time last_msg_time_;
};

class MovingAverageSmoother
{

public:

  MovingAverageSmoother(ros::NodeHandle & node_handle)
    : node_handle_(node_handle),
      reconfigure_server_(new ReconfigureServer(config_mutex_, node_handle_)),
      rate_(ros::Rate(30.0))
  {
    double rate_d, heartbeat_time;

    node_handle_.param("rate", rate_d, 30.0);
    node_handle_.param("publish_empty", publish_empty_, true);
    node_handle_.param("heartbeat_time", heartbeat_time, 5.0);
    node_handle_.param("max_history_size", max_history_size_, 1000);

    node_handle_.param("window_size", window_size_, 5);

    double position_lifetime_d, track_lifetime_d;
    node_handle_.param("track_lifetime_with_no_detections", track_lifetime_d, 5.0);
    node_handle_.param("position_lifetime", position_lifetime_d, 0.5);
    track_lifetime_ = ros::Duration(track_lifetime_d);
    position_lifetime_ = ros::Duration(position_lifetime_d);

    generateColors();
    history_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >();
    history_cloud_index_ = 0;
    history_cloud_->header.frame_id = "world";

    heartbeat_time_duration_ = ros::Duration(heartbeat_time);

    tracking_sub_ = node_handle_.subscribe<opt_msgs::TrackArray>("input", 100, &MovingAverageSmoother::trackingCallback, this);
    tracking_pub_ = node_handle_.advertise<opt_msgs::TrackArray>("output", 1);
    marker_array_pub_ = node_handle_.advertise<visualization_msgs::MarkerArray>("markers_array", 1);
    history_pub_ = node_handle_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("history", 1);

    rate_ = ros::Rate(rate_d);

    ReconfigureServer::CallbackType callback = boost::bind(&MovingAverageSmoother::configCallback, this, _1, _2);
    reconfigure_server_->setCallback(callback);

  }

  void configCallback(Config & config, uint32_t level)
  {
    //rate_ = ros::Rate(config.rate);

    config.track_lifetime_with_no_detections = std::max(config.track_lifetime_with_no_detections, rate_.expectedCycleTime().toSec());
    track_lifetime_ = ros::Duration(config.track_lifetime_with_no_detections);

    heartbeat_time_duration_ = ros::Duration(config.heartbeat_time);
    publish_empty_ = config.publish_empty;

    window_size_ = config.window_size;
    for (std::map<int, TrackInfo>::iterator it = track_map_.begin(); it != track_map_.end(); ++it)
    {
      TrackInfo & track_info = it->second;
      track_info.track_->setWindowSize(window_size_);
    }

    config.position_lifetime = std::max(config.position_lifetime, rate_.expectedCycleTime().toSec());
    position_lifetime_ = ros::Duration(config.position_lifetime);

    max_history_size_ = config.max_history_size;

  }

  void trackingCallback(const opt_msgs::TrackArray::ConstPtr & msg)
  {
    for (size_t i = 0; i < msg->tracks.size(); ++i)
    {
      const opt_msgs::Track & track_msg = msg->tracks[i];

      std::map<int, TrackInfo>::iterator it = track_map_.find(track_msg.id);

      if (it == track_map_.end()) // Track does not exist
      {
        TrackInfo track_info;
        track_info.track_ = boost::make_shared<TrackFilter>(window_size_);
        track_map_[track_msg.id] = track_info;
      }

      TrackInfo & track_info = track_map_[track_msg.id];

      track_info.track_->addPosition(Eigen::Array2d(track_msg.x, track_msg.y), msg->header.stamp);
      track_info.last_msg_ = boost::make_shared<opt_msgs::Track>(track_msg);
      if (track_info.last_msg_->visibility < 2)
        track_info.last_msg_time_ = msg->header.stamp;
    }
  }

  void spin()
  {
    ros::Time last_heartbeat_time = ros::Time::now();

    while (ros::ok())
    {
      ros::spinOnce();

      opt_msgs::TrackArray track_msg;
      visualization_msgs::MarkerArray marker_msg;

      int n = createMsg(track_msg, marker_msg);
      ros::Time current_time = ros::Time::now();

      config_mutex_.lock();
      if (publish_empty_ or n > 0)
      {
        tracking_pub_.publish(track_msg);
        marker_array_pub_.publish(marker_msg);
        history_pub_.publish(history_cloud_);
        last_heartbeat_time = current_time;
      }
      else if (not publish_empty_)
      {
        // Publish a heartbeat message every 'heartbeat_time' seconds
        if ((current_time - last_heartbeat_time) > heartbeat_time_duration_)
        {
          opt_msgs::TrackArray heartbeat_msg;
          heartbeat_msg.header.stamp = current_time;
          heartbeat_msg.header.frame_id = "heartbeat";
          tracking_pub_.publish(heartbeat_msg);
          marker_array_pub_.publish(marker_msg);
          history_pub_.publish(history_cloud_);
          last_heartbeat_time = current_time;
        }
      }
      config_mutex_.unlock();

      rate_.sleep();
    }
  }

private:

  void generateColors()
  {
    for (size_t i = 0; i <= 4; ++i)
      for (size_t j = 0; j <= 4; ++j)
        for (size_t k = 0; k <= 4; ++k)
          color_set_.push_back(Eigen::Vector3f(i * 0.25f, j * 0.25f, k * 0.25f));

    std::random_shuffle(color_set_.begin(), color_set_.end());
  }

  void appendToHistory(const opt_msgs::Track & track_msg)
  {
//    if (data.last_msg.tracks[0].visibility == opt_msgs::Track::NOT_VISIBLE)
//      return;

    config_mutex_.lock();
    if (history_cloud_->size() < max_history_size_)
    {
      pcl::PointXYZRGB point;
      history_cloud_->push_back(point);
    }
    config_mutex_.unlock();

    toPointXYZRGB(history_cloud_->points[history_cloud_index_], track_msg);
    history_cloud_index_ = (history_cloud_index_ + 1) % max_history_size_;
  }

  int createMsg(opt_msgs::TrackArray & track_msg,
                visualization_msgs::MarkerArray & marker_msg)
  {
    int added = 0;
    ros::Time now = ros::Time::now();

    track_msg.header.stamp = now;
    track_msg.header.frame_id = "world";

    history_cloud_->header.stamp = now.toNSec() / 1000;
    std::vector<int> to_remove;

    for (std::map<int, TrackInfo>::iterator it = track_map_.begin(); it != track_map_.end(); ++it)
    {
      TrackInfo & track_info = it->second;

      config_mutex_.lock();
      bool ok = (now - track_info.last_msg_time_) < track_lifetime_;
      config_mutex_.unlock();

      if (ok)
      {
        track_info.track_->removeOldPositions(now - position_lifetime_);

        Eigen::Array2d position = track_info.track_->movingAveragePosition();
        track_info.last_msg_->x = position[0];
        track_info.last_msg_->y = position[1];

        track_msg.tracks.push_back(*track_info.last_msg_);
        createMarker(marker_msg, *track_info.last_msg_, track_msg.header);
        appendToHistory(*track_info.last_msg_);
        ++added;

      }
      else
      {
        to_remove.push_back(it->first);
      }
    }

    for (size_t i = 0; i < to_remove.size(); ++i)
      track_map_.erase(to_remove[i]);


    return added;
  }


  void createMarker(visualization_msgs::MarkerArray & msg,
                    const opt_msgs::Track & track_msg,
                    const std_msgs::Header & header)
  {
//    if(track.visibility == Track::NOT_VISIBLE)
//      return;

    visualization_msgs::Marker marker;

    marker.header.frame_id = header.frame_id;
    marker.header.stamp = header.stamp;

    marker.ns = "people";
    marker.id = track_msg.id;

    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = track_msg.x;
    marker.pose.position.y = track_msg.y;
    marker.pose.position.z = 3 * track_msg.height / 4;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.r = color_set_[track_msg.id % color_set_.size()](2);
    marker.color.g = color_set_[track_msg.id % color_set_.size()](1);
    marker.color.b = color_set_[track_msg.id % color_set_.size()](0);
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration(0.2);

    msg.markers.push_back(marker);

    //------------------------------------

    visualization_msgs::Marker text_marker;

    text_marker.header.frame_id = header.frame_id;
    text_marker.header.stamp = header.stamp;

    text_marker.ns = "numbers";
    text_marker.id = track_msg.id;

    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;

    std::stringstream ss;
    ss << track_msg.id;
    text_marker.text = ss.str();

    text_marker.pose.position.x = track_msg.x;
    text_marker.pose.position.y = track_msg.y;
    text_marker.pose.position.z = track_msg.height + 0.1;
    text_marker.pose.orientation.x = 0.0;
    text_marker.pose.orientation.y = 0.0;
    text_marker.pose.orientation.z = 0.0;
    text_marker.pose.orientation.w = 1.0;

    text_marker.scale.x = 0.2;
    text_marker.scale.y = 0.2;
    text_marker.scale.z = 0.2;

    text_marker.color.r = color_set_[track_msg.id % color_set_.size()](2);
    text_marker.color.g = color_set_[track_msg.id % color_set_.size()](1);
    text_marker.color.b = color_set_[track_msg.id % color_set_.size()](0);
    text_marker.color.a = 1.0;

    text_marker.lifetime = ros::Duration(0.2);

    msg.markers.push_back(text_marker);
  }

  void toPointXYZRGB(pcl::PointXYZRGB & p,
                     const opt_msgs::Track & track_msg)
  {
//    if(track.visibility == Track::NOT_VISIBLE)
//      return;

    p.x = float(track_msg.x);
    p.y = float(track_msg.y);
    p.z = float(3 * track_msg.height / 4);
    uchar * rgb_ptr = reinterpret_cast<uchar *>(&p.rgb);
    *rgb_ptr++ = uchar(color_set_[track_msg.id % color_set_.size()](0) * 255.0f);
    *rgb_ptr++ = uchar(color_set_[track_msg.id % color_set_.size()](1) * 255.0f);
    *rgb_ptr++ = uchar(color_set_[track_msg.id % color_set_.size()](2) * 255.0f);
  }

  ros::NodeHandle node_handle_;

  boost::recursive_mutex config_mutex_;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  ros::Subscriber tracking_sub_;
  ros::Publisher tracking_pub_;
  ros::Publisher marker_array_pub_;
  ros::Publisher history_pub_;

  bool publish_empty_;
  ros::Duration heartbeat_time_duration_;

  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > color_set_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr history_cloud_;
  int max_history_size_;
  size_t history_cloud_index_;

  ros::Rate rate_;
  int window_size_;

  ros::Duration track_lifetime_;
  ros::Duration position_lifetime_;

  std::map<int, TrackInfo> track_map_;

};

} // namespace tracking
} // namespace open_ptrack

using open_ptrack::tracking::MovingAverageSmoother;

int main(int argc, char **argv)
{
  // Initialization:
  ros::init(argc, argv, "tracking_filter");
  ros::NodeHandle nh("~");

  MovingAverageSmoother smoother(nh);
  smoother.spin();

  return 0;
}
