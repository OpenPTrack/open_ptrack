/*
 * Software License Agreement (BSD License)
 *
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
#include <tracking/TrackerSmootherConfig.h>

namespace open_ptrack
{
namespace tracking
{

typedef ::tracking::TrackerSmootherConfig Config;
typedef ::dynamic_reconfigure::Server<Config> ReconfigureServer;

struct LastData
{
  ros::Time last_visible_time;
  opt_msgs::TrackArray last_msg;
};

struct TrackState
{
  Eigen::Array2d position_, velocity_, acceleration_;

  void predict(double delta_t, Eigen::Array2d & position, Eigen::Array2d & velocity, Eigen::Array2d & acceleration)
  {
    position = position_ + delta_t * (velocity_ + delta_t * 0.5 * acceleration_);
    velocity = velocity_ + delta_t * acceleration_;
    acceleration  = acceleration_;
  }

  void predict(double delta_t)
  {
    position_ += delta_t * (velocity_ + delta_t * 0.5 * acceleration_);
    velocity_ += delta_t * acceleration_;
  }

  void update(const Eigen::Array2d & measured_position, double delta_t, double alpha, double beta, double phi)
  {
    Eigen::Array2d residual = measured_position - position_;
    position_ += alpha * residual;
    velocity_ += beta / delta_t * residual;
    acceleration_ += phi * 0.5 / (delta_t * delta_t) * residual;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

class TrackSmoother
{

public:

  TrackSmoother(ros::NodeHandle & node_handle)
    : node_handle_(node_handle),
      reconfigure_server_(new ReconfigureServer(config_mutex_, node_handle_)),
      rate_(ros::Rate(30.0))
  {
    double rate_d, track_lifetime_with_no_detections, heartbeat_time;

    node_handle_.param("rate", rate_d, 30.0);
    node_handle_.param("track_lifetime_with_no_detections", track_lifetime_with_no_detections, 1.0);
    node_handle_.param("publish_empty", publish_empty_, true);
    node_handle_.param("heartbeat_time", heartbeat_time, 5.0);
    node_handle_.param("max_history_size", max_history_size_, 1000);

    double sigma_process, sigma_noise;
    node_handle_.param("sigma_process", sigma_process, 1.0);
    node_handle_.param("sigma_noise", sigma_noise, 0.0);
    computeWeights(sigma_process, sigma_noise);

    double prediction_duration_d;
    node_handle_.param("prediction_duration", prediction_duration_d, 0.1);
    prediction_duration_ = ros::Duration(prediction_duration_d);

    generateColors();
    history_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >();
    history_cloud_index_ = 0;
    history_cloud_->header.frame_id = "world";

    time_alive_ = ros::Duration(track_lifetime_with_no_detections);
    heartbeat_time_duration_ = ros::Duration(heartbeat_time);

    tracking_sub_ = node_handle_.subscribe<opt_msgs::TrackArray>("input", 100, &TrackSmoother::trackingCallback, this);
    tracking_pub_ = node_handle_.advertise<opt_msgs::TrackArray>("output", 1);
    marker_array_pub_ = node_handle_.advertise<visualization_msgs::MarkerArray>("markers_array", 1);
    history_pub_ = node_handle_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("history", 1);

    rate_ = ros::Rate(rate_d);

    ReconfigureServer::CallbackType callback = boost::bind(&TrackSmoother::configCallback, this, _1, _2);
    reconfigure_server_->setCallback(callback);

  }

  void configCallback(Config & config, uint32_t level)
  {
    //rate_ = ros::Rate(config.rate);

    config.track_lifetime_with_no_detections = std::max(config.track_lifetime_with_no_detections, rate_.expectedCycleTime().toSec());
    time_alive_ = ros::Duration(config.track_lifetime_with_no_detections);

    heartbeat_time_duration_ = ros::Duration(config.heartbeat_time);
    publish_empty_ = config.publish_empty;

    computeWeights(config.sigma_process, config.sigma_noise);
    prediction_duration_ = ros::Duration(config.prediction_duration);

    max_history_size_ = config.max_history_size;

  }

  void trackingCallback(const opt_msgs::TrackArray::ConstPtr & tracking_msg)
  {
    for (size_t i = 0; i < tracking_msg->tracks.size(); ++i)
    {
      opt_msgs::Track msg_track = tracking_msg->tracks[i];
      opt_msgs::TrackArray msg;
      msg.header = tracking_msg->header;

      std::map<int, LastData>::iterator it = last_data_map_.find(msg_track.id);

      if (it != last_data_map_.end()) // Track exists
      {
        LastData & last_data = it->second;
        double delta_t = (msg.header.stamp - last_data.last_msg.header.stamp).toSec();
        if (delta_t <= 0)
          break;

        if (state_map_.find(msg_track.id) != state_map_.end())
        {
          TrackState & state = *state_map_[msg_track.id];
          state.predict(delta_t);
          config_mutex_.lock();
          state.update(Eigen::Array2d(msg_track.x, msg_track.y), delta_t, alpha_, beta_, gamma_);
          config_mutex_.unlock();
        }
        else
        {
          boost::shared_ptr<TrackState> state = boost::make_shared<TrackState>();
          state->position_ = Eigen::Array2d(msg_track.x, msg_track.y);
          state->velocity_ = Eigen::Array2d(0, 0);//(state->position_ - Eigen::Array2d(last_data.last_msg.tracks[0].x, last_data.last_msg.tracks[0].y)) / delta_t;
          state->acceleration_ = Eigen::Array2d(0, 0);
          state_map_[msg_track.id] = state;
        }

        TrackState & state = *state_map_[msg_track.id];
        msg_track.x = state.position_[0];
        msg_track.y = state.position_[1];
        msg.tracks.push_back(msg_track);

        last_data.last_msg = msg;
        if (msg_track.visibility != opt_msgs::Track::NOT_VISIBLE)
          last_data.last_visible_time = msg.header.stamp;

      }
      else// if (msg_track.visibility != opt_msgs::Track::NOT_VISIBLE)
      {
        LastData data;
        msg.tracks.push_back(msg_track);
        data.last_msg = msg;
        data.last_visible_time = msg.header.stamp;
        last_data_map_[msg_track.id] = data;
      }
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

  void computeWeights(double sigma_process, double sigma_noise)
  {
    if (sigma_noise < 1e-5)
    {
      alpha_ = 1.0;
      beta_ = gamma_ = 0.0;
    }
    else
    {
      double t = rate_.expectedCycleTime().toSec();
      double lambda = sigma_process * t * t / sigma_noise;
      double b = lambda / 2.0 - 3.0;
      double c = lambda / 2.0 + 3.0;
      double p = c - b * b / 3.0;
      double q = 2.0 * b * b * b / 27.0 - b * c / 3.0 - 1.0;
      double v = std::sqrt(q * q + 4 * p * p * p / 27.0);
      double z = -std::pow(q + v / 2.0, 1.0 / 3.0);
      double s = z - p / (3.0 * z) - b / 3.0;
      alpha_ = 1.0 - s * s;
      beta_ = 2.0 * (1.0 - s) * (1.0 - s);
      gamma_ = beta_ * beta_ / (2 * alpha_);
    }
  }

  void generateColors()
  {
    for (size_t i = 0; i <= 4; ++i)
      for (size_t j = 0; j <= 4; ++j)
        for (size_t k = 0; k <= 4; ++k)
          color_set_.push_back(Eigen::Vector3f(i * 0.25f, j * 0.25f, k * 0.25f));

    std::random_shuffle(color_set_.begin(), color_set_.end());
  }

  void appendToHistory(const LastData & data)
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

    toPointXYZRGB(history_cloud_->points[history_cloud_index_], data);
    history_cloud_index_ = (history_cloud_index_ + 1) % max_history_size_;
  }

  int createMsg(opt_msgs::TrackArray & track_msg,
                visualization_msgs::MarkerArray & marker_msg)
  {
    int added = 0;
    ros::Time now = ros::Time::now();

    track_msg.header.stamp = now;
    history_cloud_->header.stamp = now.toNSec() / 1000;
    std::vector<int> to_remove;

    for (std::map<int, LastData>::iterator it = last_data_map_.begin(); it != last_data_map_.end(); ++it)
    {
      opt_msgs::TrackArray & saved_msg = it->second.last_msg;
      const ros::Time & time = it->second.last_visible_time;

      config_mutex_.lock();
      bool ok = (now - time) < time_alive_;
      bool prediction = (now - time) < prediction_duration_;
      config_mutex_.unlock();

      if (ok)
      {
        if (state_map_.find(it->first) != state_map_.end() and prediction)
        {
          Eigen::Array2d position, velocity, acceleration;
          state_map_[it->first]->predict((now - time).toSec(), position, velocity, acceleration);
          saved_msg.tracks[0].x = position[0];
          saved_msg.tracks[0].y = position[1];
          track_msg.tracks.push_back(saved_msg.tracks[0]);
          track_msg.header.frame_id = "world";
          createMarker(marker_msg, it->second);
          appendToHistory(it->second);
          ++added;
        }
      }
      else
      {
        to_remove.push_back(saved_msg.tracks[0].id);
      }
    }

    for (size_t i = 0; i < to_remove.size(); ++i)
    {
      last_data_map_.erase(to_remove[i]);
      state_map_.erase(to_remove[i]);
    }

    return added;
  }


  void createMarker(visualization_msgs::MarkerArray & msg,
                    const LastData & data)
  {
    const opt_msgs::Track & track = data.last_msg.tracks[0];

//    if(track.visibility == Track::NOT_VISIBLE)
//      return;

    visualization_msgs::Marker marker;

    marker.header.frame_id = data.last_msg.header.frame_id;
    marker.header.stamp = data.last_msg.header.stamp;

    marker.ns = "people";
    marker.id = track.id;

    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = track.x;
    marker.pose.position.y = track.y;
    marker.pose.position.z = track.height / 2;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.r = color_set_[track.id % color_set_.size()](2);
    marker.color.g = color_set_[track.id % color_set_.size()](1);
    marker.color.b = color_set_[track.id % color_set_.size()](0);
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration(0.2);

    msg.markers.push_back(marker);

    //------------------------------------

    visualization_msgs::Marker text_marker;

    text_marker.header.frame_id = data.last_msg.header.frame_id;
    text_marker.header.stamp = data.last_msg.header.stamp;

    text_marker.ns = "numbers";
    text_marker.id = track.id;

    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;

    std::stringstream ss;
    ss << track.id;
    text_marker.text = ss.str();

    text_marker.pose.position.x = track.x;
    text_marker.pose.position.y = track.y;
    text_marker.pose.position.z = track.height + 0.1;
    text_marker.pose.orientation.x = 0.0;
    text_marker.pose.orientation.y = 0.0;
    text_marker.pose.orientation.z = 0.0;
    text_marker.pose.orientation.w = 1.0;

    text_marker.scale.x = 0.2;
    text_marker.scale.y = 0.2;
    text_marker.scale.z = 0.2;

    text_marker.color.r = color_set_[track.id % color_set_.size()](2);
    text_marker.color.g = color_set_[track.id % color_set_.size()](1);
    text_marker.color.b = color_set_[track.id % color_set_.size()](0);
    text_marker.color.a = 1.0;

    text_marker.lifetime = ros::Duration(0.2);

    msg.markers.push_back(text_marker);
  }

  void toPointXYZRGB(pcl::PointXYZRGB & p,
                     const LastData & data)
  {
    const opt_msgs::Track & track = data.last_msg.tracks[0];

//    if(track.visibility == Track::NOT_VISIBLE)
//      return;

    p.x = float(track.x);
    p.y = float(track.y);
    p.z = float(track.height / 2);
    uchar * rgb_ptr = reinterpret_cast<uchar *>(&p.rgb);
    *rgb_ptr++ = uchar(color_set_[track.id % color_set_.size()](0) * 255.0f);
    *rgb_ptr++ = uchar(color_set_[track.id % color_set_.size()](1) * 255.0f);
    *rgb_ptr++ = uchar(color_set_[track.id % color_set_.size()](2) * 255.0f);
  }

  ros::NodeHandle node_handle_;

  boost::recursive_mutex config_mutex_;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  ros::Subscriber tracking_sub_;
  ros::Publisher tracking_pub_;
  ros::Publisher marker_array_pub_;
  ros::Publisher history_pub_;

  ros::Duration heartbeat_time_duration_;

  bool publish_empty_;

  std::map<int, LastData> last_data_map_;
  ros::Duration time_alive_;
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > color_set_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr history_cloud_;
  int max_history_size_;
  size_t history_cloud_index_;

  ros::Rate rate_;
  double alpha_, beta_, gamma_;
  ros::Duration prediction_duration_;

  std::map<int, boost::shared_ptr<TrackState> > state_map_;

};

} // namespace tracking
} // namespace open_ptrack

using open_ptrack::tracking::TrackSmoother;

int main(int argc, char **argv)
{
  // Initialization:
  ros::init(argc, argv, "tracking_filter");
  ros::NodeHandle nh("~");

  TrackSmoother smoother(nh);
  smoother.spin();

  return 0;
}
