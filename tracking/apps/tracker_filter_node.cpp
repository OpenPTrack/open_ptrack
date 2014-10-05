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

using open_ptrack::tracking::Track;

struct LastData
{
  ros::Time last_visible_time;
  opt_msgs::TrackArray last_msg;
};

std::map<int, LastData> last_data_map;
ros::Duration time_alive;
std::vector<Eigen::Vector3f> color_set;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr history_cloud;
int max_history_size;
size_t history_cloud_index;

void
generateColors()
{
  for (size_t i = 0; i <= 4; ++i)
    for (size_t j = 0; j <= 4; ++j)
      for (size_t k = 0; k <= 4; ++k)
        color_set.push_back(Eigen::Vector3f(i * 0.25f, j * 0.25f, k * 0.25f));

  std::random_shuffle(color_set.begin(), color_set.end());

}

void
createMarker(visualization_msgs::MarkerArray & msg,
             const LastData & data)
{
  const opt_msgs::Track & track = data.last_msg.tracks[0];

  if(track.visibility == Track::NOT_VISIBLE)
    return;

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

  marker.color.r = color_set[track.id % color_set.size()](2);
  marker.color.g = color_set[track.id % color_set.size()](1);
  marker.color.b = color_set[track.id % color_set.size()](0);
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration(0.2);

  msg.markers.push_back(marker);

  ///////////////////////////////////

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

  text_marker.color.r = color_set[track.id % color_set.size()](2);
  text_marker.color.g = color_set[track.id % color_set.size()](1);
  text_marker.color.b = color_set[track.id % color_set.size()](0);
  text_marker.color.a = 1.0;

  text_marker.lifetime = ros::Duration(0.2);

  msg.markers.push_back(text_marker);

}

void
toPointXYZRGB(pcl::PointXYZRGB & p,
              const LastData & data)
{
  const opt_msgs::Track & track = data.last_msg.tracks[0];

  if(track.visibility == Track::NOT_VISIBLE)
    return;

  p.x = float(track.x);
  p.y = float(track.y);
  p.z = float(track.height / 2);
  uchar * rgb_ptr = reinterpret_cast<uchar *>(&p.rgb);
  *rgb_ptr++ = uchar(color_set[track.id % color_set.size()](0) * 255.0f);
  *rgb_ptr++ = uchar(color_set[track.id % color_set.size()](1) * 255.0f);
  *rgb_ptr++ = uchar(color_set[track.id % color_set.size()](2) * 255.0f);

}

void
trackingCallback(const opt_msgs::TrackArray::ConstPtr & tracking_msg)
{
  for (size_t i = 0; i < tracking_msg->tracks.size(); ++i)
  {
    opt_msgs::Track track = tracking_msg->tracks[i];
    opt_msgs::TrackArray msg;
    msg.header = tracking_msg->header;
    msg.tracks.push_back(track);
    std::map<int, LastData>::iterator it = last_data_map.find(track.id);

    if (it != last_data_map.end()) // Track exists
    {
      it->second.last_msg = msg;
      if (track.visibility != opt_msgs::Track::NOT_VISIBLE)
        it->second.last_visible_time = msg.header.stamp;
    }
    else if (track.visibility != opt_msgs::Track::NOT_VISIBLE)
    {
      LastData data;
      data.last_msg = msg;
      data.last_visible_time = msg.header.stamp;
      last_data_map[track.id] = data;
    }
  }

}

void
appendToHistory(const LastData & data)
{
  if (history_cloud->size() < max_history_size)
  {
    pcl::PointXYZRGB point;
    history_cloud->push_back(point);
  }

  toPointXYZRGB(history_cloud->points[history_cloud_index], data);
  history_cloud_index = (history_cloud_index + 1) % max_history_size;

}

int createMsg(opt_msgs::TrackArray & track_msg,
              visualization_msgs::MarkerArray & marker_msg)
{
  int added = 0;
  ros::Time now = ros::Time::now();

  track_msg.header.stamp = now;
  history_cloud->header.stamp = now.toNSec() / 1000;
  std::vector<int> to_remove;

  for (std::map<int, LastData>::iterator it = last_data_map.begin(); it != last_data_map.end(); ++it)
  {
    const opt_msgs::TrackArray & saved_msg = it->second.last_msg;
    const ros::Time & time = it->second.last_visible_time;
    if ((now - time) < time_alive)
    {
      track_msg.tracks.push_back(saved_msg.tracks[0]);
      track_msg.header.frame_id = saved_msg.header.frame_id;
      history_cloud->header.frame_id = saved_msg.header.frame_id;
      createMarker(marker_msg, it->second);
      appendToHistory(it->second);
      ++added;
    }
    else
    {
      //last_data_map.erase(it);
      to_remove.push_back(saved_msg.tracks[0].id);
    }

  }

  for (size_t i = 0; i < to_remove.size(); ++i)
    last_data_map.erase(to_remove[i]);

  return added;
}

int
main(int argc, char **argv)
{
  // Initialization:
  ros::init(argc, argv, "tracking_filter");
  ros::NodeHandle nh("~");

  double rate_d, track_lifetime_with_no_detections, heartbeat_time;
  bool publish_empty;
  nh.param("rate", rate_d, 30.0);
  nh.param("track_lifetime_with_no_detections", track_lifetime_with_no_detections, 1.0);
  nh.param("publish_empty", publish_empty, true);
  nh.param("heartbeat_time", heartbeat_time, 5.0);
  nh.param("max_history_size", max_history_size, 1000);

  generateColors();
  history_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >();
  history_cloud_index = 0;

  time_alive = ros::Duration(track_lifetime_with_no_detections);
  ros::Duration heartbeat_time_duration = ros::Duration(heartbeat_time);

  // ROS subscriber:
  ros::Subscriber tracking_sub = nh.subscribe<opt_msgs::TrackArray>("input", 100, trackingCallback);
  ros::Publisher tracking_pub = nh.advertise<opt_msgs::TrackArray>("output", 1);
  ros::Publisher marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("markers_array", 1);
  ros::Publisher history_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("history", 1);

  ros::Rate rate(rate_d);
  ros::Time last_heartbeat_time = ros::Time::now();

  while (ros::ok())
  {
    ros::spinOnce();

    opt_msgs::TrackArray track_msg;
    visualization_msgs::MarkerArray marker_msg;

    int n = createMsg(track_msg, marker_msg);
    ros::Time current_time = ros::Time::now();

    if (publish_empty or n > 0)
    {
      tracking_pub.publish(track_msg);
      marker_array_pub.publish(marker_msg);
      history_pub.publish(history_cloud);
      last_heartbeat_time = current_time;
    }
    else if (not publish_empty)
    {
      // Publish a heartbeat message every 'heartbeat_time' seconds
      if ((current_time - last_heartbeat_time) > heartbeat_time_duration)
      {
        opt_msgs::TrackArray heartbeat_msg;
        heartbeat_msg.header.stamp = current_time;
        heartbeat_msg.header.frame_id = "heartbeat";
        tracking_pub.publish(heartbeat_msg);
        marker_array_pub.publish(marker_msg);
        history_pub.publish(history_cloud);
        last_heartbeat_time = current_time;
      }
    }

    rate.sleep();
  }

  return 0;
}
