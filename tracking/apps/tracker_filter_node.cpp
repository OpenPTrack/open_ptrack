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

std::map<int, opt_msgs::TrackArray> last_msg_map;
ros::Duration time_alive;

void
trackingCallback(const opt_msgs::TrackArray::ConstPtr & tracking_msg)
{
  
  for (size_t i = 0; i < tracking_msg->tracks.size(); ++i)
  {
    opt_msgs::Track track = tracking_msg->tracks[i];
    opt_msgs::TrackArray msg;
    msg.header = tracking_msg->header;
    msg.tracks.push_back(track);
    last_msg_map[track.id] = msg;
  }

}

int createMsg(opt_msgs::TrackArray & msg)
{
  int added = 0;
  ros::Time now = ros::Time::now();

  msg.header.stamp = now;

  for (std::map<int, opt_msgs::TrackArray>::iterator it = last_msg_map.begin(); it != last_msg_map.end(); ++it)
  {
    opt_msgs::TrackArray & saved_msg = it->second;
    if ((now - saved_msg.header.stamp) < time_alive)
    {
      msg.tracks.push_back(saved_msg.tracks[0]);
      msg.header.frame_id = saved_msg.header.frame_id;
      ++added;
    }
    else
    {
      last_msg_map.erase(it);
    }

  }
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

  time_alive = ros::Duration(track_lifetime_with_no_detections);
  ros::Duration heartbeat_time_duration = ros::Duration(heartbeat_time);

  // ROS subscriber:
  ros::Subscriber tracking_sub = nh.subscribe<opt_msgs::TrackArray>("input", 1, trackingCallback);
  ros::Publisher tracking_pub = nh.advertise<opt_msgs::TrackArray>("output", 1);

  ros::Rate rate(rate_d);
  ros::Time last_heartbeat_time = ros::Time::now();
  
  while (ros::ok())
  {
    ros::spinOnce();
    
    opt_msgs::TrackArray msg;
    int n = createMsg(msg);
    if (publish_empty or n > 0)
      tracking_pub.publish(msg);
    
    if (not publish_empty)
    { // Publish a heartbeat message every 'heartbeat_time' seconds:
      ros::Time current_time = ros::Time::now();
      if ((current_time - last_heartbeat_time) > heartbeat_time_duration)
      {
        opt_msgs::TrackArray msg;
        msg.header.stamp = current_time;
        msg.header.frame_id = "heartbeat";
        tracking_pub.publish(msg);
        last_heartbeat_time = current_time;
      }
    }

    rate.sleep();
  }

  return 0;
}
