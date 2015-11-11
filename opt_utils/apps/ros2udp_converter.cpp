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
 * Author: Matteo Munaro [matteo.munaro@dei.unipd.it]
 *
 */

#include <ros/ros.h>
#include <opt_msgs/TrackArray.h>
#include <opt_msgs/IDArray.h>
#include <open_ptrack/opt_utils/udp_messaging.h>
#include <open_ptrack/opt_utils/json.h>

// Global variables:
int udp_buffer_length;  // UDP message buffer length
int udp_port;           // UDP port
std::string hostip;     // UDP host
int json_indent_size;   // indent size for JSON message
bool json_newline;      // use newlines (true) or not (false) in JSON messages
bool json_spacing;      // use spacing (true) or not (false) in JSON messages
bool json_use_tabs;     // use tabs (true) or not (false) in JSON messages
struct ComData udp_data;  // parameters for UDP messaging
open_ptrack::opt_utils::UDPMessaging udp_messaging(udp_data);   // instance of class UDPMessaging
ros::Time last_heartbeat_time;
double heartbeat_interval;

void
trackingCallback(const opt_msgs::TrackArray::ConstPtr& tracking_msg)
{
  /// Create JSON-formatted message:
  Jzon::Object root, header, stamp;

  /// Add header (84 characters):
  header.Add("seq", int(tracking_msg->header.seq));
  stamp.Add("sec", int(tracking_msg->header.stamp.sec));
  stamp.Add("nsec", int(tracking_msg->header.stamp.nsec));
  header.Add("stamp", stamp);
  header.Add("frame_id", tracking_msg->header.frame_id);
  root.Add("header", header);

  /// Add tracks array:
  // >50 characters for every track
  Jzon::Array tracks;
  for (unsigned int i = 0; i < tracking_msg->tracks.size(); i++)
  {
    Jzon::Object current_track;
    current_track.Add("id", tracking_msg->tracks[i].id);
    current_track.Add("x", tracking_msg->tracks[i].x);
    current_track.Add("y", tracking_msg->tracks[i].y);
    current_track.Add("height", tracking_msg->tracks[i].height);
    current_track.Add("age", tracking_msg->tracks[i].age);
    current_track.Add("confidence", tracking_msg->tracks[i].confidence);

    tracks.Add(current_track);
  }
  root.Add("tracks", tracks);

  /// Convert JSON object to string:
  Jzon::Format message_format = Jzon::StandardFormat;
  message_format.indentSize = json_indent_size;
  message_format.newline = json_newline;
  message_format.spacing = json_spacing;
  message_format.useTabs = json_use_tabs;
  Jzon::Writer writer(root, message_format);
  writer.Write();
  std::string json_string = writer.GetResult();
//  std::cout << "String sent: " << json_string << std::endl;

  /// Copy string to message buffer:
  udp_data.si_num_byte_ = json_string.length()+1;
  char buf[udp_data.si_num_byte_];
  for (unsigned int i = 0; i < udp_data.si_num_byte_; i++)
  {
    buf[i] = 0;
  }
  sprintf(buf, "%s", json_string.c_str());
  udp_data.pc_pck_ = buf;         // buffer where the message is written

  /// Send message:
  udp_messaging.sendFromSocketUDP(&udp_data);
}

void
aliveIDsCallback(const opt_msgs::IDArray::ConstPtr& alive_ids_msg)
{
  ros::Time msg_time = ros::Time(alive_ids_msg->header.stamp.sec, alive_ids_msg->header.stamp.nsec);
  if ((msg_time - last_heartbeat_time).toSec() > heartbeat_interval)
  {
    /// Create JSON-formatted message:
    Jzon::Object root, header, stamp;

    /// Add header:
    header.Add("seq", int(alive_ids_msg->header.seq));
    stamp.Add("sec", int(alive_ids_msg->header.stamp.sec));
    stamp.Add("nsec", int(alive_ids_msg->header.stamp.nsec));
    header.Add("stamp", stamp);
    header.Add("frame_id", "heartbeat");
    root.Add("header", header);

    Jzon::Array alive_IDs;
    for (unsigned int i = 0; i < alive_ids_msg->ids.size(); i++)
    {
      alive_IDs.Add(alive_ids_msg->ids[i]);
    }
    root.Add("alive_IDs", alive_IDs);
    root.Add("max_ID", alive_ids_msg->max_ID);

    /// Convert JSON object to string:
    Jzon::Format message_format = Jzon::StandardFormat;
    message_format.indentSize = json_indent_size;
    message_format.newline = json_newline;
    message_format.spacing = json_spacing;
    message_format.useTabs = json_use_tabs;
    Jzon::Writer writer(root, message_format);
    writer.Write();
    std::string json_string = writer.GetResult();
    //  std::cout << "String sent: " << json_string << std::endl;

    /// Copy string to message buffer:
    udp_data.si_num_byte_ = json_string.length()+1;
    char buf[udp_data.si_num_byte_];
    for (unsigned int i = 0; i < udp_data.si_num_byte_; i++)
    {
      buf[i] = 0;
    }
    sprintf(buf, "%s", json_string.c_str());
    udp_data.pc_pck_ = buf;         // buffer where the message is written

    /// Send message:
    udp_messaging.sendFromSocketUDP(&udp_data);

    last_heartbeat_time = msg_time;
  }
}

typedef unsigned long uint32;
// convert a string represenation of an IP address into its numeric equivalent
static uint32 Inet_AtoN(const char * buf)
{

   uint32 ret = 0;
   int shift = 24;  // fill out the MSB first
   bool startQuad = true;
   while((shift >= 0)&&(*buf))
   {
      if (startQuad)
      {
         unsigned char quad = (unsigned char) atoi(buf);
         ret |= (((uint32)quad) << shift);
         shift -= 8;
      }
      startQuad = (*buf == '.');
      buf++;
   }
   return ret;
}

int
main(int argc, char **argv)
{
  // Initialization:
  ros::init(argc, argv, "ros2udp_converter");
  ros::NodeHandle nh("~");

  // Read input parameters:
  nh.param("udp/port", udp_port, 21234);
  nh.param("udp/hostip", hostip, std::string("127.0.0.1"));
  nh.param("udp/buffer_length", udp_buffer_length, 2048);
  nh.param("json/indent_size", json_indent_size, 0);
  nh.param("json/newline", json_newline, false);
  nh.param("json/spacing", json_spacing, false);
  nh.param("json/use_tabs", json_use_tabs, false);
  nh.param("json/heartbeat_interval", heartbeat_interval, 0.25);

  // ROS subscriber:
  ros::Subscriber tracking_sub = nh.subscribe<opt_msgs::TrackArray>("input_topic", 1, trackingCallback);
  ros::Subscriber alive_ids_sub = nh.subscribe<opt_msgs::IDArray>("alive_ids_topic", 1, aliveIDsCallback);

  // Initialize UDP parameters:
  char buf[0];
  udp_data.si_port_ = udp_port;      // port
  udp_data.si_retry_ = 1;
  udp_data.si_num_byte_ = udp_buffer_length; // number of bytes to write (2048 -> about 30 tracks)
  udp_data.pc_pck_ = buf;         // buffer where the message is written
  udp_data.si_timeout_ = 4;
  udp_data.sj_addr_ = Inet_AtoN(hostip.c_str());

  /// Create object for UDP messaging:
  udp_messaging = open_ptrack::opt_utils::UDPMessaging(udp_data);

  /// Create client socket:
  udp_messaging.createSocketClientUDP(&udp_data);

  // Execute callbacks:
  ros::spin();

  // Close socket:
  udp_messaging.closeSocketUDP(&udp_data);

  return 0;
}
