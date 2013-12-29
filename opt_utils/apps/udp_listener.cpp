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
#include <open_ptrack/opt_utils/udp_messaging.h>

int main(int argc, char **argv)
{
  // Initialization:
  ros::init(argc, argv, "ros2udp_converter");
  ros::NodeHandle nh("~");

  // Read input parameters:
  int udp_buffer_length;
  int udp_port;
  nh.param("udp/port", udp_port, 21234);
  nh.param("udp/buffer_length", udp_buffer_length, 2048);

//  // Initialize UDP data structure:
  char buf[udp_buffer_length];

  struct ComData udp_data;
  udp_data.si_port_ = udp_port;      // port to listen to
  udp_data.si_retry_ = 4;
  udp_data.si_num_byte_ = udp_buffer_length; // how many bites to read
  udp_data.pc_pck_ = buf;         // buffer where the message is written
  udp_data.si_timeout_ = 4;
  udp_data.sj_addr_ = INADDR_ANY;

  // Create object for UDP messaging:
  open_ptrack::opt_utils::UDPMessaging udp_messaging(udp_data);
  char *buff;

  // Create UDP server:
  udp_messaging.createSocketServerUDP(&udp_data);
  while (true)
  {
    // Listen to UDP messages and write them to console:
    udp_messaging.receiveFromSocketUDP(&udp_data);

    buff = udp_data.pc_pck_;
    if (udp_data.si_num_byte_ > 0)
    {
      buf[udp_data.si_num_byte_] = 0;
      printf("\"%s\"\n", buff);
    }

    ros::spinOnce();
  }

  // Close UDP server:
  udp_messaging.closeSocketUDP(&udp_data);

  return 0;
}
