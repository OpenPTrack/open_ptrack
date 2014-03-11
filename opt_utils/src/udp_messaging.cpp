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

#include <open_ptrack/opt_utils/udp_messaging.h>

namespace open_ptrack
{
  namespace opt_utils
  {
    UDPMessaging::UDPMessaging(ComData udp_data)
    {
      udp_data_ = udp_data;
    }

    int
    UDPMessaging::createSocketServerUDP(ComData* udp_data)
    {
      int sock = 0;

      sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

      if (sock < 0) {
        printf("sock %d %d\n", sock, errno);
        return (-1);
      }

	int broadcast=1;

	setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof broadcast);

      memset((char *) (&(connect_UDP_)), 0, sizeof (struct sockaddr_in));
      connect_UDP_.sin_family = AF_INET;
      connect_UDP_.sin_addr.s_addr = htonl(INADDR_ANY);
//      connect_UDP_.sin_addr.s_addr = htonl(127000001);
      connect_UDP_.sin_port = htons(udp_data->si_port_);

      if (bind(sock, (struct sockaddr *) &connect_UDP_, sizeof (struct sockaddr_in)) < 0)
        return (-1);

      udp_data->si_socket_ = sock;

      return (1);
    }

    int
    UDPMessaging::closeSocketUDP(ComData* udp_data)
    {
      close(udp_data->si_socket_);
    }

    int
    UDPMessaging::sendFromSocketUDP(ComData* udp_data)
    {
      socklen_t si_sock_addr_size;

      si_sock_addr_size = sizeof (connect_UDP_);

      if (udp_data->si_num_byte_ <= 0)
        return WRONG_BUFF_LENGHT;

      return (sendto(udp_data->si_socket_, (char *) udp_data->pc_pck_, udp_data->si_num_byte_, 0,
          (struct sockaddr *) &connect_UDP_, si_sock_addr_size));
    }

    int
    UDPMessaging::createSocketClientUDP(ComData* udp_data)
    {
      int si_sts = RET_OK;

      int sock = 0;
      int i;

      for (i = udp_data->si_retry_; i >= 0; i--) {
        memset((char *) (&(connect_UDP_)), 0, sizeof (struct sockaddr_in));

        connect_UDP_.sin_family = AF_INET;
        connect_UDP_.sin_addr.s_addr = htonl(udp_data->sj_addr_);
        connect_UDP_.sin_port = htons(udp_data->si_port_);

        if ((sock = socket(AF_INET, SOCK_DGRAM,  IPPROTO_UDP)) < 0)
          si_sts = -1;
        else
          break;
      }

      if (i < 0)
        return (-1);
      else
        if (si_sts == RET_OK){
	int broadcast=1;
	setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof broadcast);
          udp_data->si_socket_ = sock;
	}
      return (si_sts);
    }

    int
    UDPMessaging::receiveFromSocketUDP(ComData* udp_data)
    {

      int ret_val;
      int rec_dim;
      char *buff;
      struct timeval t_timeout;
      fd_set fdvar;
      struct sockaddr_in sx_addr;
      socklen_t si_sock_addr_size;

      si_sock_addr_size = sizeof (sx_addr);

      if (udp_data->si_timeout_ <= 0)
        return WRONG_TIMEOUT;

      if (udp_data->si_num_byte_ <= 0)
        return WRONG_BUFF_LENGHT;

      rec_dim = 0;
      buff = udp_data->pc_pck_;

      ret_val = recvfrom(udp_data->si_socket_, buff, udp_data->si_num_byte_, 0,
          (struct sockaddr *) &sx_addr, &si_sock_addr_size);

      connect_UDP_.sin_addr.s_addr = sx_addr.sin_addr.s_addr;
      connect_UDP_.sin_port = sx_addr.sin_port;

      return (ret_val);
    }
  } /* namespace opt_utils */
} /* namespace open_ptrack */

