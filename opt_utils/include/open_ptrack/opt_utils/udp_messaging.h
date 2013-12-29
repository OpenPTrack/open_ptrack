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

#ifndef OPEN_PTRACK_OPT_UTILS_UDP_MESSAGING_H_
#define OPEN_PTRACK_OPT_UTILS_UDP_MESSAGING_H_

#include <sys/socket.h>
#include <netinet/in.h> /* needed for sockaddr_in */
#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <unistd.h>
#include <cerrno>
#include <cstdio>
#include <iostream>

#define RET_OK     0
#define WRONG_TIMEOUT           -3
#define WRONG_BUFF_LENGHT       -4

struct ComData
{
    unsigned int sj_addr_;
    int  si_port_;
    int  si_retry_;
    int  si_socket_;
    char *pc_pck_;
    int  si_num_byte_;
    int  si_timeout_;
};

namespace open_ptrack
{
  namespace opt_utils
  {
    /** \brief UDPMessaging provides methods for message passing via UDP */
    class UDPMessaging
    {
      protected:

        /*
         * \brief ComData structure for communication in bootstrap and startup phase.
         */
        struct ComData udp_data_;

        struct sockaddr_in connect_UDP_;

      public:

        /** \brief Constructor. */
        UDPMessaging(ComData udp_data);

        /*
         * \brief Open a server UDP connection.
         *
         * param[in] udp_data Struct with UDP parameters.
         */
        int
        createSocketServerUDP(ComData* udp_data);

        /*
         * \brief Close a UDP connection.
         *
         * param[in] udp_data Struct with UDP parameters.
         */
        int
        closeSocketUDP(ComData* udp_data);

        /*
         * \brief Send a UDP message.
         *
         * param[in] udp_data Struct with UDP parameters.
         */
        int
        sendFromSocketUDP(ComData* udp_data);

        /*
         * \brief Open a client UDP connection.
         *
         * param[in] udp_data Struct with UDP parameters.
         */
        int
        createSocketClientUDP(ComData* udp_data);

        /*
         * \brief Receive a UDP message.
         *
         * param[in] udp_data Struct with UDP parameters.
         */
        int
        receiveFromSocketUDP(ComData* udp_data);
    };
  } /* namespace opt_utils */
} /* namespace open_ptrack */
#endif /* OPEN_PTRACK_OPT_UTILS_UDP_MESSAGING_H_ */
