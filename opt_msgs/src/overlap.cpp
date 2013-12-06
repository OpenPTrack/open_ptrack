/*
Software License Agreement (BSD License)

Copyright (c) 2013, Southwest Research Institute
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
 * Neither the name of the Southwest Research Institute, nor the names
     of its contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <open_ptrack/opt_msgs/overlap.h>

using opt_msgs::RoiRect;
using opt_msgs::RoisConstPtr;
using opt_msgs::RoisPtr;

float calcOverlapMax(RoiRect r1, RoiRect r2)
{
  float max_x = (r1.x > r2.x) ? r1.x : r2.x;
  float max_y = (r1.y > r2.y) ? r1.y : r2.y;
  float min_x = (r1.x + r1.width < r2.x + r2.width) ? r1.x + r1.width : r2.x + r2.width;
  float min_y = (r1.y + r1.height < r2.y + r2.height) ? r1.y + r1.height : r2.y + r2.height;
  float A1 = r1.width * r1.height;
  float A2 = r2.width * r2.height;
  float minArea = (A1 < A2) ? A1 : A2;
  if ((min_x > max_x) && (min_y > max_y) && (minArea > 0.0))
  {
    return (min_x - max_x) * (min_y - max_y) / minArea;
  }
  else
    return 0.0;
}  // calcOverlapMax

void remove_overlap_Rois(Rois& rois_msg_input, double max_overlap, Rois& rois_msg_output)
{
  float o;
  int n = (int) rois_msg_input.rois.size();
  rois_msg_output.rois.clear();
  if(n>0)rois_msg_output.rois.push_back(rois_msg_input.rois[0]);
  for(int i = 1;i<n;i++){
    bool overlap = false;
    for(unsigned int j = 0;j<rois_msg_output.rois.size();j++){ // check against those on list
      if ((o=calcOverlapMax(rois_msg_input.rois[i],rois_msg_output.rois[j])) > max_overlap){
        //	ROS_ERROR("overlap %d %d = %lf",i,j,o);
        overlap = true;
        break;
      }
    }
    if(!overlap) {
      //      ROS_ERROR("NO OVERLAP %d",i);
      rois_msg_output.rois.push_back(rois_msg_input.rois[i]);
    }
  }
}

