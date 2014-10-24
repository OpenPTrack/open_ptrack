#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2014-, Open Perception, Inc.
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
# * Neither the name of the copyright holder(s) nor the names of its
# contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Filippo Basso [filippo.basso@dei.unipd.it]
#
######################################################################

import roslib; roslib.load_manifest('swissranger_camera')
import rospy
import os
from sensor_msgs.srv import SetCameraInfo
from camera_info_manager import *

class SetCameraInfoSrv :

  def __init__(self) :
    self.camera_id = rospy.get_param('~camera_id', 'swissranger')
    self.camera_info_url = rospy.get_param('~camera_info_url', '')
    self.scale_factor = rospy.get_param('~scale_factor', 1)
    self.srv = rospy.Service('~set_camera_info', SetCameraInfo, self.handle_camera_info)
  
    
  def handle_camera_info(self, request) :
    #rospy.loginfo(request.camera_info)
    #saveCalibration(request.camera_info, self.camera_info_url, self.camera_id)
    try:
      file_name = self.camera_info_url[7:]

      K = [k / self.scale_factor for k in request.camera_info.K]
      D = [d / self.scale_factor for d in request.camera_info.D]
      P = [p / self.scale_factor for p in request.camera_info.P]
              
      if not os.path.exists(os.path.dirname(file_name)) :
        os.makedirs(os.path.dirname(file_name))
        
      file = open(file_name, 'w')
      
      file.write('image_width: ' + str(request.camera_info.width / self.scale_factor) + '\n')
      file.write('image_height: ' + str(request.camera_info.height / self.scale_factor) + '\n')
      file.write('camera_name: ' + self.camera_id + '\n')
      file.write('camera_matrix:\n')
      file.write('  rows: 3\n')
      file.write('  cols: 3\n')
      file.write('  data: [' + str(K[0]) + ', 0, ' + str(K[2]) + ', 0, ' + str(K[4]) + ', ' + str(K[5]) + ', 0, 0, 1]\n')
      file.write('distortion_model: plumb_bob\n')
      file.write('distortion_coefficients:\n')
      file.write('  rows: 1\n')
      file.write('  cols: 5\n')
      file.write('  data: [' + str(D[0]) + ', ' + str(D[1]) + ', ' + str(D[2]) + ', ' + str(D[3]) + ', ' + str(D[4]) + ']\n')
      file.write('rectification_matrix:\n')
      file.write('  rows: 3\n')
      file.write('  cols: 3\n')
      file.write('  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]\n')
      file.write('projection_matrix:\n')
      file.write('  rows: 3\n')
      file.write('  cols: 4\n')
      file.write('  data: [' + str(P[0]) + ', 0, ' + str(P[2]) + ', 0, 0, ' + str(P[5]) + ', ' + str(P[6]) + ', 0, 0, 0, 1, 0]\n')
    
      file.close()
    
      return (True, self.camera_info_url + ' created!')
      
    except CameraInfoError as ex :
      return (False, ex.strerror)
      
    
if __name__ == '__main__' :
  
  rospy.init_node('save_camera_info')
  
  try:
    
    saver = SetCameraInfoSrv()
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
