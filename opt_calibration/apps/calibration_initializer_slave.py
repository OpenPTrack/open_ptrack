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
# Author: Matteo Munaro [matteo.munaro@dei.unipd.it]
#         Filippo Basso [filippo.basso@dei.unipd.it]
#
######################################################################

import roslib; roslib.load_manifest('opt_calibration')
import rospy
import rospkg
from opt_msgs.msg import OPTSensorSet, OPTSensor

class CalibrationInitializerSlave :

  def __init__(self) :
    
    #self.pc_name = rospy.get_param('~pc_name')
    self.launch_file_path = rospy.get_param('~launch_file_path')
    rospy.Subscriber('sensors', OPTSensorSet, self.callback)
    
  
  def callback(self, msg) :
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
if __name__ == '__main__' :
  
  rospy.init_node('calibration_initializer_slave')
  
  try:
    
    initializer = CalibrationInitializerSlave()
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
