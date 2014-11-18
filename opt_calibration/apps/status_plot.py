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

import roslib; roslib.load_manifest('opt_calibration')
import rospy
from opt_msgs.msg import CalibrationStatus
import matplotlib.pyplot as plot
import numpy
import thread

class CalibrationStatusPlotter :

  def __init__(self) :
    rospy.Subscriber("~calibration_status", CalibrationStatus, self.callback)
    self.initialized = False
    self.has_mgs = False
    self.lock = thread.allocate_lock()
  #####################
    
  def callback(self, msg) :
    self.lock.acquire()
    self.msg = msg
    self.has_mgs = True
    self.lock.release()
  #####################
    
  def spinOnce(self) :
    self.lock.acquire()
    if not self.initialized :
      y = numpy.arange(len(self.msg.sensor_ids))

      width = self.msg.images_acquired
      width_max = max(width)
      threshold_yellow = width_max / 2.
      threshold_red = width_max / 4.
      colors = map(lambda x: '#ff0000' if x < threshold_red else '#ffff00' if x < threshold_yellow else '#00ff00', width)
      
      plot.ion()
      (self.fig, self.axis) = plot.subplots()
      plot.subplots_adjust(left = 0.25)
      self.fig.canvas.set_window_title('Calibration Status') 
      
      self.axis.set_title('Images acquired')
      self.axis.set_yticks(y + 0.4)
      self.axis.set_yticklabels(self.msg.sensor_ids, ha = "right")
      self.axis.set_xlim(xmax = max(width_max + 1, 10))
      self.axis.set_ylim([-0.2, max(y) + 1])
      
      self.bar_plot = self.axis.barh(y, width, color = colors, linewidth = 0)
      
      self.initialized = True
    else :
      width = self.msg.images_acquired
      width_max = max(width)
      threshold_yellow = width_max / 2.
      threshold_red = width_max / 4.
      colors = map(lambda x: '#ff0000' if x < threshold_red else '#ffff00' if x < threshold_yellow else '#00ff00', width)
      
      self.axis.set_xlim(xmax = max(width_max + 1, 10))
      for r in zip(self.bar_plot, width, colors):
        r[0].set_width(r[1])
        r[0].set_color(r[2])
        
    self.has_mgs = False;
    self.lock.release()
    
    plot.draw()
  #####################

  def spin(self) :
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() :
      if self.has_mgs :
        self.spinOnce()
      rate.sleep()
  #####################
           
if __name__ == '__main__' :
  rospy.init_node('opt_calibration_status_plot')
  plotter = CalibrationStatusPlotter()
  plotter.spin()
