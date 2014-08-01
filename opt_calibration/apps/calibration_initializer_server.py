#!/usr/bin/env python

###
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
###

import roslib; roslib.load_manifest('opt_calibration')
import rospy
import rospkg
from std_msgs.msg import String

def parser() :
  rospy.init_node('calibration_initializer_server')
  
  num_sensors = 0
  sensor_list = []
  
  network = rospy.get_param('~network')
  for item in network:
    pc = item['pc']
    use_serials = item['use_serials']
    sensors = item['sensors']
    for sensor_item in sensors:
      sensor_type = sensor_item['type']
      sensor_id = sensor_item['id']
      num_sensors = num_sensors + 1
      sensor_list = sensor_list + [sensor_item]
      
  checkerboard = rospy.get_param('~checkerboard')
      
  rospack = rospkg.RosPack()
  launch_file = open(rospack.get_path('opt_calibration') + '/launch/opt_calibration_master.launch', 'w')
  
  launch_file.write('<launch>\n\n')

  # Network parameters
  launch_file.write('  <!-- Network parameters -->\n')
  launch_file.write('  <arg name="num_sensors" default="' + str(num_sensors) + '" />\n\n')
  
  index = 0
  for sensor in sensor_list:
    if sensor['type'] == 'sr4500':
      sensor_name = "SR_" + sensor['id'].replace('.', '_');
      launch_file.write('  <arg name="sensor_' + str(index) + '_id"   default="' + sensor_name  + '" />\n')
    else:
      launch_file.write('  <arg name="sensor_' + str(index) + '_id"   default="' + sensor['id']  + '" />\n')
    
    launch_file.write('  <arg name="sensor_' + str(index) + '_name" default="$(arg sensor_' + str(index) + '_id)" />\n\n')
    index = index + 1
    
  # Checkerboard parameters
  launch_file.write('  <arg name="rows"        default="' + str(checkerboard['rows']) + '" />\n')
  launch_file.write('  <arg name="cols"        default="' + str(checkerboard['cols']) + '" />\n')
  launch_file.write('  <arg name="cell_width"  default="' + str(checkerboard['cell_width']) + '" />\n')
  launch_file.write('  <arg name="cell_height" default="' + str(checkerboard['cell_height']) + '" />\n\n')

  # Rviz and calibration node
  launch_file.write('  <!-- Opening Rviz for visualization-->\n')
  launch_file.write('  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find opt_calibration)/conf/opt_calibration.rviz" />\n\n')
  launch_file.write('  <!-- Launching calibration -->\n')
  launch_file.write('  <node pkg="opt_calibration" type="opt_calibration" name="opt_calibration" output="screen">\n\n')
  launch_file.write('    <param name="num_sensors" value="$(arg num_sensors)\" />\n\n')
  launch_file.write('    <param name="rows"        value="$(arg rows)" />\n')
  launch_file.write('    <param name="cols"        value="$(arg cols)" />\n')
  launch_file.write('    <param name="cell_width"  value="$(arg cell_width)" />\n')
  launch_file.write('    <param name="cell_height" value="$(arg cell_height)" />\n\n')

  #if (calibration_with_serials)??
  #  launch_file.write('    <param name="calibration_with_serials" value="true" />\n\n')
  
  index = 0
  for sensor in sensor_list:
    launch_file.write('    <param name="sensor_' + str(index) + '/name"         value="/$(arg sensor_' + str(index) + '_name)" />\n')
    if sensor['type'] == 'sr4500':
      launch_file.write('    <param name="sensor_' + str(index) + '/type"         value="pinhole_rgb" />\n')
      launch_file.write('    <remap from="~sensor_' + str(index) + '/image"       to="/$(arg sensor_' + str(index) + '_name)/intensity/image_resized" />\n')
      launch_file.write('    <remap from="~sensor_' + str(index) + '/camera_info" to="/$(arg sensor_' + str(index) + '_name)/intensity/camera_info" />\n\n')
    else:
      launch_file.write('    <param name="sensor_' + str(index) + '/type"         value="pinhole_rgb" />\n')
      launch_file.write('    <remap from="~sensor_' + str(index) + '/image"       to="/$(arg sensor_' + str(index) + '_name)/rgb/image_rect_color" />\n')
      launch_file.write('    <remap from="~sensor_' + str(index) + '/camera_info" to="/$(arg sensor_' + str(index) + '_name)/rgb/camera_info" />\n\n')

    index = index + 1

  launch_file.write('  </node>\n\n')
  launch_file.write('</launch>\n')
  
  launch_file.close()
  
  """pub = rospy.Publisher('chatter', String, queue_size=10)
  r = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    str = "hello world %s"%rospy.get_time()
    rospy.loginfo(str)
    pub.publish(str)
    r.sleep()"""

if __name__ == '__main__':
  try:
    parser()
  except rospy.ROSInterruptException: pass
