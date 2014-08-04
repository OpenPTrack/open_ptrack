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

class CalibrationInitializer :

  def __init__(self) :
    
    network = rospy.get_param('~network')
    self.checkerboard = rospy.get_param('~checkerboard')
    self.file_name = rospy.get_param('~launch_file')
    
    self.sensor_list = []
    self.pub_map = {}
    self.sensor_map = {}
    for item in network:
      pc = item['pc']
      sensors = item['sensors']
      self.pub_map[pc] = rospy.Publisher(pc + '/sensors', OPTSensorSet, queue_size=10)
      self.sensor_map[pc] = sensors
      for sensor_item in sensors:
        self.sensor_list = self.sensor_list + [sensor_item]
        
    self.id = rospy.Time.now().to_nsec()
        
  
  def createMaster(self) :
    
    file = open(self.file_name, 'w')
    file.write('<?xml version="1.0"?>')
    file.write('<!-- SESSION ID: ' + str(self.id) + ' -->\n')
    file.write('<launch>\n\n')
  
    # Network parameters
    file.write('  <!-- Network parameters -->\n')
    file.write('  <arg name="num_sensors" default="' + str(len(self.sensor_list)) + '" />\n\n')
    
    index = 0
    for sensor in self.sensor_list:
#       if sensor['type'] == 'sr4500':
#         sensor_name = "SR_" + sensor['id'].replace('.', '_');
#         file.write('  <arg name="sensor_' + str(index) + '_id"   default="' + sensor_name  + '" />\n')
#       else:
#         file.write('  <arg name="sensor_' + str(index) + '_id"   default="' + sensor['id']  + '" />\n')
      file.write('  <arg name="sensor_' + str(index) + '_id"   default="' + sensor['id']  + '" />\n')
      file.write('  <arg name="sensor_' + str(index) + '_name" default="$(arg sensor_' + str(index) + '_id)" />\n\n')
      index = index + 1
      
    # Checkerboard parameters
    file.write('  <arg name="rows"        default="' + str(self.checkerboard['rows']) + '" />\n')
    file.write('  <arg name="cols"        default="' + str(self.checkerboard['cols']) + '" />\n')
    file.write('  <arg name="cell_width"  default="' + str(self.checkerboard['cell_width']) + '" />\n')
    file.write('  <arg name="cell_height" default="' + str(self.checkerboard['cell_height']) + '" />\n\n')
  
    # Rviz and calibration node
    file.write('  <!-- Opening Rviz for visualization-->\n')
    file.write('  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find opt_calibration)/conf/opt_calibration.rviz" />\n\n')
    
    file.write('  <!-- Launching calibration -->\n')
    file.write('  <node pkg="opt_calibration" type="opt_calibration" name="opt_calibration" output="screen">\n\n')
    file.write('    <param name="num_sensors" value="$(arg num_sensors)\" />\n\n')
    file.write('    <param name="rows"        value="$(arg rows)" />\n')
    file.write('    <param name="cols"        value="$(arg cols)" />\n')
    file.write('    <param name="cell_width"  value="$(arg cell_width)" />\n')
    file.write('    <param name="cell_height" value="$(arg cell_height)" />\n\n')
  
    #if (calibration_with_serials)??
    #  file.write('    <param name="calibration_with_serials" value="true" />\n\n')
    
    index = 0
    for sensor in self.sensor_list:
      file.write('    <param name="sensor_' + str(index) + '/name"         value="/$(arg sensor_' + str(index) + '_name)" />\n')
      if sensor['type'] == 'sr4500':
        file.write('    <param name="sensor_' + str(index) + '/type"         value="pinhole_rgb" />\n')
        file.write('    <remap from="~sensor_' + str(index) + '/image"       to="/$(arg sensor_' + str(index) + '_name)/intensity/image_resized" />\n')
        file.write('    <remap from="~sensor_' + str(index) + '/camera_info" to="/$(arg sensor_' + str(index) + '_name)/intensity/camera_info" />\n\n')
      elif sensor['type'] == 'kinect1':
        file.write('    <param name="sensor_' + str(index) + '/type"         value="pinhole_rgb" />\n')
        file.write('    <remap from="~sensor_' + str(index) + '/image"       to="/$(arg sensor_' + str(index) + '_name)/rgb/image_rect_color" />\n')
        file.write('    <remap from="~sensor_' + str(index) + '/camera_info" to="/$(arg sensor_' + str(index) + '_name)/rgb/camera_info" />\n\n')
      else:
        rospy.logfatal('Sensor type "' + sensor['type'] + '" not supported yet!');
      index = index + 1
  
    file.write('  </node>\n\n')
    file.write('</launch>\n')
    
    file.close()
    
  def fileName(self) :
    return self.file_name
    
    
  def publishSensors(self) :
    
    # Create an OPTSensorSet message for each pc
    for pc in self.pub_map:
      sensor_set_msg = OPTSensorSet()
      sensor_set_msg.header.stamp = rospy.Time.now()
      
      # Create an OPTSensor message for each sensor
      for sensor_item in self.sensor_map[pc]:
        sensor_msg = OPTSensor()
        sensor_msg.id = sensor_item['id']
        if sensor_item['type'] == 'kinect1':
          sensor_msg.type = sensor_msg.TYPE_KINECT1
          if 'serial' in sensor_item:
            sensor_msg.serial = sensor_item['serial']
        elif sensor_item['type'] == 'sr4500':
          sensor_msg.type = sensor_msg.TYPE_SR4500
          sensor_msg.ip = sensor_item['ip']
        sensor_set_msg.sensors.append(sensor_msg)
      
      self.pub_map[pc].publish(sensor_set_msg)
    

if __name__ == '__main__' :
  
  rospy.init_node('calibration_initializer_master')
    
  try:
    
    initializer = CalibrationInitializer()
    
    initializer.createMaster()
    rospy.loginfo(initializer.fileName() + ' created!');
    
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
      initializer.publishSensors()
      rate.sleep()
    
  except rospy.ROSInterruptException:
    pass
