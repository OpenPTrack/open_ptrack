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
from opt_msgs.srv import *

class CalibrationInitializer :

  def __init__(self) :
    
    network = rospy.get_param('~network')
    self.checkerboard = rospy.get_param('~checkerboard')
    self.file_name = rospy.get_param('~network_calibration_launch_file')
    self.frame_file_name = rospy.get_param('~frame_calibration_launch_file')
    
    self.sensor_list = []
    self.sensor_map = {}
    for item in network:
      pc = item['pc']
      sensors = item['sensors']
      self.sensor_map[pc] = sensors
      for sensor_item in sensors:
        self.sensor_list = self.sensor_list + [sensor_item]
        
    self.session_id = rospy.Time.now().secs
        
  
  def createMaster(self) :
    
    file = open(self.file_name, 'w')
    file.write('<?xml version="1.0"?>\n')
    file.write('<!-- SESSION ID: ' + str(self.session_id) + ' -->\n')
    file.write('<launch>\n\n')
  
    # Calibration parameters
    file.write('  <!-- Calibration parameters -->\n')
    file.write('  <arg name="lock_world_frame" default="false" />\n')
    file.write('  <group if="$(arg lock_world_frame)">\n')
    file.write('    <arg name="fixed_sensor_id" />\n')
    file.write('    <rosparam command="load" file="$(find opt_calibration)/conf/camera_poses.yaml" />\n')
    file.write('  </group>\n\n')
  
    # Network parameters
    file.write('  <!-- Network parameters -->\n')
    file.write('  <arg name="num_sensors"   default="' + str(len(self.sensor_list)) + '" />\n\n')
    
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
    file.write('  <arg name="rows"          default="' + str(self.checkerboard['rows']) + '" />\n')
    file.write('  <arg name="cols"          default="' + str(self.checkerboard['cols']) + '" />\n')
    file.write('  <arg name="cell_width"    default="' + str(self.checkerboard['cell_width']) + '" />\n')
    file.write('  <arg name="cell_height"   default="' + str(self.checkerboard['cell_height']) + '" />\n\n')
  
    # Rviz and calibration node
    file.write('  <!-- Opening Rviz for visualization -->\n')
    file.write('  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find opt_calibration)/conf/opt_calibration.rviz" />\n\n')
    
    file.write('  <!-- Plot calibration status -->\n')
    file.write('  <node name="opt_calibration_status_plot" pkg="opt_calibration" type="status_plot.py" output="screen">\n')
    file.write('    <remap from="~calibration_status" to="/opt_calibration/status" />\n')
    file.write('  </node>\n\n')
    
    file.write('  <!-- Launching calibration -->\n')
    file.write('  <node pkg="opt_calibration" type="opt_calibration" name="opt_calibration" output="screen">\n\n')
    
    file.write('    <param unless="$(arg lock_world_frame)" name="world_computation" value="last_checkerboard" />\n')
    file.write('    <param     if="$(arg lock_world_frame)" name="world_computation" value="update" />\n')
    file.write('    <param     if="$(arg lock_world_frame)" name="fixed_sensor/name" value="/$(arg fixed_sensor_id)" />\n\n')
    
    file.write('    <param name="num_sensors"           value="$(arg num_sensors)" />\n\n')
    file.write('    <param name="rows"                  value="$(arg rows)" />\n')
    file.write('    <param name="cols"                  value="$(arg cols)" />\n')
    file.write('    <param name="cell_width"            value="$(arg cell_width)" />\n')
    file.write('    <param name="cell_height"           value="$(arg cell_height)" />\n\n')
  
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
        file.write('    <remap from="~sensor_' + str(index) + '/image"       to="/$(arg sensor_' + str(index) + '_name)/rgb/image_color" />\n')
        file.write('    <remap from="~sensor_' + str(index) + '/camera_info" to="/$(arg sensor_' + str(index) + '_name)/rgb/camera_info" />\n\n')
      elif sensor['type'] == 'kinect2':
        file.write('    <param name="sensor_' + str(index) + '/type"         value="pinhole_rgb" />\n')
        file.write('    <remap from="~sensor_' + str(index) + '/image"       to="/$(arg sensor_' + str(index) + '_name)/rgb_rect/image" />\n')
        file.write('    <remap from="~sensor_' + str(index) + '/camera_info" to="/$(arg sensor_' + str(index) + '_name)/rgb_rect/camera_info" />\n\n')
      elif sensor['type'] == 'stereo_pg':
        file.write('    <param name="sensor_' + str(index) + '/type"         value="pinhole_rgb" />\n')
        file.write('    <remap from="~sensor_' + str(index) + '/image"       to="/$(arg sensor_' + str(index) + '_name)/left/image_color" />\n')
        file.write('    <remap from="~sensor_' + str(index) + '/camera_info" to="/$(arg sensor_' + str(index) + '_name)/left/camera_info" />\n\n')
      else:
        rospy.logfatal('Sensor type "' + sensor['type'] + '" not supported yet!');
      index = index + 1
  
    file.write('  </node>\n\n')
    file.write('</launch>\n')
    
    file.close()

    # Create launch file for defining user reference frame:
    file = open(self.frame_file_name, 'w')
    file.write('<?xml version="1.0"?>\n')
    file.write('<!-- SESSION ID: ' + str(self.session_id) + ' -->\n')
    file.write('<launch>\n\n')
  
    # Calibration parameters
    file.write('  <!-- Calibration parameters -->\n')
    file.write('  <rosparam command="load" file="$(find opt_calibration)/conf/camera_poses.yaml" />\n\n')
    
    # Network parameters
    file.write('  <!-- Network parameters -->\n')
    file.write('  <arg name="num_sensors"   default="' + str(len(self.sensor_list)) + '" />\n\n')
    
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
      
    # Calibration node
    file.write('  <!-- Launching calibration -->\n')
    file.write('  <node pkg="opt_calibration" type="opt_define_reference_frame" name="opt_define_reference_frame" output="screen">\n')
    
    file.write('    <rosparam command="load" file="$(find opt_calibration)/conf/camera_network.yaml" />\n\n')
    
    file.write('    <param name="num_sensors"           value="$(arg num_sensors)" />\n\n')
    
    index = 0
    for sensor in self.sensor_list:
      file.write('    <param name="sensor_' + str(index) + '/name"         value="/$(arg sensor_' + str(index) + '_name)" />\n')
      if sensor['type'] == 'sr4500':
        file.write('    <param name="sensor_' + str(index) + '/type"         value="pinhole_rgb" />\n')
        file.write('    <remap from="~sensor_' + str(index) + '/image"       to="/$(arg sensor_' + str(index) + '_name)/intensity/image_resized" />\n')
        file.write('    <remap from="~sensor_' + str(index) + '/camera_info" to="/$(arg sensor_' + str(index) + '_name)/intensity/camera_info" />\n\n')
      elif sensor['type'] == 'kinect1':
        file.write('    <param name="sensor_' + str(index) + '/type"         value="pinhole_rgb" />\n')
        file.write('    <remap from="~sensor_' + str(index) + '/image"       to="/$(arg sensor_' + str(index) + '_name)/rgb/image_color" />\n')
        file.write('    <remap from="~sensor_' + str(index) + '/camera_info" to="/$(arg sensor_' + str(index) + '_name)/rgb/camera_info" />\n\n')
      elif sensor['type'] == 'kinect2':
        file.write('    <param name="sensor_' + str(index) + '/type"         value="pinhole_rgb" />\n')
        file.write('    <remap from="~sensor_' + str(index) + '/image"       to="/$(arg sensor_' + str(index) + '_name)/rgb_rect/image" />\n')
        file.write('    <remap from="~sensor_' + str(index) + '/camera_info" to="/$(arg sensor_' + str(index) + '_name)/rgb_rect/camera_info" />\n\n')
      elif sensor['type'] == 'stereo_pg':
        file.write('    <param name="sensor_' + str(index) + '/type"         value="pinhole_rgb" />\n')
        file.write('    <remap from="~sensor_' + str(index) + '/image"       to="/$(arg sensor_' + str(index) + '_name)/left/image_color" />\n')
        file.write('    <remap from="~sensor_' + str(index) + '/camera_info" to="/$(arg sensor_' + str(index) + '_name)/left/camera_info" />\n\n')
      else:
        rospy.logfatal('Sensor type "' + sensor['type'] + '" not supported yet!');
      index = index + 1
  
    file.write('  </node>\n\n')
    file.write('</launch>\n')
    
    file.close()
    
  def fileName(self) :
    return self.file_name

  def frameFileName(self) :
    return self.frame_file_name
    
  def __invokeService(self, local_service_name) :
    
    # For each pc
    for pc in self.sensor_map:
      service_name = pc + '/' + local_service_name
      rospy.loginfo('Waiting for ' + service_name + ' service...')
      rospy.wait_for_service(service_name)
      
      # For each sensor
      for sensor_item in self.sensor_map[pc]:
        
        # Create an OPTSensor message
        sensor_msg = OPTSensorRequest()
        sensor_msg.session_id = self.session_id
        sensor_msg.id = sensor_item['id']
        if sensor_item['type'] == 'kinect1':
          sensor_msg.type = OPTSensorRequest.TYPE_KINECT1
          if 'serial' in sensor_item:
            sensor_msg.serial = sensor_item['serial']   
        elif sensor_item['type'] == 'kinect2':
          sensor_msg.type = OPTSensorRequest.TYPE_KINECT2
          if 'serial' in sensor_item:
            sensor_msg.serial = sensor_item['serial']  
        elif sensor_item['type'] == 'sr4500':
          sensor_msg.type = OPTSensorRequest.TYPE_SR4500
          sensor_msg.ip = sensor_item['ip']
        elif sensor_item['type'] == 'stereo_pg':
          sensor_msg.type = OPTSensorRequest.TYPE_STEREO_PG
          sensor_msg.serial_left = sensor_item['serial_left']
          sensor_msg.serial_right = sensor_item['serial_right']
      
        # Invoke service
        try:
          add_sensor = rospy.ServiceProxy(service_name, OPTSensor)
          response = add_sensor(sensor_msg)
          if response.status == OPTSensorResponse.STATUS_OK:
            rospy.loginfo('[' + pc + '] ' + response.message);
          else:
            rospy.logerr('[' + pc + '] ' + response.message);
        except rospy.ServiceException, e:
          rospy.logerr("Service call failed: %s"%e)
  
  def createSensorLaunch(self) :
    self.__invokeService('create_sensor_launch')
          
  def createDetectorLaunch(self) :
    self.__invokeService('create_detector_launch')
    

if __name__ == '__main__' :
  
  rospy.init_node('calibration_initializer')
    
  try:
    
    initializer = CalibrationInitializer()
    
    initializer.createMaster()
    rospy.loginfo(initializer.fileName() + ' created!');
    rospy.loginfo(initializer.frameFileName() + ' created!');
    
    initializer.createSensorLaunch()
    initializer.createDetectorLaunch()
    
    rospy.loginfo('Initialization completed. Press [ctrl+c] to exit.')
    
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
