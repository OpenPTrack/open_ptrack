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
from opt_msgs.srv import *

class Listener :

  def __init__(self) :
    self.sensor_launchers_dir = rospy.get_param('~sensor_launchers_dir')
    if self.sensor_launchers_dir[len(self.sensor_launchers_dir) - 1] != '/':
      self.sensor_launchers_dir = self.sensor_launchers_dir + '/'
      
    self.detector_launchers_dir = rospy.get_param('~detector_launchers_dir')
    if self.detector_launchers_dir[len(self.detector_launchers_dir) - 1] != '/':
      self.detector_launchers_dir = self.detector_launchers_dir + '/'
      
    self.camera_poses_dir = rospy.get_param('~camera_poses_dir')
    if self.camera_poses_dir[len(self.camera_poses_dir) - 1] != '/':
      self.camera_poses_dir = self.camera_poses_dir + '/'
    
    self.create_sensor_launch_srv = rospy.Service('create_sensor_launch', OPTSensor, self.handle_create_sensor_launch)
    self.create_detector_launch_srv = rospy.Service('create_detector_launch', OPTSensor, self.handle_create_detector_launch)
    self.create_camera_poses_srv = rospy.Service('create_camera_poses', OPTTransform, self.handle_create_camera_poses)
    
  
  def handle_create_sensor_launch(self, request) :
    
    file_name = self.sensor_launchers_dir + 'sensor_' + request.id + '.launch'
    file = open(file_name, 'w')
    file.write('<?xml version="1.0"?>\n')
    file.write('<!-- SESSION ID: ' + str(request.session_id) + ' -->\n')
    file.write('<launch>\n\n')
    
    file.write('  <!-- Sensor parameters -->\n')
    
    if request.type == OPTSensorRequest.TYPE_SR4500:
      file.write('  <arg name="camera_id"   default="' + request.id + '" />\n')
      if request.ip == '':
        file.close();
        rospy.logerr('Missing "ip" field for the SR4500 sensor!')
        return (OPTSensorResponse.STATUS_ERROR, 'Missing "ip" field!')
      file.write('  <arg name="device_ip"   default="' + request.ip + '" />\n\n')
      
      file.write('  <!-- Launch sensor -->\n')
      file.write('  <include file="$(find swissranger_camera)/launch/sr_eth.launch">\n')
      file.write('    <arg name="camera_id" value="$(arg camera_id)" />\n')
      file.write('    <arg name="device_ip" value="$(arg device_ip)" />\n')
      file.write('  </include>\n\n')
      
      file.write('  <include file="$(find swissranger_camera)/launch/publisher_for_calibration.launch">\n')
      file.write('    <arg name="camera_id"       value="$(arg camera_id)" />\n')
      file.write('    <arg name="camera_info_url" value="file://$(find opt_calibration)/camera_info/$(arg camera_id).yaml" />\n')
      file.write('  </include>\n\n')
      
    elif request.type == OPTSensorRequest.TYPE_KINECT1:
      file.write('  <arg name="sensor_id"     default="' + request.id + '" />\n')
      if request.serial != '':
        file.write('  <arg name="sensor_serial" default="' + request.serial + '" />\n')
      file.write('\n')
      
      file.write('  <!-- Launch sensor -->\n')
      file.write('  <include file="$(find detection)/launch/$(env KINECT_DRIVER).launch">\n')
      if request.serial != '':
        file.write('    <arg name="device_id"           value="$(arg sensor_serial)" />\n')
        file.write('    <arg name="rgb_camera_info_url" value="file://$(find opt_calibration)/camera_info/rgb_$(arg sensor_serial).yaml" />\n')
      else:
        file.write('    <arg name="rgb_camera_info_url" value="file://$(find opt_calibration)/camera_info/rgb_$(arg sensor_id).yaml" />\n')
      file.write('    <arg name="camera"              value="$(arg sensor_id)" />\n')
      file.write('  </include>\n\n')
      
      file.write('  <!-- Publish a further transform -->\n')
      file.write('  <node pkg="tf" type="static_transform_publisher" name="$(arg sensor_id)_broadcaster" args="-0.045 0 0 1.57079 -1.57079 0 /$(arg sensor_id) /$(arg sensor_id)_link  100" />\n\n')
    
    elif request.type == OPTSensorRequest.TYPE_STEREO_PG:
      file.write('  <arg name="camera_serial_left"  default="' + request.serial_left + '" />\n')
      file.write('  <arg name="camera_serial_right" default="' + request.serial_right + '" />\n')
      file.write('  <arg name="stereo_name"         default="' + request.id + '" />\n')
      file.write('  <arg name="conf_file_left"      default="$(find opt_calibration)/conf/pg_default.yaml" />\n')
      file.write('  <arg name="conf_file_right"     default="$(find opt_calibration)/conf/pg_default.yaml" />\n\n')
      
      file.write('  <!-- Launch driver -->\n')
      file.write('  <include file="$(find opt_calibration)/launch/stereo_pg.launch">\n')
      file.write('    <arg name="stereo_name"         value="$(arg stereo_name)" />\n')
      file.write('    <arg name="camera_serial_left"  value="$(arg camera_serial_left)" />\n')
      file.write('    <arg name="camera_serial_right" value="$(arg camera_serial_right)" />\n')
      file.write('    <arg name="conf_file_left"      value="$(arg conf_file_left)" />\n')
      file.write('    <arg name="conf_file_right"     value="$(arg conf_file_right)" />\n')
      file.write('  </include>\n\n')
      file.write('  <!-- Launch stereo processing -->\n')
      file.write('  <include file="$(find opt_calibration)/launch/stereo_processing.launch">\n')
      file.write('    <arg name="stereo_namespace" value="$(arg stereo_name)" />\n')
      file.write('  </include>\n\n')  
    elif request.type == OPTSensorRequest.TYPE_KINECT2:
      file.write('  <arg name="sensor_name"     default="' + request.id + '" />\n')
      if request.serial != '':
        file.write('  <arg name="sensor_id" default="' + request.serial + '" />\n')
      file.write('\n')
      
      file.write('  <!-- Launch sensor -->\n')
      file.write('  <include file="$(find kinect2_bridge)/launch/kinect2_bridge_ir.launch">\n')
      if request.serial != '':
        file.write('    <arg name="sensor_id"           value="$(arg sensor_id)" />\n')
      #  file.write('    <arg name="rgb_camera_info_url" value="file://$(find opt_calibration)/camera_info/rgb_$(arg sensor_serial).yaml" />\n')
      #else:
      #  file.write('    <arg name="rgb_camera_info_url" value="file://$(find opt_calibration)/camera_info/rgb_$(arg sensor_id).yaml" />\n')
      file.write('    <arg name="sensor_name"         value="$(arg sensor_name)" />\n')
      file.write('    <arg name="publish_frame"       value="true" />\n')
      file.write('  </include>\n\n') 

      file.write('  <!-- Publish a further transform -->\n')
      file.write('  <node pkg="tf" type="static_transform_publisher" name="$(arg sensor_name)_broadcaster" args="0 0 0 1.57079 -1.57079 0 /$(arg sensor_name) /$(arg sensor_name)_link  100" />\n\n')     
      
    file.write('</launch>\n')
    file.close();
    rospy.loginfo(file_name + ' created!');
    
    return (OPTSensorResponse.STATUS_OK, file_name + ' created!')
    
  def handle_create_detector_launch(self, request) :
    
    file_name = self.detector_launchers_dir + 'detection_node_' + request.id + '.launch'
    file = open(file_name, 'w')
    file.write('<?xml version="1.0"?>\n')
    file.write('<!-- SESSION ID: ' + str(request.session_id) + ' -->\n')
    file.write('<launch>\n\n')
    
    file.write('  <!-- Sensor parameters -->\n')
    
    if request.type == OPTSensorRequest.TYPE_SR4500:
      file.write('  <arg name="camera_id"       default="' + request.id + '" />\n')
      if request.ip == '':
        file.close();
        rospy.logerr('Missing "ip" field for the SR4500 sensor!')
        return (OPTSensorResponse.STATUS_ERROR, 'Missing "ip" field!')
      file.write('  <arg name="device_ip"       default="' + request.ip + '" />\n')
      file.write('  <arg name="camera_info_url" default="file://$(find opt_calibration)/camera_info/$(arg camera_id).yaml" />\n\n')
      
      file.write('  <!-- Detection node -->\n')
      file.write('  <include file="$(find detection)/launch/detector_sr4500.launch">\n')
      file.write('    <arg name="camera_id"               value="$(arg camera_id)" />\n')
      file.write('    <arg name="device_ip"               value="$(arg device_ip)" />\n')
      file.write('    <arg name="ground_from_calibration" value="true" />\n')
      file.write('    <arg name="camera_info_url"         value="$(arg camera_info_url)" />\n')
      file.write('  </include>\n\n')
    
    elif request.type == OPTSensorRequest.TYPE_KINECT1:
      if request.serial != '':
        file.write('  <arg name="device_id"   default="' + request.serial + '" />\n')
      file.write('  <arg name="camera_name" default="' + request.id + '" />\n\n')
      
      file.write('  <!-- Detection node -->\n')
      file.write('  <include file="$(find detection)/launch/detector_kinect1.launch">\n')
      if request.serial != '':
        file.write('    <arg name="device_id"               value="$(arg device_id)" />\n')
        file.write('    <arg name="rgb_camera_info_url"     value="file://$(find opt_calibration)/camera_info/rgb_$(arg device_id).yaml" />\n')
      else:
        file.write('    <arg name="rgb_camera_info_url"     value="file://$(find opt_calibration)/camera_info/rgb_$(arg camera_name).yaml" />\n')
      file.write('    <arg name="camera_name"             value="$(arg camera_name)" />\n')
      file.write('    <arg name="ground_from_calibration" value="true" />\n')
      file.write('  </include>\n\n')
      
    elif request.type == OPTSensorRequest.TYPE_STEREO_PG:
      file.write('  <arg name="camera_serial_left"  default="' + request.serial_left + '" />\n')
      file.write('  <arg name="camera_serial_right" default="' + request.serial_right + '" />\n')
      file.write('  <arg name="stereo_name"         default="' + request.id + '" />\n')
      file.write('  <arg name="conf_file_left"      default="$(find opt_calibration)/conf/pg_default.yaml" />\n')
      file.write('  <arg name="conf_file_right"     default="$(find opt_calibration)/conf/pg_default.yaml" />\n\n')
     
      file.write('  <!-- Detection node -->\n')
      file.write('  <include file="$(find detection)/launch/detector_stereo_pg.launch">\n')
      file.write('    <arg name="stereo_name"             value="$(arg stereo_name)" />\n')
      file.write('    <arg name="camera_serial_left"      value="$(arg camera_serial_left)" />\n')
      file.write('    <arg name="camera_serial_right"     value="$(arg camera_serial_right)" />\n')
      file.write('    <arg name="conf_file_left"          value="$(arg conf_file_left)" />\n')
      file.write('    <arg name="conf_file_right"         value="$(arg conf_file_right)" />\n')
      file.write('    <arg name="ground_from_calibration" value="true" />\n')
      file.write('  </include>\n\n')
    elif request.type == OPTSensorRequest.TYPE_KINECT2:
      if request.serial != '':
        file.write('  <arg name="sensor_id"   default="' + request.serial + '" />\n')
      file.write('  <arg name="sensor_name" default="' + request.id + '" />\n\n')
      
      file.write('  <!-- Detection node -->\n')
      file.write('  <include file="$(find detection)/launch/detector_kinect2.launch">\n')
      if request.serial != '':
        file.write('    <arg name="sensor_id"               value="$(arg sensor_id)" />\n')
        file.write('    <arg name="rgb_camera_info_url"     value="file://$(find opt_calibration)/camera_info/rgb_$(arg sensor_id).yaml" />\n')
      else:
        file.write('    <arg name="rgb_camera_info_url"     value="file://$(find opt_calibration)/camera_info/rgb_$(arg sensor_name).yaml" />\n')
      file.write('    <arg name="sensor_name"             value="$(arg sensor_name)" />\n')
      file.write('    <arg name="ground_from_calibration" value="true" />\n')
      file.write('  </include>\n\n')
      
    file.write('</launch>\n')
    file.close();
    rospy.loginfo(file_name + ' created!');
  
    return (OPTSensorResponse.STATUS_OK, file_name + ' created!')
  
  def handle_create_camera_poses(self, request) :
    
    file_name = self.camera_poses_dir + 'camera_poses.txt'
    file = open(file_name, 'w')
    file.write('# Auto-generated file.\n')
    file.write('# CALIBRATION ID: ' + str(request.calibration_id) + '\n')
    
    data = zip(request.child_id, request.transform)
    for item in data:
      t = item[1].translation
      r = item[1].rotation
      file.write(item[0] + ': ' + str(t.x) + ' ' + str(t.y) + ' ' + str(t.z) + ' ')
      file.write(str(r.x) + ' ' + str(r.y) + ' ' + str(r.z) + ' ' + str(r.w) + '\n')
    
    file.close()
    rospy.loginfo(file_name + ' created!');
    
    return (OPTTransformResponse.STATUS_OK, file_name + ' created!')
    
if __name__ == '__main__' :
  
  rospy.init_node('listener')
  
  try:
    
    listener = Listener()
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
