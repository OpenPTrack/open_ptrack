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
from geometry_msgs.msg import Transform, Vector3, Quaternion

class DetectionInitializer :

  def __init__(self) :
    
    network = rospy.get_param('~network')
    calib_parameters = rospy.get_param('~poses')
    inv_calib_parameters = rospy.get_param('~inverse_poses')
    self.file_name = rospy.get_param('~launch_file')
    self.calibration_id = rospy.get_param('~calibration_id')
    
    self.sensor_map = {}
    for item in network:
      pc = item['pc']
      sensors = item['sensors']
      tmp_map = {}
      for sensor in sensors:
        tmp_map[sensor['id']] = {}
        tmp_map[sensor['id']]['pose'] = calib_parameters[sensor['id']]
        tmp_map[sensor['id']]['inv_pose'] = inv_calib_parameters[sensor['id']]
        tmp_map[sensor['id']]['type'] = sensor['type']
      self.sensor_map[pc] = tmp_map
      
    rospy.loginfo(self.sensor_map)
        
  
  def createTFLauncher(self) :
    
    file = open(self.file_name, 'w')
    file.write('<?xml version="1.0"?>\n')
    file.write('<!-- CALIBRATION ID: ' + str(self.calibration_id) + ' -->\n')
    file.write('<launch>\n\n')
    file.write('  <!-- Network parameters -->\n\n')
    
    for pc in self.sensor_map:
      file.write('  <!-- pc: ' + pc + ' -->\n')
      for sensor in self.sensor_map[pc]:
        t = self.sensor_map[pc][sensor]['pose']['translation']
        r = self.sensor_map[pc][sensor]['pose']['rotation']
        file.write('  <!-- sensor: ' + sensor + ' -->\n')
        file.write('  <node pkg="tf" type="static_transform_publisher" ')
        file.write('name="' + sensor +'_broadcaster"\n')
        file.write('        args="' + str(t['x']) + ' ' + str(t['y']) + ' ' + str(t['z']) + ' ')
        file.write(str(r['x']) + ' ' + str(r['y']) + ' ' + str(r['z']) + ' ' + str(r['w']) + ' ')
        file.write('/world /' + sensor +' 100" />\n')
        if self.sensor_map[pc][sensor]['type'] == 'kinect1':
          file.write('  <node pkg="tf" type="static_transform_publisher" ')
          file.write('name="' + sensor +'_broadcaster_2"\n')
          file.write('        args="-0.045 0 0 1.57 -1.57 0 ')
          file.write('/' + sensor + ' /' + sensor + '_link 100" />\n')
        elif self.sensor_map[pc][sensor]['type'] == 'kinect2':
          file.write('  <node pkg="tf" type="static_transform_publisher" ')
          file.write('name="' + sensor +'_broadcaster_2"\n')
          file.write('        args="0 0 0 1.57 -1.57 0 ')
          file.write('/' + sensor + ' /' + sensor + '_link 100" />\n')
        file.write('\n')
          
    file.write('</launch>\n')
    
    file.close()
    
  def fileName(self) :
    return self.file_name
    
  def createCameraPoses(self) :
    
    # For each pc
    for pc in self.sensor_map:
      service_name = pc + '/create_camera_poses'
      rospy.loginfo('Waiting for ' + service_name + ' service...')
      rospy.wait_for_service(service_name)
      
      # Create an OPTTransform message
      sensor_msg = OPTTransformRequest()
      sensor_msg.calibration_id = self.calibration_id
      sensor_msg.parent_id = []
      sensor_msg.child_id = []
      sensor_msg.transform = []
        
      # For each sensor
      for sensor in self.sensor_map[pc]:
        sensor_msg.parent_id = sensor_msg.parent_id + ['world']
        sensor_msg.child_id = sensor_msg.child_id + [sensor]
        t = self.sensor_map[pc][sensor]['inv_pose']['translation']
        r = self.sensor_map[pc][sensor]['inv_pose']['rotation']
        sensor_msg.transform = sensor_msg.transform + [Transform(Vector3(t['x'], t['y'], t['z']), Quaternion(r['x'], r['y'], r['z'], r['w']))]
      
      # Invoke service
      try:
        add_sensor = rospy.ServiceProxy(service_name, OPTTransform)
        response = add_sensor(sensor_msg)
        if response.status == OPTTransformResponse.STATUS_OK:
          rospy.loginfo('[' + pc + '] ' + response.message);
        else:
          rospy.logerr('[' + pc + '] ' + response.message);
      except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s"%e)
    

if __name__ == '__main__' :
  
  rospy.init_node('detection_initializer')
    
  try:
    
    initializer = DetectionInitializer()
    
    initializer.createTFLauncher()
    rospy.loginfo(initializer.fileName() + ' created!');
    
    initializer.createCameraPoses()
    
    rospy.loginfo('Initialization completed. Press [ctrl+c] to exit.')
    
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
