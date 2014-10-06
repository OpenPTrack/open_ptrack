// $Id: dev_sr.cpp 39413 2012-05-10 19:51:51Z pbeeson $

/*
 * Copyright (c) 2010 TRACLabs Inc. 
 * This node uses the libmesasr API to support both SR 3000 and 4000
 * devices.  It extends the 2008 ROS swissranger (3000 only) node that
 * used the older libUSB API.  This verison also works with ros-core
 * >= v0.9.
 * Author: Patrick Beeson (pbeeson@traclabs.com) 
 *
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * Author: Radu Bogdan Rusu <rusu@cs.tum.edu>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <sr.h>
#include <limits>
#include "ros/ros.h"

//! Macro for throwing an exception with a message
#define SR_EXCEPT(except, msg)					\
  {								\
    char buf[100];						\
    snprintf(buf, 100, "[SR::%s]: " msg, __FUNCTION__); \
    throw except(buf);						\
  }

//! Macro for throwing an exception with a message, passing args
#define SR_EXCEPT_ARGS(except, msg, ...)				\
  {									\
    char buf[100];							\
    snprintf(buf, 100, "[SR::%s]: " msg, __FUNCTION__, __VA_ARGS__); \
    throw except(buf);							\
  }

extern int SR_ROS_FuncCB(SRCAM srCam, unsigned int msg, unsigned int param, void *data);
int SR_ROS_FuncCB(SRCAM srCam, unsigned int msg, unsigned int param, void *data) {
  switch (msg)
  {
  case CM_MSG_DISPLAY:
    if (param & 0x04FF)
    {
      ROS_WARN("MSG_DISPLAY : %X : %s", param, (char *)data);
    }
    break;
  }
  return 0;
}

using namespace sr;
using namespace std;

////////////////////////////////////////////////////////////////////////////////
// Constructor
SR::SR(bool use_filter) : srCam_(NULL), imgEntryArray_(NULL), buffer_(NULL), use_filter_(use_filter) {}

SR::~SR() 
{
  SafeCleanup();
}

int SR::open(int auto_exposure, int integration_time, int modulation_freq, int amp_threshold, std::string &ether_addr)
{
  // ---[ Open the camera ]---
  int res = 0;
  if(ether_addr != "")
  {
    // ---[ set callback function ] ---
    SR_SetCallback((SR_FuncCB *)SR_ROS_FuncCB);
    res = SR_OpenETH (&srCam_, ether_addr.c_str());
  }
  else
    res = SR_OpenUSB (&srCam_, 0); //returns the device ID used in

  if (res <= 0)
  {
    SafeCleanup();
    SR_EXCEPT(sr::Exception, "Failed to open device!");
    return (-1);
  }
  
  device_id_   = getDeviceString ();
  lib_version_ = getLibraryVersion ();
  // ---[ Get the number of rows, cols, ... ]---
  int rows_ = SR_GetRows (srCam_);
  int cols_ = SR_GetCols (srCam_);

  // ---[ Set the acquisition mode ]---
  SR_SetMode (srCam_, MODE);

  int inr_  = SR_GetImageList (srCam_, &imgEntryArray_);

  ROS_INFO ("[SwissRanger device::open] Number of images available: %d", inr_);

  if ( (cols_ != SR_COLS) || (rows_ != SR_ROWS) || (inr_ < SR_IMAGES) || (imgEntryArray_ == 0) )
  {
    SafeCleanup();
    SR_EXCEPT_ARGS(sr::Exception,
                   "Invalid data images: %d %dx%d images received from camera!\n Expected %d %dx%d images.",
                   inr_, cols_, rows_, SR_IMAGES, SR_COLS, SR_ROWS);
    return (-1);
  }
  
  if (auto_exposure >= 0)
    setAutoExposure(auto_exposure);

  if (integration_time >=0 && integration_time != getIntegrationTime())
    setIntegrationTime(integration_time);

  if (modulation_freq >=0 && modulation_freq != getModulationFrequency())
    setModulationFrequency(modulation_freq);

  if (amp_threshold >=0 && amp_threshold != getAmplitudeThreshold())
    setAmplitudeThreshold(amp_threshold);

  // Points array
  size_t buffer_size = rows_ * cols_ * 3 * sizeof (float);
  buffer_ = (float*)malloc (buffer_size);
  memset (buffer_, 0xaf, buffer_size);

  xp_ = buffer_;
  yp_ = &xp_[rows_*cols_];
  zp_ = &yp_[rows_*cols_];

  return 0;
}

// //////////////////////////////////////////////////////////////////////////////
// Safe Cleanup
void SR::SafeCleanup()
{
  if (srCam_)
  {
    SR_Close (srCam_);
  }
  if (buffer_)
    free(buffer_);

  srCam_ = NULL;
  buffer_ = NULL;
}



int SR::close()
{
  if (srCam_)
    if (SR_Close (srCam_))
      ROS_WARN("unable to stop sr");

  // Free resources
  SafeCleanup();

  return 0;
}

// ////////////////////////////////////////////////////////////////////////////////
// // Store an image frame into the 'frame' buffer
void SR::readData(sensor_msgs::PointCloud &cloud,
                  sensor_msgs::PointCloud2 &cloud2,
                  sensor_msgs::Image &image_d,
                  sensor_msgs::Image &image_i,
                  sensor_msgs::Image &image_c,
                  sensor_msgs::Image &image_d16) {
  
  if (srCam_ == NULL) {
    SR_EXCEPT(sr::Exception, "Read attempted on NULL SwissRanger port!");
    return;
  }


  int res;

  double time1 = ros::Time::now().toSec();
  res = SR_Acquire (srCam_);
  double time2 = ros::Time::now().toSec();
  if (res < 0)
  {
    SR_EXCEPT(sr::Exception, "Unable to capture data");
    return;
  }

  double timestamp=(time1+time2)/2;
  cloud.header.stamp=cloud2.header.stamp=image_d.header.stamp=image_d16.header.stamp=
      image_i.header.stamp=image_c.header.stamp=ros::Time(timestamp);


  size_t image_size = imgEntryArray_->width * imgEntryArray_->height ;

  // Pointers to data
  uint8_t *distance_image   = (unsigned char*)SR_GetImage (srCam_, 0);
  uint8_t *intensity_image  = (unsigned char*)SR_GetImage (srCam_, 1);
  uint8_t *confidence_image = (unsigned char*)SR_GetImage (srCam_, 2);
  
  // Points array
  res = SR_CoordTrfFlt (srCam_, xp_, yp_, zp_, sizeof (float),
                        sizeof (float), sizeof (float));

  cloud.points.resize(image_size);
  cloud.channels.resize (2);
  cloud.channels[1].name = "confidence";
  cloud.channels[1].values.resize(image_size);
  cloud.channels[0].name = "intensity";
  cloud.channels[0].values.resize(image_size);

  cloud2.height = imgEntryArray_->height;
  cloud2.width = imgEntryArray_->width;
  cloud2.fields.resize (5);
  cloud2.fields[0].name = "x";
  cloud2.fields[0].offset = 0;
  cloud2.fields[0].count = 1;
  cloud2.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  cloud2.fields[1].name = "y";
  cloud2.fields[1].offset = 4;
  cloud2.fields[1].count = 1;
  cloud2.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  cloud2.fields[2].name = "z";
  cloud2.fields[2].offset = 8;
  cloud2.fields[2].count = 1;
  cloud2.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  cloud2.fields[3].name = "intensity";
  cloud2.fields[3].offset = 12;
  cloud2.fields[3].count = 1;
  cloud2.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  cloud2.fields[4].name = "confidence";
  cloud2.fields[4].offset = 16;
  cloud2.fields[4].count = 1;
  cloud2.fields[4].datatype = sensor_msgs::PointField::FLOAT32;
  //cloud2.is_bigendian = false; ???
  cloud2.point_step = 20;
  cloud2.row_step = cloud2.point_step * cloud2.width;
  cloud2.data.resize (cloud2.row_step * cloud2.height);
  cloud2.is_dense = true;

  image_d.width  = SR_COLS;
  image_d.height = SR_ROWS;
  image_d.encoding = "mono8";
  image_d.step=SR_COLS;
  image_d.data.resize (image_size);

  image_d16.width  = SR_COLS;
  image_d16.height = SR_ROWS;
  image_d16.encoding = "mono16";
  image_d16.step=SR_COLS*2;
  image_d16.data.resize (image_size*2);

  image_i.width  = SR_COLS;
  image_i.height = SR_ROWS;
  image_i.encoding = "mono8";
  image_i.step=SR_COLS;
  image_i.data.resize (image_size);

  image_c.width  = SR_COLS;
  image_c.height = SR_ROWS;
  image_c.encoding  = "mono8";
  image_c.step=SR_COLS;
  image_c.data.resize (image_size);

  geometry_msgs::Point32 pt;
  uint count=0;

  for (uint i = 0; i < image_size; i++)
  {
    //check if images contain data, mostly needed because SR3K does
    //not provide a confidence image
    if (distance_image != 0x0)
    {
      image_d.data[i] = ((distance_image[i * 2 + 0] << 0) + (distance_image[i * 2 + 1] << 8)) * (255 / 65535.0);
      image_d16.data[i * 2 + 0] = distance_image[i * 2 + 0];
      image_d16.data[i * 2 + 1] = distance_image[i * 2 + 1];
    }

    if (intensity_image != 0x0)
      image_i.data[i] = ((intensity_image[i * 2 + 0] << 0) + (intensity_image[i * 2 + 1] << 8)) * (255 / 65535.0);

    if (confidence_image != 0x0)
      image_c.data[i] = ((confidence_image[i * 2 + 0] << 0) + (confidence_image[i * 2 + 1] << 8)) * (255 / 65535.0);

    if (!use_filter_ || zp_[i] > 0.15)
    {
      pt.x=-xp_[i];
      pt.y=-yp_[i];
      pt.z=zp_[i];

      cloud.points[count]=pt;
      cloud.channels[0].values[count]= ((intensity_image[i * 2 + 0] << 0) + (intensity_image[i * 2 + 1] << 8));
      cloud.channels[1].values[count]= ((confidence_image[i * 2 + 0] << 0) + (confidence_image[i * 2 + 1] << 8));
      count++;

      memcpy (&cloud2.data[i * cloud2.point_step + 0], &pt.x, sizeof (float));
      memcpy (&cloud2.data[i * cloud2.point_step + 4], &pt.y, sizeof (float));
      memcpy (&cloud2.data[i * cloud2.point_step + 8], &pt.z, sizeof (float));
      float intensity = ((intensity_image[i * 2 + 0] << 0) + (intensity_image[i * 2 + 1] << 8));
      memcpy (&cloud2.data[i * cloud2.point_step + 12], &intensity, sizeof (float));
      float confidence = ((confidence_image[i * 2 + 0] << 0) + (confidence_image[i * 2 + 1] << 8));
      memcpy (&cloud2.data[i * cloud2.point_step + 16], &confidence, sizeof (float));
    }
    else
    {
      float bad_point = std::numeric_limits<float>::quiet_NaN ();
      memcpy (&cloud2.data[i * cloud2.point_step + 0], &bad_point, sizeof (float));
      memcpy (&cloud2.data[i * cloud2.point_step + 4], &bad_point, sizeof (float));
      memcpy (&cloud2.data[i * cloud2.point_step + 8], &bad_point, sizeof (float));
      memcpy (&cloud2.data[i * cloud2.point_step + 12], &bad_point, sizeof (float));
      memcpy (&cloud2.data[i * cloud2.point_step + 16], &bad_point, sizeof (float));
    }

  }

  cloud.points.resize(count);
  cloud.channels[0].values.resize(count);
  cloud.channels[1].values.resize(count);

  return;
}


////////////////////////////////////////////////////////////////////////////////
int
SR::setAutoExposure (bool on)
{
  int res;
  if (on)
#ifdef USE_SR4K
    res = SR_SetAutoExposure (srCam_, 1, 150, 5, 70);
#else
    res = SR_SetAutoExposure (srCam_, 2, 255, 10, 45);
#endif
  else
    res = SR_SetAutoExposure (srCam_, 255, 0, 0, 0);
  return (res);
}

////////////////////////////////////////////////////////////////////////////////
int
SR::setIntegrationTime (int time)
{
  // ---[ Set integration time
  return (SR_SetIntegrationTime (srCam_, time));
}

////////////////////////////////////////////////////////////////////////////////
int
SR::getIntegrationTime ()
{
  // ---[ Set integration time
  return (SR_GetIntegrationTime (srCam_));
}

////////////////////////////////////////////////////////////////////////////////
int
SR::setModulationFrequency (int freq)
{
  // ---[ Set modulation frequency
  return (SR_SetModulationFrequency (srCam_, (ModulationFrq)freq));
}

////////////////////////////////////////////////////////////////////////////////
int
SR::getModulationFrequency ()
{
  // ---[ Set modulation frequency
  return (SR_GetModulationFrequency (srCam_));
}

////////////////////////////////////////////////////////////////////////////////
int
SR::setAmplitudeThreshold (int thresh)
{
  // ---[ Set amplitude threshold
  return (SR_SetAmplitudeThreshold (srCam_, thresh));
}

////////////////////////////////////////////////////////////////////////////////
int
SR::getAmplitudeThreshold ()
{
  // ---[ Set amplitude threshold
  return (SR_GetAmplitudeThreshold (srCam_));
}

////////////////////////////////////////////////////////////////////////////////
// Obtain the device product name
std::string
SR::getDeviceString ()
{
  char *buf = new char[256];
  int *buflen = new int;
  SR_GetDeviceString (srCam_, buf, *buflen);

  // VendorID:0x%04x, ProductID:0x%04x, Manufacturer:'%s', Product:'%s'
  std::string sensor (buf);
  std::string::size_type loc = sensor.find ("Product:", 0);
  if (loc != std::string::npos)
  {
    sensor = sensor.substr (loc + 9, *buflen);
    loc = sensor.find ("'", 0);
    if (loc != std::string::npos)
      sensor = sensor.substr (0, loc);
  }
  else
    sensor = "";

  delete buflen;
  delete [] buf;
  return (sensor);
}

////////////////////////////////////////////////////////////////////////////////
// Obtain the libMesaSR library version
std::string
SR::getLibraryVersion ()
{
  unsigned short version[4];
  char buf[80];
  SR_GetVersion (version);
  snprintf (buf, sizeof (buf), "%d.%d.%d.%d", version[3], version[2], version[1], version[0]);
  return (std::string (buf));
}
