/*Software License Agreement (BSD License)

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

#include "ros/ros.h"//Used for launch file parameter parsing
#include <string>//Used for rois message vector
#include <vector>
#include <sstream>

//Included for files
#include <iostream>
#include <fstream>
#include "stdio.h"
#include "dirent.h"

//Publish Messages
#include "std_msgs/String.h"
#include "opt_msgs/DetectionArray.h"

//Time Synchronizer
// NOTE: Time Synchronizer conflicts with QT includes may need to investigate
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

//Subscribe Messages
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

// Image Transport
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// Used to display OPENCV images
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Open PTrack
#include "open_ptrack/opt_utils/conversions.h"

using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters::sync_policies;
using namespace sensor_msgs::image_encodings;
using namespace opt_msgs;
using namespace cv;

//NOTE: Where is the best place to put these
//typedef ApproximateTime<Image, DisparityImage> ApproximatePolicy;
//typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;

cv::Scalar darkBlue(130,0,0);
cv::Scalar white(255,255,255);

class roiViewerNode
{
  private:
    // Define Node
    ros::NodeHandle node_;

    // Subscribe to Messages
    message_filters::Subscriber<Image> sub_image_;
    message_filters::Subscriber<opt_msgs::DetectionArray> sub_detections_;

    // Define the Synchronizer
    typedef ApproximateTime<Image, opt_msgs::DetectionArray> ApproximatePolicy;
    typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
    boost::shared_ptr<ApproximateSync> approximate_sync_;

    // Launch file Parameters
    int label;
    bool show_confidence;
    bool color_image;

    // Object of class Conversions:
    open_ptrack::opt_utils::Conversions converter;

  public:

    explicit roiViewerNode(const ros::NodeHandle& nh):
    node_(nh)
    {

      label = 0;
      show_confidence = false;

      //Read mode from launch file
      std::string mode="";
      node_.param(ros::this_node::getName() + "/mode", mode, std::string("none"));
      ROS_INFO("Selected mode: %s",mode.c_str());

      if(mode.compare("roi_display")==0){
        ROS_INFO("MODE: %s",mode.c_str());

        //Get the image width and height
        node_.param(ros::this_node::getName()+"/label",label,0);

        //Read parameter for showing roi confidence
        node_.param(ros::this_node::getName()+"/show_confidence",show_confidence,false);

        //Read parameter stating if the image is grayscale or with colors
        node_.param(ros::this_node::getName()+"/color_image", color_image, true);

        // Subscribe to Messages
        sub_image_.subscribe(node_,"input_image",20);
        sub_detections_.subscribe(node_,"input_detections",20);

        // Sync the Synchronizer
        approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(20),
            sub_image_,
            sub_detections_));

        approximate_sync_->registerCallback(boost::bind(&roiViewerNode::imageCb,
            this,
            _1,
            _2));
      }else{

        ROS_INFO("Unknown mode:%s  Please set to {roi_display} in roiViewer.launch",mode.c_str());
      }
      // Visualization
      cv::namedWindow("Detections", 0 ); // non-autosized
      cv::startWindowThread();

    }

    void imageCb(const ImageConstPtr& image_msg,
        const opt_msgs::DetectionArray::ConstPtr& detection_msg){

      std::string filename = image_msg->header.frame_id.c_str();
      std::string imgName = filename.substr(filename.find_last_of("/")+1);

      ROS_INFO("roiViewer Callback called for image: %s", imgName.c_str());

      //Use CV Bridge to convert images
      cv_bridge::CvImagePtr cv_ptr;
      if (color_image)
      {
        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
      }
      else
      {
        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
      }

      // Read camera intrinsic parameters:
      Eigen::Matrix3f intrinsic_matrix;
      for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
          intrinsic_matrix(i, j) = detection_msg->intrinsic_matrix[i * 3 + j];

      //For each roi in rois message
      //ROS_INFO("ROIS size: %d",rois_msg->rois.size());
      for(unsigned int i=0;i<detection_msg->detections.size();i++)
      {
        // theoretical person centroid:
        Eigen::Vector3f centroid3d(detection_msg->detections[i].centroid.x, detection_msg->detections[i].centroid.y, detection_msg->detections[i].centroid.z);
        Eigen::Vector3f centroid2d = converter.world2cam(centroid3d, intrinsic_matrix);

        // theoretical person top point:
        Eigen::Vector3f top3d(detection_msg->detections[i].top.x, detection_msg->detections[i].top.y, detection_msg->detections[i].top.z);
        Eigen::Vector3f top2d = converter.world2cam(top3d, intrinsic_matrix);

        // Define Rect and make sure it is not out of the image:
        int h = centroid2d(1) - top2d(1);
        int w = h * 2 / 3.0;
        int x = std::max(0, int(centroid2d(0) - w / 2.0));
        int y = std::max(0, int(top2d(1)));
        h = std::min(int(cv_ptr->image.rows - y), int(h));
        w = std::min(int(cv_ptr->image.cols - x), int(w));

        Point ptUpperLeft = Point(x,y);
        Point ptLowerRight = Point(x+w,y+h);

        // Draw a rectangle around the detected person:
        rectangle(cv_ptr->image,ptUpperLeft,ptLowerRight,Scalar(255,255,255));

        if (show_confidence)
        {
          // Draw roi confidence
          float confidenceToDisplay = float(int(detection_msg->detections[i].confidence*100))/100;
          std::stringstream conf_ss;
          conf_ss << confidenceToDisplay;
          double scale_factor = std::min(1.0, 2*double(image_msg->height) / 480);
          cv::rectangle(cv_ptr->image, cv::Point(x, y - 15 * scale_factor),	cv::Point(x + 60 * scale_factor, y), darkBlue, CV_FILLED, 8);
          cv::putText(cv_ptr->image, conf_ss.str(), cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.5 * scale_factor, white, 1.7, CV_AA);
        }
      }

      // Display the cv image
      cv::imshow("Detections",cv_ptr->image);
    }

    ~roiViewerNode()
    {
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "roiViewer");
  ros::NodeHandle n;
  roiViewerNode roiViewerNode(n);
  ros::spin();

  return 0;
}

