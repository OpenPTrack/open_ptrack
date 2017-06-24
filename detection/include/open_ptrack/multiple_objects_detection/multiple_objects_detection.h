/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author: Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp>



#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Eigen>
#include <boost/foreach.hpp>


#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <kinect2_definitions.h>

#include "open_ptrack/multiple_objects_detection/object_detector.h"
#include "open_ptrack/multiple_objects_detection/roi_zz.h"

//publish the detections
#include <opt_msgs/Detection.h>
#include <opt_msgs/DetectionArray.h>
#include <opt_msgs/BoundingBox3D.h>
#include <opt_msgs/TrackArray.h>
#include <opt_msgs/Image2D_roi_file.h>

#include <visualization_msgs/MarkerArray.h>

#include <opt_msgs/Image2D_roi.h>
#include <opt_msgs/Image2D_roi_array.h>

#include <opt_msgs/ObjectName.h>
#include <opt_msgs/ObjectNameArray.h>

#include <ros/package.h>

using namespace opt_msgs;
using namespace sensor_msgs;
using namespace std;

using namespace cv;

class Multiple_Objects_Detection
{
public:

private:

  ////////////////////////For receiving color and depth////////////////////////
  std::mutex lock;
  std::string topicColor, topicDepth;
  const bool useExact, useCompressed;
  bool updateImage;
  bool running;
  const size_t queueSize;
  cv::Mat color,depth;
  cv::Mat cameraMatrixColor, cameraMatrixDepth;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactSyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateSyncPolicy;
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner;
  image_transport::ImageTransport it;
  //  image_transport::Subscriber roi_image_sub;
  ros::Subscriber roi_image_sub;
  image_transport::SubscriberFilter *subImageColor, *subImageDepth;
  message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor, *subCameraInfoDepth;
  message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
  message_filters::Synchronizer<ApproximateSyncPolicy> *syncApproximate;
  ////////////////////////For receiving color and depth//////////////////////////


  ////////////For roi selection////////////
  bool objects_selected;
  bool finished_select_rois_from_file;
  //    std::vector<Rect> rois_from_gui;
  //

  std::vector<pair<string,Rect>> rois_from_gui;

  std::vector<Mat> rois_from_file;
  std::vector<std::string> rois_from_file_namelist;


  //for roi projection
  tf::TransformListener tf_listener;
  tf::StampedTransform transform_camera2world;

  ros::Subscriber sub_image2D_rois;

  ros::Subscriber sub_object_names_from_tracker;
  ///////////For roi selection/////////////


  ///////////For background removal///////////
  cv::Mat depth_max;
  bool use_background_removal;
  int background_calculate_frames;
  int threshold_4_detecting_foreground;
  ///////////For background removal///////////


  ///////////For camshift recover from occlusion///////////
  std::vector<bool> occludes;
  ///////////For camshift recover from occlusion///////////


  ///////////For generating 3d position///////////
  float fx ,fy,cx,cy;
  ///////////For generating 3d position///////////


  ///////////For publish detection topic///////////
  std::string output_detection_topic;
  ros::Publisher detection_pub;
  std::string color_header_frameId;
  ///////////For publish detection topic///////////


  bool set_object_names;//if to use this detector to set the object names


  ///////////For main detection///////////
  std::vector<Object_Detector> Object_Detectors;
  std::vector<Rect> current_detected_boxes;
  cv::Mat main_color,main_depth_16,main_depth_8;
  ///////////For main detection///////////


  ///////////For display///////////
  bool show_2D_tracks;
  std::vector<std::list<Rect>> tracks_2D;
  ///////////For display///////////

  //test time cost
  std::chrono::time_point<std::chrono::high_resolution_clock> start, now;



public:
  Multiple_Objects_Detection(const std::string &output_detection_topic,const bool set_object_names,const bool useExact, const bool useCompressed,
                             const bool use_background_removal, const int background_calculate_frames,const int threshold_4_detecting_foreground,const bool show_2D_tracks)
    : output_detection_topic(output_detection_topic),set_object_names(set_object_names),useExact(useExact), useCompressed(useCompressed),updateImage(false), running(false),
      use_background_removal(use_background_removal),objects_selected(false), finished_select_rois_from_file(false),background_calculate_frames(background_calculate_frames),threshold_4_detecting_foreground(threshold_4_detecting_foreground), queueSize(5),
      nh(), spinner(0), it(nh) ,show_2D_tracks(show_2D_tracks)
  {
    std::string cameraName = "kinect2_head";
    topicColor = "/" + cameraName + "/" + K2_TOPIC_LORES_COLOR K2_TOPIC_RAW;
    topicDepth = "/" + cameraName + "/" + K2_TOPIC_LORES_DEPTH K2_TOPIC_RAW;

    cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
    cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
    detection_pub=nh.advertise<DetectionArray>(output_detection_topic,3);
  }

  ~Multiple_Objects_Detection()
  {
  }

  void run_detection(){

    start_reciver();// define some subscribers

    for(; running && ros::ok();)
    {
      if(updateImage)//if recieved color and depth msg
      {
        lock.lock();
        main_color = this->color;
        main_depth_16 = this->depth;
        updateImage = false;
        lock.unlock();

        // if accept background_removal ,generate the new color image without background
        if(use_background_removal)
        {
          if ((background_calculate_frames>0))// if the background has not been got
          {
            background_calculation();
          }
          else//after getting the bakground (depth_max), use it to do background removal,generate new color
          {
            background_removal();
          }
        }

        // if don't accept background_removal or  background is already removed
        if(!use_background_removal||(use_background_removal&&background_calculate_frames==0))
        {
          //!!!!!!!!!!!!!!!!!!!!!!!main loop!!!!!!!!!!!!!!!!!!!!!!!

          if(finished_select_rois_from_file)//keep checkin gif there are new rois
            select_rois_from_file();

          if(!rois_from_gui.empty())//keep checkin gif there are new rois
            select_rois_from_gui();

          multiple_objects_detection_main();// main detection

          if(show_2D_tracks)
          {
            show_2D_tracks_on_image();
          }
        }
      }
    }
  }
private:
  void start_reciver()
  {

    sub_image2D_rois = nh.subscribe("/kinect2_head/image2D_rois_from_gui", 5, &Multiple_Objects_Detection::image2D_rois_from_gui_Callback, this);
    //    roi_image_sub = it.subscribe("/kinect2_head/image2D_rois_from_file/image",1,&Multiple_Objects_Detection::image2D_rois_from_file_Callback,this,image_transport::TransportHints("raw"));
    roi_image_sub = nh.subscribe("/kinect2_head/image2D_rois_from_file/image",1,&Multiple_Objects_Detection::image2D_rois_from_file_Callback,this);

    if(!set_object_names)
      sub_object_names_from_tracker=nh.subscribe("/tracker/object_names",5,&Multiple_Objects_Detection::object_names_from_tracker_callback,this);


    running = true;

    //////////////////////////subscribe the color and depth data//////////////////////////
    std::string topicCameraInfoColor = topicColor.substr(0, topicColor.rfind('/')) + "/camera_info";
    std::string topicCameraInfoDepth = topicDepth.substr(0, topicDepth.rfind('/')) + "/camera_info";

    image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");
    image_transport::TransportHints hintsDepth(useCompressed ? "compressedDepth" : "raw");
    subImageColor = new image_transport::SubscriberFilter(it, topicColor, queueSize, hints);
    subImageDepth = new image_transport::SubscriberFilter(it, topicDepth, queueSize, hintsDepth);
    subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoColor, queueSize);
    subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoDepth, queueSize);

    if(useExact)
    {
      syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
      syncExact->registerCallback(boost::bind(&Multiple_Objects_Detection::image_callback, this, _1, _2, _3, _4));
    }
    else
    {
      syncApproximate = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
      syncApproximate->registerCallback(boost::bind(&Multiple_Objects_Detection::image_callback, this, _1, _2, _3, _4));
    }
    //////////////////////////subscribe the color and depth data//////////////////////////


    spinner.start();
    std::chrono::milliseconds duration(1);
    while(!updateImage )
    {
      if(!ros::ok())
      {
        return;
      }
      std::this_thread::sleep_for(duration);
    }
  }

  void background_calculation()
  {
    Mat test_backgroundfile=imread("/tmp/depth_background.png",CV_LOAD_IMAGE_UNCHANGED);
    if(!test_backgroundfile.empty())// if the background successfully read from file
    {
      depth_max = test_backgroundfile;
      background_calculate_frames=0;
      printf("use background removal and background is successfully read from file......\n");
      printf("please press P to select ROIs......\n");
    }
    else//background can't be read from file, being calculated
    {
      printf("use background removal but background can't be read from file, being calculated......\n");
      if(depth_max.empty())
        depth_max=Mat::zeros(main_depth_16.size(),CV_16UC1);
      else
        depth_max=cv::max(depth_max,main_depth_16);
      background_calculate_frames--;
      if(background_calculate_frames==0)//save background file
      {
        std::vector<int> compression_params;
        compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
        compression_params.push_back(0);
        //  pcl::io::savePCDFileASCII ("/tmp/background_" + frame_id.substr(1, frame_id.length()-1) + ".pcd", *background_cloud);
        cv::imwrite("/tmp/depth_background.png", depth_max, compression_params);
        printf("background is sucessfully calculated and saved......\n");
        printf("please press P to select ROIs......\n");
      }
    }
  }

  void background_removal()
  {
    Mat depth_diff=depth_max-main_depth_16;

    int nr=depth_diff.rows;
    int nc=depth_diff.cols;
    if(depth_diff.isContinuous()&&main_color.isContinuous())
    {
      nr=1;
      nc=nc*depth_diff.rows*depth_diff.channels();
    }
    for(int i=0;i<nr;i++)
    {
      ushort* depth_diffData=depth_diff.ptr<ushort>(i);
      uchar* colorData=main_color.ptr<uchar>(i*3);
      for(int j=0;j<nc;j++)
      {
        if (*depth_diffData<threshold_4_detecting_foreground)
        {
          *colorData++=0;
          *colorData++=0;
          *colorData++=0;
          depth_diffData++;
        }
        else
        {
          colorData++;
          colorData++;
          colorData++;
          depth_diffData++;
        }
      }
    }
  }

  void select_rois_from_file()
  {


    if(rois_from_file.empty())//when the "image2D_rois_from_file_Callback" is activted, the "rois_from_file" will be filled with cv::Rect
      return;
    for(std::vector<std::string>::iterator it =rois_from_file_namelist.begin();it!=rois_from_file_namelist.end();it++)
    {
      size_t lastindex = (*it).find_last_of(".");
      *it = (*it).substr(0, lastindex);
    }


    for(int roiIndex=0; roiIndex<rois_from_file.size();roiIndex++)
    {
      Object_Detector newDetector;//create new detector
      if(set_object_names)
        newDetector.setObjectName(rois_from_file_namelist[roiIndex]);
      else
        newDetector.setObjectName("default");

      newDetector.setCurrentRect(Rect(0,0,main_color.size().width,main_color.size().height));//set the whole image as the ROI
      newDetector.occluded=true;//set the occlusion into true so that the detector will keep searching in the whole image for the object untill it shows up
      newDetector.roi_from_file=rois_from_file[roiIndex];// set the roi from the cv::Rect in rois_from_file
      Object_Detectors.push_back(newDetector);
      current_detected_boxes.push_back(Rect(0,0,1,1));
      std::list<Rect> tracks_2D_(10);
      tracks_2D.push_back(tracks_2D_);
      bool occlude_=true;
      occludes.push_back(occlude_);
      Object_Detector::current_detected_boxes=current_detected_boxes;
    }
    objects_selected=true;
    finished_select_rois_from_file=false;
    std::cout<<rois_from_file.size()<<" objects are selected from file"<<std::endl;
  }


  void select_rois_from_gui()
  {
    if(rois_from_gui.empty())//when the "image2D_rois_from_gui_Callback" is activted, the "rois_from_gui" will be filled with cv::Rect
      return;
    for (std::vector<pair<string,Rect>>::const_iterator roi_it= rois_from_gui.begin();roi_it!=rois_from_gui.end();roi_it++)
    {
      Object_Detector newDetector;
      if(set_object_names)
        newDetector.setObjectName((*roi_it).first);
      else
        newDetector.setObjectName("default");
      newDetector.setCurrentRect((*roi_it).second);

      Object_Detectors.push_back(newDetector);
      current_detected_boxes.push_back((*roi_it).second);
      std::list<Rect> tracks_2D_(10);
      tracks_2D.push_back(tracks_2D_);
      bool occlude_=false;
      occludes.push_back(occlude_);
      Object_Detector::current_detected_boxes=current_detected_boxes;
    }
    objects_selected=true;
    std::cout<<rois_from_gui.size()<<" objects are selected from gui"<<std::endl;
    rois_from_gui.clear();
  }


  void multiple_objects_detection_main()    //main_detection
  {
    if( main_color.empty()||Object_Detectors.empty())
      return;


    ////////////////////////////////////////set the input(color+depth) of every detector////////////////////////////////////////
    Object_Detector::setMainColor(main_color);// set all the detectors' "main_color"
    if(Object_Detector::Backprojection_Mode=="HSD")// if use HSD, convert the depth from 16bit into 8bit
    {
      //devide (1000mm~9000mm) into 255 parts
      ushort Max=9000,Min=1000;
      main_depth_16.convertTo(main_depth_8, CV_8U,255.0/(Max-Min),-255.0*Min/(Max-Min));
      Object_Detector::setMainDepth(main_depth_8);
    }
    else
    {
      Object_Detector::setMainDepth(main_depth_16);
    }
    ////////////////////////////////////////set the input(color+depth) of every detector////////////////////////////////////////



    //!!!!!!!!!!!!!!!!!!!!!!!create detection_array_msg!!!!!!!!!!!!!!!!!!!!!!!
    /// Write detection message:
    DetectionArray::Ptr detection_array_msg(new DetectionArray);
    // Set camera-specific fields:
    detection_array_msg->header.frame_id = color_header_frameId;
    detection_array_msg->header.stamp = ros::Time::now();

    for(int i = 0; i < 3; i++)
      for(int j = 0; j < 3; j++)
        detection_array_msg->intrinsic_matrix.push_back(cameraMatrixColor.at<double>(i, j));

    //set the parms so that we can use the openptrack's tracking node
    detection_array_msg->confidence_type = std::string("hog+svm");
    detection_array_msg->image_type = std::string("rgb");
    //!!!!!!!!!!!!!!!!!!!!!!!detection_array_msg msg!!!!!!!!!!!!!!!!!!!!!!!

    //detect objects one by one
    for(size_t i=0; i<Object_Detectors.size(); i++)
    {
      ////////////main detection, return a detection with a RotatedRect
      RotatedRect current_trackBox=Object_Detectors[i].detectCurrentRect(i);
      string object_name=Object_Detectors[i].object_name;
      /////////////main detection, return a detection with a RotatedRect


      occludes[i]=Object_Detectors[i].occluded;//update the occlusion flag

      if(occludes[i]==false)// if no occlusion
      {

        ////////////////////for display the detection ellipse and track points////////////////////
        current_detected_boxes[i]=current_trackBox.boundingRect();
        cv::ellipse(main_color, current_trackBox, cv::Scalar(250*(i), 250*(i-1), 250*(i-2)), 2, CV_AA);
        //cv::rectangle(color, current_trackBox.boundingRect(), cv::Scalar(255, 0, 0), 2, CV_AA);
        tracks_2D[i].push_back(current_detected_boxes[i]);
        tracks_2D[i].pop_front();
        ////////////////////for display the detection ellipse and track points////////////////////


        Object_Detector::current_detected_boxes=current_detected_boxes;// update the detected_boxes, it is used to generate the other objects mask for the following detection , in the following detection ,this Rect will be blocked in the image


        //////////////////////// genearate detection msg!!!!!!!!!!!!!!!!!!!!!!!!!!
        Detection detection_msg;
        Point2d current_center2D_d(cvFloor(current_trackBox.center.x),cvFloor(current_trackBox.center.y));//center point in image2D coordinate
        double current_center_depth=main_depth_16.at<ushort>(current_center2D_d.y,current_center2D_d.x)/1000.0;//meter ,not mm

        tf::Vector3 camera_3D;//center point in camera3D coordinate
        camera_3D.setX((current_center2D_d.x-cx)*fx*current_center_depth);
        camera_3D.setY((current_center2D_d.y-cy)*fy*current_center_depth);
        camera_3D.setZ(current_center_depth);

        // set items of the detection msg
        /////set every 3D point of the detection box with the center point in camera3D coordinate/////
        detection_msg.box_3D.p1.z=camera_3D.z();
        detection_msg.box_3D.p1.x=camera_3D.x();
        detection_msg.box_3D.p1.y=camera_3D.y();

        detection_msg.box_3D.p2.z=detection_msg.box_3D.p1.z;
        detection_msg.box_3D.p2.x=detection_msg.box_3D.p1.x;
        detection_msg.box_3D.p2.y=detection_msg.box_3D.p1.y;

        detection_msg.centroid.x=detection_msg.box_3D.p1.x;
        detection_msg.centroid.y=detection_msg.box_3D.p1.y;
        detection_msg.centroid.z=detection_msg.box_3D.p1.z;

        detection_msg.top.x=detection_msg.box_3D.p1.x;
        detection_msg.top.y=detection_msg.box_3D.p1.y;
        detection_msg.top.z=detection_msg.box_3D.p1.z;

        detection_msg.bottom.x=detection_msg.box_3D.p1.x;
        detection_msg.bottom.y=detection_msg.box_3D.p1.y;
        detection_msg.bottom.z=detection_msg.box_3D.p1.z;
        /////set every 3D point of the detection box with the center point in camera3D coordinate/////


        detection_msg.box_2D.x = current_detected_boxes[i].x;
        detection_msg.box_2D.y = current_detected_boxes[i].y;
        detection_msg.box_2D.width = current_detected_boxes[i].width;
        detection_msg.box_2D.height = current_detected_boxes[i].height;


        detection_msg.confidence = 10;//set a fixed value

        //the distance between the camera and the object
        detection_msg.distance = std::sqrt(detection_msg.centroid.x * detection_msg.centroid.x + detection_msg.centroid.z * detection_msg.centroid.z);


        // set the the height of the detection in world_3D, this will pass to the tracker ,the only reason to do this is it is needed in json message.
        tf::Vector3 world_3D;
        world_3D = transform_camera2world(camera_3D);
        detection_msg.height =world_3D.z()*4/3 ;

        detection_msg.object_name=object_name;

        if(current_center_depth>0.01)
          detection_array_msg->detections.push_back(detection_msg);//push to the msg array
      }
      else{//if occluded ,use a empty_rect because of the tracker will use this to generate the other_objects_mask
        Rect empty_rect(0,0,1,1);
        current_detected_boxes[i]=empty_rect;
        Object_Detector::current_detected_boxes=current_detected_boxes;
      }
    }

    detection_pub.publish(detection_array_msg);		 // publish message
  }

  void show_2D_tracks_on_image()//show track points on 2D image
  {
    for(size_t i=0; i<tracks_2D.size(); i++)
    {
      std::list<Rect> current_track=tracks_2D[i];
      for( std::list<Rect> ::iterator it = current_track.begin(); it!=current_track.end(); it++)
      {
        Rect test_Rect((*it).x+((*it).width)/2,(*it).y+((*it).height)/2,1,1);
        test_Rect=test_Rect&Rect(0,0,main_color.size().width,main_color.size().height);
        cv::rectangle(main_color, test_Rect, cv::Scalar(250*(i), 250*(i-1), 250*(i-2)), 2, CV_AA);
      }
    }
    cv::namedWindow("show_2D_tracks");
    cv::imshow( "show_2D_tracks", main_color );
    cv::waitKey(10);
  }



  //recieve the color and depth image
  void image_callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
                      const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth)
  {
    cv::Mat _color, _depth;
    readImage(imageColor, _color);
    readImage(imageDepth, _depth);

    color_header_frameId=imageColor->header.frame_id;

    if(!objects_selected)// the camrea info just need to be read once
    {
      readCameraInfo(cameraInfoColor, cameraMatrixColor);
      readCameraInfo(cameraInfoDepth, cameraMatrixDepth);

      fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
      fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
      cx = cameraMatrixColor.at<double>(0, 2);
      cy = cameraMatrixColor.at<double>(1, 2);

      std::string world_frame_id="/world";
      try{
        tf_listener.lookupTransform(world_frame_id, color_header_frameId, ros::Time(0), transform_camera2world);
      }
      catch(tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }
    }


    if(_color.type() == CV_16U)
    {
      cv::Mat tmp;
      _color.convertTo(tmp, CV_8U, 0.02);
      cv::cvtColor(tmp, _color, CV_GRAY2BGR);
    }

    lock.lock();
    this->color = _color;
    this->depth = _depth;
    updateImage = true;
    lock.unlock();
  }


  //recieve the roi msg(x,y,width,height) from marking in the gui
  void image2D_rois_from_gui_Callback(const opt_msgs::Image2D_roi_array::ConstPtr& image2D_roi_msg)
  {
    for( std::vector<opt_msgs::Image2D_roi>::const_iterator it = image2D_roi_msg->Rois.begin();
         it != image2D_roi_msg->Rois.end(); it++)
    {
      string object_name=(*it).name;
      cv::Rect selection((*it).x,(*it).y,(*it).width,(*it).height);
      selection=selection&Rect(0,0,main_color.size().width,main_color.size().height);

      std::pair<string,Rect> new_rois_from_gui(object_name,selection);

      lock.lock();
      rois_from_gui.push_back(new_rois_from_gui);
      lock.unlock();
      std::cout<<"got image msg comes from gui"<<std::endl;
    }
  }


  //recieve the roi msg(image) from file
  void image2D_rois_from_file_Callback(const opt_msgs::Image2D_roi_filePtr &msg)
  {
    if (msg->name!="end_flag")
    {
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg->image_roi, sensor_msgs::image_encodings::BGR8);
        cv::Mat conversion_mat_;
        cv_ptr->image.copyTo(conversion_mat_);
        lock.lock();
        rois_from_file.push_back(conversion_mat_);
        lock.unlock();

        string object_name=msg->name;
        rois_from_file_namelist.push_back(object_name);

        std::cout<<"got image msg comes from file"<<std::endl;
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
    }
    else
    {
      finished_select_rois_from_file=true;
      std::cout<<"finish receiving image msg comes from file"<<std::endl;
      roi_image_sub.shutdown();//shut down it when all the rois from file finished
    }
  }





  void object_names_from_tracker_callback(const opt_msgs::ObjectNameArrayConstPtr& object_names_msg)
  {
    if (("/"+object_names_msg->header.frame_id)==color_header_frameId)
    {
      for( std::vector<opt_msgs::ObjectName>::const_iterator it = object_names_msg->object_names.begin();
           it!=object_names_msg->object_names.end();it++)
      {
        int number=(*it).no;
        if(Object_Detectors[number].object_name=="default")
        {
          Object_Detectors[number].object_name=(*it).object_name;
          std::cout<<"set number "<<number<< "object name to "<<(*it).object_name<<std::endl;
        }
      }
    }
  }



  void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const
  {
    cv_bridge::CvImageConstPtr pCvImage;
    pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
    pCvImage->image.copyTo(image);
  }

  void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const
  {
    double *itC = cameraMatrix.ptr<double>(0, 0);
    for(size_t i = 0; i < 9; ++i, ++itC)
    {
      *itC = cameraInfo->K[i];
    }
  }
};


