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
#include <visualization_msgs/MarkerArray.h>

using namespace opt_msgs;
using namespace sensor_msgs;


using namespace cv;

class Multiple_Objects_Detection
{
public:

private:

    ////////////For receiving color and depth////////////
    std::mutex lock;
    std::string cameraName;
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
    image_transport::SubscriberFilter *subImageColor, *subImageDepth;
    message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor, *subCameraInfoDepth;

    message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
    message_filters::Synchronizer<ApproximateSyncPolicy> *syncApproximate;
    ////////////For receiving color and depth//////////////



    ////////////For roi selection////////////
    bool objects_selected,paused;
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



    ///////////For main detection///////////
    //  cv::Mat color, depth;
    std::vector<Object_Detector> Object_Detectors;
    std::vector<Rect> current_detected_boxes;
    cv::Mat main_color,main_depth;
    ///////////For main detection///////////




    ///////////For display///////////
    bool show_2D_tracks;
    std::vector<std::list<Rect>> tracks_2D;
    ///////////For display///////////


public:
    Multiple_Objects_Detection(const std::string _cameraName, const std::string &output_detection_topic,const bool useExact, const bool useCompressed,
                               const bool use_background_removal, const int background_calculate_frames,const int threshold_4_detecting_foreground,const bool show_2D_tracks)
        : cameraName(_cameraName), output_detection_topic(output_detection_topic),useExact(useExact), useCompressed(useCompressed),updateImage(false), running(false),
          use_background_removal(use_background_removal), objects_selected(false),paused(false),background_calculate_frames(background_calculate_frames),threshold_4_detecting_foreground(threshold_4_detecting_foreground), queueSize(5),
          nh(), spinner(0), it(nh) ,show_2D_tracks(show_2D_tracks)
    {

        topicColor = "/" + _cameraName + "/" + K2_TOPIC_LORES_COLOR K2_TOPIC_RAW;
        topicDepth = "/" + _cameraName + "/" + K2_TOPIC_LORES_DEPTH K2_TOPIC_RAW;
        cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
        cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
        detection_pub=nh.advertise<DetectionArray>(output_detection_topic,3);
    }

    ~Multiple_Objects_Detection()
    {
    }



    void run_detection(){

        start_reciver();

        for(; running && ros::ok();)
        {
            if(updateImage)
            {

                lock.lock();
                main_color = this->color;
                main_depth = this->depth;
                updateImage = false;
                lock.unlock();

                // if accept background_removal ,generate the new color image without background
                if(use_background_removal)
                {
                    if ((background_calculate_frames>0))// if the back ground has not been got
                    {
                        background_calculation();
                    }
                    else//after get the bakground (depth_max),using it to do background removal,generate new color
                    {
                        background_removal();
                    }
                }

                // if don't accept background_removal ||  background is already removed
                if(!use_background_removal||(use_background_removal&&background_calculate_frames==0))
                {
                    if(!objects_selected&&paused)// pause to select object
                    {
                        multiple_objects_detector_initialization();
                    }

                    //!!!!!!!!!!!!!!!!!!!!!!!main loop!!!!!!!!!!!!!!!!!!!!!!!
                    else if(objects_selected&&!paused)// start to track after selecting objects
                    {
                        multiple_objects_detection_main();
                        //draw tracks_2D (cotinuely 10 detections)
                        if(show_2D_tracks)
                        {
                            show_2D_tracks_on_image();
                        }
                    }

                    cv::imshow( "Object_Detector", main_color );
                    char c = (char)cv::waitKey(10);
                    if( c == 27 )
                        break;
                    switch(c)
                    {
                    case 'p':
                        paused = !paused;
                        break;
                    case 'k':
                        Object_Detectors.clear();
                        break;
                    default:
                        ;
                    }
                }
            }
        }
    }


private:
    void start_reciver()
    {
        running = true;
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
            syncExact->registerCallback(boost::bind(&Multiple_Objects_Detection::callback, this, _1, _2, _3, _4));
        }
        else
        {
            syncApproximate = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
            syncApproximate->registerCallback(boost::bind(&Multiple_Objects_Detection::callback, this, _1, _2, _3, _4));
        }

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
                depth_max=Mat::zeros(main_depth.size(),CV_16UC1);
            else
                depth_max=cv::max(depth_max,main_depth);
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
        Mat depth_diff=depth_max-main_depth;

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


    void multiple_objects_detector_initialization()
    {
        printf("please press P to select ROIs......\n");
        std::vector<Rect> objects;
        roi_zz select_roi;
        select_roi.select("selectroi",main_color,objects,false);
        cv::destroyWindow("selectroi");

        //quit when the tracked object(s) is not provided
        if(objects.size()<1)
            return;


        for(std::vector<Rect>::iterator it =objects.begin();it!=objects.end();it++)
        {

            Object_Detector newDetector;
            cv::Rect selection=*it;
            selection=selection&Rect(0,0,main_color.size().width,main_color.size().height);

            newDetector.setCurrentRect(selection);
            Object_Detectors.push_back(newDetector);

            current_detected_boxes.push_back(selection);

            std::list<Rect> tracks_2D_(10);
            tracks_2D.push_back(tracks_2D_);

            bool occlude_=false;
            occludes.push_back(occlude_);

        }

        Object_Detector::current_detected_boxes=current_detected_boxes;

        objects_selected=true;
        paused=false;
        std::cout<<Object_Detectors.size()<<" objects are selected,start tracking"<<std::endl;
    }



//main_detection
    void multiple_objects_detection_main()
    {

        if( main_color.empty() )
            return;
        Object_Detector::setMainColor(main_color);
        Object_Detector::setMainDepth(main_depth);

        //detection_array_msg!!!!!!!!!!!!!!!!!!!!!!!
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
        //detection_array_msg msg!!!!!!!!!!!!!!!!!!!!!!!



        //detect one by one
        for(size_t i=0; i<Object_Detectors.size(); i++)
        {
            RotatedRect current_trackBox=Object_Detectors[i].detectCurrentRect(i);
            occludes[i]=Object_Detectors[i].occluded;
            if(occludes[i]==false)
            {
                //                                std::cout<<i<<" : not occluded"<<std::endl;
                current_detected_boxes[i]=current_trackBox.boundingRect();//main detection
                cv::ellipse(main_color, current_trackBox, cv::Scalar(250*(i), 250*(i-1), 250*(i-2)), 2, CV_AA);
                //cv::rectangle(color, current_trackBox.boundingRect(), cv::Scalar(255, 0, 0), 2, CV_AA);
                Object_Detector::current_detected_boxes=current_detected_boxes;


                tracks_2D[i].push_back(current_detected_boxes[i]);
                tracks_2D[i].pop_front();


                // genearate detection msg!!!!!!!!!!!!!!!!!!!!!!!!!!
                Detection detection_msg;
                Point2d current_center2D_d(cvFloor(current_trackBox.center.x),cvFloor(current_trackBox.center.y));
                double current_center_depth=main_depth.at<ushort>(current_center2D_d.y,current_center2D_d.x)/1000.0;//meter ,not mm

                detection_msg.box_3D.p1.z=current_center_depth;
                detection_msg.box_3D.p1.x=(current_center2D_d.x-cx)*fx*current_center_depth;
                detection_msg.box_3D.p1.y=(current_center2D_d.y-cy)*fy*current_center_depth;

                detection_msg.box_3D.p2.z=detection_msg.box_3D.p1.z;
                detection_msg.box_3D.p2.x=detection_msg.box_3D.p1.x;
                detection_msg.box_3D.p2.y=detection_msg.box_3D.p1.y;

                detection_msg.box_2D.x = current_detected_boxes[i].x;
                detection_msg.box_2D.y = current_detected_boxes[i].y;
                detection_msg.box_2D.width = current_detected_boxes[i].width;
                detection_msg.box_2D.height = current_detected_boxes[i].height;
                detection_msg.height = 0.002;
                detection_msg.confidence = 10;

                detection_msg.centroid.x=detection_msg.box_3D.p1.x;
                detection_msg.centroid.y=detection_msg.box_3D.p1.y;
                detection_msg.centroid.z=detection_msg.box_3D.p1.z;

                detection_msg.distance = std::sqrt(detection_msg.centroid.x * detection_msg.centroid.x + detection_msg.centroid.z * detection_msg.centroid.z);

                detection_msg.top.x=detection_msg.box_3D.p1.x;
                detection_msg.top.y=detection_msg.box_3D.p1.y;
                detection_msg.top.z=detection_msg.box_3D.p1.z;

                detection_msg.bottom.x=detection_msg.box_3D.p1.x;
                detection_msg.bottom.y=detection_msg.box_3D.p1.y;
                detection_msg.bottom.z=detection_msg.box_3D.p1.z;
                if(current_center_depth>0.01)
                    detection_array_msg->detections.push_back(detection_msg);
            }
            else{//if occluded ,use a empty_rect because of the tracker will use this to generate the other_objects_mask
                //                                std::cout<<i<<" :occluded"<<std::endl;
                //                                cv::ellipse(color, current_trackBox, cv::Scalar(250*(i), 250*(i-1), 250*(i-2)), 2, CV_AA);
                Rect empty_rect(0,0,1,1);
                current_detected_boxes[i]=empty_rect;
                Object_Detector::current_detected_boxes=current_detected_boxes;
            }
        }

        detection_pub.publish(detection_array_msg);		 // publish message

    }


    void show_2D_tracks_on_image()
    {
        for(size_t i=0; i<tracks_2D.size(); i++)
        {
            std::list<Rect> current_track=tracks_2D[i];

            for( std::list<Rect> ::iterator it = current_track.begin(); it!=current_track.end(); it++)
            {
                Rect test_Rect=*it;
                cv::rectangle(main_color, test_Rect, cv::Scalar(250*(i), 250*(i-1), 250*(i-2)), 2, CV_AA);
            }
        }
    }


    void callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
                  const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth)
    {
        cv::Mat _color, _depth;

        if(objects_selected==0)
        {
            readCameraInfo(cameraInfoColor, cameraMatrixColor);
            readCameraInfo(cameraInfoDepth, cameraMatrixDepth);

            fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
            fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
            cx = cameraMatrixColor.at<double>(0, 2);
            cy = cameraMatrixColor.at<double>(1, 2);
        }

        readImage(imageColor, _color);
        readImage(imageDepth, _depth);

        color_header_frameId=imageColor->header.frame_id;

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



