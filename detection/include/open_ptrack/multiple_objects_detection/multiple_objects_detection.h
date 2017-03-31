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
#include <visualization_msgs/MarkerArray.h>

#include <opt_msgs/Image2D_roi.h>
#include <opt_msgs/Image2D_roi_array.h>
#include <opt_msgs/World3D_roi.h>
#include <opt_msgs/World3D_roi_array.h>

using namespace opt_msgs;
using namespace sensor_msgs;


using namespace cv;

class Multiple_Objects_Detection
{
public:

private:

    ////////////For receiving color and depth////////////
    std::mutex lock;
    //    std::string cameraName;
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
    bool objects_selected;
    //    std::vector<Rect> objects;
    //for roi projection
    tf::TransformListener tf_listener;
    ros::Subscriber sub_image2D_rois;
    ros::Subscriber sub_world3D_rois;
    ros::Publisher world3D_rois_pub;
    World3D_roi current_World3D_roi_msg;
    World3D_roi_array World3D_rois_msg;
    bool publish_world3D_rois;
    int numberofrois=0;
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
    cv::Mat main_color,main_depth_16,main_depth_8;
    ///////////For main detection///////////


    ///////////For display///////////
    bool show_2D_tracks;
    std::vector<std::list<Rect>> tracks_2D;
    ///////////For display///////////

    //test time cost
    std::chrono::time_point<std::chrono::high_resolution_clock> start, now;



public:
    Multiple_Objects_Detection(const std::string &output_detection_topic,const bool useExact, const bool useCompressed,
                               const bool use_background_removal, const int background_calculate_frames,const int threshold_4_detecting_foreground,const bool show_2D_tracks, const bool publish_world3D_rois)
        : output_detection_topic(output_detection_topic),useExact(useExact), useCompressed(useCompressed),updateImage(false), running(false),
          use_background_removal(use_background_removal), objects_selected(false),background_calculate_frames(background_calculate_frames),threshold_4_detecting_foreground(threshold_4_detecting_foreground), queueSize(5),
          nh(), spinner(0), it(nh) ,show_2D_tracks(show_2D_tracks) ,publish_world3D_rois(publish_world3D_rois)
    {
        std::string cameraName = "kinect2_head";
        topicColor = "/" + cameraName + "/" + K2_TOPIC_LORES_COLOR K2_TOPIC_RAW;
        topicDepth = "/" + cameraName + "/" + K2_TOPIC_LORES_DEPTH K2_TOPIC_RAW;

        cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
        cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
        detection_pub=nh.advertise<DetectionArray>(output_detection_topic,3);

        std::string output_World3D_roi_topic = "/World3D_rois";
        world3D_rois_pub=nh.advertise<World3D_roi_array>(output_World3D_roi_topic,3);
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
                main_depth_16 = this->depth;
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
                        //                        start = std::chrono::high_resolution_clock::now();
                        background_removal();
                        //                        now = std::chrono::high_resolution_clock::now();
                        //                        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
                        //                        std::cout<<elapsed<<std::endl;
                    }
                }

                // if don't accept background_removal ||  background is already removed
                if(!use_background_removal||(use_background_removal&&background_calculate_frames==0))
                {
                    //!!!!!!!!!!!!!!!!!!!!!!!main loop!!!!!!!!!!!!!!!!!!!!!!!
                    if(objects_selected)// start to track after selecting objects
                    {

                        //                                             start = std::chrono::high_resolution_clock::now();
                        multiple_objects_detection_main();
                        //                                                now = std::chrono::high_resolution_clock::now();
                        //                                                double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
                        //                                                std::cout<<elapsed<<std::endl;


                        //draw tracks_2D (cotinuely 10 detections)
                        if(show_2D_tracks)
                        {
                            show_2D_tracks_on_image();
                        }
                    }
                }
            }
        }
    }

private:
    void start_reciver()
    {

        sub_image2D_rois = nh.subscribe("/kinect2_head/rgb_lowres/image2D_rois", 5, &Multiple_Objects_Detection::image2D_rois_Callback, this);
        sub_world3D_rois=  nh.subscribe("/World3D_rois", 5, &Multiple_Objects_Detection::world3D_rois_Callback, this);

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
            syncExact->registerCallback(boost::bind(&Multiple_Objects_Detection::image_callback, this, _1, _2, _3, _4));
        }
        else
        {
            syncApproximate = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
            syncApproximate->registerCallback(boost::bind(&Multiple_Objects_Detection::image_callback, this, _1, _2, _3, _4));
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

    //main_detection
    void multiple_objects_detection_main()
    {
        if( main_color.empty() )
            return;
        Object_Detector::setMainColor(main_color);

        if(Object_Detector::Backprojection_Mode=="HSD")
        {
            //          //devide (1000mm~9000mm) into 255 parts
            ushort Max=9000,Min=1000;
            main_depth_16.convertTo(main_depth_8, CV_8U,255.0/(Max-Min),-255.0*Min/(Max-Min));
            Object_Detector::setMainDepth(main_depth_8);
        }

        else
        {
            Object_Detector::setMainDepth(main_depth_16);
        }

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
                double current_center_depth=main_depth_16.at<ushort>(current_center2D_d.y,current_center2D_d.x)/1000.0;//meter ,not mm

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
                Rect test_Rect((*it).x+((*it).width)/2,(*it).y+((*it).height)/2,1,1);
                test_Rect=test_Rect&Rect(0,0,main_color.size().width,main_color.size().height);
                cv::rectangle(main_color, test_Rect, cv::Scalar(250*(i), 250*(i-1), 250*(i-2)), 2, CV_AA);
            }
        }
        cv::imshow( "show_2D_tracks", main_color );
        cv::waitKey(10);
    }

    void image_callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
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

    void image2D_rois_Callback(const opt_msgs::Image2D_roi_array::ConstPtr& image2D_roi_msg)
    {

        for( std::vector<opt_msgs::Image2D_roi>::const_iterator it = image2D_roi_msg->Rois.begin();
             it != image2D_roi_msg->Rois.end(); it++)
        {
            cv::Rect selection((*it).x,(*it).y,(*it).width,(*it).height);
            selection=selection&Rect(0,0,main_color.size().width,main_color.size().height);

            //////////////initialize one detector with one selection//////////////
            if(selection.area()>1)
            {
                Object_Detector newDetector;
                newDetector.setCurrentRect(selection);
                Object_Detectors.push_back(newDetector);
                current_detected_boxes.push_back(selection);
                std::list<Rect> tracks_2D_(10);
                tracks_2D.push_back(tracks_2D_);
                bool occlude_=false;
                occludes.push_back(occlude_);
                Object_Detector::current_detected_boxes=current_detected_boxes;
                //////////////~~initialize one detector with one selection//////////////


                //////////////project theselection from current_frame to other cameras//////////////
                //1. project from currentframe to world
                std::string world_frame_id="/world";
                std::string current_frame_id=color_header_frameId;
                tf::StampedTransform transform_current2world;
                try{
                    tf_listener.lookupTransform(world_frame_id, current_frame_id, ros::Time(0), transform_current2world);
                }
                catch(tf::TransformException ex){
                    ROS_ERROR("%s",ex.what());
                    ros::Duration(1.0).sleep();
                }
                Point2d current_frame_image2D_1(cvFloor(selection.x),cvFloor(selection.y));
                Point2d current_frame_image2D_2(cvFloor(selection.x+selection.width),cvFloor(selection.y+selection.height));
                std::cout<<"current_frame_image2D_11111111111111111: "<<current_frame_image2D_1.x<<"  "<<current_frame_image2D_1.y<<std::endl;
                std::cout<<"current_frame_image2D_22222222222222222: "<<current_frame_image2D_2.x<<"  "<<current_frame_image2D_2.y<<std::endl;

                double current_frame_depth_1=main_depth_16.at<ushort>(current_frame_image2D_1.y,current_frame_image2D_1.x)/1000.0;//meter ,not mm
                double current_frame_depth_2=main_depth_16.at<ushort>(current_frame_image2D_2.y,current_frame_image2D_2.x)/1000.0;//meter ,not mm

                tf::Vector3 world3D_1=image2D_to_world3D(current_frame_image2D_1,current_frame_depth_1,transform_current2world);
                tf::Vector3 world3D_2=image2D_to_world3D(current_frame_image2D_2,current_frame_depth_2,transform_current2world);

                //publish
                current_World3D_roi_msg.no=numberofrois++;
                current_World3D_roi_msg.x1=world3D_1.getX();
                current_World3D_roi_msg.y1=world3D_1.getY();
                current_World3D_roi_msg.z1=world3D_1.getZ();
                current_World3D_roi_msg.x2=world3D_2.getX();
                current_World3D_roi_msg.y2=world3D_2.getY();
                current_World3D_roi_msg.z2=world3D_2.getZ();
                World3D_rois_msg.Rois.push_back(current_World3D_roi_msg);

                //                cv::rectangle(main_color, selection, cv::Scalar(255, 0, 0), 2, CV_AA);
            }
        }
        cv::imwrite("/tmp/show_the_set_recs.png",main_color);
        objects_selected=true;
        std::cout<<Object_Detectors.size()<<" objects are selected"<<std::endl;


        world3D_rois_pub.publish(World3D_rois_msg);
        std::cout<<World3D_rois_msg.Rois.size()<<" World3D_rois are published"<<std::endl;
        World3D_rois_msg.Rois.clear();
    }


    //project form image2D coordinate of currentframe to world3D coordinate,
    // the intrisinc martrix comes from the "cameraMatrixColor"
    //two steps : image2D--camera3D--world3D
    tf::Vector3 image2D_to_world3D(Point2d i, double depth_, tf::StampedTransform transform_)
    {
        tf::Vector3 v;
        v.setX((i.x-cx)*fx*depth_);
        v.setY((i.y-cy)*fy*depth_);
        v.setZ(depth_);
        v = transform_(v);
        std::cout<<"world3D"<<v.getX()<<" "<<v.getY()<<" "<<v.getZ()<<std::endl;
        return v;
    }


    void world3D_rois_Callback(const opt_msgs::World3D_roi_array::ConstPtr& world3D_rois_msg)
    {
        if(publish_world3D_rois)
        {
            sub_world3D_rois.shutdown();
            return;
        }
        for( std::vector<opt_msgs::World3D_roi>::const_iterator it = world3D_rois_msg->Rois.begin();
             it != world3D_rois_msg->Rois.end(); it++)
        {
            tf::Vector3 world3D_1,world3D_2;
            world3D_1.setX((*it).x1);
            world3D_1.setY((*it).y1);
            world3D_1.setZ((*it).z1);
            world3D_2.setX((*it).x2);
            world3D_2.setY((*it).y2);
            world3D_2.setZ((*it).z2);

            tf::StampedTransform transform_world2projectedframe;
            std::string projected_frame_id=color_header_frameId;
            std::string world_frame_id= "/world";
            try{
                tf_listener.lookupTransform(projected_frame_id, world_frame_id, ros::Time(0), transform_world2projectedframe);
            }
            catch(tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
            Point2d projected_frame_image2D_1=world3D_to_image2D(world3D_1,transform_world2projectedframe);
            Point2d projected_frame_image2D_2=world3D_to_image2D(world3D_2,transform_world2projectedframe);
            std::cout<<"projected_frame_image2D_11111111111111111: "<<projected_frame_image2D_1.x<<"  "<<projected_frame_image2D_1.y<<std::endl;
            std::cout<<"projected_frame_image2D_22222222222222222: "<<projected_frame_image2D_2.x<<"  "<<projected_frame_image2D_2.y<<std::endl;

            cv::Rect selection(projected_frame_image2D_1,projected_frame_image2D_2);////////
            selection=selection&Rect(0,0,main_color.size().width,main_color.size().height);

            //////////////initialize one detector with one selection//////////////
            /// \brief newDetector
            if(selection.area()>1)
            {
                Object_Detector newDetector;
                newDetector.setCurrentRect(selection);
                Object_Detectors.push_back(newDetector);
                current_detected_boxes.push_back(selection);
                std::list<Rect> tracks_2D_(10);
                tracks_2D.push_back(tracks_2D_);
                bool occlude_=false;
                occludes.push_back(occlude_);
                Object_Detector::current_detected_boxes=current_detected_boxes;
                cv::rectangle(main_color, selection, cv::Scalar(255, 0, 0), 2, CV_AA);
            }

        }
        cv::imwrite("/tmp/show_the_project_recs.png",main_color);
        objects_selected=true;
        std::cout<<Object_Detectors.size()<<" objects are selected"<<std::endl;
    }
    Point2d world3D_to_image2D(tf::Vector3 v,tf::StampedTransform inverse_transform_)
    {
        v = inverse_transform_(v);
        //        std::cout<<"cam3D"<<v.getX()<<" "<<v.getY()<<" "<<v.getZ()<<std::endl;
        double depth_=v.getZ();
        Point2d i;
        i.x=cx+ (v.getX())/(fx*depth_);
        i.y=cy+ (v.getY())/(fy*depth_);
        return i;
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


