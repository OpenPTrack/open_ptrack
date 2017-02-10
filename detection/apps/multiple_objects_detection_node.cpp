#include <iostream>
#include <cstring>
#include <ctime>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <dynamic_reconfigure/server.h>
#include "open_ptrack/multiple_objects_detection/multiple_objects_detectionConfig.h"
#include "open_ptrack/multiple_objects_detection/multiple_objects_detection.h"

using namespace std;
using namespace cv;

//typedef dynamic_reconfigure::Server<Config> ReconfigureServer;

std::string cameraName;
bool useExact = false;
bool useCompressed = false;

std::string output_detection_topic;

bool use_background_removal;
int background_calculate_frames;
int threshold_4_detecting_foreground;//threshold of depth difference,use the difference to ditinguish the background and foreground
bool show_2D_tracks;


void configCb(multiple_objects_detection::multiple_objects_detectionConfig &config, uint32_t level)
{
    Object_Detector::HMin=config.HMin;
    Object_Detector::HMin=config.HMin;
    Object_Detector::SMin=config.SMin;
    Object_Detector::VMin=config.VMin;
    Object_Detector::HMax=config.HMax;
    Object_Detector::SMax=config.HMax;
    Object_Detector::VMax=config.VMax;
    Object_Detector::use_hs=config.use_hs;
    Object_Detector::h_bins=config.h_bins;
    Object_Detector::s_bins=config.s_bins;
    Object_Detector::AREA_TOLERANCE=config.AREA_TOLERANCE;
    Object_Detector::QUALITY_TOLERANCE=config.QUALITY_TOLERANCE;
    Object_Detector::DENSITY_TOLORENCE=config.DENSITY_TOLORENCE;
}


int main(int argc, char** argv){

    ros::init(argc, argv, "multiple_objects_detection");
    ros::NodeHandle nh("~");

    if(!ros::ok())
    {
        return 0;
    }

    nh.param("sensor_name", cameraName, std::string("kinect2_head"));
    nh.param("use_background_removal",use_background_removal,false);
    nh.param("background_calculate_frames",background_calculate_frames,100);
    nh.param("threshold_4_detecting_foreground",threshold_4_detecting_foreground,100);
    nh.param("show_2D_track",show_2D_tracks,false);
    nh.param("output_detection_topic",output_detection_topic,std::string("/objects_detector/detections"));



    //set the stastic varables of object_detector class,the instruction of the varable can be found in the header of the class
    nh.param("HMin",Object_Detector::HMin,0);
    nh.param("SMin",Object_Detector::SMin,80);
    nh.param("VMin",Object_Detector::VMin,100);
    nh.param("HMax",Object_Detector::HMax,360);
    nh.param("SMax",Object_Detector::SMax,256);
    nh.param("VMax",Object_Detector::VMax,256);
    nh.param("use_hs",Object_Detector::use_hs,true);
    nh.param("h_bins",Object_Detector::h_bins,36);
    nh.param("s_bins",Object_Detector::s_bins,18);
    nh.param("AREA_TOLERANCE",Object_Detector::AREA_TOLERANCE,20);
    nh.param("QUALITY_TOLERANCE",Object_Detector::QUALITY_TOLERANCE,20000);
    nh.param("DENSITY_TOLORENCE",Object_Detector::DENSITY_TOLORENCE,4.0);


    //  std::cout << "output detection topic: " << output_detection_topic << std::endl;


    // Set up dynamic reconfiguration
    dynamic_reconfigure::Server<multiple_objects_detection::multiple_objects_detectionConfig> server;
    dynamic_reconfigure::Server<multiple_objects_detection::multiple_objects_detectionConfig>::CallbackType f;
    f = boost::bind(&configCb, _1, _2);
    server.setCallback(f);


    Multiple_Objects_Detection _multiple_objects_detection(cameraName,output_detection_topic,useExact, useCompressed,
                                                           use_background_removal,background_calculate_frames,threshold_4_detecting_foreground,show_2D_tracks);

    std::cout << "start detecting..." << std::endl;
    _multiple_objects_detection.run_detection();

    ros::shutdown();
    return 0;
}
