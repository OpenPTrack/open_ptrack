#include <iostream>
#include <cstring>
#include <ctime>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <dynamic_reconfigure/server.h>
#include <detection/multiple_objects_detectionConfig.h>
#include "open_ptrack/multiple_objects_detection/multiple_objects_detection.h"

using namespace std;
using namespace cv;


//Parms for receiving color and depth image
std::string cameraName;
bool useExact = false;
bool useCompressed = false;


std::string output_detection_topic;

bool set_object_names;//if to use this detector to set the object names

//Parms for use_background_removal
bool use_background_removal;
int background_calculate_frames;
int threshold_4_detecting_foreground;//threshold of depth difference,use the difference to ditinguish the background and foreground

bool show_2D_tracks;//show 2d tracks or not

void configCb(detection::multiple_objects_detectionConfig &config, uint32_t level)
{
    Object_Detector::HMin=config.HMin;
    Object_Detector::HMin=config.HMin;
    Object_Detector::SMin=config.SMin;
    Object_Detector::VMin=config.VMin;
    Object_Detector::HMax=config.HMax;
    Object_Detector::SMax=config.HMax;
    Object_Detector::VMax=config.VMax;
    Object_Detector::h_bins=config.h_bins;
    Object_Detector::s_bins=config.s_bins;
    Object_Detector::d_bins=config.d_bins;
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

    nh.param("use_background_removal",use_background_removal,false);
    nh.param("background_calculate_frames",background_calculate_frames,100);
    nh.param("threshold_4_detecting_foreground",threshold_4_detecting_foreground,100);
    nh.param("show_2D_track",show_2D_tracks,false);
    nh.param("output_detection_topic",output_detection_topic,std::string("/objects_detector/detections"));
    nh.param("set_object_names",set_object_names,false);


    //set the stastic varables of object_detector class,the instruction of the varable can be found in the header of the class
    nh.param("HMin",Object_Detector::HMin,0);
    nh.param("SMin",Object_Detector::SMin,80);
    nh.param("VMin",Object_Detector::VMin,100);
    nh.param("HMax",Object_Detector::HMax,360);
    nh.param("SMax",Object_Detector::SMax,256);
    nh.param("VMax",Object_Detector::VMax,256);
    nh.param("h_bins",Object_Detector::h_bins,36);
    nh.param("s_bins",Object_Detector::s_bins,18);
    nh.param("d_bins",Object_Detector::d_bins,100);

    nh.param("AREA_TOLERANCE",Object_Detector::AREA_TOLERANCE,20);
    nh.param("QUALITY_TOLERANCE",Object_Detector::QUALITY_TOLERANCE,20000);
    nh.param("DENSITY_TOLORENCE",Object_Detector::DENSITY_TOLORENCE,4.0);

    nh.param("Backprojection_Mode",Object_Detector::Backprojection_Mode,std::string("HSD"));


    //  std::cout << "output detection topic: " << output_detection_topic << std::endl;

    std::cout << "HSV range:"<< std::endl
              << "H: (" << Object_Detector::HMin <<"~"<< Object_Detector::HMax <<")"<< std::endl
              << "S: (" << Object_Detector::SMin <<"~"<< Object_Detector::SMax <<")"<< std::endl
              << "V: (" << Object_Detector::VMin <<"~"<< Object_Detector::VMax <<")"<< std::endl
              << std::endl;


    std::cout << "Occlusion Parms:"<< std::endl
              <<"AREA_TOLERANCE: "<<Object_Detector::AREA_TOLERANCE<< std::endl
             <<"QUALITY_TOLERANCE: "<<Object_Detector::QUALITY_TOLERANCE<< std::endl
            <<"DENSITY_TOLORENCEE: "<<Object_Detector::DENSITY_TOLORENCE<< std::endl
           << std::endl;

    std::cout << "Backprojection_Mode: "<<Object_Detector::Backprojection_Mode<< std::endl;


    // Set up dynamic reconfiguration
    dynamic_reconfigure::Server<detection::multiple_objects_detectionConfig> server;
    dynamic_reconfigure::Server<detection::multiple_objects_detectionConfig>::CallbackType f;
    f = boost::bind(&configCb, _1, _2);
    server.setCallback(f);


    // main class for detection
    Multiple_Objects_Detection _multiple_objects_detection(output_detection_topic,set_object_names,useExact, useCompressed,
                                                           use_background_removal,background_calculate_frames,threshold_4_detecting_foreground,show_2D_tracks);

    std::cout << "start detecting..." << std::endl;
    _multiple_objects_detection.run_detection();

    ros::shutdown();
    return 0;
}
