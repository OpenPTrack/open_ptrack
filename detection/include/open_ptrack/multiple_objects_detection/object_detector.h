#ifndef Object_Detector_H
#define Object_Detector_H
//#include <QMessageBox>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"
using namespace cv;
class Object_Detector
{
public:
    bool occluded;
    double half_occluded_frames;

    //    ///////////For hsv mask///////////
    static int HMin, SMin, VMin;
    static int HMax,SMax, VMax;
    //    ///////////For hsv mask///////////

    //    ///////////For camshift///////////
    static bool use_hs;//if  use_hs==true, use hue and s to generate the hitograme to do the bacprojection , if not, just used the hue image
    static int h_bins,s_bins;//for the hue backrojection hist ,divide (HMin,HMax) to h_bins parts
    //    ///////////For camshift///////////



    ///////////For camshift recover from occlusion///////////
    //for camshift, for the window in backproject image,the value represent the similirity with the ROI. the value of the pixel is equal to 255*percentage ,the percentage come from the histogram(problility)
    // QUALITY_TOLERANCE here means the TOLERANCE of sum of the window in backproject image,it represent the quality of the window.
    // DENSITY_TOLORENCE here means the TOLERANCE of the DENSITY,DENSITY=QUALITY/window.area();
    // AREA_TOLERANCE here means the TOLERANCE of the x,y ,window.it means the objects maybe apprear in the next frame ,in the window which increase it's size by AREA_TOLERANCE every frame.so the bigger AREA_TOLERANCE,the objects can move faster.
    // if the QUALITY<QUALITY_TOLERANCE, the object is "totally occluded",
    // then when QUALITY<QUALITY_TOLERANCE, it will appeare, but at first ,not totaly appear,the window may be much bigger then the size of object,
    // then we calculate the DENSITY,if DENSITY<DENSITY_TOLORENCE.it means the window not just included the object,and the objects are "half occluded"
    // then the camshift will decrese the size of the window,untill DENSITY>DENSITY_TOLORENCE,it will be "not occluded";
    static int AREA_TOLERANCE;//AREA_TOLERANCE is also used to create the position_maks
    static int QUALITY_TOLERANCE;
    static double DENSITY_TOLORENCE;
    ///////////For camshift recover from occlusion///////////

    static std::vector<Rect> current_detected_boxes;//this one come from class:multiple_objects_detection, every time of every object's detection ,multiple_objects_detection will update this varable ,we use it to generate the other_object_mask

private:
    static cv::Mat mainColor;
    static cv::Mat mainDepth;


    bool firstRun;
    cv::Rect currentRect;

    cv::Rect detectWindow;
    cv::Mat hsv, hue, hsv_mask, hist, backproj; //, histimg;


    cv::Rect selection;
    cv::Mat Color,Depth;

    Mat other_object_mask,position_mask,depth_mask;


public:
    Object_Detector()
        :firstRun(true),occluded(false),half_occluded_frames(10)
    {
    }

    static void setMainColor(const cv::Mat _mainColor);
    static cv::Mat getMainColor();

    static void setMainDepth(const cv::Mat _mainDepth);
    static cv::Mat getMainDepth();

    void setCurrentRect(const cv::Rect _currentRect);
    cv::Rect getCurrentRect();

    void setcurrent_detected_boxes(std::vector<Rect> _current_detected_boxes);
    std::vector<Rect>  getcurrent_detected_boxes();

    cv::RotatedRect object_shift(InputArray _probColor,Rect& window, TermCriteria criteria);

    cv::RotatedRect detectCurrentRect(int id);
};

//stastic varable defination
int Object_Detector::HMax;
int Object_Detector::SMax;
int Object_Detector::VMax;
int Object_Detector::HMin;
int Object_Detector::SMin;
int Object_Detector::VMin;

bool Object_Detector::use_hs;
int Object_Detector::h_bins;
int Object_Detector::s_bins;

int Object_Detector::AREA_TOLERANCE;
int Object_Detector::QUALITY_TOLERANCE;
double Object_Detector::DENSITY_TOLORENCE;

std::vector<Rect> Object_Detector::current_detected_boxes;
#endif // Object_Detector_H
