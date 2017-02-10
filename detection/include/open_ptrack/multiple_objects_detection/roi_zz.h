#ifndef ROI_ZZ_H
#define ROI_ZZ_H

//#include "opencv2/core/utility.hpp"

#include <opencv2/video/video.hpp>



#include "opencv2/ocl/ocl.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"

#include "opencv2/imgproc/types_c.h"
//#include "feature.hpp"
//#include "onlineMIL.hpp"
//#include "onlineBoosting.hpp"

using namespace cv;

//class roi_zz
//{
//public:
//    roi_zz();
//};



class roi_zz {
public:
    Rect select(Mat img, bool fromCenter = true);
    Rect select(const cv::String& windowName, Mat img, bool showCrossair = true, bool fromCenter = true);
    void select(const cv::String& windowName, Mat img, std::vector<Rect> & boundingBox, bool fromCenter = true);

    struct handlerT{
        // basic parameters
        bool isDrawing;
        Rect box;
        Mat image;

        // parameters for drawing from the center
        bool drawFromCenter;
        Point2f center;

        // initializer list
        handlerT() : isDrawing(false), drawFromCenter(true) {};
    }selectorParams;

    // to store the tracked objects
    std::vector<handlerT> objects;

private:
    static void mouseHandler(int event, int x, int y, int flags, void *param);
    void opencv_mouse_callback(int event, int x, int y, int, void *param);

    // save the keypressed characted
    int key;
};









#endif // ROI_ZZ_H
