#include "open_ptrack/multiple_objects_detection/object_detector.h"
#include <iostream>
// Static member definition ...
cv::Mat Object_Detector::mainColor;
cv::Mat Object_Detector::mainDepth;

void Object_Detector::setMainColor(const cv::Mat _mainColor)
{
    _mainColor.copyTo(mainColor);
}

cv::Mat Object_Detector::getMainColor()
{
    return mainColor;
}

void Object_Detector::setMainDepth(const cv::Mat _mainDepth)
{
    _mainDepth.copyTo(mainDepth);
}

cv::Mat Object_Detector::getMainDepth()
{
    return mainDepth;
}


void Object_Detector::setCurrentRect(const cv::Rect _currentRect)
{
    currentRect = _currentRect;
    selection = currentRect;
}

cv::Rect Object_Detector::getCurrentRect()
{
    return currentRect;
}

void Object_Detector::setcurrent_detected_boxes(std::vector<Rect> _current_detected_boxes)
{
    current_detected_boxes=_current_detected_boxes;
}

std::vector<Rect> Object_Detector::getcurrent_detected_boxes()
{
    return current_detected_boxes;
}

cv::RotatedRect Object_Detector::object_shift(InputArray _probColor,Rect& window, TermCriteria criteria)
{
    //  CV_INSTRUMENT_REGION();

    Size size;
    Mat mat;
    mat = _probColor.getMat(), size = mat.size();

    cv::meanShift( _probColor, window, criteria );

    window.x -= AREA_TOLERANCE;
    window.y -= AREA_TOLERANCE;
    window.width += 2 * AREA_TOLERANCE;
    window.height += 2 * AREA_TOLERANCE;
    window=window&Rect(0,0,size.width,size.height);

    // Calculating moments in new center mass
    //    Moments m = isUMat ? moments(umat(window)) : moments(mat(window));
    Moments m = moments(mat(window));


    double m00 = m.m00, m10 = m.m10, m01 = m.m01;
    double mu11 = m.mu11, mu20 = m.mu20, mu02 = m.mu02;



    //    // use 1 as the qulity to calculate the percentage of the object to the window
    //    Mat hsv_mask_window=hsv_mask(window);
    //    Moments m_hsv_mask_window = moments(hsv_mask_window);
    //    double m_hsv_mask_window00 = m_hsv_mask_window.m00;



    ////////////difference from cv::camshift//////////
    // occluded process
    if( fabs(m00) < QUALITY_TOLERANCE )
    {
        occluded=true;
        //        std::cout<<"totally occluded: window density "<<fabs(m_hsv_mask_window00)/window.area()<<std::endl;
        //        std::cout<<"totally occluded: window density "<<fabs(m00)/window.area()<<std::endl;
        //        std::cout<<"occluded: half_occluded_frames: "<<half_occluded_frames<<std::endl;

        return RotatedRect();
    }

    else if(fabs(m00)/window.area()<DENSITY_TOLORENCE)
        //    else if(fabs(m_hsv_mask_window00)/window.area()<DENSITY_TOLORENCE)

    {
        occluded=true;
        //        std::cout<<"half occluded: window density "<<fabs(m_hsv_mask_window00)/window.area()<<std::endl;
//        std::cout<<"half occluded: window density "<<fabs(m00)/window.area()<<std::endl;
        //        std::cout<<"half occluded: half_occluded_frames: "<<half_occluded_frames<<std::endl;

        //half occlude means when the detecteor detected the similar background ,but the desity is not big enough ,or just a little part of the object appears
        //if there are a little similar thing in the background ,the detector will keep the search window around this similar thing and nolonger expand
        //in this situation ,if the real object appear ,it maybe not in the search window,do it will never been detect untill it move to the small search window
        //so ,we set half_occluded_frames=10,if the detector keep the search window around this similar thing for more than 10 frames,we will assume this thing is not the object and than expand the search window to the whole image
        //if this little similar thing is really belong to the object ,it will converged in 10 frames,so 10 is enough

        if(--half_occluded_frames==0)//if there are a little similar thing in the background ,the detector will keep the window around that ,but it's desity is not big enough so the detector will not asuue
        {
            half_occluded_frames=10;
            window=Rect(0,0,size.width,size.height);
            //            std::cout<<"set window to the whole image: half_occluded_frames: "<<half_occluded_frames<<std::endl;
            return RotatedRect();
        }

    }
    else{
        occluded=false;
        half_occluded_frames=10;
        //        std::cout<<"not occluded: window density "<<fabs(m_hsv_mask_window00)/window.area()<<std::endl;

//        std::cout<<"not occluded: window density "<<fabs(m00)/window.area()<<std::endl;
    }
    ////////////difference from cv::camshift////////////


    double inv_m00 = 1. / m00;
    int xc = cvRound( m10 * inv_m00 + window.x );
    int yc = cvRound( m01 * inv_m00 + window.y );
    double a = mu20 * inv_m00, b = mu11 * inv_m00, c = mu02 * inv_m00;

    // Calculating width & height
    double square = std::sqrt( 4 * b * b + (a - c) * (a - c) );

    // Calculating orientation
    double theta = atan2( 2 * b, a - c + square );

    // Calculating width & length of figure
    double cs = cos( theta );
    double sn = sin( theta );

    double rotate_a = cs * cs * mu20 + 2 * cs * sn * mu11 + sn * sn * mu02;
    double rotate_c = sn * sn * mu20 - 2 * cs * sn * mu11 + cs * cs * mu02;
    double length = std::sqrt( rotate_a * inv_m00 ) * 4;
    double width = std::sqrt( rotate_c * inv_m00 ) * 4;

    // In case, when tetta is 0 or 1.57... the Length & Width may be exchanged
    if( length < width )
    {
        std::swap( length, width );
        std::swap( cs, sn );
        theta = CV_PI*0.5 - theta;
    }


    // Saving results
    int _xc = cvRound( xc );
    int _yc = cvRound( yc );

    int t0 = cvRound( fabs( length * cs ));
    int t1 = cvRound( fabs( width * sn ));

    t0 = MAX( t0, t1 ) + 2;
    window.width = MIN( t0, (size.width - _xc) * 2 );

    t0 = cvRound( fabs( length * sn ));
    t1 = cvRound( fabs( width * cs ));

    t0 = MAX( t0, t1 ) + 2;
    window.height = MIN( t0, (size.height - _yc) * 2 );

    window.x = MAX( 0, _xc - window.width / 2 );
    window.y = MAX( 0, _yc - window.height / 2 );

    window.width = MIN( size.width - window.x, window.width );
    window.height = MIN( size.height - window.y, window.height );

    RotatedRect box;
    box.size.height = (float)length;
    box.size.width = (float)width;
    box.angle = (float)((CV_PI*0.5+theta)*180./CV_PI);
    while(box.angle < 0)
        box.angle += 360;
    while(box.angle >= 360)
        box.angle -= 360;
    if(box.angle >= 180)
        box.angle -= 180;
    box.center = Point2f( window.x + window.width*0.5f, window.y + window.height*0.5f);

    return box;
}

cv::RotatedRect Object_Detector::detectCurrentRect(int id)
{


    //  cv::Mat Color;
    mainColor.copyTo(Color);

    cv::cvtColor(Color, hsv, CV_BGR2HSV);

    cv::inRange(hsv, cv::Scalar(HMin, SMin, MIN(VMin,VMax)),
                cv::Scalar(HMax, SMax, MAX(VMin, VMax)), hsv_mask);

    //used for just hue
    float h_ranges[] = {0,HMax};
    const float* ph_ranges = h_ranges;
    int h_channels[] = {0, 0};
    if(use_hs==false)
    {
        hue.create(hsv.size(), hsv.depth());
        cv::mixChannels(&hsv, 1, &hue, 1, h_channels, 1);
    }
    //used for just hue


    //used for  h s
    int hs_size[] = { h_bins, s_bins };
    float h_range[] = {HMin,HMax};
    float s_range[] = { SMin, SMax };
    const float* phs_ranges[] = { h_range, s_range };
    int channels[] = { 0, 1 };
    //used for  h s


    if( firstRun )
    {
        if(use_hs)
        {
            //h s rather than h
            cv::Mat roi(hsv, selection), maskroi(hsv_mask, selection);
            cv::calcHist(&roi, 1, channels, maskroi, hist, 2, hs_size, phs_ranges, true, false);
            cv::normalize(hist, hist, 0, 255, CV_MINMAX);
        }
        else{
            //just hue
            cv::Mat roi(hue, selection), maskroi(hsv_mask, selection);
            cv::calcHist(&roi, 1, 0, maskroi, hist, 1, &h_bins, &ph_ranges);
            cv::normalize(hist, hist, 0, 255, CV_MINMAX);
        }

        detectWindow = selection;
        firstRun = false;
    }


    if(use_hs)
        cv::calcBackProject( &hsv, 1, channels, hist, backproj, phs_ranges, 1, true );
    else
        cv::calcBackProject(&hue, 1, 0, hist, backproj, &ph_ranges);


//     imshow("backproj first",backproj);
    // calculate the other_object_mask //where to put this ,slove bling bling
    other_object_mask=Mat::ones(Color.size(), CV_8U)*255;
    for (int i=0; i<current_detected_boxes.size(); i++)
    {
        if(i!=id)
        {
            uchar tmp =0;
            Rect current_tracked_box =current_detected_boxes[i];
            //                current_tracked_box.x=current_tracked_box.x-10;
            //                current_tracked_box.y=current_tracked_box.y-10;
            //                current_tracked_box.width=current_tracked_box.width+20;
            //                current_tracked_box.height=current_tracked_box.height+20;
            current_tracked_box=current_tracked_box&Rect(0,0,Color.size().width,Color.size().height);
            other_object_mask(current_tracked_box)=tmp;
        }
    }

    detectWindow=detectWindow&Rect(0,0,hsv.size().width,hsv.size().height);
    if(occluded==false&&detectWindow.area()>1)
    {
        //use x y to generate position_mask
        position_mask=Mat::zeros(Color.size(),CV_8UC1);
        // creat a bigger search search_window based on detectWindow
        int detectWindow_XL=detectWindow.x-AREA_TOLERANCE,detectWindow_XR=detectWindow.x+detectWindow.width+AREA_TOLERANCE;
        int detectWindow_YT=detectWindow.y-AREA_TOLERANCE,detectWindow_YB=detectWindow.y+detectWindow.height+AREA_TOLERANCE;
        Rect search_window=Rect(detectWindow_XL,detectWindow_YT,detectWindow_XR-detectWindow_XL,detectWindow_YB-detectWindow_YT)&Rect(0,0,Color.size().width,Color.size().height);
        position_mask(search_window)=255;

//        // calculate the other_object_mask
//        other_object_mask=Mat::ones(Color.size(), CV_8U)*255;
//        for (int i=0; i<current_detected_boxes.size(); i++)
//        {
//            if(i!=id)
//            {
//                uchar tmp =0;
//                Rect current_tracked_box =current_detected_boxes[i];
//                //                current_tracked_box.x=current_tracked_box.x-10;
//                //                current_tracked_box.y=current_tracked_box.y-10;
//                //                current_tracked_box.width=current_tracked_box.width+20;
//                //                current_tracked_box.height=current_tracked_box.height+20;
//                current_tracked_box=current_tracked_box&Rect(0,0,Color.size().width,Color.size().height);
//                other_object_mask(current_tracked_box)=tmp;
//            }
//        }
        position_mask &= hsv_mask;
        backproj &= position_mask;
        backproj &= other_object_mask;
    }
    else{
        backproj &= hsv_mask;
        backproj &= other_object_mask;
    }

    cv::RotatedRect tmp_detectedBox;
    if(detectWindow.area()>1)
        tmp_detectedBox = object_shift(backproj, detectWindow,
                                       cv::TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));
//    cv::rectangle(backproj, detectWindow, cv::Scalar(255, 0, 0), 2, CV_AA);
//    imshow("backproj final",backproj);
    return tmp_detectedBox;

}
