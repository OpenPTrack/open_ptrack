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


void Object_Detector::setObjectName(const std::string _object_name)
{
    object_name=_object_name;
}
std::string Object_Detector::getObjectName()
{
    return object_name;
}


void Object_Detector::setcurrent_detected_boxes(std::vector<Rect> _current_detected_boxes)
{
    current_detected_boxes=_current_detected_boxes;
}

std::vector<Rect> Object_Detector::getcurrent_detected_boxes()
{
    return current_detected_boxes;
}

void Object_Detector::H_backprojection()
{
    float h_ranges[] = {0,(float)HMax};
    const float* ph_ranges = h_ranges;
    int h_channels[] = {0, 0};
    hue.create(hsv.size(), hsv.depth());
    cv::mixChannels(&hsv, 1, &hue, 1, h_channels, 1);

    if( firstRun )
    {
        if(!occluded)
        {
        cv::Mat roi(hue, selection), maskroi(hsv_mask, selection);
        cv::calcHist(&roi, 1, 0, maskroi, h_hist, 1, &h_bins, &ph_ranges);
        cv::normalize(h_hist, h_hist, 0, 255, CV_MINMAX);
        detectWindow = selection;
        firstRun = false;
        //        std::cout<<"H mode"<<std::endl;
        }
        else{
            Mat roi_color= roi_from_file;
//            cv::imshow("roi from file",roi_color);
//            cv::waitKey(30);
            Mat roi_hsv,roi_hsv_mask;
            cv::cvtColor(roi_color, roi_hsv, CV_BGR2HSV);
            cv::inRange(roi_hsv, cv::Scalar(HMin, SMin, MIN(VMin,VMax)),
                        cv::Scalar(HMax, SMax, MAX(VMin, VMax)), roi_hsv_mask);
            cv::calcHist(&roi_hsv, 1, h_channels, roi_hsv_mask, h_hist, 1, &h_bins, &ph_ranges);
            cv::normalize(h_hist, h_hist, 0, 255, CV_MINMAX);
            detectWindow = selection;
            firstRun = false;
        }

    }
    cv::calcBackProject(&hue, 1, 0, h_hist, backproj, &ph_ranges,1,true);
}

void Object_Detector::HS_backprojection()
{
    int hs_size[] = { h_bins, s_bins };
    float h_range[] = {(float)HMin,(float)HMax};
    float s_range[] = { (float)SMin, (float)SMax };
    const float* phs_ranges[] = { h_range, s_range };
    int hs_channels[] = { 0, 1 };

    if( firstRun )
    {
        if(!occluded)
        {
            cv::Mat roi(hsv, selection), maskroi(hsv_mask, selection);
            cv::calcHist(&roi, 1, hs_channels, maskroi, hs_hist_for_HS, 2, hs_size, phs_ranges, true, false);
            cv::normalize(hs_hist_for_HS, hs_hist_for_HS, 0, 255, CV_MINMAX);
            detectWindow = selection;
            firstRun = false;
            //std::cout<<"HS mode"<<std::endl;
        }
        else{
            Mat roi_color= roi_from_file;
//            cv::imshow("roi from file",roi_color);
//            cv::waitKey(30);
            Mat roi_hsv,roi_hsv_mask;
            cv::cvtColor(roi_color, roi_hsv, CV_BGR2HSV);
            cv::inRange(roi_hsv, cv::Scalar(HMin, SMin, MIN(VMin,VMax)),
                        cv::Scalar(HMax, SMax, MAX(VMin, VMax)), roi_hsv_mask);
            cv::calcHist(&roi_hsv, 1, hs_channels, roi_hsv_mask, hs_hist_for_HS, 2, hs_size, phs_ranges, true, false);
            cv::normalize(hs_hist_for_HS , hs_hist_for_HS , 0, 255, CV_MINMAX);
            detectWindow = selection;
            firstRun = false;
        }
    }
    cv::calcBackProject( &hsv, 1, hs_channels, hs_hist_for_HS, backproj, phs_ranges, 1, true );

}

void Object_Detector::HSD_backprojection()
{

    const int hsd_size[] = { h_bins, s_bins ,d_bins};
    const int hs_size[] =  { h_bins, s_bins };

    float h_range[] = { (float)HMin, (float)HMax };
    float s_range[] = { (float)SMin, (float)SMax };
    float d_range[] = { 0, 255 };

    const float* pd_ranges= d_range;
    const float* phs_ranges[] = { h_range, s_range };
    const float* phsd_ranges[] = {h_range, s_range ,d_range };

    int hs_channels[] = { 0, 1 };
    int hsd_channels[] = {0,1,2};
    int hsd_channels_formix[] = {0, 2};

    cv::mixChannels(&Depth, 1, &hsv, 3, hsd_channels_formix, 1);//hsv-->hsd

    if( firstRun )
    {
        if(!occluded)//if the roi is from the file, the first frame will be set to occluded, in this situation,we just calculate the hs pdf and use it to search the object
        {
            cv::Mat roi(hsv, selection), maskroi(hsv_mask, selection);
            cv::calcHist(&roi, 1, hs_channels, maskroi, hs_hist_for_HSD, 2, hs_size, phs_ranges, true, false);
            cv::normalize(hs_hist_for_HSD , hs_hist_for_HSD , 0, 255, CV_MINMAX);

            double sum_hs_hist=sum(hs_hist_for_HSD)[0];
            hs_hist_pdf=hs_hist_for_HSD/sum_hs_hist;

            //used to generate the initial hsd_hist(use this just for the right data format )
            cv::calcHist(&roi, 1, hsd_channels, maskroi, hsd_hist, 3, hsd_size, phsd_ranges, true, false);

            cv::Mat roi_depth(Depth, selection);

            //calculate the the current_trackBox(rotatedrect) mask,named depth_mask(in this mask ,just the the value in the area :current_detectBox(rotatedrect) is 255)
            Point2f vertices[4];
            current_detectedBox.points(vertices);
            std::vector< std::vector<Point> >  co_ordinates;
            co_ordinates.push_back(std::vector<Point>());
            co_ordinates[0].push_back(vertices[0]);
            co_ordinates[0].push_back(vertices[1]);
            co_ordinates[0].push_back(vertices[2]);
            co_ordinates[0].push_back(vertices[3]);
            depth_mask=Mat::zeros(Color.size(),CV_8UC1);
            drawContours( depth_mask,co_ordinates,0, Scalar(255),CV_FILLED, 8 );
            depth_mask&=hsv_mask;
            cv::Mat  depth_mask_roi(depth_mask, detectWindow);
            cv::calcHist(&roi_depth, 1, 0, depth_mask_roi, tmp_depth_hist_pdf, 1, &d_bins, &pd_ranges);
            double sum_tmp_depth_hist_pdf=sum(tmp_depth_hist_pdf)[0];
            tmp_depth_hist_pdf=tmp_depth_hist_pdf/sum_tmp_depth_hist_pdf;


            //combine the hs and depth
            for (int i=0; i<hsd_size[0]; i++) {
                for (int j=0; j<hsd_size[1]; j++) {
                    for (int k=0; k<hsd_size[2]; k++) {
                        hsd_hist.at<float>(i,j,k)=255*hs_hist_pdf.at<float>(i,j)*tmp_depth_hist_pdf.at<float>(k);
                    }
                }
            }

            //normalize hsd_hist
            double hMin,hMax;
            minMaxIdx(hsd_hist, &hMin, &hMax);
            hsd_hist = 255 * hsd_hist / hMax;

            detectWindow = selection;
            firstRun = false;
        }
        else{ // if we get the roi from file ,we just calculate the hs pdf

            Mat roi_color= roi_from_file;
            Mat roi_hsv,roi_hsv_mask;
            cv::cvtColor(roi_color, roi_hsv, CV_BGR2HSV);
            cv::inRange(roi_hsv, cv::Scalar(HMin, SMin, MIN(VMin,VMax)),
                        cv::Scalar(HMax, SMax, MAX(VMin, VMax)), roi_hsv_mask);
            //            imshow("roi_hsv_mask",roi_hsv_mask);
            //            cv::waitKey(10);

            cv::calcHist(&roi_hsv, 1, hs_channels, roi_hsv_mask, hs_hist_for_HSD, 2, hs_size, phs_ranges, true, false);
            cv::normalize(hs_hist_for_HSD , hs_hist_for_HSD , 0, 255, CV_MINMAX);

            double sum_hs_hist=sum(hs_hist_for_HSD)[0];
            hs_hist_pdf=hs_hist_for_HSD/sum_hs_hist;

            //used to generate the initial hsd_hist(use this just for the right data format )
            cv::calcHist(&roi_hsv, 1, hsd_channels, roi_hsv_mask, hsd_hist, 3, hsd_size, phsd_ranges, true, false);

            detectWindow = selection;
            firstRun = false;
        }
    }

    else//main loop to get the hsd pdf, just update the depth pdf , and combine it with the initial hs pdf
    {
        if(!occluded&&!half_occluded)
        {
            cv::Mat roi_depth(Depth, detectWindow);

            //calculate the the current_trackBox(rotatedrect) mask,named depth_mask(in this mask ,just the the value in the area :current_detectBox(rotatedrect) is 255)
            Point2f vertices[4];
            current_detectedBox.points(vertices);
            std::vector< std::vector<Point> >  co_ordinates;
            co_ordinates.push_back(std::vector<Point>());
            co_ordinates[0].push_back(vertices[0]);
            co_ordinates[0].push_back(vertices[1]);
            co_ordinates[0].push_back(vertices[2]);
            co_ordinates[0].push_back(vertices[3]);

            depth_mask=Mat::zeros(Color.size(),CV_8UC1);
            drawContours( depth_mask,co_ordinates,0, Scalar(255),CV_FILLED, 8 );
            depth_mask&=hsv_mask;

            cv::Mat  depth_mask_roi(depth_mask, detectWindow);

            cv::calcHist(&roi_depth, 1, 0, depth_mask_roi, tmp_depth_hist_pdf, 1, &d_bins, &pd_ranges);
            double sum_tmp_depth_hist_pdf=sum(tmp_depth_hist_pdf)[0];
            tmp_depth_hist_pdf=tmp_depth_hist_pdf/sum_tmp_depth_hist_pdf;

            //////////////////////cost a lot of time  //////////////////////
            for (int i=0; i<hsd_size[0]; i++) {
                for (int j=0; j<hsd_size[1]; j++) {
                    for (int k=0; k<hsd_size[2]; k++) {
                        hsd_hist.at<float>(i,j,k)=255*hs_hist_pdf.at<float>(i,j)*tmp_depth_hist_pdf.at<float>(k);
                    }
                }
            }
            //////////////////////rcost a lot of time  //////////////////////

            double hMin,hMax;
            minMaxIdx(hsd_hist, &hMin, &hMax);
            hsd_hist = 255 * hsd_hist / hMax;
        }
    }


    if(!occluded&&!half_occluded)//if not occluded, use hsd pdf
    {
        cv::calcBackProject( &hsv, 1, hsd_channels, hsd_hist, backproj, phsd_ranges, 1, true );
    }
    else//if occluded, use hs pdf
    {
        cv::calcBackProject( &hsv, 1, hs_channels, hs_hist_for_HSD, backproj, phs_ranges, 1, true );
    }

}

//camshift + occlusion handle
cv::RotatedRect Object_Detector::object_shift(InputArray _probColor,Rect& window, TermCriteria criteria)
{
    Size size;
    Mat mat;
    mat = _probColor.getMat(), size = mat.size();
    cv::meanShift( _probColor, window, criteria );
    //    std::cout<<"real QUALITY_TOLERANCE"<<QUALITY_TOLERANCE<<std::endl;
    window.x -= AREA_TOLERANCE;
    window.y -= AREA_TOLERANCE;
    window.width += 2 * AREA_TOLERANCE;
    window.height += 2 * AREA_TOLERANCE;
    window=window&Rect(0,0,size.width,size.height);

    // Calculating moments in new center mass
    //  Moments m = isUMat ? moments(umat(window)) : moments(mat(window));
    Moments m = moments(mat(window));
    double m00 = m.m00, m10 = m.m10, m01 = m.m01;
    double mu11 = m.mu11, mu20 = m.mu20, mu02 = m.mu02;



    ////////////difference from cv::camshift//////////
    // occluded process
    //    QUALITY_TOLERANCE=3000;
    if( fabs(m00) < QUALITY_TOLERANCE )
    {
        occluded=true;
        //        std::cout<<"totally occluded: window QUALITY:  "<<fabs(m00)<<std::endl;

        //        std::cout<<"totally occluded: window density "<<fabs(m_hsv_mask_window00)/window.area()<<std::endl;
        //                std::cout<<"totally occluded: window density "<<fabs(m00)/window.area()<<std::endl;
        //        std::cout<<"occluded: half_occluded_frames: "<<half_occluded_frames<<std::endl;

        return RotatedRect();
    }

    else if(fabs(m00)/window.area()<DENSITY_TOLORENCE)
        //    else if(fabs(m_hsv_mask_window00)/window.area()<DENSITY_TOLORENCE)

    {
        occluded=true;
        half_occluded=true;
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
            //          std::cout<<"set window to the whole image: half_occluded_frames: "<<half_occluded_frames<<std::endl;
            return RotatedRect();
        }
    }
    else{
        occluded=false;
        half_occluded=false;
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

    mainColor.copyTo(Color);
    mainDepth.copyTo(Depth);

    cv::cvtColor(Color, hsv, CV_BGR2HSV);

    //calculate the hsv_mask by the range
    cv::inRange(hsv, cv::Scalar(HMin, SMin, MIN(VMin,VMax)), cv::Scalar(HMax, SMax, MAX(VMin, VMax)), hsv_mask);

    if(Backprojection_Mode=="H")
    {
        H_backprojection();
    }
    else if(Backprojection_Mode=="HS")
    {
        HS_backprojection();
    }
    else
    {
        HSD_backprojection();
    }
    //    imshow("backproj first",backproj);


    // calculate the other_object_mask with the current_detected_boxes
    other_object_mask=Mat::ones(Color.size(), CV_8U)*255;
    for (int i=0; i<current_detected_boxes.size(); i++)
    {
        if(i!=id)
        {
            uchar tmp =0;
            Rect current_tracked_box =current_detected_boxes[i];
            current_tracked_box=current_tracked_box&Rect(0,0,Color.size().width,Color.size().height);
            other_object_mask(current_tracked_box)=tmp;
        }
    }


    detectWindow=detectWindow&Rect(0,0,hsv.size().width,hsv.size().height);
    if(occluded==false&&detectWindow.area()>1)
    {
        //use x y to generate position_mask,the object can move in the window which is AREA_TOLERANCE size bigger than the last detected window
        position_mask=Mat::zeros(Color.size(),CV_8UC1);
        int detectWindow_XL=detectWindow.x-AREA_TOLERANCE,detectWindow_XR=detectWindow.x+detectWindow.width+AREA_TOLERANCE;
        int detectWindow_YT=detectWindow.y-AREA_TOLERANCE,detectWindow_YB=detectWindow.y+detectWindow.height+AREA_TOLERANCE;
        Rect search_window=Rect(detectWindow_XL,detectWindow_YT,detectWindow_XR-detectWindow_XL,detectWindow_YB-detectWindow_YT)&Rect(0,0,Color.size().width,Color.size().height);
        position_mask(search_window)=255;
        position_mask &= hsv_mask;
        backproj &= position_mask;
        backproj &= other_object_mask;
    }
    else{
        backproj &= hsv_mask;
        backproj &= other_object_mask;
    }

    if(detectWindow.area()>1)
        current_detectedBox = object_shift(backproj, detectWindow,
                                           cv::TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));

    //    cv::rectangle(backproj, detectWindow, cv::Scalar(255, 0, 0), 2, CV_AA);
    //    imshow("backproj final",backproj);

    return current_detectedBox;

}

