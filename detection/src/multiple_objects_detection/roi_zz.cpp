#include "open_ptrack/multiple_objects_detection/roi_zz.h"

void roi_zz::mouseHandler(int event, int x, int y, int flags, void *param){
    roi_zz *self =static_cast<roi_zz*>(param);
    self->opencv_mouse_callback(event,x,y,flags,param);
}

void roi_zz::opencv_mouse_callback( int event, int x, int y, int , void *param ){
   handlerT * data = (handlerT*)param;
  switch( event ){
    // update the selected bounding box
    case EVENT_MOUSEMOVE:
      if( data->isDrawing ){
        if(data->drawFromCenter){
          data->box.width = 2*(x-data->center.x)/*data->box.x*/;
          data->box.height = 2*(y-data->center.y)/*data->box.y*/;
          data->box.x = data->center.x-data->box.width/2.0;
          data->box.y = data->center.y-data->box.height/2.0;
        }else{
          data->box.width = x-data->box.x;
          data->box.height = y-data->box.y;
        }
      }
    break;

    // start to select the bounding box
    case EVENT_LBUTTONDOWN:
      data->isDrawing = true;
      data->box = cvRect( x, y, 0, 0 );
      data->center = Point2f((float)x,(float)y);
    break;

    // cleaning up the selected bounding box
    case EVENT_LBUTTONUP:
      data->isDrawing = false;
      if( data->box.width < 0 ){
        data->box.x += data->box.width;
        data->box.width *= -1;
      }
      if( data->box.height < 0 ){
        data->box.y += data->box.height;
        data->box.height *= -1;
      }
    break;
  }
}

Rect roi_zz::select(const cv::String& windowName, Mat img, bool showCrossair, bool fromCenter){

  key=0;

  // set the drawing mode
  selectorParams.drawFromCenter = fromCenter;

  // show the image and give feedback to user
  imshow(windowName,img);

  // copy the data, rectangle should be drawn in the fresh image
  selectorParams.image=img.clone();

  // select the object
  setMouseCallback( windowName, mouseHandler, (void *)&selectorParams );

  // end selection process on SPACE (32) ESC (27) or ENTER (13)
  while(!(key==32 || key==27 || key==13)){
    // draw the selected object
    rectangle(
      selectorParams.image,
      selectorParams.box,
      Scalar(255,0,0),2,1
    );

    // draw cross air in the middle of bounding box
    if(showCrossair){
      // horizontal line
      line(
        selectorParams.image,
        Point((int)selectorParams.box.x,(int)(selectorParams.box.y+selectorParams.box.height/2)),
        Point((int)(selectorParams.box.x+selectorParams.box.width),(int)(selectorParams.box.y+selectorParams.box.height/2)),
        Scalar(255,0,0),2,1
      );

      // vertical line
      line(
        selectorParams.image,
        Point((int)(selectorParams.box.x+selectorParams.box.width/2),(int)selectorParams.box.y),
        Point((int)(selectorParams.box.x+selectorParams.box.width/2),(int)(selectorParams.box.y+selectorParams.box.height)),
        Scalar(255,0,0),2,1
      );
    }

    // show the image bouding box
    imshow(windowName,selectorParams.image);

    // reset the image
    selectorParams.image=img.clone();

    //get keyboard event, extract lower 8 bits for scancode comparison
    key=waitKey(1) & 0xFF;
  }


  return selectorParams.box;
}

void roi_zz::select(const cv::String& windowName, Mat img, std::vector<Rect> & boundingBox, bool fromCenter){
  std::vector<Rect> box;
  Rect temp;
  key=0;

  // show notice to user
  std::cout<<"Select an object to track and then press SPACE or ENTER button!"<<std::endl ;
  std::cout<<"Finish the selection process by pressing ESC button!"<<std::endl;

  // while key is not ESC (27)
  for(;;) {
    temp=select(windowName, img, true, fromCenter);
    if(key==27) break;
    if(temp.width>0 && temp.height>0)
      box.push_back(temp);
  }
  boundingBox=box;
}

Rect roi_zz::select(Mat img, bool fromCenter){
  return select("ROI selector", img, fromCenter);
}
