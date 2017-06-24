#ifndef OPT_Streaming_Component_H
#define OPT_Streaming_Component_H

//QT
#include <QQuickPaintedItem>
#include <QImage>
#include <QPainter>

//ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opt_msgs/Image2D_roi.h>
#include <opt_msgs/Image2D_roi_array.h>


#include <opt_msgs/Image2D_roi_file.h>

//OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"


class OPT_Streaming_Component : public QQuickPaintedItem {
  //make this a Qt Widget
  Q_OBJECT
  // defines a qml value for the topic
  Q_PROPERTY(QString topic READ get_topic WRITE set_topic NOTIFY topic_changed)
  Q_PROPERTY(QString rois_dir_path READ get_rois_dir_path WRITE set_rois_dir_path NOTIFY rois_dir_path_changed)
  Q_PROPERTY(QString all_rois_dir_path READ get_all_rois_dir_path)


public:
  // Constructor, takes parent widget, which defaults to null
  OPT_Streaming_Component(QQuickItem * parent = 0);

  void paint(QPainter *painter);
  Q_INVOKABLE void setup();//set up the subscriber(image msg from sensor) and advertiser(roi msg)

  //getters and setters for Q_PROPERTYs
  void set_topic(const QString &new_value);
  QString get_topic() const;

  void set_rois_dir_path(const QString rois_dir_path_);//set the roi dir for the current camera view
  QString get_rois_dir_path() const;

  QString get_all_rois_dir_path() const;// get the main folder of all the rois from all the cameras


  Q_INVOKABLE void save_ROI(QString sensor_name,int no_ROI, int rect_X,int rect_Y, int rect_width, int rect_height );//save the selected rois as iamges

  Q_INVOKABLE void add_roi(int no_ROI, QString roi_name, int rect_X,int rect_Y, int rect_width, int rect_height);// add every selected roi to msg:image2D_rois_msg

  Q_INVOKABLE void publish_rois_from_gui();
  Q_INVOKABLE void publish_rois_from_file();

signals:
  void topic_changed();
  void rois_dir_path_changed();

private:
  void receive_image(const sensor_msgs::Image::ConstPtr & msg);//call back function to recieve the images for displaying

  // ROS
  ros::NodeHandle nh;
  image_transport::ImageTransport * img_trans;
  image_transport::Subscriber image_sub;
  QString topic_value;//topic name
  QString rois_dir_path_value;//roi file path for the current sensor
  QString all_rois_dir_path_value;//roi file main path for all the sensors
  bool ros_ready;
  // Used for buffering the image
  QImage current_image;
  cv::Mat conversion_mat_;
  uchar * current_buffer;

  //for publish msgs
  int number_of_rois;
  opt_msgs::Image2D_roi current_image2D_roi_msg;
  opt_msgs::Image2D_roi_array image2D_rois_msg;
  ros::Publisher image2D_rois_from_gui_pub;
  //        image_transport::Publisher image2D_rois_from_file_pub;
  ros::Publisher image2D_rois_from_file_pub;
};

#endif // OPT_Streaming_Component_H
