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

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"

#include <opt_msgs/Image2D_roi.h>
#include <opt_msgs/Image2D_roi_array.h>
class OPT_Streaming_Component : public QQuickPaintedItem {
    //make this a Qt Widget
    Q_OBJECT
    // defines a qml value for the topic
    Q_PROPERTY(QString topic READ get_topic WRITE set_topic NOTIFY topic_changed)

    public:
        // Constructor, takes parent widget, which defaults to null
        OPT_Streaming_Component(QQuickItem * parent = 0);

        void paint(QPainter *painter);
        void setup(ros::NodeHandle * nh);

        //getters and setters
        void set_topic(const QString &new_value);
        QString get_topic() const;

        Q_INVOKABLE void save_ROI(QString sensor_name,int no_ROI,QString roi_name, int rect_X,int rect_Y, int rect_width, int rect_height );
        Q_INVOKABLE void set_roi_name(int index, QString roi_name);
        Q_INVOKABLE void publish_rois();
signals:
        void topic_changed();

    private:
        void receive_image(const sensor_msgs::Image::ConstPtr & msg);

        // ROS
        ros::NodeHandle * nh;
        image_transport::ImageTransport * img_trans;
        image_transport::Subscriber image_sub;
        QString topic_value;
        bool ros_ready;
        // Used for buffering the image
//        QImage * current_image;
        QImage current_image;
        cv::Mat conversion_mat_;
        uchar * current_buffer;

        //for publish msgs
        int number_of_rois;
        opt_msgs::Image2D_roi current_image2D_roi_msg;
        opt_msgs::Image2D_roi_array image2D_rois_msg;
        ros::Publisher image2D_rois_pub;
//        gui::Roi_object_array::Ptr rois(new Roi_object_array);

};

#endif // OPT_Streaming_Component_H
