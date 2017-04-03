#include "../../include/component/opt_streaming_component.hpp"

OPT_Streaming_Component::OPT_Streaming_Component(QQuickItem * parent) :
    QQuickPaintedItem(parent),
    current_image(NULL),
    current_buffer(NULL),
    topic_value("/cam0"),
    ros_ready(false),
    img_trans(NULL),number_of_rois(0) {
}

void OPT_Streaming_Component::setup(ros::NodeHandle * nh) {
    img_trans = new image_transport::ImageTransport(*nh);
    //TODO: make these parameters of the component
    image_sub = img_trans->subscribe(
                topic_value.toStdString(),
                1,
                &OPT_Streaming_Component::receive_image,
                this,
                image_transport::TransportHints("raw")
                );

    std::string image2D_rois_pub_name = topic_value.toStdString().substr(0, topic_value.toStdString().rfind('/')) + "/image2D_rois";
    image2D_rois_pub=nh->advertise<opt_msgs::Image2D_roi_array>(image2D_rois_pub_name,3);
    ros_ready = true;
    ROS_INFO("Setup of video component complete");
}

/*
void OPT_Streaming_Component::receive_image(const sensor_msgs::Image::ConstPtr &msg) {
    // check to see if we already have an image frame, if we do then we need to
    // delete it to avoid memory leaks
    if( current_image ) {
        delete current_image;
    }
    // allocate a buffer of sufficient size to contain our video frame
    uchar * temp_buffer = (uchar *) malloc(sizeof(uchar) * msg->data.size());

    // and copy the message into the buffer
    // we need to do this because QImage api requires the buffer we pass in to
    // continue to exist whilst the image is in use, but the msg and it's data will
    // be lost once we leave this context
    current_image = new QImage(
          temp_buffer,
          msg->width,
          msg->height,
          QImage::Format_RGB888 // TODO: detect the type from the message
    );

    ROS_INFO("Recieved Message");

    // We then swap out of bufer to avoid memory leaks
    if(current_buffer) {
        delete current_buffer;
        current_buffer = temp_buffer;
    }

    // finally we need to re-render the component to display the new image
    update();
}


*/

void OPT_Streaming_Component::receive_image(const sensor_msgs::Image::ConstPtr &msg) {
    image2D_rois_msg.header=msg->header;

    //-----------------------------This part comes from rtq_image_view----------------------------------
    try
    {
        // First let cv_bridge do its magic
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
        conversion_mat_ = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
        try
        {
            // If we're here, there is no conversion that makes sense, but let's try to imagine a few first
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
            if (msg->encoding == "CV_8UC3")
            {
                // assuming it is rgb
                conversion_mat_ = cv_ptr->image;
            } else if (msg->encoding == "8UC1") {
                // convert gray to rgb
                cv::cvtColor(cv_ptr->image, conversion_mat_, CV_GRAY2RGB);
            } else if (msg->encoding == "16UC1" || msg->encoding == "32FC1") {
                // scale / quantify
                double min = 0;
                double max = 10;
                if (msg->encoding == "16UC1") max *= 1000;
                // dynamically adjust range based on min/max in image
                cv::minMaxLoc(cv_ptr->image, &min, &max);
                if (min == max) {
                    // completely homogeneous images are displayed in gray
                    min = 0;
                    max = 2;
                }
                cv::Mat img_scaled_8u;
                cv::Mat(cv_ptr->image-min).convertTo(img_scaled_8u, CV_8UC1, 255. / (max - min));
                cv::cvtColor(img_scaled_8u, conversion_mat_, CV_GRAY2RGB);
            } else {
                qWarning("ImageView.callback_image() could not convert image from '%s' to 'rgb8' (%s)", msg->encoding.c_str(), e.what());
                return;
            }
        }
        catch (cv_bridge::Exception& e)
        {
            qWarning("ImageView.callback_image() while trying to convert image from '%s' to 'rgb8' an exception was thrown (%s)", msg->encoding.c_str(), e.what());
            return;
        }
    }
    //-----------------------------This part comes from rtq_image_view----------------------------------

    // image must be copied since it uses the conversion_mat_ for storage which is asynchronously overwritten in the next callback invocation
    current_image=QImage(conversion_mat_.data,conversion_mat_.cols,conversion_mat_.rows,QImage::Format_RGB888);
    // finally we need to re-render the component to display the new image
    update();
}

void OPT_Streaming_Component::paint(QPainter * painter) {
    painter->drawImage(boundingRect().adjusted(1, 1, -1, -1), current_image);
}

void OPT_Streaming_Component::set_topic(const QString & new_value) {
    if(topic_value != new_value) {
        topic_value = new_value;
        if(ros_ready) {
            image_sub.shutdown();
            image_sub = img_trans->subscribe(
                        topic_value.toStdString(),
                        1,
                        &OPT_Streaming_Component::receive_image,
                        this,
                        image_transport::TransportHints("raw")
                        );
        }
        emit topic_changed();
    }
}

QString OPT_Streaming_Component::get_topic() const {
    return topic_value;
}

void OPT_Streaming_Component::save_ROI(QString sensor_name,int no_ROI,QString roi_name, int rect_X,int rect_Y, int rect_width, int rect_height )
{

    //get the roi with the bouding box you created in qml main_view, save it as a file
    cv::Rect roi_rec=cv::Rect(rect_X,rect_Y,rect_width,rect_height) & cv::Rect(0,0,conversion_mat_.size().width,conversion_mat_.size().height);
//    if(roi_rec.size().area()>1)
//    {
        cv::Mat roi(conversion_mat_,roi_rec);
        cv::cvtColor(roi, roi, CV_BGR2RGB);
        QString roi_file_name="/tmp/"+sensor_name+"_roi_"+ QString::number(no_ROI)+".png";
        cv::imwrite(roi_file_name.toStdString(),roi);

        //create a image2D_roi_msg with the roi and push it to image2D_roi_array_msg called image2D_rois_msg
        current_image2D_roi_msg.no=no_ROI;
        current_image2D_roi_msg.name=roi_name.toStdString();
        current_image2D_roi_msg.x=rect_X;
        current_image2D_roi_msg.y=rect_Y;
        current_image2D_roi_msg.width=rect_width;
        current_image2D_roi_msg.height=rect_height;
        image2D_rois_msg.Rois.push_back(current_image2D_roi_msg);
    }
//}

void OPT_Streaming_Component::set_roi_name(int index, QString roi_name)
{
    image2D_rois_msg.Rois[index-number_of_rois].name=roi_name.toStdString();
}

void OPT_Streaming_Component::publish_rois()
{
    image2D_rois_pub.publish(image2D_rois_msg);
    number_of_rois=number_of_rois+image2D_rois_msg.Rois.size();
    std::cout<<image2D_rois_msg.Rois.size()<<" image2D_rois are published"<<std::endl;
    image2D_rois_msg.Rois.clear();//use clear so that we can create new rois after starting detection
}

