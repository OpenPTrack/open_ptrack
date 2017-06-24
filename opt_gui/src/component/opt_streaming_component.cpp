#include "../../include/component/opt_streaming_component.hpp"


OPT_Streaming_Component::OPT_Streaming_Component(QQuickItem * parent) :
  nh(),
  QQuickPaintedItem(parent),
  current_image(NULL),
  current_buffer(NULL),
  topic_value("Click_on_one_of_the_camera_views_on_the_left_to_start"),
  ros_ready(false),
  img_trans(NULL),number_of_rois(0) {
  all_rois_dir_path_value=QString::fromStdString("file:"+ros::package::getPath("opt_gui")+"/data/");
}

//set up the subscriber(image msg from sensor) and advertiser(roi msg)
void OPT_Streaming_Component::setup() {
  img_trans = new image_transport::ImageTransport(nh);

  image_sub = img_trans->subscribe(
        topic_value.toStdString(),
        1,
        &OPT_Streaming_Component::receive_image,
        this,
        image_transport::TransportHints("raw")
        );

  std::string sensor_name=topic_value.toStdString().substr(1, (topic_value.toStdString().size()-17));
  std::string image2D_rois_from_gui_pub_name = "/"+sensor_name + "/image2D_rois_from_gui";
  image2D_rois_from_gui_pub=nh.advertise<opt_msgs::Image2D_roi_array>(image2D_rois_from_gui_pub_name,3);

  std::string image2D_rois_from_file_pub_name = "/"+sensor_name + "/image2D_rois_from_file/image";
//  image2D_rois_from_file_pub=nh.advertise<sensor_msgs::Image>(image2D_rois_from_file_pub_name,3);
   image2D_rois_from_file_pub=nh.advertise<opt_msgs::Image2D_roi_file>(image2D_rois_from_file_pub_name,3);

  ros_ready = true;
  ROS_INFO("Setup of video component complete");
}


//call back function to recieve images from sensor
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

  // Only Qimage can be painted
  current_image=QImage(conversion_mat_.data,conversion_mat_.cols,conversion_mat_.rows,QImage::Format_RGB888);

  // finally we need to re-render the component to display the new image
  update();
}


//This function will run  after "update();" is called
void OPT_Streaming_Component::paint(QPainter * painter) {
  painter->drawImage(boundingRect().adjusted(1, 1, -1, -1), current_image);//paint the image to fit the size of the Rectangle
}


//When the sensor name is modified in the camera view, change the topics with the new sensor name in the subscriber and advertiser
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

      image2D_rois_msg.Rois.clear();

      std::string sensor_name=topic_value.toStdString().substr(1, (topic_value.toStdString().size()-17));
      std::string image2D_rois_from_gui_pub_name = "/"+sensor_name + "/image2D_rois_from_gui";
      image2D_rois_from_gui_pub=nh.advertise<opt_msgs::Image2D_roi_array>(image2D_rois_from_gui_pub_name,3);

      std::string image2D_rois_from_file_pub_name = "/"+sensor_name + "/image2D_rois_from_file/image";
      image2D_rois_from_file_pub=nh.advertise<opt_msgs::Image2D_roi_file>(image2D_rois_from_file_pub_name,3);

    }
    emit topic_changed();
  }
}

QString OPT_Streaming_Component::get_topic() const {
  return topic_value;
}


void OPT_Streaming_Component::set_rois_dir_path(const QString rois_dir_path_)
{
  rois_dir_path_value=rois_dir_path_;
  emit rois_dir_path_changed();
}

QString OPT_Streaming_Component::get_rois_dir_path() const {
  return rois_dir_path_value;
}
QString OPT_Streaming_Component::get_all_rois_dir_path() const {
  return all_rois_dir_path_value;
}


//save the selected rois in the gui as images to their folder named with the corresponding sensor names
void OPT_Streaming_Component::save_ROI(QString sensor_name,int no_ROI, int rect_X,int rect_Y, int rect_width, int rect_height )
{
  //get the roi with the bouding box you created in qml main_view, save it as a file
  cv::Rect roi_rec=cv::Rect(rect_X,rect_Y,rect_width,rect_height) & cv::Rect(0,0,conversion_mat_.size().width,conversion_mat_.size().height);
  cv::Mat roi(conversion_mat_,roi_rec);
  cv::cvtColor(roi, roi, CV_BGR2RGB);
  std::string roi_file_name=ros::package::getPath("opt_gui")+"/data/"+sensor_name.toStdString()+"_roi_"+ QString::number(no_ROI).toStdString()+".png";
  cv::imwrite(roi_file_name,roi);
}

// Add rois one by one to the msg in the qml after push the button named "publish_rois_from_gui"
void OPT_Streaming_Component::add_roi(int no_ROI,QString roi_name, int rect_X,int rect_Y, int rect_width, int rect_height)
{
  current_image2D_roi_msg.no=no_ROI;
  current_image2D_roi_msg.name=roi_name.toStdString();
  current_image2D_roi_msg.x=rect_X;
  current_image2D_roi_msg.y=rect_Y;
  current_image2D_roi_msg.width=rect_width;
  current_image2D_roi_msg.height=rect_height;
  image2D_rois_msg.Rois.push_back(current_image2D_roi_msg);
}

//After Add all the rois , publish the msg, this is also excuted in qml as the "add_roi" function
void OPT_Streaming_Component::publish_rois_from_gui()
{
  image2D_rois_from_gui_pub.publish(image2D_rois_msg);
  std::cout<<image2D_rois_msg.Rois.size()<<" image2D_rois are published"<<std::endl;
  image2D_rois_msg.Rois.clear();//use clear so that we can create new rois after starting detection
}

/*
//read the rois from previous save image files in the folder correspong to the sensor name, and publish them one by one
void OPT_Streaming_Component::publish_rois_from_file()
{
  std::string sensor_name=topic_value.toStdString().substr(1, (topic_value.toStdString().size()-17));
  cv::Directory dir;
  std::string rois_dir_path=ros::package::getPath("opt_gui")+"/data/"+sensor_name;
  std::vector<std::string> roi_filenames = dir.GetListFiles(rois_dir_path, ".png");

  //publish roi iamge one by one
  for(std::vector<std::string>::iterator it =roi_filenames.begin(); it!=roi_filenames.end();it++)
  {
    cv::Mat image = cv::imread(rois_dir_path+*it);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    image2D_rois_from_file_pub.publish(msg);
    std::cout<<"image msg for file has published"<<std::endl;
    ros::Duration(0.3).sleep(); // sleep for half a second
  }

  //create a empty image to publish so that the reciever can know all the rois images have been published
  cv::Mat image_;
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_).toImageMsg();
  image2D_rois_from_file_pub.publish(msg);
  std::cout<<"Finish publishing roi msg from file"<<std::endl;
}
*/

//read the rois from previous save image files in the folder correspong to the sensor name, and publish them one by one
void OPT_Streaming_Component::publish_rois_from_file()
{
  std::string sensor_name=topic_value.toStdString().substr(1, (topic_value.toStdString().size()-17));
  cv::Directory dir;
  std::string rois_dir_path=ros::package::getPath("opt_gui")+"/data/"+sensor_name;
  std::vector<std::string> roi_filenames = dir.GetListFiles(rois_dir_path, ".png");

  //publish roi iamge one by one
  for(std::vector<std::string>::iterator it =roi_filenames.begin(); it!=roi_filenames.end();it++)
  {
    cv::Mat image = cv::imread(rois_dir_path+*it);


    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    opt_msgs::Image2D_roi_file current_image2D_roi_file_msg;
    current_image2D_roi_file_msg.image_roi=*image_msg;
    current_image2D_roi_file_msg.name=*it;

    image2D_rois_from_file_pub.publish(current_image2D_roi_file_msg);
    std::cout<<"image msg for file has published"<<std::endl;
    ros::Duration(0.3).sleep(); // sleep for half a second
  }

  opt_msgs::Image2D_roi_file current_image2D_roi_file_msg;
  current_image2D_roi_file_msg.name="end_flag";
  image2D_rois_from_file_pub.publish(current_image2D_roi_file_msg);
  std::cout<<"Finish publishing roi msg from file"<<std::endl;
}
