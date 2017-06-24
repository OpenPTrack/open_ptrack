#include <QTimer>
#include "../include/component/opt_streaming_component.hpp"
#include <ros/node_handle.h>
#include "../../opt_gui/include/opt_gui/main_application.hpp"

Main_Application::Main_Application():
    nh("~")
{}
void Main_Application::run() {


////////////////////////////////////////////////////////////////////////////////////
////Read sensor_name from camera_network.yaml adn use them to create topicrList ,
////then create topic_model which can be used in qml
////////////////////////////////////////////////////////////////////////////////////

    QStringList topicrList;
//    std::vector<std::string> keys;
 //   nh.getParamNames(keys);
//    ROS_INFO_STREAM("These are all the parameters: ");
//    std::copy(keys.begin(), keys.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
//    ROS_INFO_STREAM("Reading network!");
    XmlRpc::XmlRpcValue list;
    if (nh.getParam("network", list))
    {
        //ROS_INFO_STREAM("initial: " << list);
        for(int i =0; i < list.size(); ++i)
        {
            //ROS_INFO_STREAM("item: " << list[i]);
            XmlRpc::XmlRpcValue v(list[i]["sensors"]);
            //ROS_INFO_STREAM("Sensors: " << v);
            for(int k = 0; k < v.size(); ++k)
            {
                // ROS_INFO_STREAM("id"<<i<<", sensor"<<k<<": " << v[k]["id"]);
                std::string sensor_name_=v[k]["id"];
                std::string topic_="/"+sensor_name_+"/rgb_lowres/image";
                topicrList.append(QString::fromStdString(topic_));
            }
        }
    }
    else
    {
        ROS_ERROR_STREAM("Can't read sensor name from file, use the default sensor name:kinect2_head!");
        std::string topic_="/kinect2_head/rgb_lowres/image";
        topicrList.append(QString::fromStdString(topic_));
    }
    ////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////


    QQmlContext* context = this->rootContext();
    context->setContextProperty("topic_model", QVariant::fromValue(topicrList));

    qmlRegisterType<OPT_Streaming_Component>("OPT_Streaming_Component", 1, 0, "OptStreamingComponent");
    // this loads the qml file we are about to create
    this->load(QUrl(QStringLiteral("qrc:/main_window.qml")));
    // Setup a timer to get the application's idle loop
    QTimer * timer  = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(main_loop()));
    timer->start(0);
}

void Main_Application::main_loop() {
    ros::spinOnce();
}
