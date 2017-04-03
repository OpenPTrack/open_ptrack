#include <QTimer>
#include "../include/component/opt_streaming_component.hpp"
#include "../../opt_gui/include/opt_gui/main_application.hpp"

Main_Application::Main_Application() {

}

void Main_Application::run() {


    nh.param("view_topic_1",view_topic1,std::string("/kinect2_head/rgb_lowres/image"));
    nh.param("view_topic_2",view_topic2,std::string("/kinect2_far/rgb_lowres/image"));
    nh.param("view_topic_3",view_topic3,std::string("/kinect2_lenovo/rgb_lowres/image"));
    nh.param("view_topic_4",view_topic4,std::string("/kinect2_head/rgb_lowres/image"));
    nh.param("view_topic_5",view_topic5,std::string("/kinect2_head/rgb_lowres/image"));
    nh.param("view_topic_6",view_topic6,std::string("/kinect2_head/rgb_lowres/image"));


    qmlRegisterType<OPT_Streaming_Component>("OPT_Streaming_Component", 1, 0, "OptStreamingComponent");
    // this loads the qml file we are about to create
    this->load(QUrl(QStringLiteral("qrc:/main_window.qml")));

    // Setup a timer to get the application's idle loop
    QTimer * timer  = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(main_loop()));
    timer->start(0);

    // setup the video component
    OPT_Streaming_Component * video1 = this->rootObjects()[0]->findChild<OPT_Streaming_Component*>(QString("videoStream1"));
    video1->set_topic(QString::fromStdString(view_topic1));
    video1->setup(&nh);

    OPT_Streaming_Component * video2 = this->rootObjects()[0]->findChild<OPT_Streaming_Component*>(QString("videoStream2"));
    video2->set_topic(QString::fromStdString(view_topic2));
    video2->setup(&nh);

    OPT_Streaming_Component * video3 = this->rootObjects()[0]->findChild<OPT_Streaming_Component*>(QString("videoStream3"));
    video3->set_topic(QString::fromStdString(view_topic3));
    video3->setup(&nh);

    OPT_Streaming_Component * video4 = this->rootObjects()[0]->findChild<OPT_Streaming_Component*>(QString("videoStream4"));
    video4->set_topic(QString::fromStdString(view_topic4));
    video4->setup(&nh);

    OPT_Streaming_Component * video5 = this->rootObjects()[0]->findChild<OPT_Streaming_Component*>(QString("videoStream5"));
    video5->set_topic(QString::fromStdString(view_topic5));
    video5->setup(&nh);

    OPT_Streaming_Component * video6 = this->rootObjects()[0]->findChild<OPT_Streaming_Component*>(QString("videoStream6"));
    video6->set_topic(QString::fromStdString(view_topic6));
    video6->setup(&nh);

    OPT_Streaming_Component * video_main = this->rootObjects()[0]->findChild<OPT_Streaming_Component*>(QString("main_videoStream"));
    video_main->set_topic(QString::fromStdString(view_topic1));
    video_main->setup(&nh);
}

void Main_Application::main_loop() {

    ros::spinOnce();

}
