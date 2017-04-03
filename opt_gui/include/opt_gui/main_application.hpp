#ifndef MAIN_APPLICATION_H
#define MAIN_APPLICATION_H

#include <ros/ros.h>
#include <QQmlApplicationEngine>

class Main_Application : public QQmlApplicationEngine {
    Q_OBJECT
    public:
        Main_Application();
        // this method is used to setup all the ros functionality we need
        // before the application starts runnings
        void run();

    // this defines a slot that will be called when the application is idle
    public slots:
        void main_loop();

    private:
        ros::NodeHandle nh;
        std::string view_topic1;
        std::string view_topic2;
        std::string view_topic3;
        std::string view_topic4;
        std::string view_topic5;
        std::string view_topic6;

};

#endif // MAIN_APPLICATION_H
