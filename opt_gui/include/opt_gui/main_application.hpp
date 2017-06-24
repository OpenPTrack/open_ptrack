#ifndef MAIN_APPLICATION_H
#define MAIN_APPLICATION_H

#include <ros/ros.h>
#include <QQmlApplicationEngine>
#include <QGuiApplication>
#include <QQmlContext>
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
        std::string sensor_name1;
        std::string sensor_name2;
        std::string sensor_name3;
        std::string sensor_name4;
        std::string sensor_name5;
        std::string sensor_name6;

};

#endif // MAIN_APPLICATION_H
