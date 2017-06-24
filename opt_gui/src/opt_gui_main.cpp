#include <ros/ros.h>
#include <QGuiApplication>
#include"../include/opt_gui/main_application.hpp"
int main(int argc, char **argv) {
  // Set up ROS.
  ros::init(argc, argv, "opt_gui");

  QGuiApplication app(argc, argv);
  Main_Application engine;
  engine.run();

  return app.exec();
}
