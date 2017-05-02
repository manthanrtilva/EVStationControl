
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cerrno>
#include <cstddef>
#include <thread>

#include <gtkmm.h>

#include <json/json.h>

class MainWindow : public Gtk::Window {
public:
  MainWindow() {}
  ~MainWindow() {}
};

int main(int argc, char **argv) {
  std_msgs::String msg;
  ros::init(argc, argv, "station_control_ui");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::String>("schnieder_l2", 1000);
  msg.data = "boot";
  pub.publish(msg);
  ros::Rate loop_rate(1);
  // int count = 0;
  // SchniederL2Station oStation(pub);
  // ros::Subscriber sub = n.subscribe("schnieder_l2_control", 1000, &SchniederL2Station::onCommand,&oStation);
  // while (ros::ok()) {
  //   oStation.task();
  //   ros::spinOnce();
  //   loop_rate.sleep();
  //   ++count;
  // }

  return 0;
}
