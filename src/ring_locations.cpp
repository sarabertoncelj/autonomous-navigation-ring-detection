#include "ros/ros.h"
#include "task2/xy.h"
#include <visualization_msgs/Marker.h>

ros::Publisher coordinates_pub;

int ring_number = 0;
float position_x [4] = { -1.76, -0.677, 0.36, -2.11};
float position_y [4] = { 0.015, 0.816, 0.82, 1.49};

int main(int argc, char** argv){
  ros::init(argc, argv, "ring_locations");
  ros::NodeHandle n;

  coordinates_pub = n.advertise<task2::xy>("ring_locations", 10);


  while(ros::ok()) {
    if (ring_number < 4){
      task2::xy msg;
      msg.x = position_x[ring_number];
      msg.y = position_y[ring_number];
      msg.timestamp = ros::Time::now();
      ROS_INFO ("new ring: %f, %f", msg.x, msg.y);
      //ring_number++;
      coordinates_pub.publish(msg);
      //ros::topic::waitForMessage<visualization_msgs::Marker>("/goals", n);

    }
    ros::spinOnce();
  }

  return 0;
}
