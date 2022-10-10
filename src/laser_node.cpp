#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <nav_msgs/GetMap.h>
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf2_ros/transform_listener.h>

using namespace std;
using namespace cv;

Mat cv_map;
float map_resolution = 0;
geometry_msgs::TransformStamped map_transform;

ros::Publisher marker_pub;
ros::Subscriber laser_sub;

float avg;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& input) {
	avg = 0;
	int count = 0;
	float tmp = 0;
    for (int i = 0; i < input->ranges.size(); ++i) {
		const float &range = input->ranges[i];
		if (range > 0) {
			tmp = tmp + range;
			count = count +1;
		}
		//else {
			//ROS_INFO("ranges: [%f]",range);

		//}
	}
	avg = tmp / count;
	ROS_INFO("range size : %f", avg);


}

int main(int argc, char** argv) {

    ros::init(argc, argv, "laser_node");
    ros::NodeHandle n;

    laser_sub = n.subscribe("/scan", 1, &scanCallback);

    while(ros::ok()) {
      ros::spinOnce();
    }
    return 0;

}
