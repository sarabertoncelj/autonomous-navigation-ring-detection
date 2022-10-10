#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <nav_msgs/GetMap.h>
#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/LaserScan.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatus.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf2_ros/transform_listener.h>
#include "task2/xy.h"
#include "task2/goal.h"
#include <geometry_msgs/Twist.h>
#define PI 3.14159265

using namespace std;
using namespace cv;

tf2_ros::Buffer tf_buf;

// publishers & subscribers
ros::Publisher goal_pub;
ros::Publisher marker_pub;
ros::Publisher vel_pub;
ros::Subscriber goal_sub;
ros::Subscriber pose_sub;
ros::Subscriber ring_sub;
ros::Subscriber map_sub;
ros::Subscriber detection_sub;
ros::Subscriber laser_sub;
ros::ServiceClient nav_client;

// spremenljivke za transformacije
Mat cv_map;
float map_resolution = 0;
geometry_msgs::TransformStamped map_transform;
geometry_msgs::TransformStamped world_transform;
geometry_msgs::TransformStamped robot_world;

int box_margin = 1;
// int offset = 6;

// spremenljivke za navigacijo
visualization_msgs::Marker marker;
geometry_msgs::Point ring_position;
geometry_msgs::Pose approach_goal;
geometry_msgs::Pose nav_goal;
move_base_msgs::MoveBaseActionGoal current_goal;
float angle = 0;
float spin_angle = 0;
float avg;

// hardcoded stuff
int goal_index = 0;
// float position_x [6] = { -2.89, -2.346, -1.81, -0.131, -1.86, -2.396};
// float position_y [6] = { 1.054, 1.65, 0.052, 1.42, 1.09, 0.646};

// spremenljivke za izbiranje goalov
bool select_goal = true;
bool collect_ring = false;
bool ring_detected = false;
bool spin = true;
bool next_goal = false;
bool navigation = false;

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
	}
	avg = tmp / count;
	//ROS_INFO("range size : %f", avg);
}

void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg_map) {
    int size_x = msg_map->info.width;
    int size_y = msg_map->info.height;

    if ((size_x < 3) || (size_y < 3) ) {
        ROS_INFO("Map size is only x: %d,  y: %d . Not running map to image conversion", size_x, size_y);
        return;
    }

    // resize cv image if it doesn't have the same dimensions as the map
    if ( (cv_map.rows != size_y) && (cv_map.cols != size_x)) {
        cv_map = cv::Mat(size_y, size_x, CV_8U);
    }

    map_resolution = msg_map->info.resolution;
    map_transform.transform.translation.x = msg_map->info.origin.position.x;
    map_transform.transform.translation.y = msg_map->info.origin.position.y;
    map_transform.transform.translation.z = msg_map->info.origin.position.z;

    map_transform.transform.rotation = msg_map->info.origin.orientation;

    world_transform.transform.translation.x = - msg_map->info.origin.position.x;
    world_transform.transform.translation.y = - msg_map->info.origin.position.y;
    world_transform.transform.translation.z = - msg_map->info.origin.position.z;

    world_transform.transform.rotation = msg_map->info.origin.orientation;

    //tf2::poseMsgToTF(msg_map->info.origin, map_transform);

    const std::vector<int8_t>& map_msg_data (msg_map->data);

    unsigned char *cv_map_data = (unsigned char*) cv_map.data;

    //We have to flip around the y axis, y for image starts at the top and y for map at the bottom
    int size_y_rev = size_y-1;

    for (int y = size_y_rev; y >= 0; --y) {

        int idx_map_y = size_x * (size_y -y);
        int idx_img_y = size_x * y;

        for (int x = 0; x < size_x; ++x) {

            int idx = idx_img_y + x;

            switch (map_msg_data[idx_map_y + x])
            {
            case -1:
                cv_map_data[idx] = 127;
                break;

            case 0:
                cv_map_data[idx] = 255;
                break;

            case 100:
                cv_map_data[idx] = 0;
                break;
            }
        }
    }

}

void position_in_map (const move_base_msgs::MoveBaseActionFeedback::ConstPtr& robot_pose) {

  robot_world.transform.translation.x = robot_pose->feedback.base_position.pose.position.x;
  robot_world.transform.translation.y = robot_pose->feedback.base_position.pose.position.y;
  robot_world.transform.translation.z = robot_pose->feedback.base_position.pose.position.z;
  robot_world.transform.rotation = robot_pose->feedback.base_position.pose.orientation;

}

void goal_status(const move_base_msgs::MoveBaseActionResult::ConstPtr& goal) {

      ROS_INFO("status: %d", goal->status.status);
      if (goal->status.status == 3 && spin  && navigation){
        ROS_INFO("getting new goal from navigation");
        select_goal = true;
        task2::goal nav;
        if (nav_client.call(nav)){
          uint32_t shape = visualization_msgs::Marker::CUBE;
    			marker.header.frame_id = "/map";
    			marker.header.stamp = ros::Time::now();
    			marker.ns = "map_goals";
    			marker.id = 0;
    			marker.type = shape;
    			marker.action = visualization_msgs::Marker::ADD;
    			marker.pose.position.x = nav.response.x;
    			marker.pose.position.y = nav.response.y;
    			marker.scale.x = 0.2;
    			marker.scale.y = 0.2;
    			marker.scale.z = 0.2;
    			marker.color.r = 1.0;
    			marker.color.g = 0.8;
    			marker.color.b = 0.9;
    			marker.color.a = 0.8;
    			marker.lifetime = ros::Duration();
    			marker_pub.publish(marker);
          ROS_INFO("new goal from navigation: (%f, %f)", nav.response.x, nav.response.y);
          float x = robot_world.transform.translation.x - nav.response.x;
          float y = robot_world.transform.translation.y - nav.response.y;
          if (sqrt(x*x + y*y) < 0.5) spin = true;
          else {
            nav_goal.position.x = nav.response.x;
            nav_goal.position.y = nav.response.y;
            nav_goal.position.z = 0.0;
            spin = false;
            next_goal = true;
            spin_angle = 0;
          }
        }
      }
      if (goal->status.status == 3  && ring_detected && !select_goal){
        collect_ring = true;
      }
      if (goal->status.status == 4 && ring_detected && !select_goal){
        select_goal = true;
        ring_detected = false;
      }
      if (goal->status.status == 3  && next_goal && !select_goal && navigation){
        select_goal = true;
        spin = true;
        next_goal = false;
      }
      if (goal->status.status == 4  && next_goal && !select_goal && navigation){
        select_goal = true;
        spin = true;
        next_goal = false;
      }
}

geometry_msgs::Point forward(geometry_msgs::Point o, int colour, int approach_offset, int box_offset){
  geometry_msgs::Point result;
  int dist = 500;
  for (int x = o.x; x < cv_map.cols-2; x++) {
    for(int i = -box_margin; i<box_margin+1; i++){
      int v1 = cv_map.at<unsigned char>((int)o.y+i, x);
      int v2 = cv_map.at<unsigned char>((int)o.y+i, x+2);
      if (v1 == colour && v2 == colour){
        geometry_msgs::Point p;
				if (abs(o.x - x) >= box_offset) p.x = o.x;
				else p.x = x-box_offset;
        p.y = (int) o.y + approach_offset;
        // cv_map.at<unsigned char>((int)o.y, x) = 200;
        ROS_INFO("distance = %f", abs(o.x - x));
        if (abs(o.x - x) < dist){
          result = p;
          angle = PI/2;
          dist = abs(o.x - x);
        }
        else{
          break;
        }
      }
    }
  }
  for (int x = o.x; x > 2; x--) {
    for(int i = -box_margin; i<box_margin+1; i++){
      int v1 = cv_map.at<unsigned char>((int)o.y+i, x);
      int v2 = cv_map.at<unsigned char>((int)o.y+i, x-2);
      if (v1 == colour && v2 == colour){
        geometry_msgs::Point p;
				if (abs(o.x - x) >= box_offset) p.x = o.x;
				else p.x = x+box_offset;
        p.y = (int) o.y - approach_offset;
        // cv_map.at<unsigned char>((int)o.y, x) = 200;
        ROS_INFO("distance = %f", abs(o.x - x));
        if (abs(o.x - x) < dist){
          result = p;
          angle = -PI/2;
          dist = abs(o.x - x);
        }
        else{
          break;
        }
      }
    }
   }
   for (int y = o.y; y< cv_map.rows-2; y++) {
     for(int i = -box_margin; i<box_margin+1; i++){
       int v1 = cv_map.at<unsigned char>(y, (int)o.x+i);
       int v2 = cv_map.at<unsigned char>(y+2,(int)o.x+i);
       if (v1 == colour && v2 == colour){
         geometry_msgs::Point p;
         p.x = (int) o.x - approach_offset;
				 if (abs(o.y - y) >= box_offset) p.y = o.y;
				 else p.y = y-box_offset;
         // cv_map.at<unsigned char>(y, (int)o.x) = 200;
         ROS_INFO("distance = %f", abs(o.y - y));
         if (abs(o.y - y) < dist){
           angle =  0;
           result = p;
           dist = abs(o.y - y);
         }
         else{
           break;
         }
       }
     }
   }
   for (int y = o.y; y > 2; y--) {
    for(int i = -box_margin; i<box_margin+1; i++){
      int v1 = cv_map.at<unsigned char>(y, (int)o.x+i);
      int v2 = cv_map.at<unsigned char>(y-2,(int)o.x+i);
      if (v1 == colour && v2 == colour){
				geometry_msgs::Point p;
				p.x = (int) o.x + approach_offset;
				if (abs(o.y - y) >= box_offset) p.y = o.y;
				else p.y = y+box_offset;
        // cv_map.at<unsigned char>(y, (int)o.x) = 200;
        ROS_INFO("distance = %f", abs(o.y - y));
        if (abs(o.y - y) < dist){
          angle = PI;
          result = p;
          dist = abs(o.y - y);
        }
        else{
          break;
        }
     }
    }
  }
  return result;
}

void calculate_approach (geometry_msgs::Point pt, int distance){

  geometry_msgs::Point transformed_pt;
  tf2::doTransform(pt, transformed_pt, world_transform);
  transformed_pt.x = transformed_pt.x / map_resolution;
  transformed_pt.y = cv_map.rows - (transformed_pt.y / map_resolution);

  geometry_msgs::Point point = forward(transformed_pt, 0, distance, 6);
  ROS_INFO("found ring location");
  cv_map.at<unsigned char>(point.y, point.x) = 150;
  // if (!cv_map.empty()) imshow("Map", cv_map);
  // waitKey(0);

  geometry_msgs::Point approach_pt;

  point.x = (float)point.x * map_resolution;
  point.y = (float)(cv_map.rows - point.y) * map_resolution;
  point.z = 0.0;
  tf2::doTransform(point, approach_pt, map_transform);

  tf2::Quaternion goal_orientation;
  goal_orientation.setRPY(0, 0, angle);

  approach_goal.position.x = approach_pt.x;
  approach_goal.position.y = approach_pt.y;
  approach_goal.position.z = approach_pt.z;
  approach_goal.orientation.x = goal_orientation.x();
  approach_goal.orientation.y = goal_orientation.y();
  approach_goal.orientation.z = goal_orientation.z();
  approach_goal.orientation.w = goal_orientation.w();
}

void detected_ring(const task2::xy::ConstPtr& msg) {
	ROS_INFO("new ring");
  if (!ring_detected){
		ROS_INFO("ring location recieved");

	  geometry_msgs::Point pt;

	  pt.x = msg->x;
	  pt.y = msg->y;
	  pt.z = 0.0;

	  calculate_approach(pt, 3);
	  ring_detected = true;
	  select_goal = true;
	}

}

void new_candidate(const task2::xy::ConstPtr& msg) {
  if (!ring_detected && navigation){
		ROS_INFO("wait");
		geometry_msgs::Twist base_cmd;
		base_cmd.linear.x=0.0;
		base_cmd.linear.y=0.0;
		base_cmd.linear.z=0.0;
		base_cmd.angular.z=0.0;
		int step = 0;
		ros::Rate r = 10;
		while (step <2) {
			vel_pub.publish(base_cmd);
			r.sleep();
			step = step+1;
		}
	}

}

int main(int argc, char** argv) {

    ros::init(argc, argv, "ring_collector");
    ros::NodeHandle n;

    map_sub = n.subscribe("map", 10, &mapCallback);
    goal_sub = n.subscribe("/move_base/result", 10, &goal_status);
    pose_sub = n.subscribe("move_base/feedback", 10, &position_in_map);
    ring_sub = n.subscribe("/clustered_coordinates_rings", 10, &detected_ring);
    detection_sub = n.subscribe("/ring_coordinates", 10, &new_candidate);
    laser_sub = n.subscribe("/scan", 1, &scanCallback);

    goal_pub = n.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 10);
    marker_pub = n.advertise<visualization_msgs::Marker>("goals", 10);
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 50);
    nav_client = n.serviceClient<task2::goal>("navigation");

    ros::Rate r = 3;

    tf2_ros::TransformListener tfListener(tf_buf);

    robot_world = tf_buf.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(10.0) );
    ROS_INFO("ring collector ready");
    while(ros::ok()) {
      waitKey(30);
      if (collect_ring){
        ROS_INFO("collet ring");
      	geometry_msgs::Twist base_cmd;
        int step = 0;
				while(step < 8){
					ROS_INFO("scan: %f", avg);
					base_cmd.linear.x=0.3;
					vel_pub.publish(base_cmd);
					r.sleep();
					step = step+1;
				}
				base_cmd.linear.x=0.0;
				vel_pub.publish(base_cmd);
        ROS_INFO("ring colleted");
        collect_ring = false;
				ring_detected = false;
        select_goal = true;
      }
      if (select_goal && !cv_map.empty()){
        ROS_INFO("selecting new goal");
        current_goal.goal.target_pose.header.frame_id = "/map";
        current_goal.goal.target_pose.header.stamp = ros::Time::now();
        uint32_t shape = visualization_msgs::Marker::CUBE;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "map_goals";
        marker.id = 0;
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.r = 1.0;
        marker.color.g = 0.8;
        marker.color.b = 0.9;
        marker.color.a = 0.8;
        marker.lifetime = ros::Duration();
        select_goal = false;
        if (spin && navigation){
          ROS_INFO("look around");
          current_goal.goal.target_pose.pose.position.x = robot_world.transform.translation.x;
          current_goal.goal.target_pose.pose.position.y = robot_world.transform.translation.y;
          current_goal.goal.target_pose.pose.position.z = robot_world.transform.translation.z;
          current_goal.goal.target_pose.pose.position.z = robot_world.transform.translation.z;
          tf2::Quaternion goal_orientation;
          spin_angle = spin_angle+(PI/2);
          goal_orientation.setRPY(0, 0, spin_angle);
          current_goal.goal.target_pose.pose.orientation.x = goal_orientation.x();
          current_goal.goal.target_pose.pose.orientation.y = goal_orientation.y();
          current_goal.goal.target_pose.pose.orientation.z = goal_orientation.z();
          current_goal.goal.target_pose.pose.orientation.w = goal_orientation.w();
    			marker.pose = current_goal.goal.target_pose.pose;

	        goal_pub.publish(current_goal);
	        marker_pub.publish(marker);
        }
        if (next_goal && navigation){
          ROS_INFO("go to next not searched spot");
          current_goal.goal.target_pose.pose.position = nav_goal.position;
          shape = visualization_msgs::Marker::ARROW;
      		marker.type = shape;
      		marker.pose = current_goal.goal.target_pose.pose;

	        goal_pub.publish(current_goal);
	        marker_pub.publish(marker);
        }
        if (ring_detected){
          ROS_INFO("approach the ring");
          current_goal.goal.target_pose.pose = approach_goal;
          shape = visualization_msgs::Marker::ARROW;
    			marker.type = shape;
    			marker.pose = current_goal.goal.target_pose.pose;

	        goal_pub.publish(current_goal);
	        marker_pub.publish(marker);
          //if (!cv_map.empty()) imshow("Map", cv_map);
          //waitKey(0);
        }
        ROS_INFO("new goal: (%f, %f)", current_goal.goal.target_pose.pose.position.x, current_goal.goal.target_pose.pose.position.y);

      }

      ros::spinOnce();
    }
    return 0;

}
