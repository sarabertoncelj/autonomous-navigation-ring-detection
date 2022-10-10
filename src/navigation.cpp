#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <nav_msgs/GetMap.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf2_ros/transform_listener.h>
#include "task2/goal.h"

#define PI 3.14159265

using namespace std;
using namespace cv;


ros::Publisher marker_pub;
ros::Subscriber pose_sub;
ros::Subscriber status_sub;

Mat cv_map;
Mat searched;
float map_resolution = 0;
geometry_msgs::TransformStamped map_transform;
geometry_msgs::TransformStamped world_transform;
geometry_msgs::TransformStamped robot_world;

tf2_ros::Buffer tf_buf;
// tf2_ros::TransformListener tfListener(tf_buf);
// geometry_msgs::Pose approach_goal;

float angle;
int offset = 6;
ros::Subscriber map_sub;

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

bool same_colour(geometry_msgs::Point o, int n, int colour){
  vector<geometry_msgs::Point> result;
  int v;
  for (int i = 1; i < n; i++) {
    v = (int)cv_map.at<unsigned char>(o.y+i, o.x);
    if (v == colour) return false;
    v = (int)cv_map.at<unsigned char>(o.y-i, o.x);
    if (v == colour) return false;
    v = (int)cv_map.at<unsigned char>(o.y, o.x+i);
    if (v == colour) return false;
    v = (int)cv_map.at<unsigned char>(o.y, o.x-i);
    if (v == colour) return false;
    v = (int)cv_map.at<unsigned char>(o.y+i, o.x+i);
    if (v == colour) return false;
    v = (int)cv_map.at<unsigned char>(o.y-i, o.x-i);
    if (v == colour) return false;
    v = (int)cv_map.at<unsigned char>(o.y-i, o.x+i);
    if (v == colour) return false;
    v = (int)cv_map.at<unsigned char>(o.y+i, o.x-i);
    if (v == colour) return false;
 }
 return true;
}

geometry_msgs::Point closest(geometry_msgs::Point o, int colour){
  ROS_INFO("finding closest empty spot");
  geometry_msgs::Point p;
  float min_dist = 500 * 500;
  for (int x = 1; x < cv_map.cols; x++) {
    for (int y = 1; y < cv_map.rows; y++) {
      geometry_msgs::Point temp;
      temp.x = x;
      temp.y = y;
      int v = (int)cv_map.at<unsigned char>(y, x);
      if (v == colour && same_colour(temp, 10, 0) && same_colour(temp, 5, 100)){
        int dx = abs(o.x - x);
        int x2 = dx*dx;
        int dy = abs(o.y - y);
        int y2= dy*dy;
        int d2 = x2 + y2;
        if (d2 < min_dist){
          min_dist = d2;
          p.x = x;
          p.y = y;
        }
      }
   }
 }

ROS_INFO("closest empty spot at (%f, %f)", p.x, p.y);
 return p;
}

bool new_goal(task2::goal::Request &req, task2::goal::Response &res) {
  ROS_INFO("requesting new goal");
  geometry_msgs::Point robot_pt;
  geometry_msgs::Point robot_map;
  robot_pt.x = robot_world.transform.translation.x;
  robot_pt.y = robot_world.transform.translation.y;
  robot_pt.z = robot_world.transform.translation.z;
  tf2::doTransform(robot_pt, robot_map, world_transform);
  robot_map.x = robot_map.x / map_resolution;
  robot_map.y = cv_map.rows - (robot_map.y / map_resolution);
  cv_map.at<unsigned char>(robot_map.y, robot_map.x) = 100;

  geometry_msgs::Point pt = closest(robot_map, 255);
  geometry_msgs::Point transformed_pt;
  ROS_INFO("new goal at: (%f, %f)", pt.x, pt.x);
  pt.x = (float)pt.x * map_resolution;
  pt.y = (float)(cv_map.rows - pt.y) * map_resolution;
  pt.z = 0.0;

  tf2::doTransform(pt, transformed_pt, map_transform);
  ROS_INFO("new goal located");
  res.x = transformed_pt.x;
  res.y = transformed_pt.y;
  // if (!cv_map.empty()) imshow("Map", cv_map);
  // waitKey(0);
  return true;
}

void mark_searched (const move_base_msgs::MoveBaseActionFeedback::ConstPtr& robot_pose) {
  robot_world.transform.translation.x = robot_pose->feedback.base_position.pose.position.x;
  robot_world.transform.translation.y = robot_pose->feedback.base_position.pose.position.y;
  robot_world.transform.translation.z = robot_pose->feedback.base_position.pose.position.z;
  robot_world.transform.rotation = robot_pose->feedback.base_position.pose.orientation;

  geometry_msgs::Point robot_forward;
  robot_forward.x = 0.6;
  robot_forward.y = 0.0;
  robot_forward.z = 0.0;
  geometry_msgs::Point world_forward;
  tf2::doTransform(robot_forward, world_forward, robot_world);
  geometry_msgs::Point map_forward;

  geometry_msgs::Point robot_pt;
  geometry_msgs::Point robot_map;
  robot_pt.x = robot_world.transform.translation.x;
  robot_pt.y = robot_world.transform.translation.y;
  robot_pt.z = robot_world.transform.translation.z;
  tf2::doTransform(robot_pt, robot_map, world_transform);
  robot_map.x = robot_map.x / map_resolution;
  robot_map.y = cv_map.rows - (robot_map.y / map_resolution);
  // cv_map.at<unsigned char>(robot_map.y, robot_map.x) = 100;

  // uint32_t shape = visualization_msgs::Marker::CUBE;
  // visualization_msgs::Marker marker;
  // marker.header.frame_id = "/map";
  // marker.header.stamp = ros::Time::now();
  // marker.ns = "map_goals";
  // marker.id = 0;
  // marker.type = shape;
  // marker.action = visualization_msgs::Marker::ADD;
  // marker.pose.position.x = world_forward.x;
  // marker.pose.position.y = world_forward.y;
  // marker.pose.position.z = world_forward.z;
  // marker.scale.x = 0.2;
  // marker.scale.y = 0.2;
  // marker.scale.z = 0.2;
  // marker.color.r = 0.4f;
  // marker.color.g = 0.0f;
  // marker.color.b = 0.6f;
  // marker.color.a = 0.8;
  // marker.lifetime = ros::Duration();
  // marker_pub.publish(marker);

  tf2::doTransform(world_forward, map_forward, world_transform);
  map_forward.x = map_forward.x / map_resolution;
  map_forward.y = cv_map.rows - (map_forward.y / map_resolution);
  // cv_map.at<unsigned char>(map_forward.y, map_forward.x) = 100;
  // a -> robot_map
  // b -> map_forward
  float k;
  float n;
  if (map_forward.x == robot_map.x){
    n = robot_map.x;
  }
  else {
    k = (map_forward.y - robot_map.y) / (map_forward.x - robot_map.x);
    n = robot_map.y - k * robot_map.x;
  }
  for (int x = robot_map.x; x <= map_forward.x; x++) {
    int y = round(k * x + n);
    int p = cv_map.at<unsigned char>(y, x);
    if (p == 0) {
      ROS_INFO("black");
      break;
    }
    // ROS_INFO("x: %d, y: %d", x, y);
    cv_map.at<unsigned char>(y+1, x+1) = 100;
    cv_map.at<unsigned char>(y+1, x) = 100;
    cv_map.at<unsigned char>(y, x+1) = 100;
    cv_map.at<unsigned char>(y, x) = 100;
    cv_map.at<unsigned char>(y-1, x-1) = 100;
    cv_map.at<unsigned char>(y-1, x) = 100;
    cv_map.at<unsigned char>(y, x-1) = 100;
    cv_map.at<unsigned char>(y+1, x-1) = 100;
    cv_map.at<unsigned char>(y-1, x+1) = 100;
  }
  for (int x = robot_map.x; x >= map_forward.x; x--) {
    int y = round(k * x + n);
    int p = cv_map.at<unsigned char>(y, x);
    if (p == 0) {
      ROS_INFO("black");
      break;
    }
    // ROS_INFO("x: %d, y: %d", x, y);
    cv_map.at<unsigned char>(y+1, x+1) = 100;
    cv_map.at<unsigned char>(y+1, x) = 100;
    cv_map.at<unsigned char>(y, x+1) = 100;
    cv_map.at<unsigned char>(y, x) = 100;
    cv_map.at<unsigned char>(y-1, x-1) = 100;
    cv_map.at<unsigned char>(y-1, x) = 100;
    cv_map.at<unsigned char>(y, x-1) = 100;
    cv_map.at<unsigned char>(y+1, x-1) = 100;
    cv_map.at<unsigned char>(y-1, x+1) = 100;
  }
  // if (!cv_map.empty()) imshow("Map", cv_map);
  // waitKey(30);
}
void goal_status(const move_base_msgs::MoveBaseActionResult::ConstPtr& goal) {

      ROS_INFO_STREAM("status:" << goal->status.text);

      if (goal->status.status == 3){
        if (!cv_map.empty()) imshow("Map", cv_map);
        waitKey(30);
      }
}
int main(int argc, char** argv) {

    ros::init(argc, argv, "navigation");
    ros::NodeHandle n;
    map_sub = n.subscribe("map", 10, &mapCallback);
    pose_sub = n.subscribe("move_base/feedback", 10, &mark_searched);
    status_sub = n.subscribe("move_base/result", 10, &goal_status);
    marker_pub = n.advertise<visualization_msgs::Marker>("navigation_debug", 10);
    ros::ServiceServer service = n.advertiseService("navigation", new_goal);
    ROS_INFO("navigation ready");
    while(ros::ok()) {

      // if (!cv_map.empty()) imshow("Map", cv_map);
      // waitKey(30);

      ros::spinOnce();
    }
    return 0;

}
