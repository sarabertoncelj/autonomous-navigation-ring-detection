#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <nav_msgs/GetMap.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatus.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf2_ros/transform_listener.h>
#include "task2/xy.h"

using namespace std;
using namespace cv;

Mat cv_map;
float map_resolution = 0;
geometry_msgs::TransformStamped map_transform;
geometry_msgs::TransformStamped world_transform;
geometry_msgs::Pose approach_goal;
move_base_msgs::MoveBaseActionGoal current_goal;
int active_goals;


int goal_index = 0;

ros::Publisher goal_pub;
ros::Publisher marker_pub;
ros::Publisher cancel_goal;
ros::Subscriber goal_sub;
ros::Subscriber circle_sub;
ros::Subscriber map_sub;

float position_x [4] = {-0.46, -1.28, 0.49, -2.4};
float position_y [4] = {2.02, 0.7, 0.28, 2.42};

bool select_goal = true;
bool circle_detected = false;
bool cancel_all = true;

struct pixel {
    float x;
    float y;
};

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
void goal_status(const move_base_msgs::MoveBaseActionResult::ConstPtr& goal) {

      ROS_INFO_STREAM("status:" << goal->status.text);
      if (goal->status.status == 3 && !circle_detected){
        ROS_INFO("goal reached");
        select_goal = true;
        goal_index++;
      }
      if (goal->status.status == 4 && !circle_detected){
        ROS_INFO("goal aborted");
        select_goal = true;
        goal_index++;
      }
      if (goal->status.status == 3 && circle_detected && !select_goal){
        select_goal = true;
        cancel_all = true;
        circle_detected = false;
      }
      if (goal->status.status == 4 && circle_detected && !select_goal){
        select_goal = true;
        cancel_all = true;
        circle_detected = false;
      }
}

vector<geometry_msgs::Point> pixels_in_radius(geometry_msgs::Point o, int n, int colour){
  vector<geometry_msgs::Point> result;
  for (int x = 1; x < cv_map.cols; x++) {
    int dx = abs(o.x - x);
    int x2 = dx*dx;
    for (int y = 1; y < cv_map.rows; y++) {
      int v = (int)cv_map.at<unsigned char>(y, x);
      int dy = abs(o.y - y);
      int y2= dy*dy;
      int d2 = x2 + y2;
      if (v == colour && d2 <= n*n){
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;

        cv_map.at<unsigned char>(y, x) = 200;
        result.push_back(p);
      }
   }
 }
 return result;
}

void detected_circle(const task2::xy::ConstPtr& msg) {
    ROS_INFO("circle_detected");
    tf2_ros::Buffer tf_buf;
    tf2_ros::TransformListener tfListener(tf_buf);

    geometry_msgs::Point pt;
    geometry_msgs::Point transformed_pt;

    geometry_msgs::TransformStamped robot_world;
    robot_world = tf_buf.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(10.0) );
    geometry_msgs::Point robot_pt;
    geometry_msgs::Point robot_map;
    robot_pt.x = robot_world.transform.translation.x;
    robot_pt.y = robot_world.transform.translation.y;
    robot_pt.z = robot_world.transform.translation.z;
    tf2::doTransform(robot_pt, robot_map, world_transform);
    robot_map.x = robot_map.x / map_resolution;
    robot_map.y = cv_map.rows - (robot_map.y / map_resolution);
    cv_map.at<unsigned char>(robot_map.y, robot_map.x) = 100;
    pt.x = msg->x;
    pt.y = msg->y;
    pt.z = 0.0;


    tf2::doTransform(pt, transformed_pt, world_transform);
    transformed_pt.x = transformed_pt.x / map_resolution;
    transformed_pt.y = cv_map.rows - (transformed_pt.y / map_resolution);
    vector<geometry_msgs::Point> black_pixels = pixels_in_radius(transformed_pt, 4, 0);
    ROS_INFO("found the wall");
    int max_distance = 0;
    int max_i, max_j;
    for (int i = 0; i < black_pixels.size(); i++){
      for (int j = i+1; j < black_pixels.size(); j++){
        ROS_INFO("i: %d, j: %d", i,j);
        int dx = abs(black_pixels[i].x - black_pixels[j].x);
        int dy = abs(black_pixels[i].y - black_pixels[j].y);
        int d = dx*dx + dy+dy;
        if (d > max_distance){
          max_distance=d;
          max_i = i;
          max_j = j;
        }
      }
    }
    ROS_INFO("max: (%f, %f), (%f, %f)", black_pixels[max_i].x, black_pixels[max_i].y, black_pixels[max_j].x, black_pixels[max_j].y);
    int vx = black_pixels[max_i].x - black_pixels[max_j].x;
    int vy = black_pixels[max_i].y - black_pixels[max_j].y;

    int x1 = transformed_pt.x - vy;
    int y1 = transformed_pt.y + vx;

    cv_map.at<unsigned char>(transformed_pt.y, transformed_pt.x) = 240;

    ROS_INFO("pixel at (%d, %d)", x1, y1);
    int v1 = (int)cv_map.at<unsigned char>(y1, x1);
    ROS_INFO("2 | pixel at (%d, %d): %d", x1, y1, v1);
    cv_map.at<unsigned char>(y1, x1) = 200;

    int x2 = transformed_pt.x + vy;
    int y2 = transformed_pt.y - vx;
    int v2 = (int)cv_map.at<unsigned char>(y2, x2);
    ROS_INFO("2 | pixel at (%d, %d): %d", x2, y2, v2);
    cv_map.at<unsigned char>(y2, x2) = 200;

    geometry_msgs::Point approach_pt;
    geometry_msgs::Point pt2;

    if (v1 == 255 && v2 == 255) {

      int d1 = (robot_map.x - x1)*(robot_map.x - x1) + (robot_map.y - y1)+(robot_map.y - y1);
      int d2 = (robot_map.x - x2)*(robot_map.x - x2) + (robot_map.y - y2)+(robot_map.y - y2);
      if (d1<d2){

        pt2.x = (float)x1 * map_resolution;
        pt2.y = (float)(cv_map.rows - y1) * map_resolution;
        pt2.z = 0.0;

        tf2::doTransform(pt2, approach_pt, map_transform);
      }
      else {

        pt2.x = (float)x2 * map_resolution;
        pt2.y = (float)(cv_map.rows - y2) * map_resolution;
        pt2.z = 0.0;

        tf2::doTransform(pt2, approach_pt, map_transform);
      }
    }

    else if (v1 == 255) {

      pt2.x = (float)x1 * map_resolution;
      pt2.y = (float)(cv_map.rows - y1) * map_resolution;
      pt2.z = 0.0;

      tf2::doTransform(pt2, approach_pt, map_transform);
  	}

    else if (v2 == 255) {

      pt2.x = (float)x2 * map_resolution;
      pt2.y = (float)(cv_map.rows - y2) * map_resolution;
      pt2.z = 0.0;

      tf2::doTransform(pt2, approach_pt, map_transform);
    }

    float orientation_angle = acos(sqrt((pt2.y-pt.y)/((pt2.y-pt.y)*(pt2.y-pt.y)+(pt2.x-pt.x)*(pt2.x-pt.x))));
    tf2::Quaternion goal_orientation;
    goal_orientation.setRPY(0, 0, orientation_angle);

    approach_goal.position.x = approach_pt.x;
    approach_goal.position.y = approach_pt.y;
    approach_goal.position.z = approach_pt.z;
    approach_goal.orientation.x = goal_orientation.x();
    approach_goal.orientation.y = goal_orientation.y();
    approach_goal.orientation.z = goal_orientation.z();
    approach_goal.orientation.w = goal_orientation.w();

    select_goal = true;
    circle_detected = true;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "goal_processor");
    ros::NodeHandle n;

    map_sub = n.subscribe("map", 10, &mapCallback);
    goal_pub = n.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 10);
    marker_pub = n.advertise<visualization_msgs::Marker>("goals", 10);
    goal_sub = n.subscribe("/move_base/result", 10, &goal_status);
    circle_sub = n.subscribe("/clustered_coordinates", 1, &detected_circle);
    cancel_goal = n.advertise<actionlib_msgs::GoalID>("move_base/cancel", 10);

    while(ros::ok()) {
      //current_goal.goal_id.id = "current";
      //if (!cv_map.empty()) imshow("Map", cv_map);
      //waitKey(30);
      if (select_goal){
        cancel_goal.publish(current_goal.goal_id);
        if (!circle_detected && goal_index<4){
          ROS_INFO("goal number %d.", goal_index);
          geometry_msgs::Pose new_goal;

          new_goal.position.x = position_x[goal_index];
          new_goal.position.y = position_y[goal_index];
          new_goal.position.z = 0.0;
          new_goal.orientation.w = 1.0;

          move_base_msgs::MoveBaseActionGoal move_to_goal;
          move_to_goal.goal.target_pose.header.frame_id = "/map";
          move_to_goal.goal.target_pose.header.stamp = ros::Time::now();
          move_to_goal.goal.target_pose.pose = new_goal;
          //move_to_goal.goal_id.id = "current";
          current_goal = move_to_goal;
          goal_pub.publish(move_to_goal);
          ROS_INFO("goal: %f, %f.", move_to_goal.goal.target_pose.pose.position.x, move_to_goal.goal.target_pose.pose.position.y );
          uint32_t shape = visualization_msgs::Marker::CUBE;
    			visualization_msgs::Marker marker;
    			marker.header.frame_id = "/map";
    			marker.header.stamp = ros::Time::now();
    			marker.ns = "map_goals";
    			marker.id = 0;
    			marker.type = shape;
    			marker.action = visualization_msgs::Marker::ADD;
    			marker.pose = move_to_goal.goal.target_pose.pose;
    			marker.scale.x = 0.2;
    			marker.scale.y = 0.2;
    			marker.scale.z = 0.2;
    			marker.color.r = 0.4f;
    			marker.color.g = 0.0f;
    			marker.color.b = 0.0f;
    			marker.color.a = 0.8;
    			marker.lifetime = ros::Duration();
    			marker_pub.publish(marker);

          select_goal = false;
        }
        else if (circle_detected){

          current_goal.goal.target_pose.header.frame_id = "/map";
          current_goal.goal.target_pose.header.stamp = ros::Time::now();
          current_goal.goal.target_pose.pose = approach_goal;
          goal_pub.publish(current_goal);
          ROS_INFO("approaching circle");
          ROS_INFO("goal: %f, %f.", current_goal.goal.target_pose.pose.position.x, current_goal.goal.target_pose.pose.position.y );

          uint32_t shape = visualization_msgs::Marker::CUBE;
    			visualization_msgs::Marker marker;
    			marker.header.frame_id = "/map";
    			marker.header.stamp = ros::Time::now();
    			marker.ns = "map_goals";
    			marker.id = 0;
    			marker.type = shape;
    			marker.action = visualization_msgs::Marker::ADD;
    			marker.pose = current_goal.goal.target_pose.pose;
    			marker.scale.x = 0.2;
    			marker.scale.y = 0.2;
    			marker.scale.z = 0.2;
    			marker.color.r = 0.4f;
    			marker.color.g = 0.0f;
    			marker.color.b = 0.6f;
    			marker.color.a = 0.8;
    			marker.lifetime = ros::Duration();
    			marker_pub.publish(marker);

          select_goal = false;
        }

      }

      ros::spinOnce();
    }
    return 0;

}
