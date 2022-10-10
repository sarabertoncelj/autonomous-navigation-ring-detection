#include "ros/ros.h"
//#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>


#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
//#include <geometry_msgs/Twist.h>

#include <math.h>
#include <sensor_msgs/Image.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#include <sound_play/sound_play.h>
#include <unistd.h>
#include "task2/xy.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;


tf2_ros::Buffer tf_buf;
ros::Publisher goal_pub;
ros::Subscriber image_sub;
ros::Subscriber depth_sub;
ros::Publisher circle_coor_pub;
ros::Publisher vel_pub;
ros::Subscriber pose_sub;

geometry_msgs::TransformStamped trans;

float dims[3] = {0.0, 0.0, 0.0};
geometry_msgs::Pose detected_ring_pose;

void position_in_map (const move_base_msgs::MoveBaseActionFeedback::ConstPtr& robot_pose) {

  trans.transform.translation.x = robot_pose->feedback.base_position.pose.position.x;
  trans.transform.translation.y = robot_pose->feedback.base_position.pose.position.y;
  trans.transform.translation.z = robot_pose->feedback.base_position.pose.position.z;
  trans.transform.rotation = robot_pose->feedback.base_position.pose.orientation;

}

void image_callback(const ImageConstPtr& data, const ImageConstPtr& depth_img){
  ROS_INFO("I got a new image");
  ros::NodeHandle n;
  cv_bridge::CvImagePtr bridge_img;
  try
  {
    bridge_img = cv_bridge::toCvCopy(data, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& error)
  {
    ROS_ERROR("error");
  }
  Mat cv_image;
  cv_image = bridge_img->image;
  dims[0] = cv_image.rows;
  dims[1] = cv_image.cols;
  Mat gray;
  cvtColor(cv_image, gray, COLOR_BGR2GRAY);
  Mat img;
  equalizeHist(gray, img);
  //Mat blurred;
  //GaussianBlur(img, blurred, Size(7, 7), 0);
  Mat tresh;
  threshold(gray, tresh, 120, 255, THRESH_BINARY);
  Mat bilateral;
  int kernel_length = 30;
  bilateralFilter(tresh, bilateral, kernel_length, kernel_length * 2, kernel_length / 2);
  //bilateralFilter(tresh, bilateral, 9, 75, 75);
  // Mat canny;
  // Canny(bilateral, canny, 10, 255);
  //Mat blurred2;
  //GaussianBlur(canny, blurred2, Size(3, 3), 0);
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  findContours(bilateral, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  Scalar color = Scalar(0,0,255);
  //drawContours( tresh, contours, -1, color, CV_FILLED, 8);
  // imshow("bilateral", bilateral);
  // waitKey(30);

  vector<RotatedRect> elps;
  for( int i = 0; i < contours.size(); i++ )
  {
    if (contours[i].size()>=20)
      elps.push_back( fitEllipse( (contours[i]) ));
  }
  vector<RotatedRect> candidates;
  for (int i =0; i < elps.size(); i++){
    for (int j = i + 1; j < elps.size(); j++){
      RotatedRect e1 = elps[i];
      RotatedRect e2 = elps[j];
      float dist = sqrt(((e1.center.x - e2.center.x) * (e1.center.x - e2.center.x) + (e1.center.y - e2.center.y) * (e1.center.y - e2.center.y)));
      if (dist<5){
          if (e1.center.y > cv_image.rows/3 && e1.center.y < 2*cv_image.rows/3) {
              if (abs( 1 - (e1.size.width / e1.size.height)) < 0.2 && abs( 1 - (e2.size.width / e2.size.height)) < 0.2) {
                  if(e1.size.width > 50 || e2.size.width > 50) {
                      candidates.push_back(e1);
                      candidates.push_back(e2);
                  }
                  else {
                      ROS_INFO("Discarded because too small");
                  }
          }
        }
      }
    }
  }
  //Mat drawingOn = cv_image;
  ROS_INFO("Processing is done! Found %lu candidates", candidates.size()/2);
  int nmb_rings_detected = candidates.size()/2;

  if (nmb_rings_detected > 0){

    for (int i = 0; i<candidates.size(); i+=2)
    {
      RotatedRect e1 = candidates[i];
      RotatedRect e2 = candidates[i+1];
      float size = (e1.size.width+e1.size.height)/2;
      float center[2] = {e1.center.x, e1.center.y};

      //drawing ellipses on cv_image
      ellipse(cv_image, e1, (0, 0, 255), 2);
      ellipse(cv_image, e2, (0, 0, 255), 2);

      int x1 = int(center[0] - size / 2);
      int x2 = int(center[0] + size / 2);
      float x_min;
      if (x1>0){
        x_min = x1;
      }
      else{
        x_min = 0;
      }
      float x_max;
      if (x2<cv_image.rows){
        x_max = x2;
      }

      else{
        x_max = cv_image.rows;
      }
      int y1 = int(center[1] - size / 2);
      int y2 = int(center[1] + size / 2);

      float y_min;
      if (y1>0){
        y_min = y1;
      }
      else{
        y_min = 0;
      }
      float y_max;
      if (y2<cv_image.cols){
        y_max = y2;
      }

      else{
        y_max = cv_image.cols;
      }

      cv_bridge::CvImagePtr depth_image;
      try
      {
        depth_image = cv_bridge::toCvCopy(depth_img, sensor_msgs::image_encodings::TYPE_16UC1);
      }
      catch (cv_bridge::Exception& error)
      {
        ROS_ERROR("error");
      }

      //if (candidates.size() > 0) {
          // imshow("cv_image", cv_image);
          // waitKey(30);
      //}

      Mat elipse;
      Rect box = Rect(x_min,y_min,x_max-x_min,y_max-y_min);
      if (0 <= box.x  && 0 <= box.width && box.x + box.width <= depth_image->image.cols && 0 <= box.y && 0 <= box.height && box.y + box.height <= depth_image->image.rows){
        depth_image->image(Rect(box)).copyTo(elipse);

        Scalar temp = mean( elipse );
        float distance = temp.val[0]/1000.0;
        //if (mean > 0.6){
        //angle_distance = get_pose(e1, mean);

        int k_f = 525;
        float elipse_x = dims[1]/2 - e1.center.x;
        float elipse_y = dims[0]/2 - e1.center.y;
        float angle_to_target = atan2(elipse_x,k_f);

        float x = cos(angle_to_target) * (distance);
        float y = sin(angle_to_target) * (distance);

        //pozicija kroga na zemljevidu
        geometry_msgs::PointStamped point_s;
        point_s.point.x = x;
        point_s.point.y = y;
        point_s.point.z = 0.3;
        point_s.header.frame_id = "base_link";
        point_s.header.stamp = ros::Time::now();
        geometry_msgs::PointStamped point_world;
        tf2::doTransform(point_s, point_world, trans);

        detected_ring_pose.position.x = point_world.point.x;
        detected_ring_pose.position.y = point_world.point.y;
        //detected_ring_pose.position.z = point_world.point.z;

        //}
        // else{
        //     ROS_INFO("Too close.");
        // }

        task2::xy msg;
        std::stringstream ss;
        //ss << elipse_coordiantes;
        msg.x = detected_ring_pose.position.x;
        msg.y = detected_ring_pose.position.y;
        msg.timestamp = ros::Time::now();
        //msg.z_coordinate = goalZ
        circle_coor_pub.publish(msg);
        //ROS_INFO("%s", msg.data.c_str());

      }
      else{
          // detected_ring_pose.position.x = 100;
          // detected_ring_pose.position.y = 100;
          // detected_ring_pose.position.z = 100;
          ROS_INFO("Bad ring detected");
      }
    }
  }
}


int main(int argc, char** argv){
  ROS_INFO("starting circle detection");
  ros::init(argc, argv, "rings");
  ros::NodeHandle nh;
  sound_play::SoundClient sc;


  tf2_ros::TransformListener tfListener(tf_buf);
  trans = tf_buf.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(10.0) );

  pose_sub = nh.subscribe("move_base/feedback", 10, &position_in_map);
  circle_coor_pub = nh.advertise<task2::xy>("circle_coordinates", 1000);


  message_filters::Subscriber<Image> image_sub(nh, "/camera/rgb/image_raw", 1);
  message_filters::Subscriber<Image> depth_sub(nh, "/camera/depth_registered/image_raw", 1);
  typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, depth_sub);
  sync.registerCallback(boost::bind(&image_callback, _1, _2));

  while(ros::ok()) {
      ros::spinOnce();
  }

  return 0;
}
