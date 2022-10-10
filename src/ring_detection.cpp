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

#include <math.h>
#include <sensor_msgs/Image.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#include <sound_play/sound_play.h>
#include <unistd.h>
#include "task2/xy.h"
#include "task2/ColorRecognition.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <algorithm>

using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher goal_pub;
ros::Publisher marker_pub;
ros::Subscriber image_sub;
ros::Subscriber depth_sub;
ros::Subscriber pose_sub;
ros::Publisher ring_coor_pub;
ros::Publisher vel_pub;

tf2_ros::Buffer tf_buf;
geometry_msgs::TransformStamped trans;
visualization_msgs::Marker marker;

Mat histRed; Mat histGreen; Mat histBlue; Mat histBlack;
int maximum_depth = 80;
float dims[3] = {0.0, 0.0, 0.0};
geometry_msgs::Pose detected_ring_pose;
//0 = red, 1 = green, 2 = blue, 3 = black
float color[4];

Mat get_average_histograms(std::string path) {
    vector<cv::String> fn;
    glob(path, fn, false);

    Mat avg;
    size_t count = fn.size(); //number of png files in images folder
    for (size_t i=0; i<count; i++) {
        Mat img = imread(fn[i]);
        Mat imgHSV;
        cv::cvtColor(img, imgHSV, CV_BGR2HSV);
        //cv::calcHist(&imgHSV, [0, 1, 2], None, [8, 8, 8], range)
        Mat hist;
        int h_bins = 50, s_bins = 60; int histSize[] = { h_bins, s_bins };
        float h_ranges[] = { 0, 180 }; float s_ranges[] = { 0, 256 }; const float* ranges[] = { h_ranges, s_ranges };
        int channels[] = { 0, 1 };
        calcHist( &imgHSV, 1, channels, Mat(), hist, 2, histSize, ranges, true, false );
        normalize( hist, hist, 0, 1, NORM_MINMAX, -1, Mat() );

        if (i == 0) avg = hist;
        else cv::add(avg, hist, avg);
    }

    for(int i=0;i<avg.total();i++)
        ((unsigned char*)avg.data)[i]/=avg.total();

    return avg;
}


void position_in_map (const move_base_msgs::MoveBaseActionFeedback::ConstPtr& robot_pose) {

  trans.transform.translation.x = robot_pose->feedback.base_position.pose.position.x;
  trans.transform.translation.y = robot_pose->feedback.base_position.pose.position.y;
  trans.transform.translation.z = robot_pose->feedback.base_position.pose.position.z;
  trans.transform.rotation = robot_pose->feedback.base_position.pose.orientation;

  uint32_t shape = visualization_msgs::Marker::CUBE;
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "map_goals";
  marker.id = 0;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = robot_pose->feedback.base_position.pose;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.r = 1.0;
  marker.color.g = 0.8;
  marker.color.b = 0.9;
  marker.color.a = 0.8;
  marker.lifetime = ros::Duration();
  marker_pub.publish(marker);

}

void image_callback(const ImageConstPtr& data, const ImageConstPtr& depth_img){
  //ROS_INFO("I got a new image");
  ros::NodeHandle n;

  cv_bridge::CvImagePtr bridge_img;
  try
  {
    bridge_img = cv_bridge::toCvCopy(depth_img, sensor_msgs::image_encodings::TYPE_16UC1);
  }
  catch (cv_bridge::Exception& error)
  {
    ROS_ERROR("error");
  }

  Mat cv_image;
  cv_image = bridge_img->image;
  dims[0] = cv_image.rows;
  dims[1] = cv_image.cols;

  double min;
  double max;
  cv::minMaxIdx(cv_image, &min, &max);
  cv::Mat adjMap;
  cv::convertScaleAbs(cv_image, adjMap, 255 / max);

  cv::Mat treshold;
  cv::inRange( adjMap, cv::Scalar( maximum_depth, maximum_depth, maximum_depth), cv::Scalar( 255, 255, 255 ), treshold );
  adjMap = adjMap - treshold;

  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  findContours(adjMap, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  //Scalar color = Scalar(0,0,255);
  //drawContours( tresh, contours, -1, color, CV_FILLED, 8);

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
      if (dist<10){
          //zgornja tretina slike
          if (e1.center.y < 2*cv_image.rows/3) {
              //razmerje med velikostjo krogov ni vec kot 40% njune velikosti
              if (abs(e1.size.width - e2.size.width) / std::max(e1.size.width, e2.size.width) < 0.4) {
                  //krog ni vecji od tretine slike
                  if (e1.size.height < cv_image.rows/3 && e1.size.height < cv_image.rows/3 && e2.size.width < cv_image.rows/3 && e2.size.height < cv_image.rows/3) {

                  //if (abs( 1 - (e1.size.width / e1.size.height)) < 0.2 && abs( 1 - (e2.size.width / e2.size.height)) < 0.2) {
                          candidates.push_back(e1);
                          candidates.push_back(e2);
                  }
                  else {
                      ROS_INFO("krog vecji od tretine slike");
                  }
              }
              else {
                  ROS_INFO("razmerje med velikostjo");
              }
          }
             else {
                 ROS_INFO("ni zgornja tretina");
             }
          //}
      }
      else {
          ROS_INFO("distance");
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
      // ellipse(adjMap, e1, (0, 0, 255), 2);
      // ellipse(adjMap, e2, (0, 0, 255), 2);

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

      //get color image
      cv_bridge::CvImagePtr bridge_rgb;
      try
      {
        bridge_rgb = cv_bridge::toCvCopy(data, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& error)
      {
        ROS_ERROR("error");
      }
      Mat cv_image_rgb;
      cv_image_rgb = bridge_rgb->image;
      dims[0] = cv_image_rgb.rows;
      dims[1] = cv_image_rgb.cols;

      imshow("depth image", adjMap);
      waitKey(30);
      Mat elipse;

      Rect box = Rect(x_min,y_min,x_max-x_min,y_max-y_min);
      if (0 <= box.x  && 0 <= box.width && box.x + box.width <= bridge_img->image.cols && 0 <= box.y && 0 <= box.height && box.y + box.height <= bridge_img->image.rows){
        adjMap(Rect(box)).copyTo(elipse);

        //creating image with mask for color ColorRecognition
        Mat treshold2;
        cv::inRange(elipse, cv::Scalar( 1, 1, 1), cv::Scalar( 255, 255, 255), treshold2);
        Mat color_boxed;
        cv_image_rgb(Rect(box)).copyTo(color_boxed);
        Mat masked;
        color_boxed.copyTo(masked, treshold2);

        Mat imgHSV;
        cv::cvtColor(masked, imgHSV, CV_BGR2HSV);
        Mat hist;
        int h_bins = 50, s_bins = 60; int histSize[] = { h_bins, s_bins };
        float h_ranges[] = { 0, 180 }; float s_ranges[] = { 0, 256 }; const float* ranges[] = { h_ranges, s_ranges };
        int channels[] = { 0, 1 };
        calcHist( &imgHSV, 1, channels, Mat(), hist, 2, histSize, ranges, true, false );
        normalize( hist, hist, 0, 1, NORM_MINMAX, -1, Mat() );
        color[0] = compareHist(histRed, hist, CV_COMP_CORREL);
        color[1] = compareHist(histGreen, hist, CV_COMP_CORREL);
        color[2] = compareHist(histBlue, hist, CV_COMP_CORREL);
        color[3] = compareHist(histBlack, hist, CV_COMP_CORREL);

        // red = 0, green = 1, blue = 2, black = 3
        float max_prob = 0; int max_color = 0;
        for (int c = 0; c < 4; c++) {
            if (color[c] > max_prob) {
                max_color = c;
                max_prob = color[c];
            }
        }

        double averageValue = 0;
        int quantity = 0;
        for (int j = 0; j < elipse.rows; j++) {
            for (int k = 0; k < elipse.cols; k++) {
                Scalar colourScalar = elipse.at<unsigned char>(j, k);
                if (colourScalar[0] > 0.0) {
                    averageValue = averageValue + colourScalar[0];
                    quantity++;
                }
                else{
                  // elipse.at<unsigned char>(j, k) = 255;
                }
            }
        }

        //imshow("depth image", elipse);
        //waitKey(30);
        averageValue = averageValue / quantity;

        //Scalar temp = mean( elipse );
        //ROS_INFO("mean %f, average %f", temp[0], averageValue);
        float distance = (averageValue*(max/255))/1000.0;
        //if (mean > 0.6){
        //angle_distance = get_pose(e1, mean);
        //ROS_INFO("%f", distance);
        if (distance > 0.8) {
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

    	//if (max_prob > 0.4) {

    		task2::xy msg;
    		std::stringstream ss;
    		//ss << elipse_coordiantes;
    		msg.x = detected_ring_pose.position.x;
    		msg.y = detected_ring_pose.position.y;
    		msg.color = max_color;
    		msg.probability = max_prob;
    		msg.timestamp = ros::Time::now();
    		//msg.z_coordinate = goalZ
    		ring_coor_pub.publish(msg);
        }
		//ROS_INFO("%s", msg.data.c_str());
	//}
	//else {
		//ROS_INFO("Dicarded because too low color color probability.");
	//}

      }
      else{
          ROS_INFO("Bad ring detected");
      }
    }
  }
}


int main(int argc, char** argv){
  ROS_INFO("setting up ring detection");
  ros::init(argc, argv, "ring");
  ros::NodeHandle nh;
  sound_play::SoundClient sc;

  pose_sub = nh.subscribe("move_base/feedback", 10, &position_in_map);
  ring_coor_pub = nh.advertise<task2::xy>("ring_coordinates", 1000);
  marker_pub = nh.advertise<visualization_msgs::Marker>("position", 10);

  tf2_ros::TransformListener tfListener(tf_buf);
  trans = tf_buf.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(10.0) );

  message_filters::Subscriber<Image> image_sub(nh, "/camera/rgb/image_raw", 1);
  message_filters::Subscriber<Image> depth_sub(nh, "/camera/depth_registered/image_raw", 1);
  typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, depth_sub);
  sync.registerCallback(boost::bind(&image_callback, _1, _2));

  //get histograms
  histRed = get_average_histograms("/home/pikanogavicka/ROS/src/color_classificator/red/*.png");
  histGreen = get_average_histograms("/home/pikanogavicka/ROS/src/color_classificator/green/*.png");
  histBlue = get_average_histograms("/home/pikanogavicka/ROS/src/color_classificator/blue/*.png");
  histBlack = get_average_histograms("/home/pikanogavicka/ROS/src/color_classificator/black/*.png");

  ROS_INFO("starting ring detection");

  while(ros::ok()) {
      ros::spinOnce();
  }

  return 0;
}
