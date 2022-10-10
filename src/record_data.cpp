#include "ros/ros.h"
//#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

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

ros::Subscriber image_sub;
ros::Subscriber depth_sub;

int maximum_depth = 80;
float dims[3] = {0.0, 0.0, 0.0};
geometry_msgs::Pose detected_ring_pose;
int counter = 0;

void image_callback(const ImageConstPtr& data, const ImageConstPtr& depth_img){
  ROS_INFO("I got a new image");
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
          if (e1.center.y < 2*cv_image.rows/3) {
              //if (abs( 1 - (e1.size.width / e1.size.height)) < 0.2 && abs( 1 - (e2.size.width / e2.size.height)) < 0.2) {
                      candidates.push_back(e1);
                      candidates.push_back(e2);
          //}
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

      Mat elipse;
      Mat elipse_depth;
      Rect box = Rect(x_min,y_min,x_max-x_min,y_max-y_min);
      if (0 <= box.x  && 0 <= box.width && box.x + box.width <= bridge_img->image.cols && 0 <= box.y && 0 <= box.height && box.y + box.height <= bridge_img->image.rows){
        adjMap(Rect(box)).copyTo(elipse_depth);
        Mat treshold2;
        cv::inRange( elipse_depth, cv::Scalar( 1, 1, 1), cv::Scalar( 255, 255, 255), treshold2 );
        cv_image(Rect(box)).copyTo(elipse);
        Mat masked;
        elipse.copyTo(masked, treshold2);
        imshow("elipse", masked);
        waitKey(30);

        std::string colour = "/home/pikanogavicka/ROS/src/task2/src/black/black";
        std::string ending = ".png";
        std::stringstream file_name;
        vector<int> compression_params;
        compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
        compression_params.push_back(9);
        file_name << colour << counter << ending;

        try {
            imwrite(file_name.str(),  masked, compression_params);
            counter++;
        }
        catch (runtime_error& ex) {
            fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
        }

        fprintf(stdout, "Saved PNG file with alpha data.\n");

      }
      else{
          ROS_INFO("Bad ring detected");
      }
    }
  }
}


int main(int argc, char** argv){
  ROS_INFO("starting ring detection");
  ros::init(argc, argv, "ring");
  ros::NodeHandle nh;

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
