#include "ros/ros.h"
#include "task2/xy.h"
#include <limits>
#include <visualization_msgs/MarkerArray.h>

ros::Subscriber circle_coor_sub;
ros::Publisher markers_pub;
ros::Publisher coordinates_pub;

float clusterDistance = 0.8;
int maxPointsInCluster = 4;
int minPointsToPublish = 4;
double detectionLifespam = 15;

struct coordinates {
    float x;
    float y;
    ros::Time timestamp;
};

struct closestClustersStruct {
    int firstCluster;
    int secondCluster;
    float distance;
};


//std::vector<coordinates> data;
std::vector< std::vector<coordinates> > clusters;
std::vector <coordinates> clusterCenters;
int marker_num = 0;
visualization_msgs::MarkerArray marker_array;

float row_distance(coordinates r1, coordinates r2) {
    //evklidska razdalja med dvema podtkoma
    return sqrt(pow(fabs(r1.x - r2.x), 2) + pow(fabs(r1.y - r2.y), 2));
}

float cluster_distance(std::vector< std::vector<coordinates> > clusters, int c1, int c2) {
    //single linkage?
    //recieving indekses of clusters in array
    float minDistance = std::numeric_limits<int>::max();
    for (int i = 0; i < clusters[c1].size(); i++) {
        for (int j = 0; j < clusters[c2].size(); j++) {
            float newDistance = row_distance(clusters[c1][i], clusters[c2][j]);
            if (newDistance < minDistance) {
                minDistance = newDistance;
            }
        }
    }

    return minDistance;

}

closestClustersStruct closest_clusters(std::vector< std::vector<coordinates> > clusters){

    //calculate the smallest distance
    float minDistance = std::numeric_limits<int>::max();
    int firstCluster = 0;
    int secondCluster = 0;
    for (int i = 0; i < clusters.size(); i++) {
        for (int j = 0; j < clusters.size(); j++) {
            if (j > i) {
                float newDistance = cluster_distance(clusters, i, j);
                if (newDistance < minDistance) {
                    minDistance = newDistance;
                    firstCluster = i;
                    secondCluster = j;
                }
            }
        }
    }
    closestClustersStruct toReturn = {firstCluster, secondCluster, minDistance};
    return toReturn;
}

std::vector< std::vector<coordinates> > clustering () {

    closestClustersStruct closest = closest_clusters(clusters);
    int ind1 = closest.firstCluster;
    int ind2 = closest.secondCluster;
    float distance = closest.distance;
    while (distance < clusterDistance && clusters.size() > 1) {
        ROS_INFO("distance %f clusters %lu", distance, clusters.size());

        std::vector <coordinates> newCluster;
        for (int i = 0; i < clusters[ind1].size(); i++) {
            newCluster.push_back(clusters[ind1][i]);
        }
        for (int i = 0; i < clusters[ind2].size(); i++) {
            newCluster.push_back(clusters[ind2][i]);
        }
        clusters.erase (clusters.begin() + ind1);
        clusters.erase (clusters.begin() + ind2);
        clusters.push_back(newCluster);

        closestClustersStruct closest = closest_clusters(clusters);
        int ind1 = closest.firstCluster;
        int ind2 = closest.secondCluster;
        distance = closest.distance;
    }

    return clusters;
}

void markerPublishing (coordinates point) {
    //if clusters have five coordinates send message

    geometry_msgs::Pose pose;
    pose.position.x = point.x;
    pose.position.y = point.y;

    visualization_msgs::Marker marker;
    marker_num += 1;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.id = marker_num;
    uint32_t shape = visualization_msgs::Marker::CUBE;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose;
    marker.frame_locked = false;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.color.r = 0.0;
    marker.color.g = 0.1;
    marker.color.b = 0.7;
    marker.color.a = 0.7;

    marker.lifetime = ros::Duration(200);
    marker_array.markers.push_back(marker);
    markers_pub.publish(marker_array);
}

coordinates checkNewMarkers() {
    coordinates tmp;
    //recalculate the average
    for (int i = 0; i < clusters.size(); i++) {
        if (clusters[i].size() >  minPointsToPublish) {

            float averageX = 0; float averageY = 0;
            for (int j = 0; j < clusters[i].size(); j++) {
                averageX = averageX + clusters[i][j].x;
                averageY = averageY + clusters[i][j].y;
            }
            averageX = averageX / clusters[i].size();
            averageY = averageY / clusters[i].size();

            tmp.x = averageX;
            tmp.y = averageY;
            tmp.timestamp = clusters[i][0].timestamp;
            clusterCenters.push_back(tmp);

            clusters.erase(clusters.begin() + i);

            return tmp;
        }
    }

    tmp.x = 100;
    tmp.y = 100;
    return tmp;
}

void clean_clusters() {

    for (int i = 0; i < clusters.size(); i++) {
        for (int j = 0; j < clusters[i].size(); j++) {
            if (ros::Time::now().toSec() - clusters[i][j].timestamp.toSec() > detectionLifespam) {
                clusters[i].erase(clusters[i].begin() + j);
                //ROS_INFO("deleting");
            }
        }
    //ROS_INFO("length: %lud %lud", clusters.size(), clusters[i].size());
    }

}

void new_circle_callback(const task2::xy::ConstPtr& msg) {

  ROS_INFO("New circle detected x: %f, y: %f.", msg->x, msg->y);
  coordinates newCircle = {msg->x, msg->y, msg->timestamp};

  //push back new point only if it's not too similar to cluster with already five points
  bool notInCluster = true;
  for (int i = 0; i < clusterCenters.size(); i++) {
      float tmp = row_distance(newCircle, clusterCenters[i]);
      if (tmp < clusterDistance) {
          notInCluster = false;
      }
  }

  if (notInCluster) {
      std::vector <coordinates> tmp;
      tmp.push_back(newCircle);
      clusters.push_back(tmp);

      //create clusters from data
      //std::vector< std::vector<coordinates> > clusters;
      clusters = clustering();

      //recalculating average and checking for new markers to send
      coordinates newMarker = checkNewMarkers();

      //check if there is new marker to publish (100 == invalid)
      if (newMarker.x != 100) {
          //publish blue markers in rviz
          markerPublishing(newMarker);

          //send coordinates to navigation node
          task2::xy msg;
          std::stringstream ss;
          //ss << elipse_coordiantes;
          msg.x = newMarker.x;
          msg.y = newMarker.y;
          msg.timestamp = newMarker.timestamp;
          coordinates_pub.publish(msg);

          //delete old detctions so that that clustering is faster
          clean_clusters();
      }
  }

}

int main(int argc, char** argv){
  ros::init(argc, argv, "processing");
  ros::NodeHandle n;

  circle_coor_sub = n.subscribe("circle_coordinates", 1, new_circle_callback);
  markers_pub = n.advertise<visualization_msgs::MarkerArray>("clustered_markers", 1);
  coordinates_pub = n.advertise<task2::xy>("clustered_coordinates", 1);


  while(ros::ok()) {
      ros::spinOnce();
  }

  return 0;
}
