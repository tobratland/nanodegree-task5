#include "ros/ros.h"
#include <visualization_msgs/Marker.h>

class AddMarkersTimed
{
public:
  AddMarkersTimed(ros::NodeHandle *nodeHandle) : n_(*nodeHandle)
  {
    marker_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    initializePosiblePositions();
  }

  void publishMarker()
  {
    createMarker();
    placeMarker();
    marker_pub_.publish(marker);
  }

  void replaceMarker()
  {
    marker_pub_.publish(marker);
    setMarkerPositionRandom();
    placeMarker();
    marker_pub_.publish(marker);
  }

private:
  ros::NodeHandle n_;
  std::vector<geometry_msgs::Point> posibilities;
  visualization_msgs::Marker marker;
  ros::Publisher marker_pub_;
  geometry_msgs::Point currentPoint;

  void placeMarker()
  {
    currentPoint.x = marker.pose.position.x;
    currentPoint.y = marker.pose.position.y;
    currentPoint.z = marker.pose.position.z;
  }

  void createMarker()
  {
    makeMarker();
    setMarkerPositionRandom();
  }

  void initializePosiblePositions()
  {
    geometry_msgs::Point pos0;
    pos0.x = 4.17;
    pos0.y = 5.77;
    pos0.z = 0.1;
    posibilities.push_back(pos0);

    geometry_msgs::Point pos1;
    pos1.x = -0.07;
    pos1.y = 4.49;
    pos1.z = 0.1;
    posibilities.push_back(pos1);
  }

  void setMarkerPositionRandom()
  {
    int i = rand() % posibilities.size();
    marker.pose.position.x = posibilities[i].x;
    marker.pose.position.y = posibilities[i].y;
    marker.pose.position.z = posibilities[i].z;
    ROS_INFO_STREAM("x: " << marker.pose.position.x);
    remove_at(posibilities, i);
  }

  void remove_at(std::vector<geometry_msgs::Point> &v, typename std::vector<geometry_msgs::Point>::size_type n)
  {
    std::swap(v[n], v.back());
    v.pop_back();
  }

  void makeMarker()
  {
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::CUBE;

    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
  }
};

int main(int argc, char **argv)
{
  ROS_INFO_ONCE("Initializing add_markers_timed");
  ros::init(argc, argv, "add_markers_timed");
  ros::NodeHandle n;

  AddMarkersTimed addMarkersTimed(&n);

  sleep(2);
  addMarkersTimed.publishMarker();
  ROS_INFO_ONCE("placed first marker");
  ros::spinOnce();
  sleep(5);
  addMarkersTimed.replaceMarker();
  ROS_INFO_ONCE("replaced marker!");
  ros::spinOnce();
  sleep(5);
  
  return 0;
}
