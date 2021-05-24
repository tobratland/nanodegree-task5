#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"

class AddMarkers
{
public:
  AddMarkers(ros::NodeHandle *nodeHandle) : n_(*nodeHandle)
  {
    marker_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    marker_pos_pub_ = n_.advertise<geometry_msgs::Point>("marker_position", 1);
    ros::Subscriber sub = n_.subscribe("odom", 15, &AddMarkers::handleOdom, this);

    initializePosiblePositions();

    while (ros::ok)
    {
      if (!firstMarkerPlaced)
      {
        createMarker();
        placeMarker();
        firstMarkerPlaced = true;
      }
    }
  }

private:
  ros::NodeHandle n_;
  bool firstMarkerPlaced;
  bool markerPickedUp;
  float distanceTolerance;
  bool markerPutDown;
  std::vector<geometry_msgs::Point> posibilities;
  visualization_msgs::Marker marker;
  ros::Publisher marker_pub_;
  ros::Publisher marker_pos_pub_;
  geometry_msgs::Point currentPoint;

  void handleOdom(const nav_msgs::Odometry::ConstPtr &msg)
  {
    if (!firstMarkerPlaced)
    {
      return;
    }

    if (!markerPickedUp && !markerPutDown)
    {
      if (distance(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z, msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z) <= distanceTolerance)
      {
        marker.action = visualization_msgs::Marker::DELETE;
        setMarkerPositionRandom();
        placeMarker();
        markerPickedUp = true;
      }
    }
    if (markerPickedUp && !markerPutDown)
    {
      if (distance(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z, msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z) <= distanceTolerance) {
        marker.action = visualization_msgs::Marker::ADD;
        placeMarker();
        markerPutDown = true;
      }
    }
  }

  float distance(float x1, float y1,
                 float z1, float x2,
                 float y2, float z2)
  {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2) + std::pow(z2 - z1, 2) * 1.0);
  }

  void placeMarker()
  {
    currentPoint.x = marker.pose.position.x;
    currentPoint.y = marker.pose.position.y;
    currentPoint.z = marker.pose.position.z;
  
    marker_pub_.publish(marker);
    marker_pos_pub_.publish(currentPoint);
  }

  void createMarker()
  {
    makeMarker();
    setMarkerPositionRandom();
  }

  void initializePosiblePositions()
  {
    distanceTolerance = 1.0;
    geometry_msgs::Point pos1;
    pos1.x = 2.2;
    pos1.y = 2.3;
    pos1.z = 1.0;
    posibilities.push_back(pos1);

    geometry_msgs::Point pos2;
    pos1.x = 2.2;
    pos1.y = 2.3;
    pos1.z = 1.0;
    posibilities.push_back(pos2);

    geometry_msgs::Point pos3;
    pos1.x = 2.2;
    pos1.y = 2.3;
    pos1.z = 1.0;
    posibilities.push_back(pos3);
  }

  void setMarkerPositionRandom()
  {
    int i = rand() % posibilities.size();
    marker.pose.position.x = posibilities[i].x;
    marker.pose.position.x = posibilities[i].y;
    marker.pose.position.x = posibilities[i].z;

    if (!firstMarkerPlaced)
    {
      posibilities.erase(posibilities.begin() + i - 1);
    }
  }

  void makeMarker()
  {
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::SPHERE;

    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.4;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;

  AddMarkers addMarkers(&n);

  ros::spin();

  return 0;
}