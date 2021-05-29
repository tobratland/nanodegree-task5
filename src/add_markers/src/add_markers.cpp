#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

class AddMarkers
{
public:
  AddMarkers(ros::NodeHandle *nodeHandle) : n_(*nodeHandle)
  {
    marker_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    marker_pos_pub_ = n_.advertise<geometry_msgs::Point>("marker_position", 1);
    odomSub = n_.subscribe("/amcl_pose", 15, &AddMarkers::handlePose, this);
    distanceTolerance = 0.25;

    initializePosiblePositions();
    createMarker();
    placeMarker();
    firstMarkerPlaced = true;
    ROS_INFO_ONCE("put first marker down!");
    ROS_INFO_STREAM("x: " << currentPoint.x << " y: " << currentPoint.y << " z: " << currentPoint.z);
    
  }

  void publishMarker()
  {
    ROS_INFO_ONCE("Published marker");
    marker_pub_.publish(marker);
    marker_pos_pub_.publish(currentPoint);
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
  ros::Subscriber odomSub;

  void handlePose(const geometry_msgs::PoseWithCovarianceStamped &msg)
  {
    if (!firstMarkerPlaced)
    {
      ROS_INFO_ONCE("FIRST MARKER NOT PLACED!");
      return;
    }

    if (!markerPickedUp && !markerPutDown)
    {
      if (distance(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z, msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z) <= distanceTolerance)
      {
        marker.action = visualization_msgs::Marker::DELETE;
        setMarkerPositionRandom();
        placeMarker();
        publishMarker();
        markerPickedUp = true;
        ROS_INFO_ONCE("PICKED UP MARKER!");
      }
      else
      {
        ROS_INFO_ONCE("NOT CLOSE ENOUGH!!");
      }
    }
    if (markerPickedUp && !markerPutDown)
    {
      if (distance(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z, msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z) <= distanceTolerance)
      {
        marker.action = visualization_msgs::Marker::ADD;
        placeMarker();
        publishMarker();
        markerPutDown = true;
        ROS_INFO_ONCE("PLACED MARKER!");
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
    pos1.x = 4.5;
    pos1.y = 6.0;
    pos1.z = 0.1;
    posibilities.push_back(pos1);

    geometry_msgs::Point pos2;
    pos2.x = 4.23;
    pos2.y = 4.78;
    pos2.z = 0.1;
    posibilities.push_back(pos2);

    geometry_msgs::Point pos3;
    pos3.x = 1.9;
    pos3.y = 5.48;
    pos3.z = 0.1;
    posibilities.push_back(pos3);

    geometry_msgs::Point pos4;
    pos4.x = -0.07;
    pos4.y = 4.49;
    pos4.z = 0.1;
    posibilities.push_back(pos4);
    
    geometry_msgs::Point pos5;
    pos5.x = 3.75;
    pos5.y = -1.26;
    pos5.z = 0.1;
    posibilities.push_back(pos5);

    
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
  ROS_INFO_ONCE("Initializing add_markers");
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;

  AddMarkers addMarkers(&n);

  ros::Rate loopRate(5);

  while(ros::ok) {
    addMarkers.publishMarker();
    ros::spinOnce();
  }

  return 0;
}