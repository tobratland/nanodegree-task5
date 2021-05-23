#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    visualization_msgs::Marker first_marker;
    visualization_msgs::Marker second_marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    first_marker.header.frame_id = "map";
    first_marker.header.stamp = ros::Time::now();

    second_marker.header.frame_id = "map"; 
    second_marker.header.stamp = ros::Time::now();
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    first_marker.ns = "basic_shapes";
    first_marker.id = 0;

    second_marker.ns = "basic_shapes";
    second_marker.id = 1;
    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    first_marker.type = shape;
    second_marker.type = shape;
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    first_marker.action = visualization_msgs::Marker::ADD;
    second_marker.action = visualization_msgs::Marker::ADD;
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    first_marker.pose.position.x = 0;
    first_marker.pose.position.y = 0;
    first_marker.pose.position.z = 0;
    first_marker.pose.orientation.x = 0.0;
    first_marker.pose.orientation.y = 0.0;
    first_marker.pose.orientation.z = 0.0;
    first_marker.pose.orientation.w = 1.0;

    second_marker.pose.position.x = 3;
    second_marker.pose.position.y = 3;
    second_marker.pose.position.z = 2;
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    first_marker.scale.x = 0.25;
    first_marker.scale.y = 0.25;
    first_marker.scale.z = 0.25;

    second_marker.scale.x = 0.25;
    second_marker.scale.y = 0.25;
    second_marker.scale.z = 0.25;
    // Set the color -- be sure to set alpha to something non-zero!
    first_marker.color.r = 0.0f;
    first_marker.color.g = 1.0f;
    first_marker.color.b = 0.0f;
    first_marker.color.a = 1.0;


    second_marker.color.r = 1.0f;
    second_marker.color.g = 0.5f;
    second_marker.color.b = 0.5f;
    second_marker.color.a = 1.0;


    first_marker.lifetime = ros::Duration(5.0);
    second_marker.lifetime = ros::Duration(5.0);
    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    ROS_INFO_ONCE("adding first marker");
    marker_pub.publish(first_marker);
    sleep(10);
    ROS_INFO_ONCE("adding second marker");
    marker_pub.publish(second_marker);

    // Cycle between different shapes
    switch (shape)
    {
    case visualization_msgs::Marker::CUBE:
      shape = visualization_msgs::Marker::SPHERE;
      break;
    case visualization_msgs::Marker::SPHERE:
      shape = visualization_msgs::Marker::ARROW;
      break;
    case visualization_msgs::Marker::ARROW:
      shape = visualization_msgs::Marker::CYLINDER;
      break;
    case visualization_msgs::Marker::CYLINDER:
      shape = visualization_msgs::Marker::CUBE;
      break;
    }

    r.sleep();
  }
}
