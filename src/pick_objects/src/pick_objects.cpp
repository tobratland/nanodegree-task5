#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

class PickObjects
{
public:
    PickObjects(ros::NodeHandle *nodeHandle) : n_(*nodeHandle)
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
}