#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

class PickObjects
{
public:
    PickObjects(ros::NodeHandle *nodeHandle) : n_(*nodeHandle)
    {
        ROS_INFO_ONCE("started PickObjects node");
        sub = n_.subscribe("marker_position", 15, &PickObjects::handleMarker, this);
        ROS_INFO_ONCE("subscribed to marker_position");
    }

private:
    ros::NodeHandle n_;
    ros::Subscriber sub;
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    move_base_msgs::MoveBaseGoal goal;

    void handleMarker(const geometry_msgs::Point &msg)
    {
        ROS_INFO_ONCE("got marker message");
        setGoal(msg);

        // Send the goal position and orientation for the robot to reach

        MoveBaseClient ac("move_base", true);
        // Wait 5 sec for move_base action server to come up
        while (!ac.waitForServer(ros::Duration(2.0)))
        {
            ROS_INFO("Waiting for the move_base action server to come up");
        }

        ROS_INFO("Sending goal");
        ac.sendGoal(goal);

        // Wait an infinite time for the results
        ac.waitForResult();

        // Check if the robot reached its goal
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The base successfully moved to the requested position");
        else
            ROS_INFO("The base failed to get to the requested position.");
    }

    void setGoal(const geometry_msgs::Point &msg)
    {
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = msg.x;
        goal.target_pose.pose.position.y = msg.y;
        goal.target_pose.pose.position.z = msg.z;

        goal.target_pose.pose.orientation.x = 0.0;
        goal.target_pose.pose.orientation.y = 0.0;
        goal.target_pose.pose.orientation.z = 0.0;
        goal.target_pose.pose.orientation.w = 1.0;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pick_objects");
    ros::NodeHandle n;

    PickObjects pickObjects(&n);

    ros::spin();

    return 0;
}