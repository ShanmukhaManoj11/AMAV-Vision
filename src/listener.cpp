#include "ros/ros.h"
#include "vision/CamMsg.h"
#include "geometry_msgs/PoseStamped.h"

void chatterCallback(const vision::CamMsg::ConstPtr& msg)
{
  ROS_INFO("I heard: [%f %f]", msg->range, msg->heading_angle);
}

void xyz_read(const geometry_msgs::PoseStamped::ConstPtr& xyzmsg)
{
      float z_in = xyzmsg->pose.position.z;
      //ROS_INFO_STREAM(flow_read);
      ROS_INFO("z: [%f]", z_in);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  ros::Subscriber flow_xyz = n.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, xyz_read);

  ros::spin();

  return 0;
}
