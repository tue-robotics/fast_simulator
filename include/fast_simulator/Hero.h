#ifndef HERO_H
#define HERO_H

#include <actionlib/server/action_server.h>
#include <ros/ros.h>

#include <tue/manipulation/reference_generator.h>

#include "fast_simulator/Robot.h"

class Hero : public Robot
{

public:

  Hero(ros::NodeHandle& nh);

  virtual ~Hero();

  void step(double dt);

protected:

  tf::StampedTransform tf_odom_to_base_link;

  ros::Time t_last_cmd_vel_;  // ToDo: move to Robot

  ros::Publisher pub_body_;
  ros::Publisher pub_head_;
  ros::Publisher pub_arm_;
  ros::Publisher pub_torso_;
  ros::Publisher pub_gripper_;
  ros::Publisher pub_odom_;

  ros::Subscriber sub_cmd_vel;
  ros::Subscriber sub_init_pose;

  ros::Subscriber sub_gripper;

  int gripper_direction_;

  void callbackCmdVel(const geometry_msgs::Twist::ConstPtr& msg);  // ToDo: move to Robot

//  void callbackGripper(const tue_msgs::GripperCommand::ConstPtr& msg);

  void callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);  // ToDo: move to Robot

  Event event_odom_pub_;
  Event event_refs_pub_;


  // New

  typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> TrajectoryActionServer;

  void goalCallback(TrajectoryActionServer::GoalHandle gh);  // ToDo: move to Robot

  void cancelCallback(TrajectoryActionServer::GoalHandle gh);  // ToDo: move to Robot

  TrajectoryActionServer* as_;

  std::map<std::string, TrajectoryActionServer::GoalHandle> goal_handles_;

};  // End of class Hero

#endif // HERO_H
