#include <urdf/model.h>

#include <geolib/ros/msg_conversions.h>

#include "../include/fast_simulator/Hero.h"

Hero::Hero(ros::NodeHandle &nh) : Robot(nh, "hero")
{
  // Torso
  initJoint("torso_lift_joint", 0.0);

  // Arm
  initJoint("arm_lift_joint", 0.0);
  initJoint("arm_roll_joint", 0.0);
  initJoint("arm_flex_joint", 0.0);
  initJoint("wrist_roll_joint", 0.0);
  initJoint("wrist_flex_joint", 0.0);

  // Gripper
  // ToDo

  // Neck
  initJoint("head_pan_joint", 0.0);
  initJoint("head_tilt_joint", 0.0);

  urdf::Model model;
  model.initParam("/hero/robot_description"); // ToDo: check parameter name

  const std::vector<std::string>& joint_names = reference_generator_.joint_names();
  for(unsigned int i = 0 ; i < joint_names.size(); ++i)
  {
      boost::shared_ptr<const urdf::Joint> joint = model.getJoint(joint_names[i]);
      reference_generator_.setPositionLimits(i, joint->limits->lower, joint->limits->upper);
      reference_generator_.setMaxVelocity(i, joint->limits->velocity);
      reference_generator_.setMaxAcceleration(i, joint->limits->effort);
  }

}

// ----------------------------------------------------------------------------------------------------

Hero::~Hero()
{
//    delete as_;
}

// ----------------------------------------------------------------------------------------------------

void Hero::step(double dt)
{
  // ToDo
}

// ----------------------------------------------------------------------------------------------------

// ----------------------------------------------------------------------------------------------------

void Hero::callbackCmdVel(const geometry_msgs::Twist::ConstPtr& msg)
{
    this->velocity_ = *msg;
    t_last_cmd_vel_ = ros::Time::now();
}

// ----------------------------------------------------------------------------------------------------

void Hero::callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    geo::Transform pose;
    geo::convert(msg->pose.pose, pose);
    setPose(pose);
}

// ----------------------------------------------------------------------------------------------------

void Hero::goalCallback(TrajectoryActionServer::GoalHandle gh)
{
    std::string id = gh.getGoalID().id;

    std::stringstream error;
    if (!reference_generator_.setGoal(*gh.getGoal(), id, error))
    {
        gh.setRejected(control_msgs::FollowJointTrajectoryResult(), error.str());
        ROS_ERROR("%s", error.str().c_str());
        return;
    }

    // Accept the goal
    gh.setAccepted();
    goal_handles_[id] = gh;
}

// ----------------------------------------------------------------------------------------------------

void Hero::cancelCallback(TrajectoryActionServer::GoalHandle gh)
{
    gh.setCanceled();
    reference_generator_.cancelGoal(gh.getGoalID().id);
    goal_handles_.erase(gh.getGoalID().id);
}
