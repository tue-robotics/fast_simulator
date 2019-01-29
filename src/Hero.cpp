#include <urdf/model.h>

#include <geolib/ros/msg_conversions.h>
#include <geolib/ros/tf_conversions.h>

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

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

//  as_ = new TrajectoryActionServer(nh, "body/joint_trajectory_action", false);
//  as_->registerGoalCallback(boost::bind(&Hero::goalCallback, this, _1));
//  as_->registerCancelCallback(boost::bind(&Hero::cancelCallback, this, _1));
//  as_->start();

}

// ----------------------------------------------------------------------------------------------------

Hero::~Hero()
{
//    delete as_;
}

// ----------------------------------------------------------------------------------------------------

void Hero::step(double dt)
{
  const std::vector<std::string>& joint_names = reference_generator_.joint_names();

  std::vector<double> references;
  if (!reference_generator_.calculatePositionReferences(dt, references))
      return;

  for(unsigned int i = 0 ; i < joint_names.size(); ++i)
      setJointPosition(joint_names[i], references[i]);

//  for(std::map<std::string, TrajectoryActionServer::GoalHandle>::iterator it = goal_handles_.begin(); it != goal_handles_.end();)
//  {
//      TrajectoryActionServer::GoalHandle& gh = it->second;
//      tue::manipulation::JointGoalStatus status = reference_generator_.getGoalStatus(gh.getGoalID().id);

//      if (status == tue::manipulation::JOINT_GOAL_SUCCEEDED)
//      {
//          gh.setSucceeded();
//          goal_handles_.erase(it++);
//      }
//      else if (status == tue::manipulation::JOINT_GOAL_CANCELED)
//      {
//          gh.setCanceled();
//          goal_handles_.erase(it++);
//      }
//      else
//      {
//          ++it;
//      }
//  }

  Robot::step(dt);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  if (ros::Time::now() - t_last_cmd_vel_ > ros::Duration(0.5))
  {
      geometry_msgs::Twist& vel = this->velocity_;

      vel.angular.x = 0;
      vel.angular.y = 0;
      vel.angular.z = 0;

      vel.linear.x = 0;
      vel.linear.y = 0;
      vel.linear.z = 0;
  }

  if (event_odom_pub_.isScheduled()) {
      /// Send tf
      tf_odom_to_base_link.stamp_ = ros::Time::now();
      geo::convert(getAbsolutePose(), tf_odom_to_base_link);
      tf_broadcaster_.sendTransform(tf_odom_to_base_link);

      /// Send odom message
      // Fill pose
      nav_msgs::Odometry odom_msg;
      odom_msg.header.frame_id = "/sergio/odom";
      odom_msg.header.stamp    = ros::Time::now();
      odom_msg.child_frame_id  = "/sergio/base_link";
      tf::Vector3 position = tf_odom_to_base_link.getOrigin();
      odom_msg.pose.pose.position.x = position.getX();
      odom_msg.pose.pose.position.y = position.getY();
      odom_msg.pose.pose.position.z = position.getZ();
      tf::Quaternion orientation = tf_odom_to_base_link.getRotation();
      odom_msg.pose.pose.orientation.x = orientation.getX();
      odom_msg.pose.pose.orientation.y = orientation.getY();
      odom_msg.pose.pose.orientation.z = orientation.getZ();
      odom_msg.pose.pose.orientation.w = orientation.getW();
      //ROS_INFO("Position = [%f, %f, %f]",position.getX(),position.getY(),position.getZ());
      // ToDo: fill covariance

      // Fill twist (assume base controller can follow this->velocity_)
      geometry_msgs::Twist base_vel = this->velocity_;
      odom_msg.twist.twist.linear.x  = base_vel.linear.x;
      odom_msg.twist.twist.linear.y  = base_vel.linear.y;
      odom_msg.twist.twist.angular.z = base_vel.angular.z;
      //ROS_INFO("Twist: [%f, %f, %f]", odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.angular.z);
      // ToDo: fill covariance

      pub_odom_.publish(odom_msg);

  }

}

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
