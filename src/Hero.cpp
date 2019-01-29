#include <urdf/model.h>

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

