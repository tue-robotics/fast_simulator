#ifndef _FAST_SIMULATOR_AMIGO_H_
#define _FAST_SIMULATOR_AMIGO_H_

#include "tue_msgs/GripperCommand.h"
#include "tue_msgs/GripperMeasurement.h"
#include <urdf/model.h>

#include "../../include/fast_simulator/Robot.h"
#include "../../include/fast_simulator/BodyPart.h"

// ----------------------------------------------------------------------------------------------------

class Amigo : public Robot {    

public:

    Amigo(ros::NodeHandle& nh);

    virtual ~Amigo();

    void step(double dt);

protected:

    tf::StampedTransform tf_odom_to_base_link;

    ros::Time t_last_cmd_vel_;

    ros::Publisher pub_body_;
    ros::Publisher pub_head_;
    ros::Publisher pub_left_arm_;
    ros::Publisher pub_right_arm_;
    ros::Publisher pub_torso_;
    ros::Publisher pub_left_gripper_;
    ros::Publisher pub_right_gripper_;
    ros::Publisher pub_odom_;

    ros::Subscriber sub_cmd_vel;
    ros::Subscriber sub_init_pose;
    ros::Subscriber sub_head;
    ros::Subscriber sub_spindle;
    ros::Subscriber sub_left_arm;
    ros::Subscriber sub_right_arm;

    ros::Subscriber sub_left_gripper;
    ros::Subscriber sub_right_gripper;

    int left_gripper_direction_;
    int right_gripper_direction_;

    void callbackCmdVel(const geometry_msgs::Twist::ConstPtr& msg);

    void callbackLeftGripper(const tue_msgs::GripperCommand::ConstPtr& msg);

    void callbackRightGripper(const tue_msgs::GripperCommand::ConstPtr& msg);    

    void callbackJointReference(const sensor_msgs::JointState::ConstPtr msg);

    BodyPart body_;

    void callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    void publishControlRefs();

    Event event_odom_pub_;
    Event event_refs_pub_;

};

#endif
