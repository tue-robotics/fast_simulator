#include <actionlib/server/action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/ros.h>
#include <urdf/model.h>

#include <tue/manipulation/reference_generator.h>

class Robot;  // Forward declaration of robot

typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> TrajectoryActionServer;

class BodyPart
{

public:

    BodyPart();

    ~BodyPart();

    void setRobot(Robot* robot) { robot_ = robot; }

    void addActionServer(ros::NodeHandle& nh, const std::string& name);

    void initJoint(const std::string& name, double pos);

    void readJointInfoFromModel(const urdf::Model& Model);

    void step(double dt);

    const std::vector<std::string>& joint_names() const { return reference_generator_.joint_names(); }

private:

    Robot* robot_;

    tue::manipulation::ReferenceGenerator reference_generator_;

    void goalCallback(TrajectoryActionServer::GoalHandle gh);

    void cancelCallback(TrajectoryActionServer::GoalHandle gh);

    std::vector<TrajectoryActionServer*> action_servers_;

    std::map<std::string, TrajectoryActionServer::GoalHandle> goal_handles_;

};
