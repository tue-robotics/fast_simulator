#include "../include/fast_simulator/Robot.h"

#include "../include/fast_simulator/BodyPart.h"

// ----------------------------------------------------------------------------------------------------

BodyPart::BodyPart() : robot_(NULL)
{
}

// ----------------------------------------------------------------------------------------------------

BodyPart::~BodyPart()
{
    for(std::vector<TrajectoryActionServer*>::iterator it = action_servers_.begin(); it != action_servers_.end(); ++it)
        delete *it;
}

// ----------------------------------------------------------------------------------------------------

void BodyPart::addActionServer(ros::NodeHandle& nh, const std::string& name)
{
    TrajectoryActionServer* as = new TrajectoryActionServer(nh, name, false);
    as->registerGoalCallback(boost::bind(&BodyPart::goalCallback, this, _1));
    as->registerCancelCallback(boost::bind(&BodyPart::cancelCallback, this, _1));
    as->start();

    action_servers_.push_back(as);
}

// ----------------------------------------------------------------------------------------------------

void BodyPart::initJoint(const std::string& name, double pos)
{
    reference_generator_.initJoint(name, 0, 0, 0, 0);
    reference_generator_.setJointState(name, pos, 0);
    robot_->setJointPosition(name, pos);
}

// ----------------------------------------------------------------------------------------------------

void BodyPart::readJointInfoFromModel(const urdf::Model& Model)
{
    const std::vector<std::string>& joint_names = reference_generator_.joint_names();
    for(unsigned int i = 0 ; i < joint_names.size(); ++i)
    {
        boost::shared_ptr<const urdf::Joint> joint = Model.getJoint(joint_names[i]);
        reference_generator_.setPositionLimits(i, joint->limits->lower, joint->limits->upper);
        reference_generator_.setMaxVelocity(i, joint->limits->velocity);
        reference_generator_.setMaxAcceleration(i, joint->limits->effort);
    }
}

// ----------------------------------------------------------------------------------------------------

void BodyPart::step(double dt)
{
    const std::vector<std::string>& joint_names = reference_generator_.joint_names();

    std::vector<double> references;
    if (!reference_generator_.calculatePositionReferences(dt, references))
        return;

    for(unsigned int i = 0 ; i < joint_names.size(); ++i)
        robot_->setJointPosition(joint_names[i], references[i]);

    for(std::map<std::string, TrajectoryActionServer::GoalHandle>::iterator it = goal_handles_.begin(); it != goal_handles_.end();)
    {
        TrajectoryActionServer::GoalHandle& gh = it->second;
        tue::manipulation::JointGoalStatus status = reference_generator_.getGoalStatus(gh.getGoalID().id);

        if (status == tue::manipulation::JOINT_GOAL_SUCCEEDED)
        {
            gh.setSucceeded();
            goal_handles_.erase(it++);
        }
        else if (status == tue::manipulation::JOINT_GOAL_CANCELED)
        {
            gh.setCanceled();
            goal_handles_.erase(it++);
        }
        else
        {
            ++it;
        }
    }
}

// ----------------------------------------------------------------------------------------------------

void BodyPart::goalCallback(TrajectoryActionServer::GoalHandle gh)
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

void BodyPart::cancelCallback(TrajectoryActionServer::GoalHandle gh)
{
    gh.setCanceled();
    reference_generator_.cancelGoal(gh.getGoalID().id);
    goal_handles_.erase(gh.getGoalID().id);
}
