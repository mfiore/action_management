#ifndef DOCK_H
#define DOCK_H

#include <ros/ros.h>

#include <action_executor/MovementAction.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>

class Undock: public MovementAction {
	public:
	Undock(ros::NodeHandle node_handle);

protected:
	bool checkPreconditions(StringMap parameters);
	void setPostconditions(StringMap parameters);
	void execute(const action_management_msgs::ManageActionGoalConstPtr& goal):

	ros::Publisher cmd_vel_pub_;
};

#endif