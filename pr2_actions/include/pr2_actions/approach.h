#ifndef APPROACH_H
#define APPROACH_H

#include <ros/ros.h>

#include <action_nodes/MovementAction.h>
#include <tf/transform_datatypes.h>


class Approach: public MovementAction {
	public:
	Approach(ros::NodeHandle node_handle);

protected:
	bool checkPreconditions(StringMap parameters);
	void setPostconditions(StringMap parameters);
	void execute(const action_management_msgs::ManageActionGoalConstPtr& goal);
};

#endif