#ifndef DOCK_H
#define DOCK_H

#include <ros/ros.h>

#include <action_executor/MovementAction.h>
#include <tf/transform_datatypes.h>
#include <stdlib.h>     /* system, NULL, EXIT_FAILURE */

class Dock: public MovementAction {
	public:
	Dock(ros::NodeHandle node_handle);

protected:
	bool checkPreconditions(StringMap parameters);
	void setPostconditions(StringMap parameters);
	void execute(const action_management_msgs::ManageActionGoalConstPtr& goal):

};

#endif