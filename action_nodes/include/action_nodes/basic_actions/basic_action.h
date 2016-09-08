#ifndef BASIC_ACTION_H
#define BASIC_ACTION_H

#include <action_nodes/ExecutableAction.h>

class BasicAction:public ExecutableAction {
public:
	BasicAction(string action_name, ros::NodeHandle node_handle);
	void execute(const action_management_msgs::ManageActionGoalConstPtr& goal);

};

#endif