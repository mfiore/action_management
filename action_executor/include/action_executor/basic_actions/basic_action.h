#ifndef BASIC_ACTION_H
#define BASIC_ACTION_H

#include <ExecutableAction.h>

class BasicAction:public ExecutableAction {
public:
	BasicAction(string action_name);
	void execute(const shary3_msgs::ManageActionGoalConstPtr& goal);

};

#endif