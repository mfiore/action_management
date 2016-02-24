#ifndef BasicPick_H
#define BasicPick_H

#include <ros/ros.h>

#include <action_nodes/BasicAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <situation_assessment_msgs::PutObjectInHand.h>

class BasicPick: public BasicAction {
	public:
	BasicPick();

protected:
	bool checkPreconditions(StringMap parameters);
	void setPostconditions(StringMap parameters);
};

#endif