#ifndef PLACE_H
#define PLACE_H

#include <ros/ros.h>

#include <action_executor/BasicAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <situation_assessment_msgs::PutObjectInHand.h>

class Place: public BasicAction {
	public:
	Place();

protected:
	bool checkPreconditions(StringMap parameters);
	void setPostconditions(StringMap parameters);
};

#endif