#ifndef PLACE_H
#define PLACE_H

#include <ros/ros.h>

#include <action_nodes/ExecutableAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <situation_assessment_msgs/PutObjectInHand.h>
#include <move_base_msgs/MoveBaseAction.h>


typedef actionlib::SimpleActionClient<action_management_msgs::ManageActionAction> Client;


class Place: public ExecutableAction {
	public:
	Place(ros::NodeHandle node_handle);

protected:
	bool checkPreconditions(StringMap parameters);
	void setPostconditions(StringMap parameters);
	void execute(const action_management_msgs::ManageActionGoalConstPtr& goal);

	
	Client approach_client_;
	Client dock_client_;
	ros::ServiceClient put_object_in_hand_client_;

};

#endif