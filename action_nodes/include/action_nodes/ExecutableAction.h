/**
Author: Michelangelo Fiore

Abstract class to represent actions that can be executed by the robot
*/

#ifndef EXECUTABLEACTION_H
#define EXECUTABLEACTION_H

#include <ros/ros.h>
#include <map>
#include <string>
#include "action_nodes/Action.h"
#include <action_management_msgs/ManageActionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp> 
#include <boost/thread/condition_variable.hpp>
#include <boost/algorithm/string.hpp>



using namespace std;

typedef actionlib::SimpleActionServer<action_management_msgs::ManageActionAction> Server;
typedef actionlib::SimpleActionClient<action_management_msgs::ManageActionAction> Client;

class ExecutableAction:public Action {
public:
	ExecutableAction(string action_name, ros::NodeHandle node_handle);
	virtual void execute(const action_management_msgs::ManageActionGoalConstPtr& goal)=0;

protected:
	action_management_msgs::ManageActionFeedback feedback_;
	action_management_msgs::ManageActionResult result_;
	Server action_server_;
	Client motion_execution_client_;

	void setResult(string status, string details, bool ok);
	void sendFeedback(string status, string details);

	virtual bool shouldStop(StringMap parameters);
	action_management_msgs::ManageActionResultConstPtr handleMotionRequest(const action_management_msgs::ManageActionGoalConstPtr& goal);
	action_management_msgs::ManageActionResultConstPtr handleMotionRequest(action_management_msgs::ManageActionGoal goal);	

	action_management_msgs::ManageActionResultConstPtr handleOtherActionRequest(const action_management_msgs::ManageActionGoalConstPtr& goal, Client *action_client);
	action_management_msgs::ManageActionResultConstPtr handleOtherActionRequest(action_management_msgs::ManageActionGoal goal, Client *action_client);


	bool isResultSuccessfull(action_management_msgs::ManageActionResultConstPtr result);

	bool checkActionName(string name);

	bool abortIfFailed(action_management_msgs::ManageActionResultConstPtr result);

	private:
	void clientExecutionDoneCB(const actionlib::SimpleClientGoalState& state,
				const action_management_msgs::ManageActionResultConstPtr& result);
	boost::mutex mutex_is_client_done_;
	bool is_client_done_;

	bool isClientDone();

};

#endif