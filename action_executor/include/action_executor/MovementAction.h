/**
Author: Michelangelo Fiore

Abstract class to represent actions where the robot needs to move
*/

#ifndef EXECUTABLEACTION_H
#define EXECUTABLEACTION_H

#include <ros/ros.h>
#include <map>
#include <string>
#include "action_executor/ExecutableAction.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp> 
#include <boost/thread/condition_variable.hpp>
#include <boost/algorithm/string.hpp>



using namespace std;

typedef actionlib::SimpleActionClient<action_management_msgs::MoveBaseAction> MoveBaseClient;

class MovementAction:public ExecutableAction {
public:
	MovementAction(string action_name, ros::NodeHandle node_handle);

protected:
	MoveBaseClient move_base_client_;

	bool handleMoveRequest(geometry_msgs::Pose pose);

	private:
	void moveExecutionDoneCB(const actionlib::SimpleClientGoalState& state,
				const move_base_msgs::MoveBaseResultConstPtr& result);
	boost::mutex mutex_is_movement_done_;
	bool is_movement_done_;
	bool isMovementDone();

};

#endif