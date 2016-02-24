/**
Author: Michelangelo Fiore

Abstract class to represent Actions
*/

#ifndef ACTION_H
#define ACTION_H

#include <ros/ros.h>
#include <map>
#include <string>
#include <action_management_msgs/GetActionParameters.h>
#include <action_management_msgs/GetActionParameters.h>
#include <situation_assessment_msgs/QueryDatabase.h>
#include <situation_assessment_msgs/DatabaseRequest.h>
#include <action_management_msgs/CheckPreconditions.h>
#include <action_management_msgs/SetPostconditions.h>
#include <common_msgs/Parameter.h>


using namespace std;

typedef map<string,string> StringMap;

class Action {
public:
	Action(string action_name, ros::NodeHandle node_handle_);
protected:
	string action_name_;
	vector<string> parameters_;
	ros::NodeHandle node_handle_;
	virtual bool checkPreconditions(StringMap parameters)=0;
	virtual void setPostconditions(StringMap parameters)=0;
	StringMap extractParametersFromMsg(vector<common_msgs::Parameter> parameters);

	string robot_name_;
	ros::ServiceClient database_query_client_;
	ros::ServiceClient database_add_facts_client_;
	ros::ServiceClient database_remove_facts_client_;
	ros::ServiceServer get_parameters_server;

private:
	bool getParameters(action_management_msgs::GetActionParameters::Request  &req,
         action_management_msgs::GetActionParameters::Response &res);

	bool checkPreconditionsService(action_management_msgs::CheckPreconditions::Request &req,
		action_management_msgs::CheckPreconditions::Response &res);
	bool setPostconditionsService(action_management_msgs::SetPostconditions::Request &req,
		action_management_msgs::SetPostconditions::Response &res);


};
#endif