/**
Author: Michelangelo Fiore

Abstract class to represent Actions
*/

#include "action_executor/Action.h"

bool lol(action_management_msgs::GetActionParameters::Request  &req,
         action_management_msgs::GetActionParameters::Response &res) {

}

Action::Action(string action_name,ros::NodeHandle node_handle):action_name_(action_name),node_handle_(node_handle) {
	string serviceName="action_management/actions/"+action_name_+"/getParameters";
//	get_parameters_server_=node_handle_.advertiseService(serviceName,boost::bind(&Action::getParameters,this,_1,_2));
	ROS_INFO("%s Started service %s",action_name.c_str(),serviceName.c_str());

	ROS_INFO("%s connecting to database",action_name.c_str());
	database_query_client_=node_handle_.serviceClient<situation_assessment_msgs::QueryDatabase>("/situation_assessment/simple_database");
	database_add_facts_client_=node_handle_.serviceClient<situation_assessment_msgs::DatabaseRequest>("/situation_assessment/add_facts");
	database_remove_facts_client_=node_handle_.serviceClient<situation_assessment_msgs::DatabaseRequest>("/situation_assessment/remove_facts");
	ROS_INFO("%s connected",action_name.c_str());
}


bool Action::getParameters(action_management_msgs::GetActionParameters::Request  &req,
         action_management_msgs::GetActionParameters::Response &res) {
	for (int i=0;i<parameters_.size();i++) {
		res.parameters.push_back(parameters_[i]);
	}
	return true;

}

StringMap Action::extractParametersFromMsg(vector<common_msgs::Parameter> parameters) {
	StringMap result;
	for (int i=0; i<parameters.size();i++) {
		result[parameters[i].name]=parameters[i].value;
	}
	return result;
}


bool Action::checkPreconditionsService(action_management_msgs::CheckPreconditions::Request &req,
	action_management_msgs::CheckPreconditions::Response &res) {
	StringMap parameters=extractParametersFromMsg(req.parameters.parameter_list);
	res.value=checkPreconditions(parameters);
	return true;
}
bool Action::setPostconditionsService(action_management_msgs::SetPostconditions::Request &req,
	action_management_msgs::SetPostconditions::Response &res) {
	setPostconditions(extractParametersFromMsg(req.parameters.parameter_list));
	res.value=true;
	return true;

}
