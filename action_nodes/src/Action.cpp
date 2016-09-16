/**
Author: Michelangelo Fiore

Abstract class to represent Actions
*/

#include "action_nodes/Action.h"

bool lol(action_management_msgs::GetActionParameters::Request  &req,
         action_management_msgs::GetActionParameters::Response &res) {

}

Action::Action(string action_name,ros::NodeHandle node_handle):action_name_(action_name),node_handle_(node_handle) {
	string serviceBaseName="action_management/actions/"+action_name_;
	string parametersServiceName=serviceBaseName+"/getParameters";
	string preconditionsServiceName=serviceBaseName+"/checkPreconditions";
	string postconditionsServiceName=serviceBaseName+"/setPostconditions";
//	get_parameters_server_=node_handle_.advertiseService(,
	// boost::bind(&Action::getParameters,this,_1,_2));

	// check_preconditions_server_=node_handle_.advertiseService(preconditionsServiceName,
	// boost::bind(&Action::checkPreconditionsService,this,_1,_2));


	check_preconditions_server_=node_handle_.advertiseService(preconditionsServiceName,
	&Action::checkPreconditionsService,this);

		// check_preconditions_server_=node_handle_.advertiseService(preconditionsServiceName,
	// &Action::checkPreconditions);
	set_postconditions_server_=node_handle_.advertiseService(postconditionsServiceName,
		&Action::setPostconditionsService,this);
	ROS_INFO("%s Started services",action_name.c_str());


	ROS_INFO("%s connecting to database",action_name.c_str());
	database_query_client_=node_handle_.serviceClient<situation_assessment_msgs::QueryDatabase>("/situation_assessment/query_database");
	database_add_facts_client_=node_handle_.serviceClient<situation_assessment_msgs::DatabaseRequest>("/situation_assessment/add_facts");
	database_remove_facts_client_=node_handle_.serviceClient<situation_assessment_msgs::DatabaseRequest>("/situation_assessment/remove_facts");
	
	database_query_client_.waitForExistence();
	database_add_facts_client_.waitForExistence();
	database_remove_facts_client_.waitForExistence();
	ROS_INFO("%s connected",action_name.c_str());
}
bool Action::checkPreconditionsService(action_management_msgs::CheckPreconditions::Request &req,
	action_management_msgs::CheckPreconditions::Response &res) {
	ROS_INFO("ACTION - received request to check preconditions");
	StringMap parameters=extractParametersFromMsg(req.parameters.parameter_list);
	res.value=checkPreconditions(parameters);
	return true;
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



bool Action::setPostconditionsService(action_management_msgs::SetPostconditions::Request &req,
	action_management_msgs::SetPostconditions::Response &res) {
	ROS_INFO("%s - Setting postconditions", action_name_.c_str());
	setPostconditions(extractParametersFromMsg(req.parameters.parameter_list));
	res.value=true;
	return true;

}
