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

	node_handle_.getParam("situation_assessment/robot_name",robot_name_);

	ROS_INFO("%s robot name is %s",action_name_.c_str(),robot_name_.c_str());

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
	database_set_facts_client_=node_handle_.serviceClient<situation_assessment_msgs::DatabaseRequest>("/situation_assessment/set_facts");
	
	database_query_client_.waitForExistence();
	database_add_facts_client_.waitForExistence();
	database_remove_facts_client_.waitForExistence();
	database_set_facts_client_.waitForExistence();

	ROS_INFO("%s connected",action_name.c_str());
}
bool Action::checkPreconditionsService(action_management_msgs::CheckPreconditions::Request &req,
	action_management_msgs::CheckPreconditions::Response &res) {
	// ROS_INFO("ACTION - received request to check preconditions");
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

bool Action::checkParameterPresence(StringMap request_parameters) {
	for (string p: parameters_) {
		if (request_parameters.find(p)==request_parameters.end()) return false;
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

string Action::queryDatabase(situation_assessment_msgs::Fact query) {

	situation_assessment_msgs::QueryDatabase srv;
	
	srv.request.query=query;

	if (!(database_query_client_.call(srv))) {
		ROS_ERROR("%s Failed to contact db",action_name_.c_str());
		return "";
	}
	if (srv.response.result.size()==0) {
		// ROS_ERROR("%s No answers in %s query",action_name_.c_str(),query.predicate[0].c_str());
		return "";
	} 
	if (srv.response.result[0].value.size()==0) {
		ROS_ERROR("%s No values in answer for %s query",action_name_.c_str(),query.predicate[0].c_str());
	}
	return srv.response.result[0].value[0];
}

std::vector<std::string> Action::queryDatabaseComplete(situation_assessment_msgs::Fact query) {

	situation_assessment_msgs::QueryDatabase srv;
	
	srv.request.query=query;

	vector<string> null_result;

	if (!(database_query_client_.call(srv))) {
		ROS_ERROR("%s Failed to contact db",action_name_.c_str());
		return null_result;
	}
	if (srv.response.result.size()==0) {
		// ROS_ERROR("%s No answers in %s query",action_name_.c_str(),query.predicate[0].c_str());
		return null_result;
	} 
	if (srv.response.result[0].value.size()==0) {
		ROS_ERROR("%s No values in answer for %s query",action_name_.c_str(),query.predicate[0].c_str());
		return null_result;
	}
	return srv.response.result[0].value;
}

void Action::setFacts(std::vector<situation_assessment_msgs::Fact> facts) {
	situation_assessment_msgs::DatabaseRequest srv;
	srv.request.fact_list=facts;
	if (!database_set_facts_client_.call(srv)) {
		ROS_ERROR("%s failed to contact db",action_name_.c_str());
	} 
}
void Action::addFacts(std::vector<situation_assessment_msgs::Fact> facts) {
	situation_assessment_msgs::DatabaseRequest srv;
	srv.request.fact_list=facts;
	if (!database_add_facts_client_.call(srv)) {
		ROS_ERROR("%s failed to contact db",action_name_.c_str());
	} 
}
void Action::removeFacts(std::vector<situation_assessment_msgs::Fact> facts) {
	situation_assessment_msgs::DatabaseRequest srv;
	srv.request.fact_list=facts;
	if (!database_remove_facts_client_.call(srv)) {
		ROS_ERROR("%s failed to contact db",action_name_.c_str());
	} 
}


