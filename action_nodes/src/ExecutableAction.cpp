#include "action_nodes/ExecutableAction.h"

ExecutableAction::ExecutableAction(string action_name, ros::NodeHandle node_handle):
	Action(action_name,node_handle),
	action_server_(node_handle, "action_management/actions/"+action_name_+"/execute/", boost::bind(&ExecutableAction::execute, 
	this, _1), false),
	motion_execution_client_("motion/motion_plan_execute",true)
 {
 	string up_action=boost::to_upper_copy(action_name);

 	ROS_INFO("%s Waiting for motion to be ready",up_action.c_str());
 	// motion_execution_client_.waitForServer();
 	action_server_.start();
 	ROS_INFO("%s Started action server",up_action.c_str());
}

void ExecutableAction::setResult(string status, string details, bool ok) {
	string up_action=boost::to_upper_copy(action_name_);
	if (!ok) {
		ROS_WARN("%s %s %s",up_action.c_str(),status.c_str(),details.c_str());
	}
	else {
		ROS_INFO("%s %s %s",up_action.c_str(),status.c_str(),details.c_str());
	}
	common_msgs::Report report;
	report.status=status;
	report.details=details;
	result_.report=report;
}
void ExecutableAction::sendFeedback(string status, string details) {
	string up_action=boost::to_upper_copy(action_name_);

	ROS_INFO("%s %s %s",up_action.c_str(),status.c_str(),details.c_str());
	common_msgs::Report report;
	report.status=status;
	report.details=details;
	feedback_.report=report;
	action_server_.publishFeedback(feedback_);
}
bool ExecutableAction::shouldStop(StringMap parameters) {
	return false;
}


void ExecutableAction::clientExecutionDoneCB(const actionlib::SimpleClientGoalState& state,
		const action_management_msgs::ManageActionResultConstPtr& result) {
	boost::lock_guard<boost::mutex> lock(mutex_is_client_done_);
	is_client_done_=true;
}

bool ExecutableAction::isClientDone() {
	boost::lock_guard<boost::mutex> lock(mutex_is_client_done_);
	return is_client_done_;
}

action_management_msgs::ManageActionResultConstPtr ExecutableAction::handleMotionRequest(const action_management_msgs::ManageActionGoalConstPtr& goal) {	
	return handleOtherActionRequest(goal,&motion_execution_client_);
}

action_management_msgs::ManageActionResultConstPtr ExecutableAction::handleMotionRequest(action_management_msgs::ManageActionGoal goal) {	
	action_management_msgs::ManageActionGoalConstPtr message_const_ptr(&goal);
	return handleOtherActionRequest(message_const_ptr,&motion_execution_client_);

}
action_management_msgs::ManageActionResultConstPtr ExecutableAction::handleOtherActionRequest(
	const action_management_msgs::ManageActionGoalConstPtr& goal, Client *action_client) {

	action_management_msgs::ManageActionGoal goal_translated;
	goal_translated.action=goal->action;
	action_client->sendGoal(goal_translated,boost::bind(&ExecutableAction::clientExecutionDoneCB,this,_1,_2),
		Client::SimpleActiveCallback(),Client::SimpleFeedbackCallback());

	StringMap parameters=extractParametersFromMsg(goal->action.parameters);
	ros::Rate r(3);
	
	action_management_msgs::ManageActionResultConstPtr result;
	while (!isClientDone() && !action_server_.isPreemptRequested() && ros::ok() && !shouldStop(parameters)) {
		r.sleep();
	}
	if (action_server_.isPreemptRequested()) {
		action_client->cancelGoal();
		setResult("PREEMPTED","",false);
		action_server_.setPreempted(result_);
		return result;
	}
	else if (!ros::ok()) {
		action_server_.setAborted();
		return result;
	}
	result=action_client->getResult();
	return result;

}

action_management_msgs::ManageActionResultConstPtr ExecutableAction::handleOtherActionRequest(action_management_msgs::ManageActionGoal goal, Client *action_client) {

	action_management_msgs::ManageActionGoalConstPtr message_const_ptr(&goal);
	return handleOtherActionRequest(message_const_ptr,action_client);

}

bool ExecutableAction::checkActionName(string name) {
	if (name!=action_name_) {
		setResult("FAILURE","wrong action name",false);
		action_server_.setAborted(result_);
		return false;
	}
	return true;
}
bool ExecutableAction::isResultSuccessfull(action_management_msgs::ManageActionResultConstPtr result) {
	return result->report.status=="COMPLETED";
}
bool ExecutableAction::abortIfFailed(action_management_msgs::ManageActionResultConstPtr result) {
	if (!isResultSuccessfull(result)) {
		setResult("FAILED",result->report.details,false);
		action_server_.setAborted(result_);
		return false;
	}
	return true;
}



