#include "action_executor/ExecutableAction.h"

ExecutableAction::ExecutableAction(string action_name, ros::NodeHandle node_handle):
	Action(action_name,node_handle),
	action_server_(node_handle, "action_management/actions/"+action_name_+"/execute/", boost::bind(&ExecutableAction::execute, 
	this, _1), false),
	motion_execution_client_("motion/motion_plan_execute",true)
 {
 	action_server_.start();
 	ROS_INFO("Started action server");
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


void ExecutableAction::motionExecutionDoneCB(const actionlib::SimpleClientGoalState& state,
		const action_management_msgs::ManageActionResultConstPtr& result) {
	boost::lock_guard<boost::mutex> lock(mutex_is_motion_done_);
	is_motion_done_=true;
}

bool ExecutableAction::isMotionDone() {
	boost::lock_guard<boost::mutex> lock(mutex_is_motion_done_);
	return is_motion_done_;
}

bool ExecutableAction::handleMotionRequest(action_management_msgs::ManageActionGoalConstPtr& goal) {
	action_management_msgs::ManageActionGoal goal_translated;
	goal_translated.action=goal->action;
	motion_execution_client_.sendGoal(goal_translated,boost::bind(&ExecutableAction::motionExecutionDoneCB,this,_1,_2),
		Client::SimpleActiveCallback(),Client::SimpleFeedbackCallback());

	StringMap parameters=extractParametersFromMsg(goal->action.parameters);
	ros::Rate r(3);
	while (!isMotionDone() && !action_server_.isPreemptRequested() && ros::ok() && !shouldStop(parameters)) {
		r.sleep();
	}
	if (action_server_.isPreemptRequested()) {
		motion_execution_client_.cancelGoal();
		setResult("PREEMPTED","",false);
		action_server_.setPreempted(result_);
		return false;
	}
	else if (!ros::ok()) {
		action_server_.setAborted();
		return false;
	}
	action_management_msgs::ManageActionResultConstPtr motion_result=motion_execution_client_.getResult();
	return motion_result->report.status=="COMPLETED"; 
}
bool ExecutableAction::checkActionName(string name) {
	if (name!=action_name_) {
		setResult("FAILURE","wrong action name",false);
		action_server_.setAborted(result_);
		return false;
	}
	return true;
}


