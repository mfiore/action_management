#include <action_nodes/basic_actions/basic_action.h>

BasicAction::BasicAction(string action_name, ros::NodeHandle node_handle):ExecutableAction(action_name,node_handle) {

}

void BasicAction::execute(const action_management_msgs::ManageActionGoalConstPtr& goal) {
	action_management_msgs::Action this_action=goal->action;
	if (!checkActionName(this_action.name)) return;

	StringMap parameters=extractParametersFromMsg(this_action.parameters);

	if (!checkPreconditions(parameters)) {
		setResult("FAILURE","preconditions not satisfied",false);
		action_server_.setAborted(result_);
		return;
	}
	action_management_msgs::ManageActionResultConstPtr motion_result=handleMotionRequest(goal);
	if (motion_result->report.status=="OK") {
		setPostconditions(parameters);
		setResult("COMPLETED","",true);
		action_server_.setSucceeded(result_);
	}
	else {
		setResult("FAILED",motion_result->report.details,false);
		action_server_.setAborted(result_);
	}
}
