#include <action_executore/basic_actions/BasicAction.h>

void BasicAction(string action_name):ExecutableAction(action_name) {

}

void BasicAction::execute(const shary3_msgs::ManageActionGoalConstPtr& goal) {
	Action this_action=goal->action;
	if (!checkActionName(this_action.name)) return;

	StringMap parameters=extractParametersFromMsg(this_action=goal.parameters);

	if (!checkPreconditions(parameters)) {
		setResult("FAILURE","preconditions not satisfied",false);
		action_server_->setAborted(result);
		return;
	}
	if (handleMotionRequest(goal) {
		setPostconditions(parameters);
		setResult("COMPLETED","",true);
		action_server_.setSucceded(result_);
	}
	else {
		setResult("FAILED",motion_result.report.details,false);
		action_server_.setAborted(result_);
	}
}
