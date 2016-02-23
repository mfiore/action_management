#include <action_executor/MovementAction.h>


MovementAction::MovementAction(ros::NodeHandle node_handle)::ExecutableAction(node_handle),
move_base_client_("move_base",true) {
	string up_case_action=boost::to_upper_copy(action_name_);
	ROS_INFO("%s waiting for move base",up_case_action.c_str());
	move_base_client_.waitForServer();
}

void MovementAction::moveExecutionDoneCB(const actionlib::SimpleClientGoalState& state,
		const move_base_msgs::MoveBaseResultConstPtr& result) {
	boost::lock_guard<boost::mutex> lock(mutex_is_movement_done_);
	is_movement_done_=true;
}

bool MovementAction::isMovementDone() {
	boost::lock_guard<boost::mutex> lock(mutex_is_movement_done_);
	return is_movement_done_;
}

}

bool handleMoveRequest(geometry_msgs::Pose pose) {

	move_base_msgs::MoveBaseGoal move_base_goal;
	move_base_goal.target_pose=pose;
	move_base_client_.sendGoal(move_base_goal,boost::bind(&MovementAction::moveExecutionDoneCB,this,_1,_2),
		Client::SimpleActiveCallback(),Client::SimpleFeedbackCallback());

	StringMap parameters=extractParametersFromMsg(goal->action.parameters);
	ros::Rate r(3);
	while (!isMovementDone() && !action_server_.isPreemptRequested() && ros::ok() && !shouldStop(parameters)) {
		r.sleep();
	}
	if (action_server_.isPreemptRequested()) {
		client->cancelGoal();
		setResult("PREEMPTED","",false);
		action_server_.setPreempted(result_);
		return false;
	}
	else if (!ros::ok()) {
		action_server_.setAborted();
		return false;
	}
	return move_base_client_.getState()==actionlib::SimpleClientGoalState::SUCCEEDED;
}