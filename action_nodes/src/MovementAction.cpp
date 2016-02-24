#include <action_nodes/MovementAction.h>


MovementAction::MovementAction(string action_name, ros::NodeHandle node_handle):ExecutableAction(action_name,node_handle),
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


bool MovementAction::handleMoveRequest(geometry_msgs::Pose pose) {

	geometry_msgs::PoseStamped pose_stamped;
	pose_stamped.header.frame_id = "map";
	pose_stamped.header.stamp = ros::Time::now();
	pose_stamped.pose=pose;

	move_base_msgs::MoveBaseGoal move_base_goal;
	move_base_goal.target_pose=pose_stamped;
	move_base_client_.sendGoal(move_base_goal,boost::bind(&MovementAction::moveExecutionDoneCB,this,_1,_2),
		MoveBaseClient::SimpleActiveCallback(),MoveBaseClient::SimpleFeedbackCallback());


	ros::Rate r(3);
	while (!isMovementDone() && !action_server_.isPreemptRequested() && ros::ok()) {
		r.sleep();
	}
	if (action_server_.isPreemptRequested()) {
		move_base_client_.cancelGoal();
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