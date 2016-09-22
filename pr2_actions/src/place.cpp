#include <pr2_actions/place.h>

Place::Place(ros::NodeHandle node_handle):ExecutableAction("place",node_handle),
approach_client_("action_management/actions/approach/execute",true), 
dock_client_("action_management/actions/dock/execute",true) 

{

	put_object_in_hand_client_=node_handle_.serviceClient<situation_assessment_msgs::PutObjectInHand>("/situation_assessment/put_object_in_hand");
	ROS_INFO("PLACE waiting for put object in hand service");
	put_object_in_hand_client_.waitForExistence();
	
	ROS_INFO("PLACE waiting for approach action");
	approach_client_.waitForServer();
	parameters_.push_back("main_object");
	parameters_.push_back("main_agent");
	parameters_.push_back("support_object");
}


bool Place::checkPreconditions(StringMap parameters) {
	situation_assessment_msgs::QueryDatabase srv;
	srv.request.query.model=robot_name_;
	srv.request.query.subject=parameters["main_agent"];
	srv.request.query.predicate.push_back("has");
	srv.request.query.value.push_back(parameters["main_object"]);

	if (database_query_client_.call(srv)) {
		return srv.response.result.size()>0;
	}
	else {
		ROS_ERROR("PLACE Failed to contact db");
	}
}

void Place::setPostconditions(StringMap parameters) {
	situation_assessment_msgs::Fact f1,f2;
	f1.model=robot_name_;
	f1.subject=parameters["main_agent"];
	f1.predicate.push_back("has");
	f1.value.push_back(parameters["main_object"]);

	f2.model=robot_name_;
	f2.subject=parameters.at("main_agent");
	f2.predicate.push_back("at");
	f2.value=parameters.at("target");

	situation_assessment_msgs::DatabaseRequest srv;
	srv.request.fact_list.push_back(f1);
	if (!database_remove_facts_client_.call(srv)) {
		ROS_ERROR("PLACE failed to contact db");
	} 


	if (parameters["main_agent"]!=robot_name_) {
		situation_assessment_msgs::PutObjectInHand put_object_in_hand_srv;
		put_object_in_hand_srv.request.object=parameters["main_object"];
		put_object_in_hand_srv.request.agent=parameters["main_agent"];
		put_object_in_hand_srv.request.has_object=false;
		if (!put_object_in_hand_client_.call(put_object_in_hand_srv)){
			ROS_ERROR("PLACE couldn't call put object in hand service");
		}
		if (!put_object_in_hand_srv.response.result) {
			ROS_ERROR("PLACE couldn't remove object from hand");
		}
	}
}



void Place::execute(const action_management_msgs::ManageActionGoalConstPtr& goal) {
	action_management_msgs::Action place_action=goal->action;
	if (!checkActionName(place_action.name)) return;

	StringMap parameters=extractParametersFromMsg(goal->action.parameters);

	if (!checkPreconditions(parameters)) {
		setResult("FAILURE","preconditions not satisfied",false);
		action_server_.setAborted(result_);
		return;
	}

	action_management_msgs::ManageActionGoal approach_goal, dock_goal, place_pose_goal;
	common_msgs::Parameter placement_location_parameter,pose_parameter;
	placement_location_parameter.name="location";
	placement_location_parameter.value=parameters["support_object"];
	pose_parameter.name="pose_name";
	pose_parameter.value="manipulationPose";

	approach_goal.action.name="approach";
	approach_goal.action.parameters.push_back(placement_location_parameter);

	place_pose_goal.action.name="moveTo";
	place_pose_goal.action.parameters.push_back(pose_parameter);

	dock_goal.action.name="dock";
	dock_goal.action.parameters.push_back(placement_location_parameter);


	action_management_msgs::ManageActionResultConstPtr action_result;
	action_result=handleOtherActionRequest(approach_goal,&approach_client_);
	if (!abortIfFailed(action_result)) return;

	action_result=handleMotionRequest(place_pose_goal);
	if (!abortIfFailed(action_result)) return;

	action_result=handleOtherActionRequest(dock_goal,&dock_client_);
	if (!abortIfFailed(action_result)) return;

	action_result=handleMotionRequest(goal);

	if (!abortIfFailed(action_result)) return;
	setPostconditions(parameters);
	setResult("COMPLETED","",true);
	action_server_.setSucceeded(result_);
}



