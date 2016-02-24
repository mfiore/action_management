#include <pr2_actions/pick.h>

Pick::Pick(ros::NodeHandle node_handle):ExecutableAction("pick",node_handle),
approach_client_("action_management/actions/approach/execute",true), 
dock_client_("action_management/actions/dock/execute",true)
 {

	put_object_in_hand_client_=node_handle_.serviceClient<situation_assessment_msgs::PutObjectInHand>("/situation_assessment/put_object_in_hand");
	ROS_INFO("PICK waiting for put object in hand service");
	put_object_in_hand_client_.waitForExistence();
	
	ROS_INFO("PICK waiting for approach action");
	approach_client_.waitForServer();
	parameters_.push_back("main_object");
	parameters_.push_back("main_agent");
}


bool Pick::checkPreconditions(StringMap parameters) {
	situation_assessment_msgs::QueryDatabase srv;
	srv.request.query.model=robot_name_;
	srv.request.query.subject=parameters["main_agent"];
	srv.request.query.predicate.push_back("has");

	if (database_query_client_.call(srv)) {
		return srv.response.result.size()<2;
	}
	else {
		ROS_ERROR("PICK Failed to contact db");
	}
}

void Pick::setPostconditions(StringMap parameters) {
	situation_assessment_msgs::Fact f;
	f.model=robot_name_;
	f.subject=parameters["main_agent"];
	f.predicate.push_back("has");
	f.value.push_back(parameters["main_object"]);

	situation_assessment_msgs::DatabaseRequest srv;
	srv.request.fact_list.push_back(f);
	if (!database_add_facts_client_.call(srv)) {
		ROS_ERROR("PICK failed to contact db");
	} 

	if (parameters["main_agent"]!=robot_name_) {
		situation_assessment_msgs::PutObjectInHand put_object_in_hand_srv;
		put_object_in_hand_srv.request.object=parameters["main_object"];
		put_object_in_hand_srv.request.agent=parameters["main_agent"];
		put_object_in_hand_srv.request.has_object=true;
		if (!put_object_in_hand_client_.call(put_object_in_hand_srv)){
			ROS_ERROR("PICK couldn't call put object in hand service");
		}
		if (!put_object_in_hand_srv.response.result) {
			ROS_ERROR("PICK couldn't put object in hand");
		}
	}
}



void Pick::execute(const action_management_msgs::ManageActionGoalConstPtr& goal) {
	action_management_msgs::Action pick_action=goal->action;
	if (!checkActionName(pick_action.name)) return;

	StringMap parameters=extractParametersFromMsg(pick_action.parameters);

	if (!checkPreconditions(parameters)) {
		setResult("FAILURE","preconditions not satisfied",false);
		action_server_.setAborted(result_);
		return;
	}

	situation_assessment_msgs::QueryDatabase query_srv;
	query_srv.request.query.model=robot_name_;
	query_srv.request.query.subject=parameters["main_object"];
	query_srv.request.query.predicate.push_back("isOn");
	if (!database_query_client_.call(query_srv)) {
		ROS_WARN("PICK couldn't contact db");
	}
	if (query_srv.response.result.size()==0 || query_srv.response.result[0].value.size()==0) {
		ROS_WARN("PICK couldn't get location of object");
		setResult("FAILEDD","couldn't get location of object",false);
		action_server_.setAborted(result_);
		return;
	}
	string object_location=query_srv.response.result[0].value[0];


	action_management_msgs::ManageActionGoal approach_goal, dock_goal, pick_pose_goal;
	common_msgs::Parameter object_location_parameter,pose_parameter;
	object_location_parameter.name="location";
	object_location_parameter.value=object_location;
	pose_parameter.name="pose_name";
	pose_parameter.value="manipulationPose";

	approach_goal.action.name="approach";
	approach_goal.action.parameters.push_back(object_location_parameter);

	pick_pose_goal.action.name="moveTo";
	pick_pose_goal.action.parameters.push_back(pose_parameter);

	dock_goal.action.name="dock";
	dock_goal.action.parameters.push_back(object_location_parameter);


	action_management_msgs::ManageActionResultConstPtr action_result;
	action_result=handleOtherActionRequest(approach_goal,&approach_client_);
	if (!abortIfFailed(action_result)) return;

	action_result=handleMotionRequest(pick_pose_goal);
	if (!abortIfFailed(action_result)) return;

	action_result=handleOtherActionRequest(dock_goal,&dock_client_);
	if (!abortIfFailed(action_result)) return;

	action_result=handleMotionRequest(goal);

	if (!abortIfFailed(action_result)) return;
	setPostconditions(parameters);
	setResult("COMPLETED","",true);
	action_server_.setSucceeded(result_);
}

