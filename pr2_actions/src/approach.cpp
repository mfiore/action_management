#include <pr2_actions/approach.h>

Approach::Approach(ros::NodeHandle node_handle):
MovementAction("approach",node_handle) {

	parameters_.push_back("location");
	parameters_.push_back("main_agent");
}

bool Approach::checkPreconditions(StringMap parameters) {
	return true;
}

bool Approach::setPostconditions(StringMap parameters) {
	situation_assessment_msgs::DatabaseRequest srv_add,srv_remove;
	situation_assessment_msgs::Fact f_to_add,f_to_remove;
	f_to_add.model=robot_name_;
	f_to_add.subject=parameters["main_agent"];
	f_to_add.predicate.push_back("isAt");
	f_to_add.value.push_back(parameters["location"]);
	f_to_add.value.push_back("in_approach");

	f_to_remove.model=robot_name_;
	f_to_add.subject=parameters["main_agent"];
	f_to_add.predicate.push_back("isAt");

	srv_remove.request.fact_list.push_back(f_to_remove);
	srv_add.request.fact_list.push_back(f_to_add);

	if (!database_remove_facts_client_.call(srv_remove)) {
		ROS_WARN("APPROACH couldn't contact database");
	}	
	if (!database_add_facts_client_.call(srv_add)) {
		ROS_WARN("APPROACH couldn't contact database");
	}

}

void Approach::execute(const action_management_msgs::ManageActionGoalConstPtr& goal) {
	if (!checkActionName(pick_action.name)) return;

	StringMap parameters=extractParametersFromMsg(pick_action=goal.parameters);


	situation_assessment_msgs::QueryDatabase srv_database;
	srv_database.request.query.model=robot_name_;
	srv_database.request.query.subject=parameters["location"];
	srv_database.request.query.predicate.push_back("pose");
	srv_database.request.query.predicate.push_back("dock");
	if (!database_query_client_.call(srv_database)) {
		ROS_ERROR("APPROACH couldn't contact database");
		setResult("FAILED","couldn't get position of location",false);
		action_server_.setAborted();
		return;
	}
	if (srv_database.response.result.size()==0 || srv_database.response.result[0].value.size()<6) {
		ROS_ERROR("APPROACH location pose not present in the db");
		setResult("FAILED","location pose not present in the db",false);
		action_server_->setAborted();
		return;
	} 

	geometry_msgs::Pose pose;
	pose.position.x=srv_database.response.result[0].value[0];
	pose.position.y=srv_database.response.result[0].value[1];
	pose.position.z=0;
	
	pose.position.x=srv_database.response.result[0].value[2];
	pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(srv_database.response.result[0].value[3],
		srv_database.response.result[0].value[4],
		srv_database.response.result[0].value[5]);

	if (handleMoveRequest(pose)) {
		setResult("SUCCEEDED","",true);
		action_server_.setSucceded(result_);
	}
	return;
}
