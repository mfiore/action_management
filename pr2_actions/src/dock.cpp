#include <pr2_actions/dock.h>

Dock::Dock(ros::NodeHandle node_handle):
MovementAction("dock",node_handle) {

	parameters_.push_back("location");
	parameters_.push_back("main_agent");
}

bool Dock::checkPreconditions(StringMap parameters) {
	return true;
}

bool Dock::setPostconditions(StringMap parameters) {
	situation_assessment_msgs::DatabaseRequest srv_add,srv_remove;
	situation_assessment_msgs::Fact f_to_add,f_to_remove;
	f_to_add.model=robot_name_;
	f_to_add.subject=parameters["main_agent"];
	f_to_add.predicate.push_back("isAt");
	f_to_add.value.push_back(parameters["location"]);
	f_to_add.value.push_back("in_dock");

	f_to_remove.model=robot_name_;
	f_to_add.subject=parameters["main_agent"];
	f_to_add.predicate.push_back("isAt");

	srv_remove.request.fact_list.push_back(f_to_remove);
	srv_add.request.fact_list.push_back(f_to_add);

	if (!database_remove_facts_client_.call(srv_remove)) {
		ROS_WARN("DOCK couldn't contact database");
	}	
	if (!database_add_facts_client_.call(srv_add)) {
		ROS_WARN("DOCK couldn't contact database");
	}

}


void Dock::switchToDockMode() {
	system("rosrun dynamic_reconfigure dynparam set /move_base_node/local_costmap/inflation_layer "{'inflation_radius':0.05}"");
	system("rosrun dynamic_reconfigure dynparam set /move_base_node/local_costmap "{'footprint': '[[-0.325,-0.325],[-0.325,0.325],[0.325,0.325],[0.325,-0.325]]'}"");
	system("rosrun dynamic_reconfigure dynparam set /move_base_node/local_costmap/obstacle_layer "{'unknown_threshold': 12}"");
	system("rosrun dynamic_reconfigure dynparam set /move_base_node/global_costmap/inflation_layer "{'inflation_radius':0.05}"");
	system("rosrun dynamic_reconfigure dynparam set /move_base_node/global_costmap "{'footprint': '[[-0.325,-0.325],[-0.325,0.325],[0.325,0.325],[0.325,-0.325]]'}"");
	system("rosrun dynamic_reconfigure dynparam set /move_base_node/global_costmap/obstacle_layer "{'unknown_threshold': 12}" ");
	system("rosrun dynamic_reconfigure dynparam set /move_base_node/DWAPlannerROS 
		"{'max_vel_x': 0.1,'max_vel_y': 0.05,'max_trans_vel': 0.05,
		'min_trans_vel': 0.01,'max_rot_vel': 0.05,'min_rot_vel': 0.01,
		'forward_point_distance': 0.165,'yaw_goal_tolerance': 0.17, 'xy_goal_tolerance': 0.05}"");
}
void Dock::switchToNavMode() {
	system("rosrun dynamic_reconfigure dynparam set /move_base_node/local_costmap/inflation_layer 
		"{'inflation_radius':0.55}"");
	system("rosrun dynamic_reconfigure dynparam set /move_base_node/local_costmap "{'footprint': 
	'[[-0.325,-0.325],[-0.325,0.325],[0.325,0.325],[0.46, 0.0],[0.325,-0.325]]'}"");
	system("rosrun dynamic_reconfigure dynparam set /move_base_node/local_costmap/obstacle_layer 
		"{'unknown_threshold': 8}"");
	system("rosrun dynamic_reconfigure dynparam set /move_base_node/global_costmap/inflation_layer 
		"{'inflation_radius':0.55}"");
	system("rosrun dynamic_reconfigure dynparam set /move_base_node/global_costmap "{'footprint': 
	'[[-0.325,-0.325],[-0.325,0.325],[0.325,0.325],[0.46, 0.0],[0.325,-0.325]]'}"");
	system("rosrun dynamic_reconfigure dynparam set /move_base_node/global_costmap/obstacle_layer 
		"{'unknown_threshold': 8}" ");
	system("rosrun dynamic_reconfigure dynparam set /move_base_node/DWAPlannerROS "{'max_vel_x': 0.55,'max_vel_y': 0.1,
	'max_trans_vel': 0.55,'min_trans_vel': 0.1,'max_rot_vel': 1.0,'min_rot_vel': 0.3,'forward_point_distance': 0.325,
	'yaw_goal_tolerance': 0.17, 'xy_goal_tolerance': 0.2}"");
}

void Dock::execute(const action_management_msgs::ManageActionGoalConstPtr& goal) {
	if (!checkActionName(pick_action.name)) return;

	StringMap parameters=extractParametersFromMsg(pick_action=goal.parameters);

	situation_assessment_msgs::QueryDatabase srv_database;
	srv_database.request.query.model=robot_name_;
	srv_database.request.query.subject=parameters["location"];
	srv_database.request.query.predicate.push_back("pose");
	srv_database.request.query.predicate.push_back("dock");
	if (!database_query_client_.call(srv_database)) {
		ROS_ERROR("DOCK couldn't contact database");
		setResult("FAILED","couldn't get position of location",false);
		action_server_.setAborted();
		return;
	}
	if (srv_database.response.result.size()==0 || srv_database.response.result[0].value.size()<6) {
		ROS_ERROR("DOCK location pose not present in the db");
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

	switchToDockMode();

	if (handleMoveRequest(pose)) {
		setResult("SUCCEEDED","",true);
		action_server_.setSucceded(result_);
	}
	switchToNavMode();
	return;
}
