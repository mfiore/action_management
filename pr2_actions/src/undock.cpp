#include <pr2_actions/undock.h>

Undock::Undock(ros::NodeHandle node_handle):
MovementAction("undock",node_handle) {

	cmd_vel_pub_=node_handle_.advertise<geometry_msgs::Twist>("/base_controller/command", 1);

	parameters_.push_back("location");
	parameters_.push_back("main_agent");
}

bool Undock::checkPreconditions(StringMap parameters) {
	situation_assessment_msgs::QueryDatabase srv;
	srv.request.query.model=robot_name_;
	srv.request.query.subject=parameters["main_agent"];
	srv.request.query.predicate.push_back("isAt");
	srv.request.query.value.push_back(parameters["location"]);
	srv.request.query.value.push_back("in_dock");


	if (!database_query_client_.call(srv)) {
		ROS_WARN("UNDOCK couldn't contact database");
	}	

	return srv.response.result.size()>0;
}

bool Undock::setPostconditions(StringMap parameters) {
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
		ROS_WARN("UNDOCK couldn't contact database");
	}	
	if (!database_add_facts_client_.call(srv_add)) {
		ROS_WARN("UNDOCK couldn't contact database");
	}

}



void Undock::execute(const action_management_msgs::ManageActionGoalConstPtr& goal) {
	if (!checkActionName(goal->action.name)) return;
	StringMap parameters=extractParametersFromMsg(goal->parameters);

	if (!checkPreconditions(parameters)) {
		setResult("FAILURE","preconditions not satisfied",false);
		action_server_->setAborted(result);
		return;
	}

	   //we will be sending commands of type "twist"
	    geometry_msgs::Twist base_cmd;

	    else if (direction=='b') {
	        base_cmd.linear.x=-0.2;
	    }

	    cmd_vel_pub_.publish(base_cmd);

	    //Get the time and store it in the time variable.
	    ros::Time time = ros::Time::now();
	//Wait a duration of one second.
	    ros::Duration d = ros::Duration(2);
	    d.sleep();

	    base_cmd.linear.x=0;
	    base_cmd.linear.y=0;
	    cmd_vel_pub_.publish(base_cmd);

		setResult("SUCCEEDED","",true);
		action_server_.setSucceded(result_);
	return;
}
