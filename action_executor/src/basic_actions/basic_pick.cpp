
#include <action_executor/basic_actions/BasicPick.h>

BasicPick::BasicPick():BasicAction("pick") {
	parameters_.push_back("main_object");
}

bool BasicPick::checkPreconditions(StringMap parameters) {
	situation_assessment_msgs::QueryDatabase srv;
	srv.request.query.model=robot_name_;
	srv.request.query.subject=parameters["main_agent"];
	srv.request.query.predicate.push_back("has");

	if (database_query_client_.call(srv)) {
		return response.result.size()<2;
	}
	else {
		ROS_ERROR("%s Failed to contact db",action_name_);
	}
}

void BasicPick::setPostconditions(StringMap parameters) {
	situation_assessment_msgs::Fact f;
	f.model=robot_name_;
	f.subject=parameters["main_agent"];
	f.predicate.push_back("has");
	f.value.push_back(parameters["main_object"]);

	situation_assessment_msgs::DatabaseRequest srv;
	srv.fact_list.push_back(f);
	if (!database_add_facts_client_.call(srv)) {
		ROS_ERROR("%s failed to contact db");
	} 
}

bool BasicPick::shouldStop(StringMap parameters) {
	return false;
}

