
#include <action_executor/basic_actions/BasicPlace.h>

BasicPlace::BasicPlace():BasicAction("BasicPlace") {
	parameters_.push_back("main_object");
	parameters_.push_back("support_object");
}

bool BasicPlace::checkPreconditions(StringMap parameters) {
	situation_assessment_msgs::QueryDatabase srv;
	srv.request.query.model=robot_name_;
	srv.request.query.subject=parameters["main_agent"];
	srv.request.query.predicate.push_back("hasInHand");
	srv.request.query.value.push_back(parameters["main_object"]);

	if (database_query_client_.call(srv)) {
		return response.result.size()<2;
	}
	else {
		ROS_ERROR("%s Failed to contact db",action_name_);
	}
}

void BasicPlace::setPostconditions(StringMap parameters) {
	situation_assessment_msgs::Fact f;
	f.model=robot_name_;
	f.subject=parameters["main_agent"];
	f.predicate.push_back("has");
	f.value.push_back(parameters["main_object"]);

	situation_assessment_msgs::DatabaseRequest srv;
	srv.fact_list.push_back(f);
	if (!database_remove_facts_client_.call(srv)) {
		ROS_ERROR("%s failed to contact db");
	} 
}

bool BasicPlace::shouldStop(StringMap parameters) {
	return false;
}

