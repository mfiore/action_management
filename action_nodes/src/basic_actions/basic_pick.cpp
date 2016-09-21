
#include <action_nodes/basic_actions/basic_pick.h>

BasicPick::BasicPick(ros::NodeHandle node_handle):BasicAction("pick",node_handle) {
	parameters_.push_back("main_object");
	parameters_.push_back("main_agent");

	put_object_in_hand_client_=node_handle_.serviceClient<situation_assessment_msgs::PutObjectInHand>("/situation_assessment/put_object_in_hand",1000);

}

bool BasicPick::checkPreconditions(StringMap parameters) {
	// ROS_INFO("%s - checking preconditions",action_name_.c_str());

 	if (!checkParameterPresence(parameters)) return false;

	situation_assessment_msgs::QueryDatabase srv;
	srv.request.query.model=robot_name_;
	srv.request.query.subject=parameters["main_agent"];
	srv.request.query.predicate.push_back("has");

	if (database_query_client_.call(srv)) {
		// ROS_INFO("%s - response is %d",action_name_.c_str(),srv.response.result.size()<2);
		return srv.response.result.size()==0;
	}
	else {
		ROS_ERROR("%s Failed to contact db",action_name_.c_str());
		return false;
	}
}

void BasicPick::setPostconditions(StringMap parameters) {
	situation_assessment_msgs::Fact f;

	// ROS_INFO("BasicPick - robo tname is %s",robot_name_.c_str());
	f.model=robot_name_;
	f.subject=parameters["main_agent"];
	f.predicate.push_back("has");
	f.value.push_back(parameters["main_object"]);

	situation_assessment_msgs::DatabaseRequest srv;
	srv.request.fact_list.push_back(f);
	if (!database_add_facts_client_.call(srv)) {
		ROS_ERROR("%s failed to contact db",action_name_.c_str());
	} 

	situation_assessment_msgs::PutObjectInHand srv_put;
	srv_put.request.object=parameters["main_object"];
	srv_put.request.agent=parameters["main_agent"];
	srv_put.request.has_object=true;

	if (!put_object_in_hand_client_.call(srv_put)) {
		ROS_ERROR("%s failed to put object in hand",action_name_.c_str());
	}

}

// bool BasicPick::shouldStop(StringMap parameters) {
	// return false;
// }

