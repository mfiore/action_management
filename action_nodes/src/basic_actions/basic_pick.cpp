
#include <action_nodes/basic_actions/basic_pick.h>

BasicPick::BasicPick(ros::NodeHandle node_handle):BasicAction("pick",node_handle) {
	parameters_.push_back("main_object");
	parameters_.push_back("main_agent");

	put_object_in_hand_client_=node_handle_.serviceClient<situation_assessment_msgs::PutObjectInHand>("/situation_assessment/put_object_in_hand",1000);

}

bool BasicPick::checkPreconditions(StringMap parameters) {
	// ROS_INFO("%s - checking preconditions",action_name_.c_str());

 	if (!checkParameterPresence(parameters)) return false;
 	string agent=parameters["main_agent"];
 	string object=parameters["main_object"];

 	situation_assessment_msgs::Fact f_loc;
 	f_loc.model=robot_name_;
 	f_loc.subject=agent+"_torso";
 	f_loc.predicate.push_back("isAt");

 	std::string agent_at=queryDatabase(f_loc);

 	f_loc.subject=object;

 	std::string object_at=queryDatabase(f_loc);

 	situation_assessment_msgs::Fact f_has;
 	f_has.model=robot_name_;
 	f_has.subject=parameters["main_agent"];
 	f_has.predicate.push_back("has");

 	string r=queryDatabase(f_has);
 	return r=="" && agent_at==object_at;
}

void BasicPick::setPostconditions(StringMap parameters) {
	string agent=parameters["main_agent"];
	string object=parameters["main_object"];

	situation_assessment_msgs::Fact f;

	f.model=robot_name_;
	f.subject=agent;
	f.predicate.push_back("has");
	f.value.push_back(object);

	std::vector<situation_assessment_msgs::Fact> add_facts={f};
	addFacts(add_facts);

	// situation_assessment_msgs::Fact query_location;
	// query_location.model=robot_name_;
	// query_location.subject=object;
	// query_location.predicate.push_back("isInArea");
	// query_location.value.push_back("agent");

	// std::vector<situation_assessment_msgs::Fact> set_facts={query_location};
	// setFacts(set_facts);

	// std::string location=queryDatabase(query_location);

	// if (location=="") {
	// 	ROS_WARN("PICK failed to get location of object");
	// }

	// situation_assessment_msgs::Fact remove_location_f;
	// remove_location_f.model=robot_name_;	
	// remove_location_f.subject=object;	
	// remove_location_f.predicate.push_back("at");	
	// remove_location_f.value.push_back(location);	

	// std::vector<situation_assessment_msgs::Fact> remove_facts={remove_location_f};

	// removeFacts(remove_facts);

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

