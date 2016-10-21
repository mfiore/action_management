
#include <action_nodes/basic_actions/basic_place.h>

BasicPlace::BasicPlace(ros::NodeHandle node_handle):BasicAction("place",node_handle) {
	parameters_.push_back("main_object");
	parameters_.push_back("target");

	put_object_in_hand_client_=node_handle_.serviceClient<situation_assessment_msgs::PutObjectInHand>("/situation_assessment/put_object_in_hand",1000);
	place_object_client_=node_handle_.serviceClient<situation_assessment_msgs::PlaceObject>("/situation_assessment/place_object",1000);

}

bool BasicPlace::checkPreconditions(StringMap parameters) {
	if (!checkParameterPresence(parameters)) return false;

	string agent=parameters["main_agent"];
	string object=parameters["main_object"];
	string target=parameters["target"];

	situation_assessment_msgs::Fact f_loc;
	f_loc.model=robot_name_;
	f_loc.subject=agent+"_torso";
	f_loc.predicate.push_back("isAt");

	std::string agent_areas=queryDatabase(f_loc);

	f_loc.subject=target;

	std::string object_areas=queryDatabase(f_loc);

	situation_assessment_msgs::Fact f_has;
	f_has.model=robot_name_;
	f_has.subject=agent;
	f_has.predicate.push_back("has");

	string human_object=queryDatabase(f_has);

	return agent_areas==object_areas && human_object!="";
}

void BasicPlace::setPostconditions(StringMap parameters) {
	string agent=parameters["main_agent"];
	string object=parameters["main_object"];
	string target=parameters["target"];

	std::vector<situation_assessment_msgs::Fact> remove_facts,add_facts;

	situation_assessment_msgs::Fact remove_has_f;
	remove_has_f.model=robot_name_;
	remove_has_f.subject=agent;
	remove_has_f.predicate.push_back("has");
	remove_has_f.value.push_back(object);

	remove_facts={remove_has_f};

	removeFacts(remove_facts);

	// situation_assessment_msgs::Fact add_at_f;
	// add_at_f.model=robot_name_;
	// add_at_f.subject=object;
	// add_at_f.predicate.push_back("isInArea");
	// add_at_f.value.push_back(target);

	// add_facts={add_at_f};

	// addFacts(add_facts);

	situation_assessment_msgs::Fact hand_pose_query;
	hand_pose_query.model=robot_name_;
	hand_pose_query.subject=agent+"_hand";
	hand_pose_query.predicate={"pose"};

	std::vector<std::string> hand_pose_string=queryDatabaseComplete(hand_pose_query);

	situation_assessment_msgs::PlaceObject srv_place;
	srv_place.request.name=parameters["main_object"];
	srv_place.request.pose.position.x=stod(hand_pose_string[0]);
	srv_place.request.pose.position.y=stod(hand_pose_string[1]);
	srv_place.request.pose.position.z=stod(hand_pose_string[2]);
	srv_place.request.pose.orientation.x=0;
	srv_place.request.pose.orientation.y=0;
	srv_place.request.pose.orientation.z=0;
	srv_place.request.pose.orientation.w=1;

	if (!place_object_client_.call(srv_place)) {
		ROS_ERROR("%s failed to place object",action_name_.c_str());
	}

	situation_assessment_msgs::PutObjectInHand srv_put;
	srv_put.request.object=parameters["main_object"];
	srv_put.request.agent=parameters["main_agent"];
	srv_put.request.has_object=false;


	if (!put_object_in_hand_client_.call(srv_put)) {
		ROS_ERROR("%s failed to put object in hand",action_name_.c_str());
	}

}

// bool BasicPlace::shouldStop(StringMap parameters) {
	// return false;
// }

