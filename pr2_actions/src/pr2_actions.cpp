#include <ros/ros.h>

#include "pr2_actions/approach.h"
#include "pr2_actions/dock.h"
#include "pr2_actions/undock.h"
#include "pr2_actions/pick.h"
#include "pr2_actions/place.h"

int main(int argc, char** argv) {
	ros::init(argc,argv,"pr2_actions");
	ros::NodeHandle node_handle;

	Approach approach(node_handle);
	Dock dock(node_handle);
	Undock undock(node_handle);
	Pick pick(node_handle);
	Place place(node_handle);

	ros::spin();
	return 0;
}