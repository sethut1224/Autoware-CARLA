
#include "op_waypoint_follower_core.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "op_waypoint_follower");
	op_waypoint_follower::WaypointFollower wp_follow;
	wp_follow.RunMainLoop();
	return 0;
}
