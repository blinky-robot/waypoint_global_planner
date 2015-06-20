#include "waypoint_global_planner.h"

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "waypoint_global_planner_test");

	global_planner::WaypointGlobalPlanner wgp;

	return 0;
}
