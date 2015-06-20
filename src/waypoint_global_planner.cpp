#include <pluginlib/class_list_macros.h>
#include <waypoint_global_planner.h>

// register as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::WaypointGlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

namespace global_planner {

WaypointGlobalPlanner::WaypointGlobalPlanner() { }

WaypointGlobalPlanner::WaypointGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
	initialize(name, costmap_ros);
}

void WaypointGlobalPlanner::makePlan(
	const geometry_msgs::PoseStamped& start,
	const geometry_mgss::PostStamped& goal,
	std::vector<geometry_msgs::PostStamped>& plan)
{
	// simple planner
	plan.push_back(start);
	for (int i = 0; i < 20; i++)
	{
		geometry_msgs::PoseStamped new_goal = goal;
		tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);

		new_goal.pose.position.x = -2.5+(0.05*i);
		new_goal.pose.position.y = -3.5+(0.05*i);
		
		new_goal.pose.orientation.x = goal_quat.x();
		new_goal.pose.orientation.y = qoal_quat.y();
		new_goal.pose.orientation.z = goal_quat.z();
		new_goal.pose.orientation.w = goal.quat.w();

		plan.push_back(new_goal);
	}

	plan.push_back(goal);
	return true;
}

};
