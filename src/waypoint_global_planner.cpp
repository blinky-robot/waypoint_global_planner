#include <fstream>
#include <pluginlib/class_list_macros.h>
#include <waypoint_global_planner.h>

// register as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::WaypointGlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

namespace global_planner {

WaypointGlobalPlanner::WaypointGlobalPlanner() 
 : waypoint_file_name("waypoint_list.txt")
{ 
	ros::NodeHandle private_nh("~/WaypointGlobalPlanner");
	private_nh.param("waypoint_list", waypoint_file_name, waypoint_file_name); 
}

WaypointGlobalPlanner::WaypointGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
 : waypoint_file_name("waypoint_list.txt")
{
	initialize(name, costmap_ros);

	ros::NodeHandle private_nh("~/" + name);
	private_nh.param("waypoint_list", waypoint_file_name, waypoint_file_name); 
}

void WaypointGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
}

bool WaypointGlobalPlanner::makePlan(
	const geometry_msgs::PoseStamped& start,
	const geometry_msgs::PoseStamped& goal,
	std::vector<geometry_msgs::PoseStamped>& plan)
{
	ROS_INFO("Making global plan based on file='%s'", waypoint_file_name.c_str());

	// deserialize waypoints from binary file
	std::vector<geometry_msgs::PoseStamped> waypoints;

	// TODO: return false if could not open file
	std::ifstream ifs(waypoint_file_name.c_str(), std::ios::in|std::ios::binary);

	if (!ifs.is_open())
	{
		ROS_ERROR("Failed to open waypoint list '%s'", waypoint_file_name.c_str());
		return false;
	}

	ifs.seekg (0, std::ios::end);
	std::streampos end = ifs.tellg();
	ifs.seekg(0, std::ios::beg);
	std::streampos begin = ifs.tellg();

	uint32_t file_size = end-begin;
	boost::shared_array<uint8_t> ibuffer(new uint8_t[file_size]);
	ifs.read((char*) ibuffer.get(), file_size);
	ros::serialization::IStream istream(ibuffer.get(), file_size);
	ros::serialization::deserialize(istream, waypoints);
	ifs.close();

	// populate plan
	for (int i = 0; i < waypoints.size(); i++)
	{
		plan.push_back(waypoints[i]);
	}

	return true;
}

};
