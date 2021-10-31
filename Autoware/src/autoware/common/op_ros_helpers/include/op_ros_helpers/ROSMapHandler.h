/// \file  ROSMapHandler.h
/// \brief Handle road network mapping functionality for OpenPlanner, such as message subscription, reading the map, measurement conversion
/// 		It handles (Vector Maps, KML, Lanelet2)
/// \author Hatem Darweesh
/// \date June 18, 2021

#ifndef ROSMAPHANDLER_H_
#define ROSMAPHANDLER_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <autoware_lanelet2_msgs/MapBin.h>
#include "vector_map_msgs/PointArray.h"
#include "vector_map_msgs/LaneArray.h"
#include "vector_map_msgs/NodeArray.h"
#include "vector_map_msgs/StopLineArray.h"
#include "vector_map_msgs/DTLaneArray.h"
#include "vector_map_msgs/LineArray.h"
#include "vector_map_msgs/AreaArray.h"
#include "vector_map_msgs/SignalArray.h"
#include "vector_map_msgs/StopLine.h"
#include "vector_map_msgs/VectorArray.h"
#include "op_planner/RoadNetwork.h"
#include "op_planner/PlannerCommonDef.h"
#include "op_planner/MatrixOperations.h"
#include "op_utility/DataRW.h"


namespace PlannerHNS
{

class MapHandler
{
private:

	PlannerHNS::MAP_SOURCE_TYPE m_MapType;
	std::string m_MapPath;
	std::string m_MapFileName;
	WayPoint m_MapOrigin;
	autoware_lanelet2_msgs::MapBin m_Lanelet2Bin;
	bool m_bMap;
	bool m_bMapBinReceived;
	bool m_bKmlMapFileNameReceived;

	UtilityHNS::MapRaw m_MapRaw;
	ros::Subscriber sub_map_file_name;
	ros::Subscriber sub_bin_map;
	ros::Subscriber sub_lanes;
	ros::Subscriber sub_points;
	ros::Subscriber sub_dt_lanes;
	ros::Subscriber sub_intersect;
	ros::Subscriber sup_area;
	ros::Subscriber sub_lines;
	ros::Subscriber sub_stop_line;
	ros::Subscriber sub_signals;
	ros::Subscriber sub_vectors;
	ros::Subscriber sub_curbs;
	ros::Subscriber sub_edges;
	ros::Subscriber sub_way_areas;
	ros::Subscriber sub_cross_walk;
	ros::Subscriber sub_nodes;

	void callbackGetkmlMapFileName(const std_msgs::String& file_name);
	void callbackGetLanelet2(const autoware_lanelet2_msgs::MapBin& msg);
	void callbackGetVMLanes(const vector_map_msgs::LaneArray& msg);
	void callbackGetVMPoints(const vector_map_msgs::PointArray& msg);
	void callbackGetVMdtLanes(const vector_map_msgs::DTLaneArray& msg);
	void callbackGetVMIntersections(const vector_map_msgs::CrossRoadArray& msg);
	void callbackGetVMAreas(const vector_map_msgs::AreaArray& msg);
	void callbackGetVMLines(const vector_map_msgs::LineArray& msg);
	void callbackGetVMStopLines(const vector_map_msgs::StopLineArray& msg);
	void callbackGetVMSignal(const vector_map_msgs::SignalArray& msg);
	void callbackGetVMVectors(const vector_map_msgs::VectorArray& msg);
	void callbackGetVMCurbs(const vector_map_msgs::CurbArray& msg);
	void callbackGetVMRoadEdges(const vector_map_msgs::RoadEdgeArray& msg);
	void callbackGetVMWayAreas(const vector_map_msgs::WayAreaArray& msg);
	void callbackGetVMCrossWalks(const vector_map_msgs::CrossWalkArray& msg);
	void callbackGetVMNodes(const vector_map_msgs::NodeArray& msg);
	void LoadKmlMap(const std::string& file_name, RoadNetwork& map);
	void LoadOpenDriveMap(const std::string& file_name, RoadNetwork& map);

public:
	MapHandler();
	virtual ~MapHandler();
	void InitMapHandler(ros::NodeHandle& nh, const std::string& source_topic, const std::string& map_path_topic, const std::string& map_origin_topic);
	bool IsMapLoaded();
	void LoadMap(RoadNetwork& map, bool bEnableLaneChange);
};

} /* namespace PlannerHNS */

#endif /* ROSMAPHANDLER_H_ */
