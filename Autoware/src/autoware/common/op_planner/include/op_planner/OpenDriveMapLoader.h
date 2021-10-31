
/// \file OpenDriveMapLoader.h
/// \brief Functions for Loading OpenDrive map file format.
/// \author Armin Straller
/// \date Mar 3, 2021

#ifndef OPENDRIVEMAPLOADER_H_
#define OPENDRIVEMAPLOADER_H_

#include <op_planner/MappingHelpers.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <ros/ros.h>
#include <op_planner/opendrive/types.hpp>

namespace PlannerHNS {

class OpenDriveMapLoader {
public:
	OpenDriveMapLoader(int map_version = 1);
	virtual ~OpenDriveMapLoader();

	/**
	 * @brief Read xml based .xodr map file format. It is a opendrive file generated from commonroad lanelet 2 based format. Conversion structure can be used for expansion to other Opendrive based files.
	 * @param opendriveFile
	 * @param map, OpenPlanner RoadNetwork map
	 */
	void LoadXODR(const std::string& opendriveFile, RoadNetwork& map);

	/**
	 * @brief Should be called before LoadXODR to take effect. stitching process should be withing the loading process , shouldn't be executed separately
	 */
	void EnableLaneStitching()
	{
		_bLaneStitch = true;
	}

	std::vector<Lane> GetLanesList(const opendrive::OpenDriveData* odr);
	std::vector<Curb> GetCurbsList(const opendrive::OpenDriveData* odr);
	std::vector<Boundary> GetBoundariesList(const opendrive::OpenDriveData* odr);
	std::vector<StopLine> GetStopLinesList(const opendrive::OpenDriveData* odr);
	std::vector<TrafficLight> GetTrafficLightsList(const opendrive::OpenDriveData* odr);
	std::vector<RoadSegment> GetRoadSegmentsList(const opendrive::OpenDriveData* odr);
	Lane GetLaneInfo(const opendrive::OpenDriveData* odr, const opendrive::RoadInformation* road, const opendrive::LaneInfo* laneInfo);
	std::vector<int> GetFromIDs(const opendrive::OpenDriveData* odr, const opendrive::RoadInformation* road, const opendrive::LaneInfo* laneInfo);
	std::vector<int> GetToIDs(const opendrive::OpenDriveData* odr, const opendrive::RoadInformation* road, const opendrive::LaneInfo* laneInfo);
	std::vector<WayPoint> GetCenterLaneData(const opendrive::OpenDriveData* odr, const opendrive::RoadInformation* road, const opendrive::LaneInfo* laneInfo);
	std::vector<WayPoint> GetOuterLaneData(const opendrive::OpenDriveData* odr, const opendrive::RoadInformation* road, const opendrive::LaneInfo* laneInfo);
	PlannerHNS::WayPoint GeneratePointFromLine(const opendrive::GeometryAttributesLine *line, const std::vector<opendrive::LaneWidth> width, int laneId, unsigned int waypointId, int sideId, double sOffset, double ds);
	PlannerHNS::WayPoint GeneratePointFromArc(const opendrive::GeometryAttributesArc *arc, const std::vector<opendrive::LaneWidth> width, int laneId, unsigned int waypointId, int sideId, double sOffset, double ds);
	std::vector<opendrive::LaneWidth> GetLaneWidths(const opendrive::OpenDriveData* odr, const opendrive::RoadInformation* road, const opendrive::LaneInfo* laneInfo, double factor);
	int FindLeftmostPointInJunctionPointsIndex(std::vector<PlannerHNS::WayPoint>& points);
	bool isPointLeftOfLine(PlannerHNS::WayPoint p0, PlannerHNS::WayPoint p1, PlannerHNS::WayPoint p2);

private:
	PlannerHNS::RoadNetwork* _pMap;
	int _map_version;
	bool _bLaneStitch;
	double _resolution = 1.0;
	unsigned int _waypointCounter = 0;

};

} /* namespace PlannerHNS */

#endif /* OPENDRIVEMAPLOADER_H_ */