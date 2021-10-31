/*
 * OpenDriveMapLoader.cpp
 *
 *  Created on: Mar 3, 2021
 *      Author: armin
 */


#include <op_planner/OpenDriveMapLoader.h>
#include <op_planner/PlanningHelpers.h>
#include <op_planner/opendrive/geometry/GeometryGenerator.hpp>
#include <op_planner/opendrive/parser/OpenDriveParser.hpp>

namespace PlannerHNS {

OpenDriveMapLoader::OpenDriveMapLoader(int map_version) : _map_version(map_version)
{
	_pMap = nullptr;
	_bLaneStitch = false;
	_waypointCounter = 1;
}

OpenDriveMapLoader::~OpenDriveMapLoader()
{
}

void OpenDriveMapLoader::LoadXODR(std::string const &file, RoadNetwork& map)
{
	// std::cout << "***** Called LoadXODR ****" << std::endl;
    opendrive::OpenDriveData odr;
    bool bSuccess =  opendrive::parser::OpenDriveParser::Parse(file.c_str(), odr, opendrive::parser::XmlInputType::FILE);

    // check if the file was loaded successfully
	if(!bSuccess)
	{
		ROS_FATAL("Unable to load Map file!");
		return;
	}

	_pMap = &map;

	// std::cout << " >> Reading Data from OpenDrive map file ... " << std::endl;

	// std::cout << " >> Load Lanes from OpenDrive file .. " << std::endl;
	std::vector<Lane> laneLinksList = GetLanesList(&odr);

	MappingHelpers::FixTwoPointsLanes(laneLinksList);

	map.roadSegments.clear();
	//map.roadSegments = GetRoadSegmentsList(&odr);
	//map.roadSegments.at(0).id = 0;

	PlannerHNS::RoadSegment rS;
	rS.id = 0;
	map.roadSegments.push_back(rS);

	//Fill the relations
	for(unsigned int j=0; j < laneLinksList.size(); j++)
	{
		//PlanningHelpers::CalcAngleAndCost(laneLinksList.at(j).points);
		// std::cout << " >> Added pRoad to Lane: " << laneLinksList.at(j).id << std::endl;
		laneLinksList.at(j).pRoad = &map.roadSegments.at(0);
		map.roadSegments.at(0).Lanes.push_back(laneLinksList.at(j));
	}

	// std::cout << " >> Load Stop Lines from OpenDrive file ... " << std::endl;
	std::vector<StopLine> stopLines = GetStopLinesList(&odr);

	// std::cout << " >> Load Traffic Lights from OpenDrive file ... " << std::endl;
	std::vector<TrafficLight> trafficLights = GetTrafficLightsList(&odr);

	// std::cout << " >> Load Curbs from OpenDrive file .. " << std::endl;
	std::vector<Curb> curbs = GetCurbsList(&odr);

	std::vector<Boundary> boundaries = GetBoundariesList(&odr);

	// std::cout << " >> Link lanes and waypoints with pointers ... " << std::endl;
	//Link Lanes by pointers
	MappingHelpers::LinkLanesPointers(map);

	//Link waypoints by pointers
	// std::cout << " >> Link missing branches and waypoints... " << std::endl;
	MappingHelpers::LinkMissingBranchingWayPointsV2(map);

	//Link lanechange waypoints by pointers
	// std::cout << " >> Link Lane change semantics ... " << std::endl;
	MappingHelpers::LinkLaneChangeWaypointsPointers(map);

	if(_bLaneStitch && map.roadSegments.size() > 0)
	{
		MappingHelpers::StitchLanes(map.roadSegments.at(0).Lanes);
	}

	// map.stopLines.clear();
	map.stopLines = stopLines;

	// map.trafficLights.clear();
	map.trafficLights = trafficLights;

	// map.curbs.clear();
	map.curbs = curbs;

	map.boundaries = boundaries;

	std::cout << " >> Link Boundaries and Waypoints ... " << std::endl;
	MappingHelpers::ConnectBoundariesToWayPoints(map);
	MappingHelpers::LinkBoundariesToWayPoints(map);

	MappingHelpers::LinkTrafficLightsIntoGroups(map);
	MappingHelpers::ConnectTrafficLightsAndStopLines(map);
	
	//Link waypoints && StopLines
	// std::cout << " >> Link Stop lines and Traffic lights ... " << std::endl;
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i = 0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			for(unsigned int itl = 0; itl < map.trafficLights.size(); itl++)
			{	
				if(map.trafficLights.at(itl).CheckLane(map.roadSegments.at(rs).Lanes.at(i).id))
				{
					map.trafficLights.at(itl).pLanes.push_back(&map.roadSegments.at(rs).Lanes.at(i));
					map.roadSegments.at(rs).Lanes.at(i).trafficlights.push_back(map.trafficLights.at(itl));
					// std::cout << " >> Added Traffic Light " << map.trafficLights.at(itl).id << " to Lane." << std::endl;
				}
			}

			for(unsigned int isl = 0; isl < map.stopLines.size(); isl++)
			{
				if(map.stopLines.at(isl).laneId == map.roadSegments.at(rs).Lanes.at(i).id)
				{
					map.stopLines.at(isl).pLane = &map.roadSegments.at(rs).Lanes.at(i);
					map.roadSegments.at(rs).Lanes.at(i).stopLines.push_back(map.stopLines.at(isl));

					// TODO: Figure out how the waypoints are matched. Issue due to lanes which are super close to each other. 
					WayPoint wp((map.stopLines.at(isl).points.at(0).pos.x+map.stopLines.at(isl).points.at(1).pos.x)/2.0, (map.stopLines.at(isl).points.at(0).pos.y+map.stopLines.at(isl).points.at(1).pos.y)/2.0, (map.stopLines.at(isl).points.at(0).pos.z+map.stopLines.at(isl).points.at(1).pos.z)/2.0, (map.stopLines.at(isl).points.at(0).pos.a+map.stopLines.at(isl).points.at(1).pos.a)/2.0);
					// WayPoint wp( map.stopLines.at(isl).points.at(0).pos.x, map.stopLines.at(isl).points.at(0).pos.y, map.stopLines.at(isl).points.at(0).pos.z, map.stopLines.at(isl).points.at(0).pos.a );
					map.roadSegments.at(rs).Lanes.at(i).points.at(PlanningHelpers::GetClosestNextPointIndexFast(map.roadSegments.at(rs).Lanes.at(i).points, wp)).stopLineID = map.stopLines.at(isl).id;
					// std::cout << " >> Added Stop Line " << map.stopLines.at(isl).id << " to Waypoint." << std::endl;
				}
			}
		}
	}

	// std::cout << " >> Find Max IDs ... " << std::endl;
	MappingHelpers::GetMapMaxIds(map);

	// std::cout << "Map loaded from OpenDrive File with (" << laneLinksList.size()  << ") lanes, First Point ( " << MappingHelpers::GetFirstWaypoint(map).pos.ToString() << ")"<< std::endl;

	_pMap = nullptr;
}

std::vector<Lane> OpenDriveMapLoader::GetLanesList(const opendrive::OpenDriveData* odr)
{
	std::vector<Lane> lList;
	// first iteration connect everyting based on ids
	for(const opendrive::RoadInformation &road : odr->roads)
    {
		for(const opendrive::LaneSection &laneSection : road.lanes.lane_sections)
    	{
			for(const opendrive::LaneInfo &laneInfoRight : laneSection.right)
			{
				if(	laneInfoRight.attributes.type == opendrive::LaneType::Driving )
				//	|| laneInfoRight.attributes.type == opendrive::LaneType::Shoulder)
				{
					Lane l;
					l = GetLaneInfo(odr, &road, &laneInfoRight);
					lList.push_back(l);
				}
			}
			for(const opendrive::LaneInfo &laneInfoLeft : laneSection.left)
			{
				if(	laneInfoLeft.attributes.type == opendrive::LaneType::Driving )
				//	|| laneInfoLeft.attributes.type == opendrive::LaneType::Shoulder)
				{
					Lane l;
					l = GetLaneInfo(odr, &road, &laneInfoLeft);
					lList.push_back(l);
				}
			}
		}
	}
	return lList;
}

std::vector<RoadSegment> OpenDriveMapLoader::GetRoadSegmentsList(const opendrive::OpenDriveData* odr)
{
	std::vector<RoadSegment> rlList;

	if(odr->roads.size() < 1)
		return rlList;

	for(const opendrive::RoadInformation &road : odr->roads)
    {
		RoadSegment rl;
		rl.id = road.attributes.id;
		rlList.push_back(rl);
	}

	return rlList;
}

Lane OpenDriveMapLoader::GetLaneInfo(const opendrive::OpenDriveData* odr,const opendrive::RoadInformation* road, const opendrive::LaneInfo* laneInfo)
{
	Lane ll;
	ll.id = 10 + road->attributes.id * 10 + laneInfo->attributes.id;
	
	// add all lanes to the same road segment
	ll.roadId = 0;
	ll.num = -1;
	ll.type = PlannerHNS::NORMAL_LANE;
	ll.length = road->attributes.length;
	if(laneInfo->lane_width.size() > 0)
		ll.width = laneInfo->lane_width[0].a;
	if(road->attributes.speed.size() > 0)
		ll.speed = road->attributes.speed[0].max;
	ll.fromIds 	= GetFromIDs(odr, road, laneInfo);
	ll.toIds 	= GetToIDs(odr, road, laneInfo);

	// Get Center Lane Data based on OpenDrive Road
	ll.points = GetCenterLaneData(odr, road, laneInfo);

	return ll;
}

// Get IDs of the Predecessing Roads
std::vector<int> OpenDriveMapLoader::GetFromIDs(const opendrive::OpenDriveData* odr, const opendrive::RoadInformation* road, const opendrive::LaneInfo* laneInfo)
{
	std::vector<int> fromIds;

	// check right side element
	// check if a predecessor exists
	if(road->road_link.predecessor && laneInfo->attributes.id < 0)
	{
		//predecessor type is a road
		if(road->road_link.predecessor->element_type == opendrive::ElementType::Road)
		{
			//check right side lanes with contact point end
			if(road->road_link.predecessor->contact_point == opendrive::ContactPoint::End)
				fromIds.push_back(10 + road->road_link.predecessor->id * 10 + laneInfo->attributes.id);
		}

		// no need for checking from Junctions since only incoming Roads are linked
	}

	//check left side element
	//check if a successor exists
	if(road->road_link.successor && laneInfo->attributes.id > 0)
	{
		//checking for roads
		if(road->road_link.successor->element_type == opendrive::ElementType::Road)
		{
			// check left side lanes with contact point end
			if(road->road_link.successor->contact_point == opendrive::ContactPoint::End)
				fromIds.push_back(10 + road->road_link.successor->id * 10 + laneInfo->attributes.id);
			// check left side lanes with contact point start
			if(road->road_link.successor->contact_point == opendrive::ContactPoint::Start)
				fromIds.push_back(10 + road->road_link.successor->id * 10 + laneInfo->attributes.id);
		}

		// no need for checking from Junctions since only incoming Roads are linked
	}

	return fromIds;
}

// Get IDs of the Successing Roads
std::vector<int> OpenDriveMapLoader::GetToIDs(const opendrive::OpenDriveData* odr, const opendrive::RoadInformation* road, const opendrive::LaneInfo* laneInfo)
{
	std::vector<int> toIds;

	//check right side element
	//check if a successor exists
	if(road->road_link.successor && laneInfo->attributes.id < 0)
	{
		//checking for roads
		if(road->road_link.successor->element_type == opendrive::ElementType::Road)
		{
			// check right side lanes with contact point end
			if(road->road_link.successor->contact_point == opendrive::ContactPoint::End)
				toIds.push_back(10 + road->road_link.successor->id * 10 - laneInfo->attributes.id);
			// check right side lanes with contact point start
			if(road->road_link.successor->contact_point == opendrive::ContactPoint::Start)
				toIds.push_back(10 + road->road_link.successor->id * 10 + laneInfo->attributes.id);
		}

		//successor type is a junction
		if(road->road_link.successor->element_type == opendrive::ElementType::Junction)
		{
			// loop trough all junctions in odr map
			for(const opendrive::Junction &junction : odr->junctions)
			{
				if(junction.attributes.id == road->road_link.successor->id)
				{
					for(const opendrive::JunctionConnection &connection: junction.connections)
					{
						if(connection.attributes.incoming_road == road->attributes.id && connection.links.size() > 0)
						{
							for(unsigned int i = 0; i < connection.links.size(); i++)
							{
								if(laneInfo->attributes.id == connection.links.at(i).from)
								{
									toIds.push_back(10 + connection.attributes.connecting_road * 10 + laneInfo->attributes.id);
								}
							}
						}
					}
				}
			}
		}
	}

	// check left side element
	// check if a predecessor exists
	if(road->road_link.predecessor && laneInfo->attributes.id > 0)
	{
		//predecessor type is a road
		if(road->road_link.predecessor->element_type == opendrive::ElementType::Road)
		{
			//check left side lanes with contact point end
			if(road->road_link.predecessor->contact_point == opendrive::ContactPoint::End)
				toIds.push_back(10 + road->road_link.predecessor->id * 10 + laneInfo->attributes.id);
			//check left side lanes with contact point start
			if(road->road_link.predecessor->contact_point == opendrive::ContactPoint::Start)
				toIds.push_back(10 + road->road_link.predecessor->id * 10 - laneInfo->attributes.id);
		}

		//predecessor type is a junction
		if(road->road_link.predecessor->element_type == opendrive::ElementType::Junction)
		{
			// loop trough all junctions in odr map
			for(const opendrive::Junction &junction : odr->junctions)
			{
				if(junction.attributes.id == road->road_link.predecessor->id)
				{
					for(const opendrive::JunctionConnection &connection: junction.connections)
					{
						if(connection.attributes.incoming_road == road->attributes.id && connection.links.size() > 0)
						{
							for(unsigned int i = 0; i < connection.links.size(); i++)
							{
								if(laneInfo->attributes.id == connection.links.at(i).from)
								{
									toIds.push_back(10 + connection.attributes.connecting_road * 10 + laneInfo->attributes.id);
								}
							}
						}
					}
				}
			}
		}
	}

	return toIds;
}

std::vector<WayPoint> OpenDriveMapLoader::GetCenterLaneData(const opendrive::OpenDriveData* odr, const opendrive::RoadInformation* road, const opendrive::LaneInfo* laneInfo)
{
	std::vector<WayPoint> gps_points;
	int laneId = 10 + road->attributes.id * 10 + laneInfo->attributes.id;

	std::vector<opendrive::LaneWidth> lW = GetLaneWidths(odr, road, laneInfo, 0.5);

	// iterate trough all geometries defining the road
	for(const std::unique_ptr<opendrive::GeometryAttributes> &geometry: road->geometry_attributes)
	{
		int sideId = laneInfo->attributes.id;
		try
		{
			switch (geometry->type)
			{
				case opendrive::GeometryType::ARC:
				{
					auto arc = static_cast<opendrive::GeometryAttributesArc *>(geometry.get());

					for(double sArc = 0.0; sArc < arc->length; sArc += _resolution)
					{
						if(sArc > arc->length)
							sArc = arc->length;
						PlannerHNS::WayPoint p = GeneratePointFromArc( arc, lW, laneId, _waypointCounter, sideId, arc->start_position, sArc);
						gps_points.push_back(p);
						_waypointCounter++;
					}
					// add one last waypoint at arc->length
					PlannerHNS::WayPoint p = GeneratePointFromArc( arc, lW, laneId, _waypointCounter, sideId, arc->start_position, arc->length );
					gps_points.push_back(p);
					_waypointCounter++;
				
					break;
				}
				break;
				case opendrive::GeometryType::LINE:
				{
					
					auto line = static_cast<opendrive::GeometryAttributesLine *>(geometry.get());

					for(double sLine = 0.0; sLine < line->length; sLine += _resolution)
					{
						if(sLine > line->length)
							sLine = line->length;
						PlannerHNS::WayPoint p = GeneratePointFromLine( line, lW, laneId, _waypointCounter, sideId, line->start_position, sLine );
						gps_points.push_back(p);
						_waypointCounter++;
					}
					// add one last waypoint at line->length
					PlannerHNS::WayPoint p = GeneratePointFromLine( line, lW, laneId, _waypointCounter, sideId, line->start_position, line->length );
					gps_points.push_back(p);
					_waypointCounter++;

					break;
				}
				break;
				case opendrive::GeometryType::SPIRAL:
				{
					ROS_FATAL(">> SPIRAL Geometries are not covered by GetCenterLaneData!");
					break;
				}
				break;
				case opendrive::GeometryType::POLY3:
				{
					ROS_FATAL(">> POLY3 Geometries are not covered by GetCenterLaneData!");
					break;
				}
				break;
				case opendrive::GeometryType::PARAMPOLY3:
				{
					ROS_FATAL(">> PARAMPOLY3 Geometries are not covered by GetCenterLaneData!");
					break;
				}
				break;
				default:
				break;
			}
		}
		catch (...)
		{
			ROS_FATAL(">> Geometries are not covered by GetCenterLaneData!");
			continue;
		}

	}

	// mirror points if left side lane
	if(laneInfo->attributes.id > 0)
	{
		std::vector<WayPoint> temp;
		int size = gps_points.size();
		for(int i = 0; i < size; i++)
		{
			temp.push_back(gps_points.back());
			gps_points.pop_back();
		}
		gps_points = temp;
	}

	return gps_points;
}

std::vector<WayPoint> OpenDriveMapLoader::GetOuterLaneData(const opendrive::OpenDriveData* odr, const opendrive::RoadInformation* road, const opendrive::LaneInfo* laneInfo)
{
	std::vector<WayPoint> gps_points;
	int laneId = 10 + road->attributes.id * 10 + laneInfo->attributes.id;

	std::vector<opendrive::LaneWidth> lW = GetLaneWidths(odr, road, laneInfo, 1.0);

	// iterate trough all geometries defining the road
	for(const std::unique_ptr<opendrive::GeometryAttributes> &geometry: road->geometry_attributes)
	{
		int sideId = laneInfo->attributes.id;
		try
		{
			switch (geometry->type)
			{
				case opendrive::GeometryType::ARC:
				{
					auto arc = static_cast<opendrive::GeometryAttributesArc *>(geometry.get());

					for(double sArc = 0.0; sArc < arc->length; sArc += _resolution)
					{
						if(sArc > arc->length)
							sArc = arc->length;
						PlannerHNS::WayPoint p = GeneratePointFromArc( arc, lW, laneId, _waypointCounter, sideId, arc->start_position, sArc);
						gps_points.push_back(p);
						_waypointCounter++;
					}
					// add one last waypoint at arc->length
					PlannerHNS::WayPoint p = GeneratePointFromArc( arc, lW, laneId, _waypointCounter, sideId, arc->start_position, arc->length );
					gps_points.push_back(p);
					_waypointCounter++;
				
					break;
				}
				break;
				case opendrive::GeometryType::LINE:
				{
					
					auto line = static_cast<opendrive::GeometryAttributesLine *>(geometry.get());

					for(double sLine = 0.0; sLine < line->length; sLine += _resolution)
					{
						if(sLine > line->length)
							sLine = line->length;
						PlannerHNS::WayPoint p = GeneratePointFromLine( line, lW, laneId, _waypointCounter, sideId, line->start_position, sLine );
						gps_points.push_back(p);
						_waypointCounter++;
					}
					// add one last waypoint at line->length
					PlannerHNS::WayPoint p = GeneratePointFromLine( line, lW, laneId, _waypointCounter, sideId, line->start_position, line->length );
					gps_points.push_back(p);
					_waypointCounter++;

					break;
				}
				break;
				case opendrive::GeometryType::SPIRAL:
				{
					ROS_FATAL(">> SPIRAL Geometries are not covered by GetCenterLaneData!");
					break;
				}
				break;
				case opendrive::GeometryType::POLY3:
				{
					ROS_FATAL(">> POLY3 Geometries are not covered by GetCenterLaneData!");
					break;
				}
				break;
				case opendrive::GeometryType::PARAMPOLY3:
				{
					ROS_FATAL(">> PARAMPOLY3 Geometries are not covered by GetCenterLaneData!");
					break;
				}
				break;
				default:
				break;
			}
		}
		catch (...)
		{
			ROS_FATAL(">> Geometries are not covered by GetCenterLaneData!");
			continue;
		}

	}

	// mirror points if left side lane
	if(laneInfo->attributes.id > 0)
	{
		std::vector<WayPoint> temp;
		int size = gps_points.size();
		for(int i = 0; i < size; i++)
		{
			temp.push_back(gps_points.back());
			gps_points.pop_back();
		}
		gps_points = temp;
	}

	return gps_points;
}


PlannerHNS::WayPoint OpenDriveMapLoader::GeneratePointFromLine(const opendrive::GeometryAttributesLine *line, const std::vector<opendrive::LaneWidth> width, int laneId, unsigned int waypointId, int sideId, double sOffset, double ds)
{
	PlannerHNS::WayPoint p;
	p.laneId = laneId;
	p.id = waypointId;
	p.bDir = PlannerHNS::FORWARD_DIR;

	p.width = -1;
	double sWidth = ds + sOffset;
	// calculate the road width at the given ds
	if(width.size() == 1)
	{
		p.width = width[0].a + sWidth * width[0].b + pow(sWidth, 2.0) * width[0].c + pow(sWidth, 3.0) * width[0].d;
	}
	else if (width.size() > 1)
	{
		for(int i = 0; i < width.size()-1; i++)
		{	
			if(sWidth >= width[i].soffset && sWidth < width[i+1].soffset)
			{
				p.width = width[i].a + sWidth * width[i].b + pow(sWidth, 2.0) * width[i].c + pow(sWidth, 3.0) * width[i].d;
				break;
			}
		}

		if(sWidth >= width[width.size()-1].soffset )
		{
			p.width = width[width.size()-1].a + sWidth * width[width.size()-1].b + pow(sWidth, 2.0) * width[width.size()-1].c + pow(sWidth, 3.0) * width[width.size()-1].d;
		}

		if(p.width == -1)
		{
			ROS_FATAL(">> No sOffset segment found for Line Geometry in lane %d!", p.laneId);
			ROS_FATAL(">> ds: %f, width.size(): %d, width[0].soffset: %f, width[width.size()-1].soffset: %f!", sWidth, width.size(), width[0].soffset, width[width.size()-1].soffset);
		}
	}
	else
	{
		ROS_FATAL(">> No width found!");
	}

	// change heading offset to negative
	if(sideId <= -1)
		sideId = -1;
	// change heading offset to positive
	if(sideId >= 1)
		sideId = 1;
	
	// calculate the heading at the given ds
	tf2::Quaternion q;
	double heading = line->heading;
	if ( sideId > 0)
		heading += M_PI;
	q.setRPY( 0, 0, heading );
	p.rot.x = q[0];
	p.rot.y = q[1];
	p.rot.z = q[2];
	p.rot.w = q[3];
	
	// line defined by s reference frame
	p.pos.x = line->start_position_x + cos(line->heading) * ds + cos(line->heading + sideId * M_PI_2) * p.width;
	p.pos.y = line->start_position_y + sin(line->heading) * ds + sin(line->heading + sideId * M_PI_2) * p.width;
	p.pos.z = 0;

	p.pos.a = heading;
	p.cost = 0.0;

	return p;
}

PlannerHNS::WayPoint OpenDriveMapLoader::GeneratePointFromArc(const opendrive::GeometryAttributesArc *arc, const std::vector<opendrive::LaneWidth> width, int laneId, unsigned int waypointId, int sideId, double sOffset, double ds)
{
	PlannerHNS::WayPoint p;
	p.laneId = laneId;
	p.id = waypointId;
	p.bDir = PlannerHNS::FORWARD_DIR;
	p.width = -1;

	double sWidth = ds + sOffset;
	// calculate the road width at the given ds
	if(width.size() == 1)
	{
		p.width = width[0].a + sWidth * width[0].b + pow(sWidth, 2.0) * width[0].c + pow(sWidth, 3.0) * width[0].d;
	}
	else if (width.size() > 1)
	{
		for(int i = 0; i < width.size()-1; i++)
		{	
			if(sWidth >= width[i].soffset && sWidth < width[i+1].soffset)
			{
				p.width = width[i].a + sWidth * width[i].b + pow(sWidth, 2.0) * width[i].c + pow(sWidth, 3.0) * width[i].d;
				break;
			}
			
		}
		if(sWidth >= width[width.size()-1].soffset )
		{
			p.width = width[width.size()-1].a + sWidth * width[width.size()-1].b + pow(sWidth, 2.0) * width[width.size()-1].c + pow(sWidth, 3.0) * width[width.size()-1].d;
		}
		if(p.width == -1)
		{
			ROS_FATAL(">> No sOffset segment found for Arc Geometry in lane %d!", p.laneId);
			ROS_FATAL(">> ds: %f, width.size(): %d, width[0].soffset: %f, width[width.size()-1].soffset: %f!", sWidth, width.size(), width[0].soffset, width[width.size()-1].soffset);
		}
	}
	else
	{
		ROS_FATAL(">> No width found!");
	}

	// calculate arc parameters
	double R = 1 / arc->curvature;
	double circumference = 2.0 * R * M_PI;
	double alpha_ds = (ds / circumference) * 2.0 * M_PI;
	double angularOffset = M_PI_2;
	int offsetSign = 0.0;	

	// Left Side and Right curve
	if( sideId > 0 && arc->curvature < 0 )
	{
		offsetSign = -1;
	}
	// Left Side and Left curve
	if ( sideId > 0 && arc->curvature >= 0 )
	{
		offsetSign = -1;
	}
	// Right Side and Right curve
	if ( sideId < 0 && arc->curvature < 0 )
	{
		offsetSign = 1;
	}
	// Right Side and Left curve
	if ( sideId < 0 && arc->curvature >= 0 )
	{
		offsetSign = 1;
	}

	double heading = arc->heading + alpha_ds;
	if ( sideId > 0)
		heading += M_PI;
	
	double circle_center_x = arc->start_position_x + cos(arc->heading + angularOffset) * R;
	double circle_center_y = arc->start_position_y + sin(arc->heading + angularOffset) * R;	

	double wOffset = offsetSign * p.width;
	double dx = (R + wOffset) * cos(M_PI + arc->heading + angularOffset + alpha_ds);
	double dy = (R + wOffset) * sin(M_PI + arc->heading + angularOffset + alpha_ds);

	// calculate the heading at the given ds
	tf2::Quaternion q;
	
	q.setRPY( 0, 0, heading);
	p.rot.x = q[0];
	p.rot.y = q[1];
	p.rot.z = q[2];
	p.rot.w = q[3];
	
	p.pos.x = circle_center_x + dx;
	p.pos.y = circle_center_y + dy;
	p.pos.z = 0;
	p.pos.a = heading;
	p.cost = 0.0;

	return p;
}

// Create Stop Lines for each signal with name == "StopLine" based on s_position, lane width and geometries
std::vector<StopLine> OpenDriveMapLoader::GetStopLinesList(const opendrive::OpenDriveData* odr)
{
	std::vector<StopLine> slList;
	// first iteration connect everyting based on ids
	for(const opendrive::RoadInformation &road : odr->roads)
    {
		if(road.traffic_signal_references.size() > 0)
		{
			// get the current road's lane ids
			int laneId = 0;
			std::vector<opendrive::LaneWidth> width;
			for(const opendrive::LaneSection &laneSection : road.lanes.lane_sections)
			{
				for(const opendrive::LaneInfo &laneInfoRight : laneSection.right)
				{
					if(	laneInfoRight.attributes.type == opendrive::LaneType::Driving
						|| laneInfoRight.attributes.type == opendrive::LaneType::Shoulder)
					{
						laneId = laneInfoRight.attributes.id;
						width = GetLaneWidths(odr, &road, &laneInfoRight, 0.5);
						break;
					}
				}
				for(const opendrive::LaneInfo &laneInfoLeft : laneSection.left)
				{
					if(	laneInfoLeft.attributes.type == opendrive::LaneType::Driving
						|| laneInfoLeft.attributes.type == opendrive::LaneType::Shoulder)
					{
						laneId = laneInfoLeft.attributes.id;
						width = GetLaneWidths(odr, &road, &laneInfoLeft, 0.5);
						break;
					}
				}
			}

			if(laneId != 0)
			{
				for(int i = 0; i < road.traffic_signal_references.size(); i++)
				{
					StopLine sl;
					//get base point of Stop Line
					double s_pos = road.traffic_signal_references.at(i).start_position;
					double t_pos = road.traffic_signal_references.at(i).track_position;

					int sideId = 0;
					if(laneId > 0)
					{
						sideId = 1;
					}
					if(laneId < 0)
					{
						sideId = -1;
					}

					sl.id = 10 + road.traffic_signal_references.at(i).id * 10;
					sl.lightIds.push_back(sl.id = 10 + road.traffic_signal_references.at(i).id * 10);

					// flip the s_pos starting point since the geometry was defined in the other direction
					if(sideId * t_pos < 0)
						s_pos = road.attributes.length - s_pos;

					PlannerHNS::WayPoint pRef;
					// iterate trough all geometries defining the road to get the geometry at s_pos
					unsigned int geoIndex = 0;
					for(unsigned int i = 0; i < road.geometry_attributes.size(); i ++)
					{
						if(i < road.geometry_attributes.size()-1)
						{
							if(road.geometry_attributes.at(i)->start_position < s_pos && road.geometry_attributes.at(i + 1)->start_position > s_pos)
								geoIndex = i;
							
						}
						if(i == road.geometry_attributes.size()-1)
						{
							if(road.geometry_attributes.at(i)->start_position < s_pos)
								geoIndex = i;
						}
					}

					const std::unique_ptr<opendrive::GeometryAttributes> &geometry = road.geometry_attributes.at(geoIndex);

					// get the gps position of the stop Line reference
					try
					{
						switch (geometry->type)
						{
							case opendrive::GeometryType::ARC:
							{
								auto arc = static_cast<opendrive::GeometryAttributesArc *>(geometry.get());
								double sArc = s_pos - arc->start_position;

								if(sArc > arc->length)
									sArc = arc->length;
								if(sArc < 0)
									sArc = 0;
								pRef = GeneratePointFromArc( arc, width, sl.laneId, 0, sideId, arc->start_position, sArc);
								break;
							}
							break;
							case opendrive::GeometryType::LINE:
							{
								auto line = static_cast<opendrive::GeometryAttributesLine *>(geometry.get());
								double sLine = s_pos - line->start_position;
								if(sLine > line->length)
									sLine = line->length;
								if(sLine < 0)
									sLine = 0;
								pRef = GeneratePointFromLine( line, width, sl.laneId, 0, sideId, line->start_position, sLine );
								break;
							}
							break;
							case opendrive::GeometryType::SPIRAL:
							{
								ROS_FATAL(">> SPIRAL Geometries are not covered by GetCenterLaneData!");
								break;
							}
							break;
							case opendrive::GeometryType::POLY3:
							{
								ROS_FATAL(">> POLY3 Geometries are not covered by GetCenterLaneData!");
								break;
							}
							break;
							case opendrive::GeometryType::PARAMPOLY3:
							{
								ROS_FATAL(">> PARAMPOLY3 Geometries are not covered by GetCenterLaneData!");
								break;
							}
							break;
							default:
							break;
						}
					}
					catch (...)
					{
						ROS_FATAL(">> In GetStopLinesList: Geometries are not covered by GetCenterLaneData!");
					}
		
					// create Points
					sl.points.clear();
					int slLength = 2;

					for(int i = 0; i < slLength; i++)
					{
						PlannerHNS::WayPoint p;
						p.pos.x = pRef.pos.x + cos(pRef.pos.a + M_PI_2) * pRef.width * (double)(1 - i * slLength);
						p.pos.y = pRef.pos.y + sin(pRef.pos.a + M_PI_2) * pRef.width * (double)(1 - i * slLength);
						p.pos.z = pRef.pos.z;

						sl.points.push_back(p);
					}

					if(laneId !=0){
						sl.laneId = 10 + road.attributes.id * 10 + laneId;
					}else{
						ROS_FATAL(">> Did not get a lane Id continuing loop! ");
						continue;
					}
					slList.push_back(sl);
					
				}
			}
			
		}
	}
	return slList;

}

// Create Traffic Light for each signal with type == 1000001
std::vector<TrafficLight> OpenDriveMapLoader::GetTrafficLightsList(const opendrive::OpenDriveData* odr)
{
	std::vector<TrafficLight> tlList;

	// iterate trough roads to identify signals
	for(const opendrive::RoadInformation &road : odr->roads)
    {
		if(road.traffic_signals.size() > 0)
		{
			// check if there is a traffic signal with traffic light 
			for(int i = 0; i < road.traffic_signals.size(); i++)
			{
				if(road.traffic_signals.at(i).type == "1000001")
				{
					
					TrafficLight tl;
					//get base point of Stop Line
					double s_pos = road.traffic_signals.at(i).start_position;
					double t_pos = road.traffic_signals.at(i).track_position;

					tl.id = 10 + road.traffic_signals.at(i).id*10;

					// iterate trough roads to identify signal references
					for(const opendrive::RoadInformation &road : odr->roads)
					{
						if(road.traffic_signal_references.size() > 0)
						{
							for(int i = 0; i < road.traffic_signal_references.size(); i++)
							{	
								if(tl.id == (10 + road.traffic_signal_references.at(i).id * 10))
								{
									// get the current road's lane id
									int laneId = 0;
									const std::vector<opendrive::LaneWidth>* width;
									for(const opendrive::LaneSection &laneSection : road.lanes.lane_sections)
									{
										for(const opendrive::LaneInfo &laneInfoRight : laneSection.right)
										{
											if(	laneInfoRight.attributes.type == opendrive::LaneType::Driving
												|| laneInfoRight.attributes.type == opendrive::LaneType::Shoulder)
											{
												laneId = laneInfoRight.attributes.id;
												width = &laneInfoRight.lane_width;
												break;
											}
										}
										for(const opendrive::LaneInfo &laneInfoLeft : laneSection.left)
										{
											if(	laneInfoLeft.attributes.type == opendrive::LaneType::Driving
												|| laneInfoLeft.attributes.type == opendrive::LaneType::Shoulder)
											{
												laneId = laneInfoLeft.attributes.id;
												width = &laneInfoLeft.lane_width;
												break;
											}
										}
									}

									if(laneId != 0)
									{										
										tl.laneIds.push_back(10 + road.attributes.id * 10 + laneId);
										tl.stopLineID = tl.id;

										if(tl.laneIds.size() > 0 && tl.stopLineID != 0)
										{
											ROS_INFO("Found %d Lanes and Stop Line %d for traffic Light: %d", tl.laneIds.size(), tl.stopLineID, tl.id);
										}else{
											ROS_FATAL("No Lanes and Stop Line found for Traffic Light: %d", tl.id);
										}

									}
								}
							}
						}
					}

					tl.lightType = RED_LIGHT;
					
					tl.vertical_angle = 2 * M_PI;
					tl.horizontal_angle = 2 * M_PI;

					// get the current road's lane id
					int laneId = 0;
					const std::vector<opendrive::LaneWidth>* width;
					for(const opendrive::LaneSection &laneSection : road.lanes.lane_sections)
					{
						for(const opendrive::LaneInfo &laneInfoRight : laneSection.right)
						{
							if(	laneInfoRight.attributes.type == opendrive::LaneType::Driving
								|| laneInfoRight.attributes.type == opendrive::LaneType::Shoulder)
							{
								laneId = laneInfoRight.attributes.id;
								width = &laneInfoRight.lane_width;
								break;
							}
						}
						for(const opendrive::LaneInfo &laneInfoLeft : laneSection.left)
						{
							if(	laneInfoLeft.attributes.type == opendrive::LaneType::Driving
								|| laneInfoLeft.attributes.type == opendrive::LaneType::Shoulder)
							{
								laneId = laneInfoLeft.attributes.id;
								width = &laneInfoLeft.lane_width;
								break;
							}
						}
					}
					ROS_INFO("Lane ID: %d", laneId);
					if(laneId != 0)
					{
						ROS_INFO("Creating traffic light: %d", tl.id);
						
						int sideId = 0;
						if(laneId > 0)
						{
							sideId = 1;
						}
						if(laneId < 0)
						{
							sideId = -1;
						}

						// flip the s_pos starting point since the geometry was defined in the other direction
						// if(sideId > 0)
						// 	s_pos = road.attributes.length - s_pos;

						PlannerHNS::WayPoint pRef;
						// iterate trough all geometries defining the road to get the geometry at s_pos
						unsigned int geoIndex = 0;
						for(unsigned int i = 0; i < road.geometry_attributes.size(); i ++)
						{
							if(i < road.geometry_attributes.size()-1)
							{
								if(road.geometry_attributes.at(i)->start_position < s_pos && road.geometry_attributes.at(i + 1)->start_position > s_pos)
									geoIndex = i;
								
							}
							if(i == road.geometry_attributes.size()-1)
							{
								if(road.geometry_attributes.at(i)->start_position < s_pos)
									geoIndex = i;
							}
						}

						const std::unique_ptr<opendrive::GeometryAttributes> &geometry = road.geometry_attributes.at(geoIndex);

						// get the gps position of the traffic Light reference
						try
						{
							switch (geometry->type)
							{
								case opendrive::GeometryType::ARC:
								{
									auto arc = static_cast<opendrive::GeometryAttributesArc *>(geometry.get());
									double sArc = s_pos - arc->start_position;
									if(sArc > arc->length)
										sArc = arc->length;
									pRef = GeneratePointFromArc( arc, *width, laneId, 0, sideId, arc->start_position, sArc);
									break;
								}
								break;
								case opendrive::GeometryType::LINE:
								{
									auto line = static_cast<opendrive::GeometryAttributesLine *>(geometry.get());
									double sLine = s_pos - line->start_position;
									if(sLine > line->length)
										sLine = line->length;
									pRef = GeneratePointFromLine( line, *width, laneId, 0, sideId, line->start_position, sLine );
									break;
								}
								break;
								case opendrive::GeometryType::SPIRAL:
								{
									ROS_FATAL(">> SPIRAL Geometries are not covered by GetCenterLaneData!");
									break;
								}
								break;
								case opendrive::GeometryType::POLY3:
								{
									ROS_FATAL(">> POLY3 Geometries are not covered by GetCenterLaneData!");
									break;
								}
								break;
								case opendrive::GeometryType::PARAMPOLY3:
								{
									ROS_FATAL(">> PARAMPOLY3 Geometries are not covered by GetCenterLaneData!");
									break;
								}
								break;
								default:
								break;
							}
						}
						catch (...)
						{
							ROS_FATAL(">> In GetTrafficLightsList: Geometries are not covered by GetCenterLaneData!");
							continue;
						}

						PlannerHNS::WayPoint p;
						p.pos.x = pRef.pos.x + cos(pRef.pos.a - M_PI_2) * (t_pos); //* sideId);
						p.pos.y = pRef.pos.y + sin(pRef.pos.a - M_PI_2) * (t_pos); //* sideId);
						p.pos.z = road.traffic_signals.at(i).zoffset;

						tl.pose = p;

						tlList.push_back(tl);
					}
				}
			}
		}
	}
	return tlList;
}

std::vector<Curb> OpenDriveMapLoader::GetCurbsList(const opendrive::OpenDriveData* odr){
	// std::cout << ">>> Check out those Cuuuuurbs" << std::endl;

	std::vector<Curb> cList;
	// first iteration connect everyting based on ids
	for(const opendrive::RoadInformation &road : odr->roads)
    {
		for(const opendrive::LaneSection &laneSection : road.lanes.lane_sections)
    	{
			for(const opendrive::LaneInfo &laneInfoRight : laneSection.right)
			{
				if(	laneInfoRight.attributes.type == opendrive::LaneType::Driving
					|| laneInfoRight.attributes.type == opendrive::LaneType::Shoulder)
				{
					int marker_size = laneInfoRight.road_marker.size();
					if(marker_size > 0) {
						for ( int i = 0; i < marker_size; i++ ) {
							if(laneInfoRight.road_marker.at(i).type == "curb") {
								Curb c;
								c.id 		= 10 + road.attributes.id * 10 + laneInfoRight.attributes.id;
								c.laneId 	= 10 + road.attributes.id * 10 + laneInfoRight.attributes.id;
								c.roadId 	= 0;
								c.width		= laneInfoRight.road_marker.at(i).width;
								c.height	= 0.15;
								c.points 	= GetOuterLaneData(odr, &road, &laneInfoRight);
								
								cList.push_back(c);
							}
						}
					}
				}
			}
			for(const opendrive::LaneInfo &laneInfoLeft : laneSection.left)
			{
				if(	laneInfoLeft.attributes.type == opendrive::LaneType::Driving
					|| laneInfoLeft.attributes.type == opendrive::LaneType::Shoulder)
				{
					int marker_size = laneInfoLeft.road_marker.size();
					if(marker_size > 0) {
						for ( int i = 0; i < marker_size; i++ ) {
							if(laneInfoLeft.road_marker.at(i).type == "curb") {
								Curb c;
								c.id 		= 10 + road.attributes.id * 10 + laneInfoLeft.attributes.id;
								c.laneId 	= 10 + road.attributes.id * 10 + laneInfoLeft.attributes.id;
								c.roadId 	= 0;
								c.width		= laneInfoLeft.road_marker.at(i).width;
								c.height	= 0.15;
								c.points 	= GetOuterLaneData(odr, &road, &laneInfoLeft);
								
								cList.push_back(c);
							}
						}
					}
					
				}
			}
		}
	}
	return cList;

}


std::vector<opendrive::LaneWidth> OpenDriveMapLoader::GetLaneWidths(const opendrive::OpenDriveData* odr, const opendrive::RoadInformation* road, const opendrive::LaneInfo* laneInfo, double factor)
{
	std::vector<opendrive::LaneWidth> lW;
	for(const opendrive::RoadInformation &r : odr->roads)
    {
		if( r.attributes.id == road->attributes.id)
		{
			for(const opendrive::LaneSection &laneSection : r.lanes.lane_sections)
			{
				if(laneInfo->attributes.id > 0)
				{
					// set size of lW
					for(const opendrive::LaneInfo &laneInfoLeft : laneSection.left)
					{
						// check if the lane id is still smaller then the requested lane
						if(laneInfoLeft.attributes.id == laneInfo->attributes.id) {
							lW.resize(laneInfoLeft.lane_width.size());
						}
					}
					for(const opendrive::LaneInfo &laneInfoLeft : laneSection.left)
					{
						// check if the lane id is still smaller then the requested lane
						if(laneInfoLeft.attributes.id <= laneInfo->attributes.id && lW.size() > 0) {
							for(int i = 0; i < laneInfoLeft.lane_width.size(); i++)
							{
								double multi = 1;
								if(laneInfoLeft.attributes.id == laneInfo->attributes.id)
									multi = factor;
								if(i < lW.size()) {
									lW.at(i).soffset = laneInfoLeft.lane_width.at(i).soffset;
									lW.at(i).a += laneInfoLeft.lane_width.at(i).a * multi;
									lW.at(i).b += laneInfoLeft.lane_width.at(i).b * multi;
									lW.at(i).c += laneInfoLeft.lane_width.at(i).c * multi;
									lW.at(i).d += laneInfoLeft.lane_width.at(i).d * multi;
								}
								
							}
						}
					}
				}

				if(laneInfo->attributes.id < 0)
				{
					// set size of lW
					for(const opendrive::LaneInfo &laneInfoRight : laneSection.right)
					{
						// check if the lane id is still smaller then the requested lane
						if(laneInfoRight.attributes.id == laneInfo->attributes.id) {
							lW.resize(laneInfoRight.lane_width.size());
						}
					}
					for(const opendrive::LaneInfo &laneInfoRight : laneSection.right)
					{
						// check if the lane id is still bigger then the requested lane
						if(laneInfoRight.attributes.id >= laneInfo->attributes.id && lW.size() > 0) {
							for(int i = 0; i < laneInfoRight.lane_width.size(); i++)
							{
								double multi = 1;
								if(laneInfoRight.attributes.id == laneInfo->attributes.id)
									multi = factor;
								if(i < lW.size()) {
									lW.at(i).soffset = laneInfoRight.lane_width.at(i).soffset;
									lW.at(i).a += laneInfoRight.lane_width.at(i).a * multi;
									lW.at(i).b += laneInfoRight.lane_width.at(i).b * multi;
									lW.at(i).c += laneInfoRight.lane_width.at(i).c * multi;
									lW.at(i).d += laneInfoRight.lane_width.at(i).d * multi;
								}
							}
						}
					}
					
				}
			}
			
			for(int i = 0; i < lW.size(); i++)
			{
				// in case there is more then one lane offset entry
				if(r.lanes.lane_offset.size() > 1) {
					for(int j = 0; j < r.lanes.lane_offset.size(); j++)
					{
						if(j < r.lanes.lane_offset.size()-1) {
							if(r.lanes.lane_offset.at(j).s <= lW.at(i).soffset && lW.at(i).soffset < r.lanes.lane_offset.at(j+1).s) {
								lW.at(i).a += r.lanes.lane_offset.at(j).a;
								lW.at(i).b += r.lanes.lane_offset.at(j).b;
								lW.at(i).c += r.lanes.lane_offset.at(j).c;
								lW.at(i).d += r.lanes.lane_offset.at(j).d;
							}
						}
						else {
							// evaluating the last element
							if(r.lanes.lane_offset.at(j).s <= lW.at(i).soffset) {
								lW.at(i).a += r.lanes.lane_offset.at(j).a;
								lW.at(i).b += r.lanes.lane_offset.at(j).b;
								lW.at(i).c += r.lanes.lane_offset.at(j).c;
								lW.at(i).d += r.lanes.lane_offset.at(j).d;
							}
						}
					}
				} else if(r.lanes.lane_offset.size() == 1) {
					if(r.lanes.lane_offset.at(0).s <= lW.at(i).soffset) {
						lW.at(i).a += r.lanes.lane_offset.at(0).a;
						lW.at(i).b += r.lanes.lane_offset.at(0).b;
						lW.at(i).c += r.lanes.lane_offset.at(0).c;
						lW.at(i).d += r.lanes.lane_offset.at(0).d;
					}
				} else {
					ROS_INFO(">> GetCenterLaneData: No Lane offset entry exists");
				}
			}
		}
	}

	return lW;
}

std::vector<Boundary> OpenDriveMapLoader::GetBoundariesList(const opendrive::OpenDriveData* odr)
{
	std::vector<Boundary> bList;

	int boundaryIdCounter = 0;
	// loop trough all junctions in odr map
	for(const opendrive::Junction &junction : odr->junctions)
	{
		/****
			Create Boundaries around all junctions		
			Therefore loop through all roads inside the junction.
			Then construct a polygon around the junction.
		****/
		Boundary b;
		b.id = boundaryIdCounter;
		boundaryIdCounter ++;
		b.roadId = 0;
		b.type = PlannerHNS::INTERSECTION_BOUNDARY;
		std::vector<WayPoint> points;
		for(const opendrive::JunctionConnection &connection: junction.connections)
		{
			for(const opendrive::RoadInformation &road : odr->roads)
    		{
				if(connection.attributes.connecting_road == road.attributes.id)
				{
					for(const opendrive::LaneSection &laneSection : road.lanes.lane_sections)
					{
						for(const opendrive::LaneInfo &laneInfoRight : laneSection.right)
						{
							points = GetCenterLaneData(odr, &road, &laneInfoRight);
							for(const WayPoint &p : points) 
							{
								b.points.push_back(p);
							}
						}
						for(const opendrive::LaneInfo &laneInfoLeft : laneSection.left)
						{
							points = GetCenterLaneData(odr, &road, &laneInfoLeft);
							for(const WayPoint &p : points)
							{
								b.points.push_back(p);
							}
						}
					}
				}
			}
		}

		// Gift Wrapping Algorithm for Junction Convex Hull calculation
		std::vector<WayPoint> hullPoints;
		int firstPointOnHullIndex = FindLeftmostPointInJunctionPointsIndex(b.points);
		int pointOnHullIndex = firstPointOnHullIndex;
		WayPoint pointOnHull = b.points.at(pointOnHullIndex);
		int endpointIndex = 0;
		WayPoint endpoint;
		int i = 0;
		do
		{
			hullPoints.push_back(pointOnHull);
			endpoint = b.points.at(0);
			endpointIndex = 0;
			for(int j = 0; j < b.points.size(); j ++)
			{
				if(endpointIndex == pointOnHullIndex || isPointLeftOfLine(hullPoints.at(i), endpoint, b.points.at(j)))
				{
					endpoint = b.points.at(j);
					endpointIndex = j;
				}
			}
			i ++;
			pointOnHull = endpoint;
			pointOnHullIndex = endpointIndex;
		} while (endpointIndex != firstPointOnHullIndex);

		b.points.clear();
		// for clean visualization and closing boundaries, the first point is added as last point for a second time
		hullPoints.push_back(hullPoints.at(0));
		b.points = hullPoints;
		bList.push_back(b);
	}

	return bList;
}

int OpenDriveMapLoader::FindLeftmostPointInJunctionPointsIndex(std::vector<PlannerHNS::WayPoint>& points)
{
	int leftmostPointIndex = 0;
	double leftmostPosY = points[leftmostPointIndex].pos.y;
	for (int i = 0; i < points.size(); i ++)
	{
		if(points[i].pos.y < points[leftmostPointIndex].pos.y)
		{
			leftmostPointIndex = i;
			leftmostPosY = points[i].pos.y;
		}
	}
	return leftmostPointIndex;
}

bool OpenDriveMapLoader::isPointLeftOfLine(PlannerHNS::WayPoint A, PlannerHNS::WayPoint B, PlannerHNS::WayPoint C)
{
	// line is defined by A and B
	// C is the point to check
	bool isLeft = false;

	double determinantABAM = (B.pos.x - A.pos.x) * (C.pos.y - A.pos.y) - (B.pos.y - A.pos.y) * (C.pos.x - A.pos.x);
	if(determinantABAM > 0)
		isLeft = true;
	return isLeft;
}

} /* namespace PlannerHNS */