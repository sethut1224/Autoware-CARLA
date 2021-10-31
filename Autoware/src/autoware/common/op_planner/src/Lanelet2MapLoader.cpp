/*
 * Lanelet2MapLoader.cpp
 *
 *  Created on: Mar 17, 2020
 *      Author: hatem
 */

#include <op_planner/Lanelet2MapLoader.h>
#include <op_planner/PlanningHelpers.h>

namespace PlannerHNS {

Lanelet2MapLoader::Lanelet2MapLoader()  {

}

Lanelet2MapLoader::~Lanelet2MapLoader() {
}

void Lanelet2MapLoader::LoadMap(const autoware_lanelet2_msgs::MapBin& msg, PlannerHNS::RoadNetwork& map)
{
	map.Clear();
	lanelet::LaneletMapPtr l2_map = std::make_shared<lanelet::LaneletMap>();

	try
	{
		lanelet::utils::conversion::fromBinMsg(msg, l2_map);
		FromLaneletToRoadNetwork(l2_map, map, nullptr);
	}
	catch(std::exception& e)
	{
		std::cout << "Failed to parse lanelet2 message, Error: " << e.what() << std::endl;
		return;
	}
}

lanelet::LaneletMapPtr Lanelet2MapLoader::LoadMap(const std::string& fileName, PlannerHNS::RoadNetwork& map)
{
	PlannerHNS::MappingHelpers::LoadProjectionData(fileName, map);
	lanelet::LaneletMapPtr l2_map = nullptr;
	lanelet::ErrorMessages errors;
	lanelet::Projector* p_proj = nullptr;

	if(map.proj == PlannerHNS::MGRS_PROJ)
	{
		std::cout << " >> Loading Map using Autoware OSM parser and MGRS projector." << std::endl;
		std::cout << "Using projection string: " << map.str_proj << std::endl;
		std::cout << "Using origin : " << map.origin.pos.ToString() << std::endl;
		p_proj = new lanelet::projection::MGRSProjector();

	}
	else
	{
		std::cout << " >> Loading Map using Autoware OSM parser and UTM projector." << std::endl;
		std::cout << "Using projection string: " << map.str_proj << std::endl;
		std::cout << "Using origin : " << map.origin.pos.ToString() << std::endl;


		if(fabs(map.origin.pos.lat) < 0.00001 && fabs(map.origin.pos.lon) < 0.00001)
		{
			if(map.str_proj.size() > 0)
			{
				PlannerHNS::MappingHelpers::xyzTolla_proj(map.str_proj, PlannerHNS::WayPoint(), 0.0, 0.0, 0.0,
							map.origin.pos.lat, map.origin.pos.lon, map.origin.pos.alt);
			}
			else
			{
				ExtractFirstLongLatFromFileAsOrigin(fileName, map);
			}
		}

		p_proj = new lanelet::projection::UtmProjector(lanelet::Origin({map.origin.pos.lat, map.origin.pos.lon, map.origin.pos.alt}));
	}

	try
	{
		l2_map = lanelet::load(fileName, "autoware_osm_handler", *p_proj, &errors);
		lanelet::utils::overwriteLaneletsCenterline(l2_map, false);
		FromLaneletToRoadNetwork(l2_map, map, p_proj);
	}
	catch(std::exception& e)
	{
		std::cout << "Failed to Load map file: " << fileName << ", Error: " << e.what() << std::endl;
		return nullptr;
	}

	if(errors.size() > 0)
	{
		for(const auto &error: errors)
		{
			std::cout << "Lanelet Error: " << error <<std::endl;
		}
		std::cout << "Failed to Load map file: " << fileName << std::endl;
		return nullptr;
	}

	if(p_proj != nullptr)
		delete p_proj;

	return l2_map;
}

void Lanelet2MapLoader::CreateLane(lanelet::routing::RoutingGraphUPtr& routingGraph, lanelet::traffic_rules::TrafficRulesPtr& traffic, lanelet::ConstLanelet& lanelet_obj, PlannerHNS::Lane& l, PlannerHNS::RoadNetwork& map, lanelet::Projector* proj)
{
	l.id = lanelet_obj.id();
	lanelet::ConstLanelets next_lanes = routingGraph->following(lanelet_obj);
	for(auto& center_next : next_lanes)
	{
	  l.toIds.push_back(center_next.id());
	}

	lanelet::ConstLineString3d center_l = lanelet_obj.centerline();

//	for(auto& attr: lanelet_obj.attributes())
//	{
//	  std::cout << "lanelet Line attr: " << attr.first << ", " << attr.second << std::endl;
//	}

	CreateWayPointsFromLineString(map, l.points, center_l, proj, l.id);

	if(lanelet_obj.hasAttribute("turn_direction"))
	{
	  DIRECTION_TYPE dir_type = FORWARD_DIR;
	  if(lanelet_obj.attributes().at("turn_direction").value().compare("left") == 0)
	  {
		  dir_type = FORWARD_LEFT_DIR;
	  }
	  else if(lanelet_obj.attributes().at("turn_direction").value().compare("right") == 0)
	  {
		  dir_type = FORWARD_RIGHT_DIR;
	  }

	  for(auto& p: l.points)
	  {
		  p.bDir = dir_type;
	  }
	}

	l.num = routingGraph->lefts(lanelet_obj).size() + 1;
	l.speed = traffic->speedLimit(lanelet_obj).speedLimit.value();

	double width_sum = 0;
	if(l.points.size() == center_l.size() && l.points.size() > 0)
	{
		for(unsigned int i=0; i < l.points.size(); i++)
		{
			l.points.at(i).width = lanelet::geometry::distance2d(center_l[i], lanelet_obj.leftBound()) + lanelet::geometry::distance2d(center_l[i], lanelet_obj.rightBound());
			width_sum += l.points.at(i).width;
			l.points.at(i).v = l.speed;
		}

		l.width = width_sum / (double)l.points.size();
	}

	PlannerHNS::PlanningHelpers::CalcAngleAndCost(l.points);
}

void Lanelet2MapLoader::FromLaneletToRoadNetwork(lanelet::LaneletMapPtr l2_map, PlannerHNS::RoadNetwork& map, lanelet::Projector* proj)
{
	lanelet::LaneletLayer& l_layer = l2_map->laneletLayer;
	  std::vector<lanelet::ConstLanelet> lets;
	  lets.insert(lets.begin(), l_layer.begin(), l_layer.end());

	  lanelet::traffic_rules::TrafficRulesPtr trafficRules =
			  lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Vehicle);

	  lanelet::routing::RoutingGraphUPtr routingGraph = lanelet::routing::RoutingGraph::build(*l2_map, *trafficRules);

	  PlannerHNS::RoadSegment segment;
	  for(auto& x : l2_map->laneletLayer)
	  {
		  //std::cout << "Lanelet Type: " << x.attributes()["type"] << ", " <<  x.attributes()["subtype"] << ", " << x.id() <<  std::endl << std::endl;
		  if(x.attributes()["subtype"].value().compare("road") == 0 || x.attributes()["subtype"].value().compare("junction_road") == 0)
		  {

			  PlannerHNS::Lane l;
			  CreateLane(routingGraph, trafficRules, x, l, map, proj);

			  std::vector<lanelet::ConstLanelet> x_lets;
			  x_lets.push_back(x);

			    // find stop lines referenced by traffic signs
			    std::vector<std::shared_ptr<const lanelet::TrafficSign> > traffic_signs = x.regulatoryElementsAs<const lanelet::TrafficSign>();
			    if (traffic_signs.size() > 0)
			    {
			      for (const auto& ts : traffic_signs)
			      {
			    	  int sign_group_id = ts->id();

			    	lanelet::ConstLineStringsOrPolygons3d pos_data = ts->trafficSigns();
			        if (ts->type() != "stop_sign")
			        {
//			        	for(unsigned int i_shape = 0; i_shape < pos_data.size(); i_shape++)
//			        	{
//			        		lanelet::ConstLineStringOrPolygon3d l_or_p =  pos_data.at(i_shape);
//
//			        		PlannerHNS::TrafficSign op_ts;
//							op_ts.id = l_or_p.id();
//							op_ts.laneIds.push_back(l.id);
//							op_ts.signType = PlannerHNS::UNKNOWN_SIGN;
//
//			        		if(l_or_p.isLineString())
//			        		{
//			        			lanelet::Optional<lanelet::ConstLineString3d> line_str = l_or_p.lineString();
//			        			if(!!line_str)
//			        				CreateWayPointsFromLineString(op_ts.points, line_str.get(), proj);
//			        		}
//			        		else if(l_or_p.polygon())
//			        		{
//			        			lanelet::Optional<lanelet::ConstPolygon3d> poly_str = l_or_p.polygon();
//								if(!!poly_str)
//									CreateWayPointsFromPolygon(op_ts.points, poly_str.get(), proj);
//			        		}
//
//			        		PlannerHNS::MappingHelpers::InsertUniqueTrafficSign(map.signs, op_ts);
//			        	}
			        }
			        else
			        {

			        	for(unsigned int i_shape = 0; i_shape < pos_data.size(); i_shape++)
						{
							lanelet::ConstLineStringOrPolygon3d l_or_p =  pos_data.at(i_shape);

							PlannerHNS::TrafficSign op_ts;
							op_ts.id = l_or_p.id();
							op_ts.groupID = sign_group_id;
							op_ts.laneIds.push_back(l.id);
							op_ts.signType = PlannerHNS::STOP_SIGN;
							  if(l.points.size() > 0)
							  {
								  op_ts.horizontal_angle = (l.points.at(0).pos.a*RAD2DEG) - 90;
							  }

							if(l_or_p.isLineString())
							{
								lanelet::Optional<lanelet::ConstLineString3d> line_str = l_or_p.lineString();
								if(!!line_str)
								{
									CreateWayPointsFromLineString(map, op_ts.points, line_str.get(), proj);
								}
							}
							else if(l_or_p.polygon())
							{
								lanelet::Optional<lanelet::ConstPolygon3d> poly_str = l_or_p.polygon();
								if(!!poly_str)
								{
									CreateWayPointsFromPolygon(map, op_ts.points, poly_str.get(), proj);
								}
							}

							PlannerHNS::WayPoint sum_p;
							for(unsigned k=0; k < op_ts.points.size(); k++)
							{
								sum_p.pos.x += op_ts.points.at(k).pos.x;
								sum_p.pos.y += op_ts.points.at(k).pos.y;
								sum_p.pos.z += op_ts.points.at(k).pos.z;
								sum_p.pos.lat += op_ts.points.at(k).pos.lat;
								sum_p.pos.lon += op_ts.points.at(k).pos.lon;
								sum_p.pos.alt += op_ts.points.at(k).pos.alt;
							}

							if(op_ts.points.size() > 0)
							{
								op_ts.pose.pos.x = sum_p.pos.x/op_ts.points.size();
								op_ts.pose.pos.y = sum_p.pos.y/op_ts.points.size();
								op_ts.pose.pos.z = sum_p.pos.z/op_ts.points.size();
								op_ts.pose.pos.lat = sum_p.pos.lat/op_ts.points.size();
								op_ts.pose.pos.lon = sum_p.pos.lon/op_ts.points.size();
								op_ts.pose.pos.alt = sum_p.pos.alt/op_ts.points.size();
							}

							PlannerHNS::MappingHelpers::InsertUniqueTrafficSign(map.signs, op_ts);
						}

						lanelet::ConstLineStrings3d traffic_sign_sl = ts->refLines();
						for(unsigned int i_sl=0; i_sl < traffic_sign_sl.size(); i_sl++)
						{
							lanelet::ConstLineString3d stop_line_points = traffic_sign_sl.at(i_sl);
							PlannerHNS::StopLine sl;
							sl.id = stop_line_points.id();
							sl.stopSignID = ts->id();
							CreateWayPointsFromLineString(map, sl.points, stop_line_points, proj);
							l.stopLines.push_back(sl);
							//PlannerHNS::MappingHelpers::InsertUniqueStopLine(map.stopLines, sl);
						}
			        }
			      }
			    }

			  std::vector<lanelet::ConstLineString3d> stop_lines = lanelet::utils::query::getTrafficLightStopLines(x_lets);
			  for(unsigned int i=0; i < stop_lines.size(); i++)
			  {
				  lanelet::ConstLineString3d stop_line_points = stop_lines.at(i);
				  PlannerHNS::StopLine sl;
				  sl.id = stop_line_points.id();
				  CreateWayPointsFromLineString(map, sl.points, stop_line_points, proj);
				  l.stopLines.push_back(sl);
				 //PlannerHNS::MappingHelpers::InsertUniqueStopLine(map.stopLines, sl);
			  }

			  std::vector<lanelet::AutowareTrafficLightConstPtr> lanelet_lights = lanelet::utils::query::autowareTrafficLights(x_lets);
			  for(unsigned int i=0; i < lanelet_lights.size(); i++)
			  {
				  std::vector<PlannerHNS::TrafficLight> tls = CreateTrafficLightsFromLanelet2(map, lanelet_lights.at(i), proj, l.id);
				  if(l.points.size() > 0)
				  {
					  for(auto& tl: tls)
					  {
						  tl.horizontal_angle = (l.points.at(0).pos.a*RAD2DEG) - 90;
					  }
				  }
				  l.trafficlights.insert(l.trafficlights.begin(), tls.begin(), tls.end());
				  for(unsigned int j=0; j < tls.size(); j++)
				  {
					  PlannerHNS::MappingHelpers::InsertUniqueTrafficLight(map.trafficLights, tls.at(j));
				  }
			  }

			  segment.Lanes.push_back(l);
		  }
	  }

	  for(auto& x : l2_map->areaLayer)
	  {
//		  std::cout << "Area Layer: " << x.attributes()["type"] << ", " <<  x.attributes()["subtype"] << ", " << x.id()
//				  << ", OuterBound: "<< x.outerBound().size() << ", InnerBound: "<< x.innerBounds().size() <<  std::endl;

		  PlannerHNS::Boundary area;
		  area.id = x.id();
		  //area.type = GetBoundaryTypeFromString(x.attributes().at("subtype").value());
		  area.type = PlannerHNS::BOUNDARY_TYPE_STR.GetEnum(x.attributes().at("subtype").value());
		  for(auto& bound : x.outerBound())
		  {
//			  std::cout << "Bound Side: " << bound.id() << ", " << bound.size() << std::endl;
			  CreateWayPointsFromLineString(map, area.points, bound, proj);
		  }
		  map.boundaries.push_back(area);
	  }

	  for(auto& x : l2_map->lineStringLayer)
	  {
//		  std::cout << "String Layer: " << x.attributes()["area"] << ", " << x.attributes()["type"] << ", " <<  x.attributes()["subtype"] << ", " << x.id()  <<  std::endl;

		  if(x.attributes()["subtype"].value().compare("parking") == 0)
		  {
			  PlannerHNS::Boundary area;
			  area.id = x.id();
			  area.type = PlannerHNS::PARKING_BOUNDARY;
			  CreateWayPointsFromLineString(map, area.points, x, proj);
			  map.boundaries.push_back(area);
		  }
	  }

	  map.roadSegments.clear();
	  map.roadSegments.push_back(segment);

	  PlannerHNS::MappingHelpers::ConnectMissingStopLinesAndLanes(map);

	  PlannerHNS::MappingHelpers::ConnectLanes(map);
	  PlannerHNS::MappingHelpers::ConnectTrafficLightsAndStopLines(map);
	  PlannerHNS::MappingHelpers::ConnectTrafficSignsAndStopLines(map);
	//Link Lanes and lane's waypoints by pointers
	std::cout << " >> Link lanes and waypoints with pointers ... " << std::endl;
	MappingHelpers::LinkLanesPointers(map);
	//Link waypoints
	std::cout << " >> Link missing branches and waypoints... " << std::endl;
	MappingHelpers::LinkMissingBranchingWayPointsV2(map);

	std::cout << " >> Connect Wayarea (boundaries) to waypoints ... " << std::endl;
	MappingHelpers::ConnectBoundariesToWayPoints(map);
	MappingHelpers::LinkBoundariesToWayPoints(map);
}

void Lanelet2MapLoader::CreateWayPointsFromLineString(const PlannerHNS::RoadNetwork& map, std::vector<PlannerHNS::WayPoint>& points, lanelet::ConstLineString3d& line_string, lanelet::Projector* proj, int lane_id)
{
	for(auto& p : line_string )
	{
		//std::cout << "Size of Point: " << p.id() << ", " <<  "( " << p.x() << ", " << p.y() << ", " << p.z() << ")" << std::endl;
		PlannerHNS::WayPoint wp;
		PlannerHNS::RoadNetwork::g_max_point_id++;
		wp.id = PlannerHNS::RoadNetwork::g_max_point_id;
		wp.laneId = lane_id;
		wp.iOriginalIndex = points.size();

		wp.pos.x = p.x();
		wp.pos.y = p.y();
		wp.pos.z = p.z();

		if(proj != nullptr)
		{
			lanelet::GPSPoint gps_p = proj->reverse(p);
			wp.pos.lon = gps_p.lon;
			wp.pos.lat = gps_p.lat;
			wp.pos.alt = gps_p.ele;

			PlannerHNS::MappingHelpers::llaToxyz_proj(map.str_proj, PlannerHNS::WayPoint(), wp.pos.lat, wp.pos.lon, wp.pos.alt,
					wp.pos.x, wp.pos.y, wp.pos.z);
		}

		points.push_back(wp);

		if(points.size() > 1)
		{
			unsigned int p1 = points.size()-2;
			unsigned int p2 = points.size()-1;
			points.at(p1).toIds.push_back(points.at(p2).id);
			points.at(p2).fromIds.push_back(points.at(p1).id);
		}
	}
}

void Lanelet2MapLoader::CreateWayPointsFromLineString(const PlannerHNS::RoadNetwork& map, std::vector<PlannerHNS::WayPoint>& points,const lanelet::LineString3d& line_string, lanelet::Projector* proj, int lane_id)
{
	for(auto& p : line_string )
	{
	  PlannerHNS::WayPoint wp;
	  PlannerHNS::RoadNetwork::g_max_point_id++;
	  wp.id = PlannerHNS::RoadNetwork::g_max_point_id;
	  wp.laneId = lane_id;
	  wp.iOriginalIndex = points.size();

		wp.pos.x = p.x();
		wp.pos.y = p.y();
		wp.pos.z = p.z();

		if(proj != nullptr)
		{
			lanelet::GPSPoint gps_p = proj->reverse(p);
			wp.pos.lon = gps_p.lon;
			wp.pos.lat = gps_p.lat;
			wp.pos.alt = gps_p.ele;

			PlannerHNS::MappingHelpers::llaToxyz_proj(map.str_proj, PlannerHNS::WayPoint(), wp.pos.lat, wp.pos.lon, wp.pos.alt,
					wp.pos.x, wp.pos.y, wp.pos.z);
		}

		if(points.size() > 0)
		{
			double d = hypot(points.back().pos.y-wp.pos.y, points.back().pos.x-wp.pos.x);
			if(d < 0.001)
			{
				continue; //ignor this point
			}
		}

		PlannerHNS::RoadNetwork::g_max_point_id++;
		wp.id = PlannerHNS::RoadNetwork::g_max_point_id;
		points.push_back(wp);
	}
}

void Lanelet2MapLoader::CreateWayPointsFromPolygon(const PlannerHNS::RoadNetwork& map, std::vector<PlannerHNS::WayPoint>& points,const lanelet::ConstPolygon3d& line_string, lanelet::Projector* proj, int lane_id)
{
	for(auto& p : line_string )
	{
	  PlannerHNS::WayPoint wp;
	  PlannerHNS::RoadNetwork::g_max_point_id++;
	  wp.id = PlannerHNS::RoadNetwork::g_max_point_id;
	  wp.laneId = lane_id;
	  wp.iOriginalIndex = points.size();

		wp.pos.x = p.x();
		wp.pos.y = p.y();
		wp.pos.z = p.z();

		if(proj != nullptr)
		{
			lanelet::GPSPoint gps_p = proj->reverse(p);
			wp.pos.lon = gps_p.lon;
			wp.pos.lat = gps_p.lat;
			wp.pos.alt = gps_p.ele;

			PlannerHNS::MappingHelpers::llaToxyz_proj(map.str_proj, PlannerHNS::WayPoint(), wp.pos.lat, wp.pos.lon, wp.pos.alt,
					wp.pos.x, wp.pos.y, wp.pos.z);
		}

		points.push_back(wp);
	}
}

std::vector<PlannerHNS::StopLine> Lanelet2MapLoader::CreateStopLinesFromLanelet2(const PlannerHNS::RoadNetwork& map, lanelet::ConstLineString3d& sl_let, lanelet::Projector* proj, int lane_id )
{
	PlannerHNS::StopLine sl;
	std::vector<PlannerHNS::StopLine> stop_lines;

	sl.id = sl_let.id();
	CreateWayPointsFromLineString(map, sl.points, sl_let, proj);

	if(lane_id != 0)
		sl.laneId = lane_id;

	stop_lines.push_back(sl);

	return stop_lines;
}

std::vector<PlannerHNS::TrafficLight> Lanelet2MapLoader::CreateTrafficLightsFromLanelet2(const PlannerHNS::RoadNetwork& map, lanelet::AutowareTrafficLightConstPtr& tl_let, lanelet::Projector* proj, int lane_id)
{
	//std::cout << "### One Light >> " << std::endl;

	std::vector<PlannerHNS::TrafficLight> traffic_lights;
	int stop_line_id = 0;
	lanelet::Optional<lanelet::ConstLineString3d> stop_l =  tl_let->stopLine();
	if (!!stop_l)
	{
		lanelet::ConstLineString3d sl = stop_l.get();
//		if(sl.hasAttribute("type") && sl.hasAttribute("subtype"))
//		{
//			std::cout << "Standard Stop Line: " << sl.attributes().at("type") << ", " << sl.attributes().at("subtype") << ", " << sl.size() << std::endl;
//		}
//		else
//		{
//			std::cout << "Stop Line Without attributes: " << sl.size() << std::endl;
//		}
		stop_line_id = sl.id();
	}

	int bulb_size = 0;
	for (auto& ls : tl_let->lightBulbs())
	{
		lanelet::ConstLineString3d l = static_cast<lanelet::ConstLineString3d>(ls);
		bulb_size = l.size();
		//std::cout << "Bulb: " << l.size() << std::endl;
		int traffic_light_group_id = l.id();

		for (auto pt : l)
		{
			PlannerHNS::TrafficLight tl;
			tl.id = pt.id();
			tl.groupID = traffic_light_group_id;
			tl.stopLineID = stop_line_id;
			tl.pose.pos.x = pt.x();
			tl.pose.pos.y = pt.y();
			tl.pose.pos.z = pt.z();

			if(proj != nullptr)
			{
				lanelet::GPSPoint gps_p = proj->reverse(pt);
				tl.pose.pos.lon = gps_p.lon;
				tl.pose.pos.lat = gps_p.lat;
				tl.pose.pos.alt = gps_p.ele;

				PlannerHNS::MappingHelpers::llaToxyz_proj(map.str_proj, PlannerHNS::WayPoint(), tl.pose.pos.lat, tl.pose.pos.lon, tl.pose.pos.alt,
						tl.pose.pos.x, tl.pose.pos.y, tl.pose.pos.z);
			}

			tl.vertical_angle = 90;

			if (pt.hasAttribute("color"))
			{
				if(pt.attribute("color").value().compare("red")==0)
				{
					tl.lightType = PlannerHNS::RED_LIGHT;
				}
				else if(pt.attribute("color").value().compare("yellow")==0)
				{
					tl.lightType = PlannerHNS::YELLOW_LIGHT;
				}
				else if(pt.attribute("color").value().compare("green")==0)
				{
					tl.lightType = PlannerHNS::GREEN_LIGHT;
				}
			}

			if(lane_id != 0)
			{
				tl.laneIds.push_back(lane_id);
			}

			traffic_lights.push_back(tl);
		}
	}

	if(bulb_size == 0)
	{
		for (auto& lsp : tl_let->trafficLights())
		{
			if (lsp.isLineString())
			{
				lanelet::ConstLineString3d ls = static_cast<lanelet::ConstLineString3d>(lsp);
//				if(ls.hasAttribute("type") && ls.hasAttribute("subtype"))
//				{
//					std::cout << "Standard Light: " << ls.attributes().at("type") << ", " << ls.attributes().at("subtype") << ", " << ls.size() <<  std::endl;
//				}
//				else
//				{
//					std::cout << "Light Without Attribute: " << tl_let->attributes().at("type") << ", " << tl_let->attributes().at("subtype") << ", " << ls.size() <<  std::endl;
//				}

				if (ls.hasAttribute("height"))
				{
				//lanelet::Attribute attr = ls.attribute("height");
				//double traffic_light_height = std::stod(attr.value());
				}

				int traffic_light_group_id = ls.id();

				for (auto pt : ls)
				{
					PlannerHNS::TrafficLight tl;
					tl.id = pt.id();
					tl.groupID = traffic_light_group_id;
					tl.stopLineID = stop_line_id;
					tl.pose.pos.x = pt.x();
					tl.pose.pos.y = pt.y();
					tl.pose.pos.z = pt.z();

					if(proj != nullptr)
					{
						lanelet::GPSPoint gps_p = proj->reverse(pt);
						tl.pose.pos.lon = gps_p.lon;
						tl.pose.pos.lat = gps_p.lat;
						tl.pose.pos.alt = gps_p.ele;

						PlannerHNS::MappingHelpers::llaToxyz_proj(map.str_proj, PlannerHNS::WayPoint(), tl.pose.pos.lat, tl.pose.pos.lon, tl.pose.pos.alt,
								tl.pose.pos.x, tl.pose.pos.y, tl.pose.pos.z);
					}

					tl.vertical_angle = 90;

					if (pt.hasAttribute("color"))
					{
						if(pt.attribute("color").value().compare("red")==0)
						{
							tl.lightType = PlannerHNS::RED_LIGHT;
						}
						else if(pt.attribute("color").value().compare("yellow")==0)
						{
							tl.lightType = PlannerHNS::YELLOW_LIGHT;
						}
						else if(pt.attribute("color").value().compare("green")==0)
						{
							tl.lightType = PlannerHNS::GREEN_LIGHT;
						}
					}

					if(lane_id != 0)
					{
						tl.laneIds.push_back(lane_id);
					}

					traffic_lights.push_back(tl);
				}
			}
			else
			{
//				lanelet::ConstPolygon3d ls = static_cast<lanelet::ConstPolygon3d>(lsp);
//
//				if(ls.hasAttribute("type") && ls.hasAttribute("subtype"))
//				{
//				std::cout << "Polygon Standard Light: " << ls.attributes().at("type") << ", " << ls.attributes().at("subtype") << std::endl;
//				}
//				else
//				{
//				std::cout << "polygon Light Without Attribute: " << tl_let->attributes().at("type") << ", " << tl_let->attributes().at("subtype") << std::endl;
//				}
			}
		}
	}
	return traffic_lights;
}

void Lanelet2MapLoader::ExtractFirstLongLatFromFileAsOrigin(const std::string& fileName, PlannerHNS::RoadNetwork& map)
{
	std::ifstream f(fileName.c_str());
	if(!f.good())
	{
		return;
	}

	TiXmlDocument doc(fileName);
	try
	{
		doc.LoadFile();
	}
	catch(std::exception& e)
	{
		std::cout << "Can't read lat long information from lanelet2 .osm file: : "<<  fileName << std::endl;
		std::cout << e.what() << std::endl;
		return;
	}


	std::vector<TiXmlElement*> elements;
	if(UtilityHNS::XmlHelpers::findFirstElement("node", doc.FirstChildElement(), elements) > 0)
	{
		map.origin.pos.lat = strtod(elements.at(0)->Attribute("lat"), NULL);
		map.origin.pos.lon = strtod(elements.at(0)->Attribute("lon"), NULL);
		std::cout << "Lanelet2 file first found: Latitude: " << map.origin.pos.lat <<  ", Longitude: " << map.origin.pos.lon << std::endl;
	}
}

} /* namespace PlannerHNS */
