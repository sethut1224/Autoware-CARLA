/*
 * Copyright 2016-2020 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <way_planner_core.h>
#include <string>
#include <vector>

namespace WayPlannerNS
{

void way_planner_core::GetTransformFromTF(const std::string parent_frame, const std::string child_frame,
                                          tf::StampedTransform &transform)
{
  static tf::TransformListener listener;

  int nFailedCounter = 0;
  while (1)
  {
    try
    {
      listener.lookupTransform(parent_frame, child_frame, ros::Time(0), transform);
      break;
    }
    catch (tf::TransformException& ex)
    {
      if (nFailedCounter > 2)
      {
        ROS_ERROR("%s", ex.what());
      }
      ros::Duration(1.0).sleep();
      nFailedCounter++;
    }
  }
}

way_planner_core::way_planner_core()
{
  m_pCurrGoal = 0;
  m_iCurrentGoalIndex = 1;
  m_bKmlMap = false;
  nh.getParam("/way_planner/pathDensity",               m_params.pathDensity);
  nh.getParam("/way_planner/enableSmoothing",           m_params.bEnableSmoothing);
  nh.getParam("/way_planner/enableLaneChange",          m_params.bEnableLaneChange);
  nh.getParam("/way_planner/enableRvizInput",           m_params.bEnableRvizInput);
  nh.getParam("/way_planner/enableReplan",              m_params.bEnableReplanning);
  nh.getParam("/way_planner/enableHMI",                 m_params.bEnableHMI);
  nh.getParam("/way_planner/fallbackMinGoalDistanceTh", m_params.fallbackMinGoalDistanceTh);
  nh.getParam("/way_planner/planningMaxAttempt",        m_params.planningMaxAttempt);

  // The planning max attempt feature parameter cannot be below zero
  if (m_params.planningMaxAttempt < 0)
  {
    m_params.planningMaxAttempt = 0;
  }

  // The fallback min goal distance threshold feature parameter cannot be below zero
  if (m_params.fallbackMinGoalDistanceTh < 0)
  {
    m_params.fallbackMinGoalDistanceTh = 0;
  }

  int iSource = 0;
  nh.getParam("/way_planner/mapSource", iSource);
  if (iSource == 0)
  {
    m_params.mapSource = MAP_AUTOWARE;
  }
  else if (iSource == 1)
  {
    m_params.mapSource = MAP_FOLDER;
  }
  else if (iSource == 2)
  {
    m_params.mapSource = MAP_KML_FILE;
  }

  nh.getParam("/way_planner/mapFileName", m_params.KmlMapPath);


  tf::StampedTransform transform;
  GetTransformFromTF("map", "world", transform);
  m_OriginPos.position.x  = transform.getOrigin().x();
  m_OriginPos.position.y  = transform.getOrigin().y();
  m_OriginPos.position.z  = transform.getOrigin().z();

  pub_Paths = nh.advertise<autoware_msgs::LaneArray>("lane_waypoints_array", 1, true);
  pub_PathsRviz = nh.advertise<visualization_msgs::MarkerArray>("global_waypoints_rviz", 1, true);
  pub_StartPointRviz = nh.advertise<visualization_msgs::Marker>("Global_StartPoint_rviz", 1, true);
  pub_GoalPointRviz = nh.advertise<visualization_msgs::MarkerArray>("Global_GoalPoints_rviz", 1, true);
  pub_NodesListRviz = nh.advertise<visualization_msgs::MarkerArray>("Goal_Nodes_Points_rviz", 1, true);
  pub_MapRviz = nh.advertise<visualization_msgs::MarkerArray>("vector_map_center_lines_rviz", 100, true);
  pub_TrafficInfoRviz = nh.advertise<visualization_msgs::MarkerArray>("Traffic_Lights_rviz", 1, true);

#ifdef ENABLE_VISUALIZE_PLAN
  m_CurrMaxCost = 1;
  m_iCurrLevel = 0;
  m_nLevelSize = 1;
  m_bSwitch = 0;
  pub_GlobalPlanAnimationRviz = nh.advertise<visualization_msgs::MarkerArray>("AnimateGlobalPlan", 1, true);
#endif

  if (m_params.bEnableHMI)
  {
    m_AvgResponseTime = 0;
    m_SocketServer.InitSocket(10001, 10002);
  }

  /** @todo To achieve perfection, you need to start sometime */

  sub_start_pose = nh.subscribe("/initialpose", 1, &way_planner_core::callbackGetStartPose, this);
  sub_goal_pose = nh.subscribe("move_base_simple/goal", 1, &way_planner_core::callbackGetGoalPose, this);

  sub_current_pose = nh.subscribe("/current_pose", 100, &way_planner_core::callbackGetCurrentPose, this);

  int bVelSource = 1;
  nh.getParam("/dp_planner/enableOdometryStatus", bVelSource);
  if (bVelSource == 0)
  {
    sub_robot_odom = nh.subscribe("/odom", 100, &way_planner_core::callbackGetRobotOdom, this);
  }
  else if (bVelSource == 1)
  {
    sub_current_velocity = nh.subscribe("/current_velocity", 100, &way_planner_core::callbackGetVehicleStatus, this);
  }
  else if (bVelSource == 2)
  {
    sub_can_info = nh.subscribe("/can_info", 100, &way_planner_core::callbackGetCANInfo, this);
  }

  sub_nodes_list = nh.subscribe("/GlobalNodesList", 1, &way_planner_core::callbackGetNodesList, this);

  if (m_params.mapSource == MAP_AUTOWARE)
  {
    sub_map_points = nh.subscribe("/vector_map_info/point",     1, &way_planner_core::callbackGetVMPoints,      this);
    sub_map_lanes  = nh.subscribe("/vector_map_info/lane",      1, &way_planner_core::callbackGetVMLanes,       this);
    sub_map_nodes  = nh.subscribe("/vector_map_info/node",      1, &way_planner_core::callbackGetVMNodes,       this);
    sup_stop_lines = nh.subscribe("/vector_map_info/stop_line", 1, &way_planner_core::callbackGetVMStopLines,   this);
    sub_dtlanes    = nh.subscribe("/vector_map_info/dtlane",    1, &way_planner_core::callbackGetVMCenterLines, this);
  }
}

way_planner_core::~way_planner_core()
{
}

void way_planner_core::callbackGetGoalPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
  PlannerHNS::WayPoint wp;
  wp = PlannerHNS::WayPoint(msg->pose.position.x+m_OriginPos.position.x, msg->pose.position.y+m_OriginPos.position.y,
      msg->pose.position.z+m_OriginPos.position.z, tf::getYaw(msg->pose.orientation));

  if (m_GoalsPos.size() == 0)
  {
    ROS_INFO("Can Not add Goal, Select Start Position Fist !");
  }
  else
  {
    m_GoalsPos.push_back(wp);
    ROS_INFO("Received Goal Pose");
  }
}

void way_planner_core::callbackGetStartPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
  m_CurrentPose = PlannerHNS::WayPoint(msg->pose.pose.position.x+m_OriginPos.position.x,
      msg->pose.pose.position.y+m_OriginPos.position.y,
      msg->pose.pose.position.z+m_OriginPos.position.z,
      tf::getYaw(msg->pose.pose.orientation));

  if (m_GoalsPos.size() <= 1)
  {
    m_GoalsPos.clear();
    m_GoalsPos.push_back(m_CurrentPose);
    ROS_INFO("Received Start pose");
  }
}

void way_planner_core::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
  m_CurrentPose = PlannerHNS::WayPoint(msg->pose.position.x, msg->pose.position.y,
      msg->pose.position.z, tf::getYaw(msg->pose.orientation));

  if (m_GoalsPos.size() <= 1)
  {
    m_GoalsPos.clear();
    m_GoalsPos.push_back(m_CurrentPose);
  }
}

void way_planner_core::callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg)
{
  m_VehicleState.speed = msg->twist.twist.linear.x;

  UtilityHNS::UtilityH::GetTickCount(m_VehicleState.tStamp);
}

void way_planner_core::callbackGetVehicleStatus(const geometry_msgs::TwistStampedConstPtr& msg)
{
  m_VehicleState.speed = msg->twist.linear.x;
  UtilityHNS::UtilityH::GetTickCount(m_VehicleState.tStamp);
}

void way_planner_core::callbackGetCANInfo(const autoware_can_msgs::CANInfoConstPtr &msg)
{
  m_VehicleState.speed = msg->speed / 3.6;
  m_VehicleState.steer = (msg->angle * 0.45) / 660;
  std::cout << "Can Info, Speed: "<< m_VehicleState.speed << ", Steering: " << m_VehicleState.steer << std::endl;
}

void way_planner_core::callbackGetVMPoints(const vector_map_msgs::PointArray& msg)
{
  ROS_INFO("Received Map Points");
  m_AwMap.points = msg;
  m_AwMap.bPoints = true;
}

void way_planner_core::callbackGetVMLanes(const vector_map_msgs::LaneArray& msg)
{
  ROS_INFO("Received Map Lane Array");
  m_AwMap.lanes = msg;
  m_AwMap.bLanes = true;
}

void way_planner_core::callbackGetVMNodes(const vector_map_msgs::NodeArray& msg)
{
}

void way_planner_core::callbackGetVMStopLines(const vector_map_msgs::StopLineArray& msg)
{
}

void way_planner_core::callbackGetVMCenterLines(const vector_map_msgs::DTLaneArray& msg)
{
  ROS_INFO("Received Map Center Lines");
  m_AwMap.dtlanes = msg;
  m_AwMap.bDtLanes = true;
}

void way_planner_core::callbackGetNodesList(const vector_map_msgs::NodeArray& msg)
{
}

bool way_planner_core::GenerateGlobalPlan(PlannerHNS::WayPoint& startPoint, PlannerHNS::WayPoint& goalPoint,
                                          std::vector<std::vector<PlannerHNS::WayPoint>>& generatedTotalPaths)
{
  generatedTotalPaths.clear();

#ifdef ENABLE_VISUALIZE_PLAN
  std::vector<int> predefinedLanesIds;
  double planning_distance = pow((m_CurrentPose.v), 2);
  if (planning_distance < MIN_EXTRA_PLAN_DISTANCE) {
    planning_distance = MIN_EXTRA_PLAN_DISTANCE;
  }

  double ret = m_PlannerH.PlanUsingDP(
      startPoint, goalPoint, MAX_GLOBAL_PLAN_SEARCH_DISTANCE, planning_distance,
      m_params.bEnableLaneChange, predefinedLanesIds, m_Map,
      generatedTotalPaths);

  m_pCurrGoal =
      PlannerHNS::MappingHelpers::GetClosestWaypointFromMap(goalPoint, m_Map);

#else
  std::vector<int> predefinedLanesIds;

  double ret = m_PlannerH.PlanUsingDP(
      startPoint, goalPoint, MAX_GLOBAL_PLAN_DISTANCE,
      m_params.bEnableLaneChange, predefinedLanesIds, m_Map,
      generatedTotalPaths, nullptr, m_params.fallbackMinGoalDistanceTh);
#endif

  if (m_params.bEnableHMI)
  {
    for (unsigned int im = 0; im < m_ModifiedWayPointsCosts.size(); im++)
    {
      m_ModifiedWayPointsCosts.at(im)->actionCost.at(0).second = 0;
    }

    m_ModifiedWayPointsCosts.clear();
  }
  if (ret == 0)
  {
    generatedTotalPaths.clear();
  }

  if ((generatedTotalPaths.size() > 0) && (generatedTotalPaths.at(0).size() > 0))
  {
    if (m_params.bEnableSmoothing)
    {
      for (unsigned int i = 0; i < generatedTotalPaths.size(); i++)
      {
        PlannerHNS::PlanningHelpers::FixPathDensity(generatedTotalPaths.at(i), m_params.pathDensity);
        PlannerHNS::PlanningHelpers::SmoothPath(generatedTotalPaths.at(i), 0.49, 0.35, 0.01);
      }
    }

    for (unsigned int i = 0; i < generatedTotalPaths.size(); i++)
    {
      PlannerHNS::PlanningHelpers::CalcAngleAndCost(generatedTotalPaths.at(i));
      std::cout << "New DP Path -> " << generatedTotalPaths.at(i).size() << std::endl;
    }

    return true;
  }
  else
  {
    std::cout << "Can't Generate Global Path for Start (" << startPoint.pos.ToString()
              << ") and Goal (" << goalPoint.pos.ToString() << ")" << std::endl;
  }
  return false;
}

void way_planner_core::VisualizeAndSend(const std::vector<std::vector<PlannerHNS::WayPoint> > generatedTotalPaths)
{
  autoware_msgs::LaneArray lane_array;
  visualization_msgs::MarkerArray pathsToVisualize;

  for (unsigned int i = 0; i < generatedTotalPaths.size(); i++)
  {
    ROSHelpers::ConvertFromPlannerHToAutowarePathFormat(generatedTotalPaths.at(i), lane_array);
  }

  std_msgs::ColorRGBA total_color;
  total_color.r = 0;
  total_color.g = 0.7;
  total_color.b = 1.0;
  total_color.a = 0.9;
  ROSHelpers::createGlobalLaneArrayMarker(total_color, lane_array, pathsToVisualize);
  ROSHelpers::createGlobalLaneArrayOrientationMarker(lane_array, pathsToVisualize);
  ROSHelpers::createGlobalLaneArrayVelocityMarker(lane_array, pathsToVisualize);

  pub_PathsRviz.publish(pathsToVisualize);
  pub_Paths.publish(lane_array);

#ifdef OPENPLANNER_ENABLE_LOGS
  for (unsigned int i = 0; i < generatedTotalPaths.size(); i++)
  {
    std::ostringstream str_out;
    str_out << UtilityHNS::UtilityH::GetHomeDirectory();
    str_out << UtilityHNS::DataRW::LoggingMainfolderName;
    str_out << "GlobalPath_";
    str_out << i;
    str_out << "_";
    PlannerHNS::PlanningHelpers::WritePathToFile(str_out.str(), generatedTotalPaths.at(i));
  }
#endif
}

#ifdef ENABLE_VISUALIZE_PLAN
void way_planner_core::CreateNextPlanningTreeLevelMarker(std::vector<PlannerHNS::WayPoint*>& level,
                                      visualization_msgs::MarkerArray& markerArray, double max_cost)
{
  if ((level.size() == 0) && m_pCurrGoal)
  {
    return;
  }

  std::vector<PlannerHNS::WayPoint*> newlevel;

  for (unsigned int i = 0; i < level.size(); i++)
  {
    visualization_msgs::Marker lane_waypoint_marker;
    lane_waypoint_marker.header.frame_id = "map";
    lane_waypoint_marker.header.stamp = ros::Time();
    lane_waypoint_marker.type = visualization_msgs::Marker::ARROW;
    lane_waypoint_marker.ns = "tree_levels";
    lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
    lane_waypoint_marker.scale.x = 1.0;
    lane_waypoint_marker.scale.y = 0.5;
    lane_waypoint_marker.scale.z = 0.5;
    lane_waypoint_marker.color.a = 0.8;
    lane_waypoint_marker.color.b = 1-0.0;

    float norm_cost = (level.at(i)->cost / max_cost) * 2.0;
    if (norm_cost <= 1.0)
    {
      lane_waypoint_marker.color.r = 1 - norm_cost;
      lane_waypoint_marker.color.g = 1 - 1.0;
    }
    else if (norm_cost > 1.0)
    {
      lane_waypoint_marker.color.r = 1 - 1.0;
      lane_waypoint_marker.color.g = 1 - (2.0 - norm_cost);
    }

    if (markerArray.markers.size() == 0)
    {
      lane_waypoint_marker.id = 0;
    }
    else
    {
      lane_waypoint_marker.id = markerArray.markers.at(markerArray.markers.size() - 1).id + 1;
    }

    lane_waypoint_marker.pose.position.x = level.at(i)->pos.x;
    lane_waypoint_marker.pose.position.y = level.at(i)->pos.y;
    lane_waypoint_marker.pose.position.z = level.at(i)->pos.z;
    double a = UtilityHNS::UtilityH::SplitPositiveAngle(level.at(i)->pos.a);
    lane_waypoint_marker.pose.orientation = tf::createQuaternionMsgFromYaw(a);
    markerArray.markers.push_back(lane_waypoint_marker);

    if (level.at(i)->pLeft)
    {
      lane_waypoint_marker.pose.orientation = tf::createQuaternionMsgFromYaw(a + M_PI_2);
      newlevel.push_back(level.at(i)->pLeft);
      lane_waypoint_marker.id = markerArray.markers.at(markerArray.markers.size()-1).id + 1;
      markerArray.markers.push_back(lane_waypoint_marker);
    }
    if (level.at(i)->pRight)
    {
      newlevel.push_back(level.at(i)->pRight);
      lane_waypoint_marker.pose.orientation = tf::createQuaternionMsgFromYaw(a - M_PI_2);
      lane_waypoint_marker.id = markerArray.markers.at(markerArray.markers.size() - 1).id + 1;
      markerArray.markers.push_back(lane_waypoint_marker);
    }

    for (unsigned int j = 0; j < level.at(i)->pFronts.size(); j++)
    {
      if (level.at(i)->pFronts.at(j))
      {
        newlevel.push_back(level.at(i)->pFronts.at(j));
      }
    }

    if (hypot(m_pCurrGoal->pos.y - level.at(i)->pos.y, m_pCurrGoal->pos.x - level.at(i)->pos.x) < 0.5)
    {
      newlevel.clear();
      break;
    }

    std::cout << "Levels: " << lane_waypoint_marker.id << ", pLeft:" << level.at(i)->pLeft << ", pRight:"
              << level.at(i)->pRight << ", nFront:" << level.at(i)->pFronts.size() << ", Cost: "
              << norm_cost << std::endl;
  }

  level = newlevel;
}
#endif

bool way_planner_core::HMI_DoOneStep()
{
  double min_distance = m_AvgResponseTime * m_VehicleState.speed;
  std::vector<PlannerHNS::WayPoint*> branches;

  PlannerHNS::WayPoint startPoint;

  if (m_GoalsPos.size() > 1)
  {
    startPoint = m_CurrentPose;
  }

  PlannerHNS::WayPoint* currOptions = 0;
  ROSHelpers::FindIncommingBranches(m_GeneratedTotalPaths, startPoint, min_distance, branches, currOptions);
  if (branches.size() > 0)
  {
    HMI_MSG msg;
    msg.type = OPTIONS_MSG;
    msg.options.clear();
    for (unsigned int i = 0; i < branches.size(); i++)
    {
      msg.options.push_back(branches.at(i)->actionCost.at(0).first);
    }

    std::cout << "Send Message (" <<  branches.size() << ") Branches (";
    for (unsigned int i = 0; i < branches.size(); i++)
    {
      if (branches.at(i)->actionCost.at(0).first == PlannerHNS::FORWARD_ACTION)
      {
        std::cout << "F,";
      }
      else if (branches.at(i)->actionCost.at(0).first == PlannerHNS::LEFT_TURN_ACTION)
      {
        std::cout << "L,";
      }
      else if (branches.at(i)->actionCost.at(0).first == PlannerHNS::RIGHT_TURN_ACTION)
      {
        std::cout << "R,";
      }
    }

    std::cout << ")" << std::endl;

    int close_index = PlannerHNS::PlanningHelpers::GetClosestNextPointIndex_obsolete(m_GeneratedTotalPaths.at(0),
                                                                                      startPoint);
    for (unsigned int i = close_index + 1; i < m_GeneratedTotalPaths.at(0).size(); i++)
    {
      bool bFound = false;
      for (unsigned int j = 0; j < branches.size(); j++)
      {
        if (branches.at(j)->id == m_GeneratedTotalPaths.at(0).at(i).id)
        {
          currOptions = branches.at(j);
          bFound = true;
          break;
        }
      }
      if (bFound)
      {
        break;
      }
    }

    if (currOptions != 0)
    {
      msg.current = currOptions->actionCost.at(0).first;
      msg.currID = currOptions->laneId;
    }

    m_SocketServer.SendMSG(msg);

    double total_d = 0;
    for (unsigned int iwp = 1; iwp < m_GeneratedTotalPaths.at(0).size(); iwp++)
    {
      total_d += hypot(m_GeneratedTotalPaths.at(0).at(iwp).pos.y - m_GeneratedTotalPaths.at(0).at(iwp-1).pos.y,
                        m_GeneratedTotalPaths.at(0).at(iwp).pos.x - m_GeneratedTotalPaths.at(0).at(iwp-1).pos.x);
    }

    HMI_MSG inc_msg;
    int bNew = m_SocketServer.GetLatestMSG(inc_msg);
    if (bNew > 0)
    {
      for (unsigned int i = 0; i < branches.size(); i++)
      {
        for (unsigned int j = 0; j < inc_msg.options.size(); j++)
        {
          if (branches.at(i)->actionCost.at(0).first == inc_msg.options.at(j))
          {
            branches.at(i)->actionCost.at(0).second = -total_d * 4.0;
            m_ModifiedWayPointsCosts.push_back(branches.at(i));
          }
        }
      }
      return true;
    }
  }

  return false;
}

void way_planner_core::PlannerMainLoop()
{
  ros::Rate loop_rate(10);
  timespec animation_timer;
  UtilityHNS::UtilityH::GetTickCount(animation_timer);
  int newPlanTry = 0;

  while (ros::ok())
  {
    ros::spinOnce();
    bool bMakeNewPlan = false;

    if (m_params.bEnableHMI)
    {
      bMakeNewPlan = HMI_DoOneStep();
    }

    if(m_params.mapSource == MAP_KML_FILE && !m_bKmlMap)
    {
      m_bKmlMap = true;
      PlannerHNS::KmlMapLoader kml_loader;
      kml_loader.LoadKML(m_params.KmlMapPath, m_Map);
      visualization_msgs::MarkerArray map_marker_array;
      ROSHelpers::ConvertFromRoadNetworkToAutowareVisualizeMapFormat(m_Map, map_marker_array);
      pub_MapRviz.publish(map_marker_array);
    }
    else if (m_params.mapSource == MAP_FOLDER && !m_bKmlMap)
    {
      m_bKmlMap = true;
      PlannerHNS::VectorMapLoader vec_loader(1, m_params.bEnableLaneChange);
      vec_loader.LoadFromFile(m_params.KmlMapPath, m_Map);
      visualization_msgs::MarkerArray map_marker_array;
      ROSHelpers::ConvertFromRoadNetworkToAutowareVisualizeMapFormat(m_Map, map_marker_array);

      pub_MapRviz.publish(map_marker_array);

    }
    else if(m_params.mapSource == MAP_AUTOWARE)
    {
      if(m_AwMap.bDtLanes && m_AwMap.bLanes && m_AwMap.bPoints)
      {
        m_AwMap.bDtLanes = m_AwMap.bLanes = m_AwMap.bPoints = false;

        UtilityHNS::MapRaw map_raw;
        map_raw.LoadFromData(m_AwMap.lanes, m_AwMap.dtlanes, m_AwMap.points);
        PlannerHNS::VectorMapLoader vec_loader(1, m_params.bEnableLaneChange);
        vec_loader.LoadFromData(map_raw, m_Map);
        visualization_msgs::MarkerArray map_marker_array;
        ROSHelpers::ConvertFromRoadNetworkToAutowareVisualizeMapFormat(m_Map, map_marker_array);
        pub_MapRviz.publish(map_marker_array);
      }
    }

    if (m_GoalsPos.size() > 1)
    {
      PlannerHNS::WayPoint startPoint = m_CurrentPose;
      PlannerHNS::WayPoint goalPoint = m_GoalsPos.at(m_iCurrentGoalIndex);

      if ((m_GeneratedTotalPaths.size() > 0) && (m_GeneratedTotalPaths.at(0).size() > 3))
      {
        if (m_params.bEnableReplanning)
        {
          PlannerHNS::RelativeInfo info;
          bool ret = PlannerHNS::PlanningHelpers::GetRelativeInfoRange(m_GeneratedTotalPaths, startPoint, 0.75, info);
          if ((ret == true) && (info.iGlobalPath >= 0) && (info.iGlobalPath < m_GeneratedTotalPaths.size())
              && (info.iFront > 0) && (info.iFront < m_GeneratedTotalPaths.at(info.iGlobalPath).size()))
          {
            double remaining_distance = m_GeneratedTotalPaths.at(info.iGlobalPath).at(m_GeneratedTotalPaths.at(
                info.iGlobalPath).size() - 1).cost -
                (m_GeneratedTotalPaths.at(info.iGlobalPath).at(info.iFront).cost + info.to_front_distance);
            if (remaining_distance <= REPLANNING_DISTANCE)
            {
              m_iCurrentGoalIndex++;
              if (m_iCurrentGoalIndex >= m_GoalsPos.size())
              {
                m_iCurrentGoalIndex = 0;
              }
              bMakeNewPlan = true;
            }
          }
        }
      }
      else
      {
        bMakeNewPlan = true;
      }

      if (bMakeNewPlan)
      {
        bool bNewPlan = GenerateGlobalPlan(startPoint, goalPoint, m_GeneratedTotalPaths);
        //  If the maximum attempt feature is enabled, increase the counter
        if (m_params.planningMaxAttempt != 0)
        {
          newPlanTry++;
        }

        if (bNewPlan)
        {
          //  Reset the newPlanTry as we have found the path
          newPlanTry = 0;
          bMakeNewPlan = false;
          VisualizeAndSend(m_GeneratedTotalPaths);
#ifdef ENABLE_VISUALIZE_PLAN
          //  calculate new max_cost
          if (m_PlanningVisualizeTree.size() > 1)
          {
            m_CurrentLevel.push_back(m_PlanningVisualizeTree.at(0));
            m_CurrMaxCost = 0;
            for (unsigned int itree = 0; itree < m_PlanningVisualizeTree.size(); itree++)
            {
              if (m_PlanningVisualizeTree.at(itree)->cost > m_CurrMaxCost)
              {
                m_CurrMaxCost = m_PlanningVisualizeTree.at(itree)->cost;
              }
            }
          }
#endif
        }
        else
        {
          //  If we retried enough, remove the goal from the queue
          if (newPlanTry >= m_params.planningMaxAttempt)
          {
            ROS_WARN("%s (tried %d times), %s",
              "way_planner: Unable to plan the path",
              m_params.planningMaxAttempt,
              "removing the goal");
            m_GoalsPos.erase(m_GoalsPos.begin() + m_iCurrentGoalIndex);
            newPlanTry = 0;
          }
          else
          {
            //  Retry for m_params.planningMaxAttempt times
            ROS_WARN("way_planner: Unable to plan the requested goal path, will retry");
          }
        }
      }

#ifdef ENABLE_VISUALIZE_PLAN
      if (UtilityHNS::UtilityH::GetTimeDiffNow(animation_timer) > 0.5)
      {
        UtilityHNS::UtilityH::GetTickCount(animation_timer);
        m_CurrentLevel.clear();

        for (unsigned int ilev = 0; (ilev < m_nLevelSize) && (m_iCurrLevel < m_PlanningVisualizeTree.size()); ilev++)
        {
          m_CurrentLevel.push_back(m_PlanningVisualizeTree.at(m_iCurrLevel));
          m_nLevelSize += m_PlanningVisualizeTree.at(m_iCurrLevel)->pFronts.size() - 1;
          m_iCurrLevel++;
        }

        if ((m_CurrentLevel.size() == 0) && (m_GeneratedTotalPaths.size() > 0))
        {
          m_bSwitch++;
          m_AccumPlanLevels.markers.clear();

          if (m_bSwitch == 2)
          {
            for (unsigned int il = 0; il < m_GeneratedTotalPaths.size(); il++)
            {
              for (unsigned int ip = 0; ip < m_GeneratedTotalPaths.at(il).size(); ip++)
              {
                m_CurrentLevel.push_back(&m_GeneratedTotalPaths.at(il).at(ip));
              }
            }
            std::cout << "Switch On " << std::endl;
            m_bSwitch = 0;
          }
          else
          {
            for (unsigned int ilev = 0; ilev < (m_PlanningVisualizeTree.size() + 200); ilev++)
            {
              m_CurrentLevel.push_back(m_PlanningVisualizeTree.at(0));
            }

            std::cout << "Switch Off " << std::endl;
          }

          CreateNextPlanningTreeLevelMarker(m_CurrentLevel, m_AccumPlanLevels, m_CurrMaxCost);
          pub_GlobalPlanAnimationRviz.publish(m_AccumPlanLevels);
        }
        else
        {
          CreateNextPlanningTreeLevelMarker(m_CurrentLevel, m_AccumPlanLevels, m_CurrMaxCost);

          if (m_AccumPlanLevels.markers.size() > 0)
          {
            pub_GlobalPlanAnimationRviz.publish(m_AccumPlanLevels);
          }
        }
      }
#endif
    }

    loop_rate.sleep();
  }
}

}  // namespace WayPlannerNS
