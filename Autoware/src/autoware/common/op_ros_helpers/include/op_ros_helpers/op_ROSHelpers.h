/// \file  ROSHelpers.h
/// \brief Helper functions for rviz visualization
/// \author Hatem Darweesh
/// \date Jun 30, 2016

#ifndef OP_ROSHELPERS_H_
#define OP_ROSHELPERS_H_

#include <ros/ros.h>
#include "op_planner/RoadNetwork.h"
#include "op_planner/PlannerCommonDef.h"
#include "op_planner/LocalPlannerH.h"

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

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "autoware_msgs/CloudClusterArray.h"
#include "autoware_msgs/DetectedObjectArray.h"

#include "libwaypoint_follower/libwaypoint_follower.h"
#include "autoware_msgs/LaneArray.h"

#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

namespace PlannerHNS
{

class ROSHelpers
{
public:
	ROSHelpers();
	virtual ~ROSHelpers();

	static void getTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::TransformListener& listener, tf::StampedTransform &transform);

	static void transformDetectedObjects(const std::string& src_frame, const std::string& dst_frame, const tf::StampedTransform& trans,
			const autoware_msgs::DetectedObjectArray& input, autoware_msgs::DetectedObjectArray& transformed_input, bool bTransformBoundary = true);

	static void ConvertFromAutowareCloudClusterObstaclesToPlannerH(const PlannerHNS::WayPoint& currState, const double& car_width,
			const double& car_length, const autoware_msgs::CloudClusterArray& clusters,
			std::vector<PlannerHNS::DetectedObject>& impObstacles, const double max_obj_size, const double& min_obj_size, const double& detection_radius,
			const int& n_poly_quarters,const double& poly_resolution, int& nOriginalPoints, int& nContourPoints);

	static visualization_msgs::Marker CreateGenMarker(const double& x, const double& y, const double& z,const double& a,
			const double& r, const double& g, const double& b, const double& scale, const int& id, const std::string& ns, const int& type);

	static void InitMatchingMarkers(const int& nMarkers, visualization_msgs::MarkerArray& connections);

	static void ConvertMatchingMarkers(const std::vector<std::pair<PlannerHNS::WayPoint, PlannerHNS::WayPoint> >& match_list,
			visualization_msgs::MarkerArray& tracked_traj_d, visualization_msgs::MarkerArray& tracked_traj, int start_id=0);

	static void InitMarkers(const int& nMarkers,
			visualization_msgs::MarkerArray& centers,
			visualization_msgs::MarkerArray& dirs,
			visualization_msgs::MarkerArray& text_info,
			visualization_msgs::MarkerArray& polygons,
			visualization_msgs::MarkerArray& trajectories);

	static int ConvertTrackedObjectsMarkers(const PlannerHNS::WayPoint& currState, const std::vector<PlannerHNS::DetectedObject>& trackedObstacles,
			visualization_msgs::MarkerArray& centers_d,
			visualization_msgs::MarkerArray& dirs_d,
			visualization_msgs::MarkerArray& text_info_d,
			visualization_msgs::MarkerArray& polygons_d,
			visualization_msgs::MarkerArray& tracked_traj_d,
			visualization_msgs::MarkerArray& centers,
			visualization_msgs::MarkerArray& dirs,
			visualization_msgs::MarkerArray& text_info,
			visualization_msgs::MarkerArray& polygons,
			visualization_msgs::MarkerArray& tracked_traj);

	static void CreateCircleMarker(const PlannerHNS::WayPoint& _center, const double& radius, const double& r, const double& g, const double& b, const int& start_id, const std::string& name_space,visualization_msgs::Marker& circle_points);

	static void TrajectorySelectedToMarkers(const std::vector<PlannerHNS::WayPoint>& path, const double& r_path, const double& g_path, const double& b_path, const double& r_circle,
			const double& g_circle, const double& b_circle, const double& radius, visualization_msgs::MarkerArray& markerArray, int skip = 0);

	static void TrajectorySelectedToCircles(const std::vector<PlannerHNS::WayPoint>& path, const double& r_path, const double& g_path, const double& b_path, const double& r_circle,
			const double& g_circle, const double& b_circle, const double& radius, visualization_msgs::MarkerArray& markerArray, int skip = 0);

	static void DrivingPathToMarkers(const std::vector<std::pair<PlannerHNS::WayPoint, PlannerHNS::PolygonShape> >& path, visualization_msgs::MarkerArray& markerArray);

	static void InitPredMarkers(const int& nMarkers, visualization_msgs::MarkerArray& paths);

	static void InitCurbsMarkers(const int& nMarkers, visualization_msgs::MarkerArray& curbs);

	static void ConvertPredictedTrqajectoryMarkers(std::vector<std::vector<PlannerHNS::WayPoint> >& paths,visualization_msgs::MarkerArray& path_markers, visualization_msgs::MarkerArray& path_markers_d);

	static void ConvertCurbsMarkers(const std::vector<PlannerHNS::DetectedObject>& curbs, visualization_msgs::MarkerArray& curbs_markers, visualization_msgs::MarkerArray& curbs_markers_d);

	static void TrajectoriesToMarkers(const std::vector<std::vector<std::vector<PlannerHNS::WayPoint> > >& paths, visualization_msgs::MarkerArray& markerArray);

	static void TrajectoriesToColoredMarkers(const std::vector<std::vector<PlannerHNS::WayPoint> >& paths,const std::vector<PlannerHNS::TrajectoryCost>& traj_costs, const int& iClosest, visualization_msgs::MarkerArray& markerArray);

	static void InitCollisionPointsMarkers(const int& nMarkers, visualization_msgs::MarkerArray& col_points);

	static void ConvertCollisionPointsMarkers(const std::vector<PlannerHNS::WayPoint>& col_pointss, visualization_msgs::MarkerArray& collision_markers, visualization_msgs::MarkerArray& collision_markers_d);

	static void InitPredParticlesMarkers(const int& nMarkers, visualization_msgs::MarkerArray& paths, bool bOld = false);

	static void ConvertParticles(std::vector<PlannerHNS::WayPoint>& points, visualization_msgs::MarkerArray& part_mkrs, visualization_msgs::MarkerArray& part_markers_d, bool bOld = false);

	static void ConvertFromPlannerHToAutowarePathFormat(const std::vector<PlannerHNS::WayPoint>& path, const int& iStart,
				autoware_msgs::Lane & trajectory);

	static void ConvertFromPlannerHRectangleToAutowareRviz(const std::vector<PlannerHNS::GPSPoint>& safety_rect,
			visualization_msgs::Marker& marker);

	static void ConvertFromPlannerHToAutowareVisualizePathFormat(const std::vector<PlannerHNS::WayPoint>& curr_path,
			const std::vector<std::vector<std::vector<PlannerHNS::WayPoint> > >& paths, const PlannerHNS::LocalPlannerH& localPlanner,
				visualization_msgs::MarkerArray& markerArray);

	static void ConvertFromPlannerHToAutowareVisualizePathFormat(const std::vector<std::vector<PlannerHNS::WayPoint> >& globalPaths, visualization_msgs::MarkerArray& markerArray);

	static void ConvertFromRoadNetworkToAutowareVisualizeMapFormat(const PlannerHNS::RoadNetwork& map,	visualization_msgs::MarkerArray& markerArray, bool show_connections = false);

	static void ConvertFromAutowareBoundingBoxObstaclesToPlannerH(const jsk_recognition_msgs::BoundingBoxArray& detectedObstacles,
			std::vector<PlannerHNS::DetectedObject>& impObstacles);

	static void ConvertFromPlannerObstaclesToAutoware(const PlannerHNS::WayPoint& currState, const std::vector<PlannerHNS::DetectedObject>& trackedObstacles,
			visualization_msgs::MarkerArray& detectedPolygons);

	static void ConvertFromLocalLaneToAutowareLane(const std::vector<PlannerHNS::WayPoint>& path, autoware_msgs::Lane& trajectory , const unsigned int& iStart = 0);

	static void ConvertFromLocalLaneToAutowareLane(const std::vector<PlannerHNS::GPSPoint>& path, autoware_msgs::Lane& trajectory);

	static void ConvertFromAutowareLaneToLocalLane(const autoware_msgs::Lane& trajectory, std::vector<PlannerHNS::WayPoint>& path);

	static PlannerHNS::PID_CONST GetPIDValues(const std::string& str_param);

	static void createGlobalLaneArrayMarker(std_msgs::ColorRGBA color, const autoware_msgs::LaneArray &lane_waypoints_array, visualization_msgs::MarkerArray& markerArray);

	static void createGlobalLaneArrayVelocityMarker(const autoware_msgs::LaneArray &lane_waypoints_array , visualization_msgs::MarkerArray& markerArray);

	static void createGlobalLaneArrayOrientationMarker(const autoware_msgs::LaneArray &lane_waypoints_array , visualization_msgs::MarkerArray& markerArray);

	static void GetTrafficLightForVisualization(std::vector<PlannerHNS::TrafficLight>& lights, visualization_msgs::MarkerArray& markerArray);

	static void ConvertFromAutowareDetectedObjectToOpenPlannerDetectedObject(const autoware_msgs::DetectedObject& det_obj, PlannerHNS::DetectedObject& obj);

	static void ConvertFromOpenPlannerDetectedObjectToAutowareDetectedObject(const PlannerHNS::DetectedObject& det_obj, const bool& bSimulationMode, autoware_msgs::DetectedObject& obj);

	static std::string GetBehaviorNameFromCode(const PlannerHNS::STATE_TYPE& behState);

	static void VisualizeBehaviorState(const PlannerHNS::WayPoint& currState, const PlannerHNS::BehaviorState& beh, const bool& bGreenLight, const int& avoidDirection, visualization_msgs::Marker& behaviorMarker, std::string ns,double size_factor = 1);

	static void VisualizeIntentionState(const PlannerHNS::WayPoint& currState, const PlannerHNS::BEH_STATE_TYPE& beh, visualization_msgs::Marker& behaviorMarker, std::string ns,double size_factor = 1);

	static void GetIndicatorArrows(const PlannerHNS::WayPoint& center, const double& width,const double& length, const PlannerHNS::LIGHT_INDICATOR& indicator, const int& id,visualization_msgs::MarkerArray& markerArray);

	static void TTC_PathRviz(const std::vector<PlannerHNS::WayPoint>& path, visualization_msgs::MarkerArray& markerArray);

	static autoware_msgs::Waypoint ConvertBehaviorStateToAutowareWaypoint(const PlannerHNS::BehaviorState& beh);

	static PlannerHNS::BehaviorState ConvertAutowareWaypointToBehaviorState(const autoware_msgs::Waypoint& state_point);

	static void CreateNextPlanningTreeLevelMarker(std::vector<PlannerHNS::WayPoint*>& level, visualization_msgs::MarkerArray& markerArray, PlannerHNS::WayPoint* pCurrGoal, double max_cost);
};

}
#endif /* ROSHELPERS_H_ */
