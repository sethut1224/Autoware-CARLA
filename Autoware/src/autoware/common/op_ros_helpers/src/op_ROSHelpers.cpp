/// \file  ROSHelpers.cpp
/// \brief Helper functions for rviz visualization
/// \author Hatem Darweesh
/// \date Jun 30, 2016

#include "op_ros_helpers/op_ROSHelpers.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include "op_ros_helpers/PolygonGenerator.h"
#include "op_planner/MappingHelpers.h"
#include "op_planner/MatrixOperations.h"
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>


namespace PlannerHNS
{

ROSHelpers::ROSHelpers() {

}

ROSHelpers::~ROSHelpers() {
}

void ROSHelpers::getTransformFromTF(const std::string src_frame, const std::string dst_frame, tf::TransformListener& listener, tf::StampedTransform &transform)
{
	int counter = 0;
	while (1)
	{
		try
		{
			listener.waitForTransform(src_frame, dst_frame, ros::Time(0), ros::Duration(1.0));
			listener.lookupTransform(dst_frame, src_frame, ros::Time(0), transform);
			break;
		}
		catch (tf::TransformException& ex)
		{
			if(counter > 2)
			{
				ROS_ERROR("%s", ex.what());
			}
			ros::Duration(1.0).sleep();
			counter++;
		}
	}
}

void ROSHelpers::transformDetectedObjects(const std::string& src_frame, const std::string& dst_frame, const tf::StampedTransform& trans,
		const autoware_msgs::DetectedObjectArray& input, autoware_msgs::DetectedObjectArray& transformed_out, bool bTransformBoundary)
{
	transformed_out.objects.clear();
  for (size_t i = 0; i < input.objects.size(); i++)
  {
    tf::Transform input_object_pose;

    autoware_msgs::DetectedObject dd;
	dd = input.objects.at(i);

    input_object_pose.setOrigin(tf::Vector3(input.objects.at(i).pose.position.x, input.objects.at(i).pose.position.y, input.objects.at(i).pose.position.z));
    tf::Quaternion quat = tf::Quaternion(input.objects.at(i).pose.orientation.x, input.objects.at(i).pose.orientation.y, input.objects.at(i).pose.orientation.z, input.objects.at(i).pose.orientation.w);
    input_object_pose.setRotation(quat);
    geometry_msgs::Pose pose_out;
    tf::poseTFToMsg(trans * input_object_pose, pose_out);
    dd.pose = pose_out;

    dd.convex_hull.polygon.points.clear();
    geometry_msgs::Point32 p;
    for(unsigned int j=0; j < input.objects.at(i).convex_hull.polygon.points.size(); j++)
    {
    	p = input.objects.at(i).convex_hull.polygon.points.at(j);
    	if(bTransformBoundary == true)
    	{
    		tf::Transform poly_pose;
    		geometry_msgs::PoseStamped point_pose_out;
			poly_pose.setOrigin(tf::Vector3(p.x, p.y, p.z));
			poly_pose.setRotation(tf::Quaternion(input.objects.at(i).pose.orientation.x, input.objects.at(i).pose.orientation.y, input.objects.at(i).pose.orientation.z, input.objects.at(i).pose.orientation.w));
			tf::poseTFToMsg(trans * poly_pose, point_pose_out.pose);
			p.x = point_pose_out.pose.position.x;
			p.y = point_pose_out.pose.position.y;
			p.z = point_pose_out.pose.position.z;
    	}
    	dd.convex_hull.polygon.points.push_back(p);
    }

    if(input.objects.at(i).pointcloud.data.size() > 0)
    {
    	pcl_ros::transformPointCloud(dst_frame, trans, input.objects.at(i).pointcloud, dd.pointcloud);
    }

    transformed_out.objects.push_back(dd);
  }
}

visualization_msgs::Marker ROSHelpers::CreateGenMarker(const double& x, const double& y, const double& z,const double& a,
		const double& r, const double& g, const double& b, const double& scale, const int& id, const std::string& ns, const int& type)
{
	visualization_msgs::Marker mkr;
	mkr.header.frame_id = "map";
	mkr.header.stamp = ros::Time();
	mkr.ns = ns;
	mkr.type = type;
	mkr.lifetime = ros::Duration(0.1); // make old path disappear
	mkr.action = visualization_msgs::Marker::ADD;
	if(type != visualization_msgs::Marker::LINE_LIST && type != visualization_msgs::Marker::LINE_STRIP && type != visualization_msgs::Marker::TEXT_VIEW_FACING)
	{
	if(type == visualization_msgs::Marker::ARROW)
		mkr.scale.y = scale/2.0;
	else
		mkr.scale.y = scale;
	}

	if(type != visualization_msgs::Marker::TEXT_VIEW_FACING)
	{
		mkr.scale.x = scale;
	}

	if(type != visualization_msgs::Marker::LINE_LIST && type != visualization_msgs::Marker::LINE_STRIP)
	{
		mkr.scale.z = scale;
	}

	if(type == visualization_msgs::Marker::CYLINDER)
	{
		mkr.scale.z = 0.01;
	}

	mkr.color.a = 0.8;
	mkr.color.r = r;
	mkr.color.g = g;
	mkr.color.b = b;
	mkr.pose.position.x = x;
	mkr.pose.position.y = y;
	mkr.pose.position.z = z;
	mkr.pose.orientation = tf::createQuaternionMsgFromYaw(a);
	mkr.id = id;
	return mkr;
}

void ROSHelpers::InitMarkers(const int& nMarkers,
		visualization_msgs::MarkerArray& centers,
		visualization_msgs::MarkerArray& dirs,
		visualization_msgs::MarkerArray& text_info,
		visualization_msgs::MarkerArray& polygons,
		visualization_msgs::MarkerArray& trajectories)
{
	centers.markers.clear();
	dirs.markers.clear();
	text_info.markers.clear();
	polygons.markers.clear();
	trajectories.markers.clear();

	for(int i=0; i<nMarkers; i++)
	{
		visualization_msgs::Marker mkr = CreateGenMarker(0,0,0,0,1,1,1,1,i,"CenterMarker", visualization_msgs::Marker::SPHERE);
		centers.markers.push_back(mkr);
	}

	for(int i=nMarkers; i<nMarkers*2; i++)
	{
		visualization_msgs::Marker mkr = CreateGenMarker(0,0,0,0,1,1,1,1,i,"Directions", visualization_msgs::Marker::ARROW);
		dirs.markers.push_back(mkr);
	}

	for(int i=nMarkers*2; i<nMarkers*3; i++)
	{
		visualization_msgs::Marker mkr = CreateGenMarker(0,0,0,0,1,1,1,1,i,"InfoText", visualization_msgs::Marker::TEXT_VIEW_FACING);
		text_info.markers.push_back(mkr);
	}

	for(int i=nMarkers*3; i<nMarkers*4; i++)
	{
		visualization_msgs::Marker mkr = CreateGenMarker(0,0,0,0,1,1,1,1,i,"detected_polygons", visualization_msgs::Marker::LINE_STRIP);
		polygons.markers.push_back(mkr);
	}

	for(int i=nMarkers*4; i<nMarkers*5; i++)
	{
		visualization_msgs::Marker mkr = CreateGenMarker(0,0,0,0,1,1,1,1,i,"tracked_trajectories", visualization_msgs::Marker::LINE_STRIP);
		trajectories.markers.push_back(mkr);
	}
}

void ROSHelpers::InitMatchingMarkers(const int& nMarkers, visualization_msgs::MarkerArray& connections)
{
	connections.markers.clear();
	for(int i=0; i<nMarkers; i++)
	{
		visualization_msgs::Marker mkr = CreateGenMarker(0,0,0,0,1,1,1,1,i,"matching_connections", visualization_msgs::Marker::LINE_STRIP);
		connections.markers.push_back(mkr);
	}
}

void ROSHelpers::ConvertMatchingMarkers(const std::vector<std::pair<PlannerHNS::WayPoint, PlannerHNS::WayPoint> >& match_list,
		visualization_msgs::MarkerArray& tracked_traj_d, visualization_msgs::MarkerArray& tracked_traj, int start_id)
{

	tracked_traj = tracked_traj_d;

	for(unsigned int i = 0; i < match_list.size(); i++)
	{
		visualization_msgs::Marker match_mkr = CreateGenMarker(0,0,0,0,1,0,0,0.2, start_id+i,"matching_connections", visualization_msgs::Marker::LINE_STRIP);
		geometry_msgs::Point point;
		point.x = match_list.at(i).first.pos.x;
		point.y = match_list.at(i).first.pos.y;
		point.z = match_list.at(i).first.pos.z;
		match_mkr.points.push_back(point);

		point.x = match_list.at(i).second.pos.x;
		point.y = match_list.at(i).second.pos.y;
		point.z = match_list.at(i).second.pos.z;
		match_mkr.points.push_back(point);

		if(i < tracked_traj.markers.size())
			tracked_traj.markers.at(i) = match_mkr;
		else
			tracked_traj.markers.push_back(match_mkr);
	}

	if(match_list.size() < tracked_traj.markers.size())
	{
		tracked_traj.markers.erase(tracked_traj.markers.begin()+match_list.size(), tracked_traj.markers.end());
	}
}

int ROSHelpers::ConvertTrackedObjectsMarkers(const PlannerHNS::WayPoint& currState, const std::vector<PlannerHNS::DetectedObject>& trackedObstacles,
		visualization_msgs::MarkerArray& centers_d,
		visualization_msgs::MarkerArray& dirs_d,
		visualization_msgs::MarkerArray& text_info_d,
		visualization_msgs::MarkerArray& polygons_d,
		visualization_msgs::MarkerArray& tracked_traj_d,
		visualization_msgs::MarkerArray& centers,
		visualization_msgs::MarkerArray& dirs,
		visualization_msgs::MarkerArray& text_info,
		visualization_msgs::MarkerArray& polygons,
		visualization_msgs::MarkerArray& tracked_traj)
{

	int text_info_i = 0;
	int polygons_i = 0;
	int centers_i = 0;
	int dirs_i = 0;
	int tracked_traj_i = 0;

	centers = centers_d;
	dirs = dirs_d;
	text_info = text_info_d;
	polygons = polygons_d;
	tracked_traj = tracked_traj_d;

	for(unsigned int i =0; i < trackedObstacles.size(); i++)
	{
		int speed = (trackedObstacles.at(i).center.v*3.6);

		//Update Stage
		visualization_msgs::Marker center_mkr = CreateGenMarker(trackedObstacles.at(i).center.pos.x,trackedObstacles.at(i).center.pos.y,trackedObstacles.at(i).center.pos.z,
				trackedObstacles.at(i).center.pos.a,1,0,0,0.5,i,"CenterMarker", visualization_msgs::Marker::SPHERE);
		centers_i++;
		if(i < centers.markers.size())
		{
			center_mkr.id = centers.markers.at(i).id;
			centers.markers.at(i) = center_mkr;
		}
		else
			centers.markers.push_back(center_mkr);

		//Directions
		if(trackedObstacles.at(i).bDirection)
		{
			visualization_msgs::Marker dir_mkr = CreateGenMarker(trackedObstacles.at(i).center.pos.x,trackedObstacles.at(i).center.pos.y,trackedObstacles.at(i).center.pos.z+0.5,
					trackedObstacles.at(i).center.pos.a,0,1,0,0.3,centers.markers.size()+i,"Directions", visualization_msgs::Marker::ARROW);
			dirs_i++;
			dir_mkr.scale.x = 0.4;
			if(i < dirs.markers.size())
			{
				dir_mkr.id = dirs.markers.at(i).id;
				dirs.markers.at(i) = dir_mkr;
			}
			else
				dirs.markers.push_back(dir_mkr);
		}


		//Text
		visualization_msgs::Marker text_mkr;
//		if(speed > 3.0)
//			text_mkr = CreateGenMarker(trackedObstacles.at(i).center.pos.x+0.5,trackedObstacles.at(i).center.pos.y+0.5,trackedObstacles.at(i).center.pos.z+1,
//					trackedObstacles.at(i).center.pos.a,1,0,0,0.75,centers.markers.size()*2+i,"InfoText", visualization_msgs::Marker::TEXT_VIEW_FACING);
//		else
		text_mkr = CreateGenMarker(trackedObstacles.at(i).center.pos.x+0.5,trackedObstacles.at(i).center.pos.y+0.5,trackedObstacles.at(i).center.pos.z+1,
							trackedObstacles.at(i).center.pos.a,1,1,1,1.2,centers.markers.size()*2+i,"InfoText", visualization_msgs::Marker::TEXT_VIEW_FACING);
		text_info_i++;

		std::ostringstream str_out;
		str_out << trackedObstacles.at(i).id << " ( " << speed << " )" << " (" << trackedObstacles.at(i).distance_to_center << ")";
		//str_out << trackedObstacles.at(i).id << " (" << speed << ")";
		text_mkr.text = str_out.str();

		if(i < text_info.markers.size())
		{
			text_mkr.id = text_info.markers.at(i).id;
			text_info.markers.at(i) = text_mkr;
		}
		else
			text_info.markers.push_back(text_mkr);


		//Polygons
		visualization_msgs::Marker poly_mkr = CreateGenMarker(0,0,0,0, 1,0.25,0.25,0.1,centers.markers.size()*3+i,"detected_polygons", visualization_msgs::Marker::LINE_STRIP);
		polygons_i++;

		for(unsigned int p = 0; p < trackedObstacles.at(i).contour.size(); p++)
		{
			geometry_msgs::Point point;
			point.x = trackedObstacles.at(i).contour.at(p).x;
			point.y = trackedObstacles.at(i).contour.at(p).y;
			point.z = trackedObstacles.at(i).contour.at(p).z;
			poly_mkr.points.push_back(point);
		}

		if(trackedObstacles.at(i).contour.size()>0)
		{
			geometry_msgs::Point point;
			point.x = trackedObstacles.at(i).contour.at(0).x;
			point.y = trackedObstacles.at(i).contour.at(0).y;
			point.z = trackedObstacles.at(i).contour.at(0).z;
			poly_mkr.points.push_back(point);
		}

		if(i < polygons.markers.size())
		{
			poly_mkr.id =  polygons.markers.at(i).id;
			polygons.markers.at(i) = poly_mkr;
		}
		else
			polygons.markers.push_back(poly_mkr);


		//Trajectories
		if(trackedObstacles.at(i).centers_list.size() > 1)
		{
		visualization_msgs::Marker traj_mkr = CreateGenMarker(0,0,0,0,1,1,0,0.1,centers.markers.size()*4+i,"tracked_trajectories", visualization_msgs::Marker::LINE_STRIP);
		tracked_traj_i++;

		for(unsigned int p = 0; p < trackedObstacles.at(i).centers_list.size(); p++)
		{
			geometry_msgs::Point point;
			point.x = trackedObstacles.at(i).centers_list.at(p).pos.x;
			point.y = trackedObstacles.at(i).centers_list.at(p).pos.y;
			point.z = trackedObstacles.at(i).centers_list.at(p).pos.z;
			traj_mkr.points.push_back(point);
		}


		if(i < tracked_traj.markers.size())
		{
			traj_mkr.id = tracked_traj.markers.at(i).id ;
			tracked_traj.markers.at(i) = traj_mkr;
		}
		else
			tracked_traj.markers.push_back(traj_mkr);
		}
	}

	//if(text_info_i>0) text_info_i--;
	if(text_info_i < text_info.markers.size())
	{
		text_info.markers.erase(text_info.markers.begin()+text_info_i, text_info.markers.end());
	}

	//if(polygons_i>0) polygons_i--;
	if(polygons_i < polygons.markers.size())
	{
		polygons.markers.erase(polygons.markers.begin()+polygons_i, polygons.markers.end());
	}

	//if(dirs_i>0) dirs_i--;
	if(dirs_i < dirs.markers.size())
	{
		dirs.markers.erase(dirs.markers.begin()+dirs_i, dirs.markers.end());
	}

	//if(centers_i>0) centers_i--;
	if(centers_i < centers.markers.size())
	{
		centers.markers.erase(centers.markers.begin()+centers_i, centers.markers.end());
	}

	//if(tracked_traj_i>0) tracked_traj_i--;
	if(tracked_traj_i < tracked_traj.markers.size())
	{
		tracked_traj.markers.erase(tracked_traj.markers.begin()+tracked_traj_i, tracked_traj.markers.end());
	}

	return text_info_i;
}

void ROSHelpers::CreateCircleMarker(const PlannerHNS::WayPoint& _center, const double& radius, const double& r, const double& g, const double& b, const int& start_id, const std::string& name_space, visualization_msgs::Marker& circle_points)
{
	//"Detection_Circles"
	circle_points = CreateGenMarker(0,0,0,0,r,g,b,0.1,start_id,name_space, visualization_msgs::Marker::LINE_STRIP);
	for (float i = 0; i < M_PI*2.0+0.1; i+=0.1)
	{
		geometry_msgs::Point point;
		point.x = _center.pos.x + (radius * cos(i));
		point.y = _center.pos.y + (radius * sin(i));
		point.z = _center.pos.z;
		circle_points.points.push_back(point);
	}
}

void ROSHelpers::InitPredMarkers(const int& nMarkers, visualization_msgs::MarkerArray& paths)
{
	paths.markers.clear();
	for(int i=0; i<nMarkers; i++)
	{
		visualization_msgs::Marker mkr = CreateGenMarker(0,0,0,0,1,1,1,1,i,"Predicted_Trajectories", visualization_msgs::Marker::CUBE);
		paths.markers.push_back(mkr);
	}
}

void ROSHelpers::InitCurbsMarkers(const int& nMarkers, visualization_msgs::MarkerArray& curbs)
{
	curbs.markers.clear();
	for(int i=0; i<nMarkers; i++)
	{
		visualization_msgs::Marker mkr = CreateGenMarker(0,0,0,0,1,1,1,1,i,"map_detected_curbs", visualization_msgs::Marker::SPHERE);
		curbs.markers.push_back(mkr);
	}
}

void ROSHelpers::ConvertPredictedTrqajectoryMarkers(std::vector<std::vector<PlannerHNS::WayPoint> >& paths,visualization_msgs::MarkerArray& path_markers, visualization_msgs::MarkerArray& path_markers_d)
{
	path_markers = path_markers_d;
	int iCount = 0;
	for(unsigned int i = 0; i < paths.size(); i++)
	{
		if(paths.at(i).size() < 2) continue;

		double additional_z = 0.1;
		double prop = 1.0;
		bool bCurrent = false;
		if(paths.at(i).size()>0)
		{
			prop = paths.at(i).at(0).collisionCost;
			if(prop > 0.5)
			{
				bCurrent = true;
				//std::cout << "Trajectory with Index << " << i << " >> Cost = " << prop << std::endl;
			}
		}


		double r = 0, g = 0, b = 0;
		if(bCurrent == true)
		{
			g = 1.0;
		}
		else
		{
			r = 1.0;
		}
//		if(i == 0)
//		{
//			r = 1.0;
//		}
//		else if(i == 1)
//		{
//			b = 1.0;
//		}
//		else if(i == 2)
//		{
//			r = 1.0;
//			b = 1.0;
//		}
//		else if(i == 3)
//		{
//			r = 1.0;
//		}
//		else
//		{
//			r = 1.0;
//		}

		visualization_msgs::Marker path_mkr = CreateGenMarker(0,0,0,0,r,g,b,0.1,iCount,"Predicted_Trajectories", visualization_msgs::Marker::LINE_STRIP);

		//visualization_msgs::Marker path_mkr = CreateGenMarker(0,0,0,0,1.0*prop,0.1*prop,0.1*prop,0.1,i,"Predicted_Trajectories", visualization_msgs::Marker::LINE_STRIP);

		for(unsigned int p = 0; p < paths.at(i).size(); p++)
		{
			geometry_msgs::Point point;
			point.x = paths.at(i).at(p).pos.x;
			point.y = paths.at(i).at(p).pos.y;
			point.z = paths.at(i).at(p).pos.z + additional_z;
			path_mkr.points.push_back(point);
		}

		if(iCount < path_markers.markers.size())
			path_markers.markers.at(iCount) = path_mkr;
		else
			path_markers.markers.push_back(path_mkr);

		iCount++;

		r = 0.9;
		g = 0.9;
		b = 0.0;
		for(unsigned int p = 0; p < paths.at(i).size(); p++)
		{
			geometry_msgs::Point point;
			point.x = paths.at(i).at(p).pos.x;
			point.y = paths.at(i).at(p).pos.y;
			point.z = paths.at(i).at(p).pos.z + additional_z;

			visualization_msgs::Marker circle_mkr = CreateGenMarker(point.x,point.y,point.z,0.4,r,g,b,0.5,iCount,"Predicted_Trajectories", visualization_msgs::Marker::CYLINDER);
			if(iCount < path_markers.markers.size())
				path_markers.markers.at(iCount) = circle_mkr;
			else
				path_markers.markers.push_back(circle_mkr);
			iCount++;
		}

		r = 0.6;
		g = 0.6;
		b = 0.6;
		for(unsigned int p = 0; p < paths.at(i).size(); p++)
		{
			geometry_msgs::Point point;
			point.x = paths.at(i).at(p).pos.x;
			point.y = paths.at(i).at(p).pos.y + 0.5;
			point.z = paths.at(i).at(p).pos.z + additional_z + 0.01;
			std::ostringstream str_out;
			str_out.precision(3);
			str_out << paths.at(i).at(p).timeCost;

			visualization_msgs::Marker txt_mkr = CreateGenMarker(point.x,point.y,point.z,0.4,r,g,b,0.5,iCount,"Predicted_Trajectories", visualization_msgs::Marker::TEXT_VIEW_FACING);
			txt_mkr.text = str_out.str();

			if(iCount < path_markers.markers.size())
				path_markers.markers.at(iCount) = txt_mkr;
			else
				path_markers.markers.push_back(txt_mkr);

			iCount++;
		}
	}

	//if(iCount > 0) iCount--;

//	if(iCount < path_markers.markers.size())
//	{
//		path_markers.markers.erase(path_markers.markers.begin()+iCount, path_markers.markers.end());
//	}
}

void ROSHelpers::ConvertCurbsMarkers(const std::vector<PlannerHNS::DetectedObject>& curbs, visualization_msgs::MarkerArray& curbs_markers, visualization_msgs::MarkerArray& curbs_markers_d)
{
	curbs_markers = curbs_markers_d;
	int iMarkerIndex = 0;
	for(unsigned int i = 0; i < curbs.size(); i++)
	{
		for(unsigned int j = 0; j < curbs.at(i).contour.size(); j++)
		{
			visualization_msgs::Marker curb_mkr = CreateGenMarker(curbs.at(i).contour.at(j).x,curbs.at(i).contour.at(j).y,curbs.at(i).contour.at(j).z,0,1,0.54,0,0.2,iMarkerIndex,"map_detected_curbs", visualization_msgs::Marker::SPHERE);

			if(iMarkerIndex < curbs_markers.markers.size())
				curbs_markers.markers.at(iMarkerIndex) = curb_mkr;
			else
				curbs_markers.markers.push_back(curb_mkr);

			iMarkerIndex++;
		}
	}
}

void ROSHelpers::InitCollisionPointsMarkers(const int& nMarkers, visualization_msgs::MarkerArray& col_points)
{
	col_points.markers.clear();
	for(int i=0; i<nMarkers; i++)
	{
		visualization_msgs::Marker mkr = CreateGenMarker(0,0,0,0,1,1,1,1,i,"collision_points_rviz", visualization_msgs::Marker::SPHERE);
		col_points.markers.push_back(mkr);
	}
}

void ROSHelpers::ConvertCollisionPointsMarkers(const std::vector<PlannerHNS::WayPoint>& col_points, visualization_msgs::MarkerArray& collision_markers, visualization_msgs::MarkerArray& collision_markers_d)
{
	collision_markers = collision_markers_d;
	for(unsigned int i = 0; i < col_points.size(); i++)
	{
		visualization_msgs::Marker mkr = CreateGenMarker(col_points.at(i).pos.x, col_points.at(i).pos.y, col_points.at(i).pos.z,0,1,0,0,0.5,i,"collision_points_rviz", visualization_msgs::Marker::SPHERE);

		if(i < collision_markers.markers.size())
			collision_markers.markers.at(i) = mkr;
		else
			collision_markers.markers.push_back(mkr);

	}
}

void ROSHelpers::ConvertFromPlannerHToAutowarePathFormat(const std::vector<PlannerHNS::WayPoint>& path, const int& iStart,
		autoware_msgs::Lane& trajectory)
{
	trajectory.waypoints.clear();
	for(unsigned int i=iStart; i < path.size(); i++)
	{
		autoware_msgs::Waypoint wp;
		wp.pose.pose.position.x = path.at(i).pos.x;
		wp.pose.pose.position.y = path.at(i).pos.y;
		wp.pose.pose.position.z = path.at(i).pos.z;
		wp.pose.pose.orientation = tf::createQuaternionMsgFromYaw(UtilityHNS::UtilityH::SplitPositiveAngle(path.at(i).pos.a));
		wp.twist.twist.linear.x = path.at(i).v;
		if(path.at(i).bDir == FORWARD_DIR)
			wp.dtlane.dir = 0;
		else if(path.at(i).bDir == FORWARD_LEFT_DIR)
			wp.dtlane.dir = 1;
		else if(path.at(i).bDir == FORWARD_RIGHT_DIR)
			wp.dtlane.dir = 2;
		//PlannerHNS::GPSPoint p = path.at(i).pos;
		//std::cout << p.ToString() << std::endl;
		trajectory.waypoints.push_back(wp);
	}
}

void ROSHelpers::ConvertFromRoadNetworkToAutowareVisualizeMapFormat(const PlannerHNS::RoadNetwork& map,	visualization_msgs::MarkerArray& markerArray, bool show_connections)
{
	visualization_msgs::Marker lane_waypoint_marker;
	lane_waypoint_marker.header.frame_id = "map";
	//lane_waypoint_marker.header.stamp = ros::Time();
	lane_waypoint_marker.ns = "road_network_vector_map";
	lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
	lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
	lane_waypoint_marker.scale.x = 0.15;
	std_msgs::ColorRGBA roll_color;
	roll_color.r = 0.9;
	roll_color.g = 0.9;
	roll_color.b = 0.9;
	roll_color.a = 0.5;

	//lane_waypoint_marker.color = roll_color;
	lane_waypoint_marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);
	lane_waypoint_marker.frame_locked = false;

	markerArray.markers.clear();
	int marker_id = 0;

	for(unsigned int i = 0; i< map.roadSegments.size(); i++)
	{
		for(unsigned int j = 0; j < map.roadSegments.at(i).Lanes.size(); j++)
		{
		  lane_waypoint_marker.points.clear();
		  lane_waypoint_marker.colors.clear();
		  lane_waypoint_marker.id = marker_id++;
		  for(unsigned int p = 0; p < map.roadSegments.at(i).Lanes.at(j).points.size(); p++)
		  {
			geometry_msgs::Point point;
			point.x = map.roadSegments.at(i).Lanes.at(j).points.at(p).pos.x;
			point.y = map.roadSegments.at(i).Lanes.at(j).points.at(p).pos.y;
			point.z = map.roadSegments.at(i).Lanes.at(j).points.at(p).pos.z;

			std_msgs::ColorRGBA edge_color = roll_color;
//			if(map.roadSegments.at(i).Lanes.at(j).points.at(p).custom_type == CUSTOM_AVOIDANCE_DISABLED)
//			{
//				edge_color.r = 0.5;
//				edge_color.b = 0.5;
//			}

			lane_waypoint_marker.colors.push_back(edge_color);
			lane_waypoint_marker.points.push_back(point);
		  }
		  markerArray.markers.push_back(lane_waypoint_marker);
		}
	}

	// visualization_msgs::Marker lane_heading_marker;
	// lane_heading_marker.header.frame_id = "map";
	// //lane_waypoint_marker.header.stamp = ros::Time();
	// lane_heading_marker.ns = "road_network_vector_map";
	// lane_heading_marker.type = visualization_msgs::Marker::ARROW;
	// lane_heading_marker.action = visualization_msgs::Marker::ADD;
	// lane_heading_marker.scale.x = 0.5;
	// lane_heading_marker.scale.y = 0.1;
	// lane_heading_marker.scale.z = 0.1;

	// roll_color.r = 0.0;
	// roll_color.g = 0.9;
	// roll_color.b = 0.0;
	// roll_color.a = 1.0;

	// for(unsigned int i = 0; i< map.roadSegments.size(); i++)
	// {
	// 	for(unsigned int j = 0; j < map.roadSegments.at(i).Lanes.size(); j++)
	// 	{
	// 		for(unsigned int p = 0; p < map.roadSegments.at(i).Lanes.at(j).points.size(); p++)
	// 	  	{
	// 			lane_heading_marker.id = marker_id++;

	// 			lane_heading_marker.pose.position.x = map.roadSegments.at(i).Lanes.at(j).points.at(p).pos.x;
	// 			lane_heading_marker.pose.position.y = map.roadSegments.at(i).Lanes.at(j).points.at(p).pos.y;
	// 			lane_heading_marker.pose.position.z = map.roadSegments.at(i).Lanes.at(j).points.at(p).pos.z;

	// 			lane_heading_marker.pose.orientation.x = map.roadSegments.at(i).Lanes.at(j).points.at(p).rot.x;
	// 			lane_heading_marker.pose.orientation.y = map.roadSegments.at(i).Lanes.at(j).points.at(p).rot.y;
	// 			lane_heading_marker.pose.orientation.z = map.roadSegments.at(i).Lanes.at(j).points.at(p).rot.z;
	// 			lane_heading_marker.pose.orientation.w = map.roadSegments.at(i).Lanes.at(j).points.at(p).rot.w;

	// 			lane_heading_marker.color = roll_color;
	// 			markerArray.markers.push_back(lane_heading_marker);
	// 		}
	// 	}
	// }

	if(show_connections)
	{
		visualization_msgs::Marker laneID_text_marker;
		laneID_text_marker.header.frame_id = "map";
		//lane_waypoint_marker.header.stamp = ros::Time();
		laneID_text_marker.ns = "road_network_vector_map";
		laneID_text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		laneID_text_marker.action = visualization_msgs::Marker::ADD;
		laneID_text_marker.scale.z = 0.5;

		roll_color.r = 1;
		roll_color.g = 1;
		roll_color.b = 1;
		roll_color.a = 1.0;

		for(unsigned int i = 0; i< map.roadSegments.size(); i++)
		{
			for(unsigned int j = 0; j < map.roadSegments.at(i).Lanes.size(); j++)
			{
				laneID_text_marker.id = marker_id++;

				int labelIndex =  map.roadSegments.at(i).Lanes.at(j).points.size() / 2;

				laneID_text_marker.pose.position.x = map.roadSegments.at(i).Lanes.at(j).points.at(labelIndex).pos.x;
				laneID_text_marker.pose.position.y = map.roadSegments.at(i).Lanes.at(j).points.at(labelIndex).pos.y;
				laneID_text_marker.pose.position.z = map.roadSegments.at(i).Lanes.at(j).points.at(labelIndex).pos.z;

				char buffer [50];
				sprintf (buffer, "LANE ID: %d", map.roadSegments.at(i).Lanes.at(j).id);

				laneID_text_marker.text = buffer;

				laneID_text_marker.color = roll_color;
				markerArray.markers.push_back(laneID_text_marker);
			}
		}

		visualization_msgs::Marker trafficLightID_marker;
		trafficLightID_marker.header.frame_id = "map";
		//lane_waypoint_marker.header.stamp = ros::Time();
		trafficLightID_marker.ns = "road_network_vector_map";
		trafficLightID_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		trafficLightID_marker.action = visualization_msgs::Marker::ADD;
		trafficLightID_marker.scale.z = 0.5;

		roll_color.r = 1;
		roll_color.g = 1;
		roll_color.b = 1;
		roll_color.a = 1.0;



		for(unsigned int j = 0; j < map.trafficLights.size(); j++)
		{

			if(map.trafficLights.at(j).lightType == RED_LIGHT)
			{
				roll_color.r = 1;
				roll_color.g = 0;
				roll_color.b = 0;
				roll_color.a = 1.0;
			}
			else if(map.trafficLights.at(j).lightType == GREEN_LIGHT)
			{
				roll_color.r = 0;
				roll_color.g = 1;
				roll_color.b = 0;
				roll_color.a = 1.0;

			}
			trafficLightID_marker.id = marker_id++;

			trafficLightID_marker.pose.position.x = map.trafficLights.at(j).pose.pos.x;
			trafficLightID_marker.pose.position.y = map.trafficLights.at(j).pose.pos.y;
			trafficLightID_marker.pose.position.z = map.trafficLights.at(j).pose.pos.z;

			if(map.trafficLights.at(j).laneIds.size() > 0)
			{
				char buffer [50];
				sprintf (buffer, "TrafficLight ID: %d\n(LANE %d)", map.trafficLights.at(j).id, map.trafficLights.at(j).laneIds.at(0));
				trafficLightID_marker.text = buffer;
			}


			trafficLightID_marker.color = roll_color;
			markerArray.markers.push_back(trafficLightID_marker);
		}
	}


	if(show_connections && map.roadSegments.size() > 0)
	{
		for(auto l: map.roadSegments.at(0).Lanes)
		{
			PlanningHelpers::CalcAngleAndCost(l.points);
			for(auto& p: l.points)
			{
				marker_id++;
				visualization_msgs::Marker mkr_l;
				mkr_l = PlannerHNS::ROSHelpers::CreateGenMarker(p.pos.x,p.pos.y,p.pos.z,p.pos.a, 0.7,1,0.5,1.0,marker_id,"road_network_vector_map", visualization_msgs::Marker::ARROW);
				mkr_l.scale.y = 0.2;
				mkr_l.scale.z = 0.2;
				mkr_l.color.a = 0.25;
				markerArray.markers.push_back(mkr_l);
				if(p.LeftPointId > 0)
				{
					marker_id++;
					mkr_l = PlannerHNS::ROSHelpers::CreateGenMarker(p.pos.x,p.pos.y,p.pos.z,p.pos.a+M_PI_2, 0.7,0.2,0.5,1.0,marker_id,"road_network_vector_map", visualization_msgs::Marker::ARROW);
					mkr_l.scale.y = 0.2;
					mkr_l.scale.z = 0.2;
					mkr_l.color.a = 0.25;
					markerArray.markers.push_back(mkr_l);
				}

				if(p.RightPointId > 0)
				{
					marker_id++;
					mkr_l = PlannerHNS::ROSHelpers::CreateGenMarker(p.pos.x,p.pos.y,p.pos.z,p.pos.a-M_PI_2, 0.7,0.2,0.5,1.0,marker_id,"road_network_vector_map", visualization_msgs::Marker::ARROW);
					mkr_l.scale.y = 0.2;
					mkr_l.scale.z = 0.2;
					mkr_l.color.a = 0.25;
					markerArray.markers.push_back(mkr_l);
				}
			}
		}
	}

	visualization_msgs::Marker stop_line_marker;
		stop_line_marker.header.frame_id = "map";
		//stop_line_marker.header.stamp = ros::Time();
		stop_line_marker.ns = "road_network_stop_line";
		stop_line_marker.type = visualization_msgs::Marker::LINE_STRIP;
		stop_line_marker.action = visualization_msgs::Marker::ADD;
		stop_line_marker.scale.x = 0.25;
		roll_color.r = 1;
		roll_color.g = 1;
		roll_color.b = 1;
		roll_color.a = 0.5;
		stop_line_marker.color = roll_color;
		stop_line_marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);
		stop_line_marker.frame_locked = false;

	for(unsigned int sl = 0; sl < map.stopLines.size(); sl++)
	{
	  stop_line_marker.points.clear();
	  stop_line_marker.id = marker_id++;
	  for(unsigned int p = 0; p < map.stopLines.at(sl).points.size(); p++)
	  {
		geometry_msgs::Point point;
		point.x = map.stopLines.at(sl).points.at(p).pos.x;
		point.y = map.stopLines.at(sl).points.at(p).pos.y;
		point.z = map.stopLines.at(sl).points.at(p).pos.z;
		stop_line_marker.points.push_back(point);
	  }
	  markerArray.markers.push_back(stop_line_marker);
	}

	visualization_msgs::Marker boundary_marker;
	boundary_marker.header.frame_id = "map";
	//stop_line_marker.header.stamp = ros::Time();
	boundary_marker.ns = "road_network_boundaries";
	boundary_marker.type = visualization_msgs::Marker::LINE_STRIP;
	boundary_marker.action = visualization_msgs::Marker::ADD;
	boundary_marker.scale.x = 0.25;
	boundary_marker.frame_locked = false;
	boundary_marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);

	for(unsigned int ib = 0; ib< map.boundaries.size(); ib++)
	{
		if(map.boundaries.at(ib).type == PlannerHNS::PARKING_BOUNDARY)
		{
			boundary_marker.color.r = 1.0;
			boundary_marker.color.g = 0.2;
			boundary_marker.color.b = 0.2;
			boundary_marker.color.a = 0.5;
		}
		else
		{
			boundary_marker.color.r = 0.5;
			boundary_marker.color.g = 0.5;
			boundary_marker.color.b = 1.0;
			boundary_marker.color.a = 0.5;
		}
		boundary_marker.points.clear();
		boundary_marker.id = marker_id++;
	  for(int p = 0; p < map.boundaries.at(ib).points.size(); p++)
	  {
		  int curr_index = p;
			geometry_msgs::Point point;
			point.x = map.boundaries.at(ib).points.at(curr_index).pos.x;
			point.y = map.boundaries.at(ib).points.at(curr_index).pos.y;
			point.z = map.boundaries.at(ib).points.at(curr_index).pos.z;
			boundary_marker.points.push_back(point);
	  }
	  markerArray.markers.push_back(boundary_marker);
	}

	visualization_msgs::Marker curb_marker;
	curb_marker.header.frame_id = "map";
	//stop_line_marker.header.stamp = ros::Time();
	curb_marker.ns = "road_network_curbs";
	curb_marker.type = visualization_msgs::Marker::LINE_STRIP;
	curb_marker.action = visualization_msgs::Marker::ADD;
	curb_marker.scale.x = 0.25;
	curb_marker.frame_locked = false;
	curb_marker.color.r = 0.2;
	curb_marker.color.g = 0.2;
	curb_marker.color.b = 1.0;
	curb_marker.color.a = 0.5;
	curb_marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);

	for(unsigned int ic = 0; ic< map.curbs.size(); ic++)
	{
		if(map.curbs.at(ic).points.size() < 2)
			continue;

		curb_marker.points.clear();
		curb_marker.id = marker_id++;
	  for(unsigned int p = 0; p < map.curbs.at(ic).points.size(); p++)
	  {
		geometry_msgs::Point point;
		point.x = map.curbs.at(ic).points.at(p).pos.x;
		point.y = map.curbs.at(ic).points.at(p).pos.y;
		point.z = map.curbs.at(ic).points.at(p).pos.z;
		curb_marker.points.push_back(point);
	  }
	  markerArray.markers.push_back(curb_marker);
	}
}

void ROSHelpers::InitPredParticlesMarkers(const int& nMarkers, visualization_msgs::MarkerArray& paths, bool bOld )
{
	paths.markers.clear();
	for(int i=0; i<nMarkers; i++)
	{
		if(bOld)
		{
			visualization_msgs::Marker mkr = CreateGenMarker(0,0,0,0,1,1,1,0.05,i,"Particles", visualization_msgs::Marker::CUBE);
			mkr.scale.x = 0.2;
			paths.markers.push_back(mkr);
		}
		else
		{
			visualization_msgs::Marker mkr = CreateGenMarker(0,0,0,0,1,1,1,0.05,i,"Particles", visualization_msgs::Marker::ARROW);
			mkr.scale.x = 0.3;
			paths.markers.push_back(mkr);
		}
	}
}

void ROSHelpers::ConvertParticles(std::vector<PlannerHNS::WayPoint>& points, visualization_msgs::MarkerArray& part_mkrs, visualization_msgs::MarkerArray& part_markers_d, bool bOld)
{
	part_mkrs = part_markers_d;
	for(unsigned int i = 0; i < points.size(); i++)
	{
		visualization_msgs::Marker mkr;
		if(bOld)
		{
			if(points.at(i).bDir == PlannerHNS::STANDSTILL_DIR)
				mkr = CreateGenMarker(points.at(i).pos.x, points.at(i).pos.y,points.at(i).pos.z,points.at(i).pos.a,1,0,0,0.1,i,"Particles", visualization_msgs::Marker::CUBE);
			else if(points.at(i).bDir == PlannerHNS::FORWARD_DIR)
				mkr = CreateGenMarker(points.at(i).pos.x, points.at(i).pos.y,points.at(i).pos.z,points.at(i).pos.a,1,1,1,0.1,i,"Particles", visualization_msgs::Marker::CUBE);
			else if(points.at(i).bDir == PlannerHNS::FORWARD_RIGHT_DIR)
				mkr = CreateGenMarker(points.at(i).pos.x, points.at(i).pos.y,points.at(i).pos.z,points.at(i).pos.a,0,1,0,0.1,i,"Particles", visualization_msgs::Marker::CUBE);
			else if(points.at(i).bDir == PlannerHNS::FORWARD_LEFT_DIR)
				mkr = CreateGenMarker(points.at(i).pos.x, points.at(i).pos.y,points.at(i).pos.z,points.at(i).pos.a,0,0,1,0.1,i,"Particles", visualization_msgs::Marker::CUBE);
			else if(points.at(i).bDir == PlannerHNS::BACKWARD_DIR)
				mkr = CreateGenMarker(points.at(i).pos.x, points.at(i).pos.y,points.at(i).pos.z,points.at(i).pos.a,1,0,1,0.1,i,"Particles", visualization_msgs::Marker::CUBE);
			else
				mkr = CreateGenMarker(points.at(i).pos.x, points.at(i).pos.y,points.at(i).pos.z,points.at(i).pos.a,1,1,0,0.1,i,"Particles", visualization_msgs::Marker::CUBE);
		}
		else
		{
			if(points.at(i).bDir == PlannerHNS::STANDSTILL_DIR)
				mkr = CreateGenMarker(points.at(i).pos.x, points.at(i).pos.y,points.at(i).pos.z,points.at(i).pos.a,1,0,0,0.15,i,"Particles", visualization_msgs::Marker::ARROW);
			else if(points.at(i).bDir == PlannerHNS::FORWARD_DIR)
				mkr = CreateGenMarker(points.at(i).pos.x, points.at(i).pos.y,points.at(i).pos.z,points.at(i).pos.a,1,1,1,0.15,i,"Particles", visualization_msgs::Marker::ARROW);
			else if(points.at(i).bDir == PlannerHNS::FORWARD_RIGHT_DIR)
				mkr = CreateGenMarker(points.at(i).pos.x, points.at(i).pos.y,points.at(i).pos.z,points.at(i).pos.a,0,1,0,0.15,i,"Particles", visualization_msgs::Marker::ARROW);
			else if(points.at(i).bDir == PlannerHNS::FORWARD_LEFT_DIR)
				mkr = CreateGenMarker(points.at(i).pos.x, points.at(i).pos.y,points.at(i).pos.z,points.at(i).pos.a,0,0,1,0.15,i,"Particles", visualization_msgs::Marker::ARROW);
			else if(points.at(i).bDir == PlannerHNS::BACKWARD_DIR)
				mkr = CreateGenMarker(points.at(i).pos.x, points.at(i).pos.y,points.at(i).pos.z,points.at(i).pos.a,1,0,1,0.15,i,"Particles", visualization_msgs::Marker::ARROW);
			else
				mkr = CreateGenMarker(points.at(i).pos.x, points.at(i).pos.y,points.at(i).pos.z,points.at(i).pos.a,1,1,0,0.15,i,"Particles", visualization_msgs::Marker::ARROW);
		}

		//mkr.scale.x = 0.3;
		if(i < part_mkrs.markers.size())
			part_mkrs.markers.at(i) = mkr;
		else
			part_mkrs.markers.push_back(mkr);
	}
}

void ROSHelpers::ConvertFromPlannerHRectangleToAutowareRviz(const std::vector<PlannerHNS::GPSPoint>& safety_rect,
		visualization_msgs::Marker& marker)
{
	//if(safety_rect.size() != 4) return;

	visualization_msgs::Marker lane_waypoint_marker;
	lane_waypoint_marker.header.frame_id = "map";
	lane_waypoint_marker.header.stamp = ros::Time();
	lane_waypoint_marker.ns = "global_lane_array_marker";
	lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
	lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
	lane_waypoint_marker.scale.x = 0.2;
	//lane_waypoint_marker.scale.y = 0.2;
	//lane_waypoint_marker.scale.z = 0.1;
	lane_waypoint_marker.frame_locked = false;
	lane_waypoint_marker.color.r = 0.0;
	lane_waypoint_marker.color.g = 1.0;
	lane_waypoint_marker.color.b = 0.0;
	lane_waypoint_marker.color.a = 0.6;
	lane_waypoint_marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);

	for(unsigned int i = 0; i < safety_rect.size(); i++)
	{
		geometry_msgs::Point p;
		p.x = safety_rect.at(i).x;
		p.y = safety_rect.at(i).y;
		p.z = safety_rect.at(i).z;

		lane_waypoint_marker.points.push_back(p);
	}
	if(safety_rect.size() > 0)
	{
		geometry_msgs::Point p;
		p.x = safety_rect.at(0).x;
		p.y = safety_rect.at(0).y;
		p.z = safety_rect.at(0).z;
		lane_waypoint_marker.points.push_back(p);
	}

//	geometry_msgs::Point p1, p2,p3,p4;
//	p1.x = safety_rect.at(0).x;
//	p1.y = safety_rect.at(0).y;
//	p1.z = safety_rect.at(0).z;
//
//	p2.x = safety_rect.at(1).x;
//	p2.y = safety_rect.at(1).y;
//	p2.z = safety_rect.at(1).z;
//
//	p3.x = safety_rect.at(2).x;
//	p3.y = safety_rect.at(2).y;
//	p3.z = safety_rect.at(2).z;
//
//	p4.x = safety_rect.at(3).x;
//	p4.y = safety_rect.at(3).y;
//	p4.z = safety_rect.at(3).z;
//
//	lane_waypoint_marker.points.push_back(p1);
//	lane_waypoint_marker.points.push_back(p2);
//	lane_waypoint_marker.points.push_back(p3);
//	lane_waypoint_marker.points.push_back(p4);
//	lane_waypoint_marker.points.push_back(p1);

	 marker = lane_waypoint_marker;

}

void ROSHelpers::TrajectoriesToMarkers(const std::vector<std::vector<std::vector<PlannerHNS::WayPoint> > >& paths, visualization_msgs::MarkerArray& markerArray)
{
	visualization_msgs::Marker lane_waypoint_marker;
	lane_waypoint_marker.header.frame_id = "map";
	lane_waypoint_marker.header.stamp = ros::Time();
	lane_waypoint_marker.ns = "global_lane_array_marker";
	lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
	lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
	lane_waypoint_marker.scale.x = 0.1;
	//lane_waypoint_marker.scale.y = 0.1;
	//lane_waypoint_marker.scale.z = 0.1;
	lane_waypoint_marker.frame_locked = false;
	std_msgs::ColorRGBA  default_color;
	default_color.r = 0;
	default_color.g = 1;
	default_color.b = 0;
	default_color.a = 0.8;
	lane_waypoint_marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);


	int count = 0;
	for (unsigned int il = 0; il < paths.size(); il++)
	{
		for (unsigned int i = 0; i < paths.at(il).size(); i++)
		{
			lane_waypoint_marker.points.clear();
			lane_waypoint_marker.colors.clear();
			lane_waypoint_marker.id = count;

			for (unsigned int j=0; j < paths.at(il).at(i).size(); j++)
			{
			  geometry_msgs::Point point;

			  point.x = paths.at(il).at(i).at(j).pos.x;
			  point.y = paths.at(il).at(i).at(j).pos.y;
			  point.z = paths.at(il).at(i).at(j).pos.z;

			  std_msgs::ColorRGBA  edge_color = default_color;
//			  if(paths.at(il).at(i).at(j).custom_type == CUSTOM_AVOIDANCE_ENABLED)
//			  {
//				  edge_color.r = 1;
//				  edge_color.g = 0.75;
//			  }

			  lane_waypoint_marker.colors.push_back(edge_color);
			  lane_waypoint_marker.points.push_back(point);
			}

			markerArray.markers.push_back(lane_waypoint_marker);
			count++;
		}
	}
}

void ROSHelpers::TrajectorySelectedToMarkers(const std::vector<PlannerHNS::WayPoint>& path, const double& r_path, const double& g_path,
		const double& b_path, const double& r_circle, const double& g_circle, const double& b_circle, const double& radius, visualization_msgs::MarkerArray& markerArray, int skip)
{
	visualization_msgs::Marker path_part, circle_part;

	int count = 0;
	path_part = CreateGenMarker(0,0,0,0,r_path,g_path,b_path,1.5,count,"Path_Part", visualization_msgs::Marker::LINE_STRIP);
	path_part.color.a = 0.75;
	for (unsigned int i = 0; i < path.size(); i++)
	{
		  geometry_msgs::Point point;
		  point.x = path.at(i).pos.x;
		  point.y = path.at(i).pos.y;
		  point.z = path.at(i).pos.z;


		  std_msgs::ColorRGBA  color = path_part.color;
		  color.b = 0;
		  color.g = (path.at(i).curvatureCost-0.75)*3.0;
		  color.r = 1.0 - ((path.at(i).curvatureCost-0.75)*3.0);

		  path_part.colors.push_back(color);
		  path_part.points.push_back(point);

		  i += skip;

//		  std::ostringstream str_out;
//		  str_out.precision(3);
//		  str_out << path.at(i).timeCost;
//			visualization_msgs::Marker txt_mkr = CreateGenMarker(point.x,point.y-0.5,point.z+0.1,0.4,0.8,0.8,0.8,0.5,count,"test_part", visualization_msgs::Marker::TEXT_VIEW_FACING);
//			txt_mkr.text = str_out.str();
//			markerArray.markers.push_back(txt_mkr);
//			count++;
	}


	markerArray.markers.push_back(path_part);
}

void ROSHelpers::TrajectorySelectedToCircles(const std::vector<PlannerHNS::WayPoint>& path, const double& r_path, const double& g_path,
		const double& b_path, const double& r_circle, const double& g_circle, const double& b_circle, const double& radius, visualization_msgs::MarkerArray& markerArray, int skip)
{
	visualization_msgs::Marker circle_part;

	int count = 0;
	std_msgs::ColorRGBA color;
	color.r = 0;
	color.g = 0;
	color.b = 0;
	color.a = 0.75;

	for (unsigned int i = 0; i < path.size(); i++)
	{
		  color.b = 0;
		  color.g = 1 - (path.at(i).curvatureCost*1.0);
		  color.r = path.at(i).curvatureCost*1.0;
		  count++;
		  CreateCircleMarker(path.at(i), radius, color.r, color.g, color.b, count, "circle_part", circle_part);
		  markerArray.markers.push_back(circle_part);
		  i += skip;
	}
}

void ROSHelpers::DrivingPathToMarkers(const std::vector<std::pair<PlannerHNS::WayPoint, PlannerHNS::PolygonShape> >& path, visualization_msgs::MarkerArray& markerArray)
{
	visualization_msgs::Marker path_line, car_rect;
	std::vector<geometry_msgs::Point> points_list;
	int counter = 1;
	markerArray.markers.clear();


	for(const auto& item: path)
	{
		geometry_msgs::Point point;
		point.x = item.first.pos.x;
		point.y = item.first.pos.y;
		point.z = item.first.pos.z;
		points_list.push_back(point);

		car_rect = CreateGenMarker(0,0,0,0, 0.9, 0.7, 0.5, 0.05, counter,"op_driving_path", visualization_msgs::Marker::LINE_STRIP);
		for(const auto& p: item.second.points)
		{
			geometry_msgs::Point car_point;
			car_point.x = p.x;
			car_point.y = p.y;
			car_point.z = p.z;
			car_rect.points.push_back(car_point);
		}

		if(item.second.points.size() > 0)
		{
			geometry_msgs::Point car_point;
			car_point.x = item.second.points.front().x;
			car_point.y = item.second.points.front().y;
			car_point.z = item.second.points.front().z;
			car_rect.points.push_back(car_point);
		}

		markerArray.markers.push_back(car_rect);
		counter++;
	}

	path_line = CreateGenMarker(0,0,0,0, 0.5, 0.7, 0.9, 0.1, counter,"op_driving_path", visualization_msgs::Marker::LINE_STRIP);
	path_line.points = points_list;
	markerArray.markers.push_back(path_line);
}

void ROSHelpers::TrajectoriesToColoredMarkers(const std::vector<std::vector<PlannerHNS::WayPoint> >& paths, const std::vector<PlannerHNS::TrajectoryCost>& traj_costs,const int& iClosest, visualization_msgs::MarkerArray& markerArray)
{
	visualization_msgs::Marker lane_waypoint_marker;
	lane_waypoint_marker.header.frame_id = "map";
	lane_waypoint_marker.header.stamp = ros::Time();
	lane_waypoint_marker.ns = "local_lane_array_marker_colored";
	lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
	lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
	lane_waypoint_marker.scale.x = 0.1;
	//lane_waypoint_marker.scale.y = 0.1;
	//lane_waypoint_marker.scale.z = 0.1;
	lane_waypoint_marker.color.a = 0.9;
	lane_waypoint_marker.color.r = 1.0;
	lane_waypoint_marker.color.g = 1.0;
	lane_waypoint_marker.color.b = 1.0;
	lane_waypoint_marker.frame_locked = false;
	lane_waypoint_marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);

	int count = markerArray.markers.size();
	for (unsigned int i = 0; i < paths.size(); i++)
	{
		lane_waypoint_marker.points.clear();
		lane_waypoint_marker.id = count;

		for (unsigned int j=0; j < paths.at(i).size(); j++)
		{
			geometry_msgs::Point point;

			point.x = paths.at(i).at(j).pos.x;
			point.y = paths.at(i).at(j).pos.y;
			point.z = paths.at(i).at(j).pos.z;

			lane_waypoint_marker.points.push_back(point);
		}

		lane_waypoint_marker.color.b = 0;

		if(traj_costs.size() == paths.size())
		{
			float norm_cost = traj_costs.at(i).cost * paths.size();
			if(norm_cost <= 1.0)
			{
				lane_waypoint_marker.color.r = norm_cost;
				lane_waypoint_marker.color.g = 1.0;
			}
			else if(norm_cost > 1.0)
			{
				lane_waypoint_marker.color.r = 1.0;
				lane_waypoint_marker.color.g = 2.0 - norm_cost;
			}
		}
		else
		{
			lane_waypoint_marker.color.r = 1.0;
			lane_waypoint_marker.color.g = 0.0;
		}

		if(traj_costs.at(i).bBlocked)
		{
			lane_waypoint_marker.color.r = 1.0;
			lane_waypoint_marker.color.g = 0.0;
			lane_waypoint_marker.color.b = 0.0;
		}

		if((int)i == iClosest)
		{
			lane_waypoint_marker.color.r = 1.0;
			lane_waypoint_marker.color.g = 0.0;
			lane_waypoint_marker.color.b = 1.0;
		}

		markerArray.markers.push_back(lane_waypoint_marker);
		count++;
	}
}

void ROSHelpers::ConvertFromPlannerHToAutowareVisualizePathFormat(const std::vector<PlannerHNS::WayPoint>& curr_path,
		const std::vector<std::vector<std::vector<PlannerHNS::WayPoint> > >& paths, const PlannerHNS::LocalPlannerH& localPlanner,
			visualization_msgs::MarkerArray& markerArray)
{
	visualization_msgs::Marker lane_waypoint_marker;
	lane_waypoint_marker.header.frame_id = "map";
	lane_waypoint_marker.header.stamp = ros::Time();
	lane_waypoint_marker.ns = "global_lane_array_marker";
	lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
	lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
	lane_waypoint_marker.scale.x = 0.1;
	lane_waypoint_marker.scale.y = 0.1;
	//lane_waypoint_marker.scale.z = 0.1;
	lane_waypoint_marker.frame_locked = false;
	std_msgs::ColorRGBA  total_color, curr_color;


	int count = 0;
	for (unsigned int il = 0; il < paths.size(); il++)
	{
		for (unsigned int i = 0; i < paths.at(il).size(); i++)
		{
			lane_waypoint_marker.points.clear();
			lane_waypoint_marker.id = count;

			for (unsigned int j=0; j < paths.at(il).at(i).size(); j++)
			{
			  geometry_msgs::Point point;

			  point.x = paths.at(il).at(i).at(j).pos.x;
			  point.y = paths.at(il).at(i).at(j).pos.y;
			  point.z = paths.at(il).at(i).at(j).pos.z;

			  lane_waypoint_marker.points.push_back(point);
			}

			lane_waypoint_marker.color.a = 0.9;
			if(localPlanner.m_TrajectoryCostsCalculatotor.m_TrajectoryCosts.size() == paths.size())
			{
				float norm_cost = localPlanner.m_TrajectoryCostsCalculatotor.m_TrajectoryCosts.at(i).cost * paths.size();
				if(norm_cost <= 1.0)
				{
					lane_waypoint_marker.color.r = norm_cost;
					lane_waypoint_marker.color.g = 1.0;
				}
				else if(norm_cost > 1.0)
				{
					lane_waypoint_marker.color.r = 1.0;
					lane_waypoint_marker.color.g = 2.0 - norm_cost;
				}
			}
			else
			{
				lane_waypoint_marker.color.r = 0.0;
				lane_waypoint_marker.color.g = 1.0;
			}

			if((int)i == localPlanner.m_iSafeTrajectory && (int)il == localPlanner.m_iCurrentTotalPathId)
			{
				lane_waypoint_marker.color.r = 1.0;
				lane_waypoint_marker.color.g = 0.0;
				lane_waypoint_marker.color.b = 1.0;
			}
			else
			{
				lane_waypoint_marker.color.b = 0;
			}

			markerArray.markers.push_back(lane_waypoint_marker);
			count++;
		}
	}
}

void ROSHelpers::ConvertFromPlannerHToAutowareVisualizePathFormat(const std::vector<std::vector<PlannerHNS::WayPoint> >& globalPaths, visualization_msgs::MarkerArray& markerArray)
{
	visualization_msgs::Marker lane_waypoint_marker;
	lane_waypoint_marker.header.frame_id = "map";
	lane_waypoint_marker.header.stamp = ros::Time();
	lane_waypoint_marker.ns = "global_lane_array_marker";
	lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
	lane_waypoint_marker.action = visualization_msgs::Marker::ADD;


	std_msgs::ColorRGBA roll_color, total_color, curr_color;
	lane_waypoint_marker.points.clear();
	lane_waypoint_marker.id = 1;
	lane_waypoint_marker.scale.x = 0.1;
	lane_waypoint_marker.scale.y = 0.1;
	total_color.r = 1;
	total_color.g = 0;
	total_color.b = 0;
	total_color.a = 0.5;
	lane_waypoint_marker.color = total_color;
	lane_waypoint_marker.frame_locked = false;

	int count = 0;
	for (unsigned int i = 0; i < globalPaths.size(); i++)
	{
		lane_waypoint_marker.points.clear();
		lane_waypoint_marker.id = count;

		for (unsigned int j=0; j < globalPaths.at(i).size(); j++)
		{
		  geometry_msgs::Point point;

		  point.x = globalPaths.at(i).at(j).pos.x;
		  point.y = globalPaths.at(i).at(j).pos.y;
		  point.z = globalPaths.at(i).at(j).pos.z;

		  lane_waypoint_marker.points.push_back(point);
		}

		markerArray.markers.push_back(lane_waypoint_marker);
		count++;
	}
}

void ROSHelpers::ConvertFromPlannerObstaclesToAutoware(const PlannerHNS::WayPoint& currState, const std::vector<PlannerHNS::DetectedObject>& trackedObstacles,
		visualization_msgs::MarkerArray& detectedPolygons)
{
	visualization_msgs::Marker lane_waypoint_marker;
	lane_waypoint_marker.header.frame_id = "map";
	lane_waypoint_marker.header.stamp = ros::Time();
	lane_waypoint_marker.ns = "detected_polygons";
	lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
	lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
	lane_waypoint_marker.scale.x = .1;
	lane_waypoint_marker.scale.y = .1;
	//lane_waypoint_marker.scale.z = .05;
	lane_waypoint_marker.color.a = 0.8;
	lane_waypoint_marker.frame_locked = false;

	visualization_msgs::Marker corner_marker;
	corner_marker.header.frame_id = "map";
	corner_marker.header.stamp = ros::Time();
	corner_marker.ns = "Polygon_Corners";
	corner_marker.type = visualization_msgs::Marker::SPHERE;
	corner_marker.action = visualization_msgs::Marker::ADD;
	corner_marker.scale.x = .1;
	corner_marker.scale.y = .1;
	corner_marker.scale.z = .1;
	corner_marker.color.a = 0.8;
	corner_marker.frame_locked = false;


	visualization_msgs::Marker quarters_marker;
	quarters_marker.header.frame_id = "map";
	quarters_marker.header.stamp = ros::Time();
	quarters_marker.ns = "Quarters_Lines";
	quarters_marker.type = visualization_msgs::Marker::LINE_STRIP;
	quarters_marker.action = visualization_msgs::Marker::ADD;
	quarters_marker.scale.x = .03;
	quarters_marker.scale.y = .03;
	quarters_marker.scale.z = .03;
	quarters_marker.color.a = 0.8;
	quarters_marker.color.r = 0.6;
	quarters_marker.color.g = 0.5;
	quarters_marker.color.b = 0;
	quarters_marker.frame_locked = false;

	visualization_msgs::Marker direction_marker;
	direction_marker.header.frame_id = "map";
	direction_marker.header.stamp = ros::Time();
	direction_marker.ns = "Object_Direction";
	direction_marker.type = visualization_msgs::Marker::ARROW;
	direction_marker.action = visualization_msgs::Marker::ADD;
	direction_marker.scale.x = .9;
	direction_marker.scale.y = .4;
	direction_marker.scale.z = .4;
	direction_marker.color.a = 0.8;
	direction_marker.color.r = 0;
	direction_marker.color.g = 1;
	direction_marker.color.b = 0;
	direction_marker.frame_locked = false;


	visualization_msgs::Marker velocity_marker;
	velocity_marker.header.frame_id = "map";
	velocity_marker.header.stamp = ros::Time();
	velocity_marker.ns = "detected_polygons_velocity";
	velocity_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	//velocity_marker.action = visualization_msgs::Marker::ADD;
	velocity_marker.scale.z = 0.9;
	velocity_marker.scale.x = 0.9;
	velocity_marker.scale.y = 0.9;
	velocity_marker.color.a = 0.5;

	velocity_marker.frame_locked = false;
	detectedPolygons.markers.clear();

	int pointID = 0;
	int quartersIds = 0;
	for(unsigned int i =0; i < trackedObstacles.size(); i++)
	{
		//double distance = hypot(currState.pos.y-trackedObstacles.at(i).center.pos.y, currState.pos.x-trackedObstacles.at(i).center.pos.x);

		lane_waypoint_marker.color.g = 0;
		lane_waypoint_marker.color.r = 0;
		lane_waypoint_marker.color.b = 1;

		velocity_marker.color.r = 1;//trackedObstacles.at(i).center.v/16.0;
		velocity_marker.color.g = 1;// - trackedObstacles.at(i).center.v/16.0;
		velocity_marker.color.b = 1;

		lane_waypoint_marker.points.clear();
		lane_waypoint_marker.id = i;
		velocity_marker.id = i;

		//std::cout << " Distance : " << distance << ", Of Object" << trackedObstacles.at(i).id << std::endl;

		for(unsigned int p = 0; p < trackedObstacles.at(i).contour.size(); p++)
		{

			geometry_msgs::Point point;

			  point.x = trackedObstacles.at(i).contour.at(p).x;
			  point.y = trackedObstacles.at(i).contour.at(p).y;
			  point.z = trackedObstacles.at(i).contour.at(p).z;

			  lane_waypoint_marker.points.push_back(point);

			  corner_marker.pose.position = point;
			  corner_marker.color.r = 0.8;
			  corner_marker.color.g = 0;
			  corner_marker.color.b = 0.7;
			  corner_marker.color.a = 0.5;
			  corner_marker.id = pointID;
			  pointID++;

			  detectedPolygons.markers.push_back(corner_marker);
		}

		if(trackedObstacles.at(i).contour.size()>0)
		{
		geometry_msgs::Point point;

		  point.x = trackedObstacles.at(i).contour.at(0).x;
		  point.y = trackedObstacles.at(i).contour.at(0).y;
		  point.z = trackedObstacles.at(i).contour.at(0).z+1;

		  lane_waypoint_marker.points.push_back(point);

		}


		geometry_msgs::Point point;

		point.x = trackedObstacles.at(i).center.pos.x;
		point.y = trackedObstacles.at(i).center.pos.y;
		point.z = trackedObstacles.at(i).center.pos.z+1;

//		geometry_msgs::Point relative_p;
		//relative_p.y = 0.5;
//		velocity_marker.pose.position = calcAbsoluteCoordinate(relative_p, point);
		velocity_marker.pose.position = point;
	    velocity_marker.pose.position.z += 0.5;

	    direction_marker.id = i;
	    direction_marker.pose.position = point;
	    direction_marker.pose.position.z += 0.5;
	    direction_marker.pose.orientation = tf::createQuaternionMsgFromYaw(UtilityHNS::UtilityH::SplitPositiveAngle(trackedObstacles.at(i).center.pos.a));


		for(unsigned int iq = 0; iq < 8; iq++)
		{
			quarters_marker.points.clear();
			quarters_marker.id = quartersIds;
			quarters_marker.points.push_back(point);
			geometry_msgs::Point point2 = point;
			double a_q = UtilityHNS::UtilityH::SplitPositiveAngle(trackedObstacles.at(i).center.pos.a+(iq*M_PI_4));
			point2.x += 2.0*cos(a_q);
			point2.y += 1.5*sin(a_q);
			quarters_marker.points.push_back(point2);

			quartersIds++;
			detectedPolygons.markers.push_back(quarters_marker);
		}

		int speed = (trackedObstacles.at(i).center.v*3.6);

	  // double to string
	  std::ostringstream str_out;
	//  if(trackedObstacles.at(i).center.v > 0.75)
	  str_out << "(" << trackedObstacles.at(i).id << " , " << speed << ")";
//	  else
//		  str_out << trackedObstacles.at(i).id;
	  //std::string vel = str_out.str();
	  velocity_marker.text = str_out.str();//vel.erase(vel.find_first_of(".") + 2);
	  //if(speed > 0.5)

	  detectedPolygons.markers.push_back(velocity_marker);
	  detectedPolygons.markers.push_back(lane_waypoint_marker);
	  detectedPolygons.markers.push_back(direction_marker);

	}
}

std::string ROSHelpers::GetBehaviorNameFromCode(const PlannerHNS::STATE_TYPE& behState)
{
	std::string str = "Unknown";
	switch(behState)
	{
	case PlannerHNS::INITIAL_STATE:
		str = "Init";
		break;
	case PlannerHNS::WAITING_STATE:
		str = "Waiting";
		break;
	case PlannerHNS::FORWARD_STATE:
		str = "Forward";
		break;
	case PlannerHNS::STOPPING_STATE:
		str = "Stop";
		break;
	case PlannerHNS::EMERGENCY_STATE:
		str = "Emergency";
		break;
	case PlannerHNS::LANE_CHANGE_STATE:
		str = "Lane Change";
		break;
	case PlannerHNS::FINISH_STATE:
		str = "End";
		break;
	case PlannerHNS::FOLLOW_STATE:
		str = "Follow";
		break;
	case PlannerHNS::OBSTACLE_AVOIDANCE_STATE:
		str = "Swerving";
		break;
	case PlannerHNS::TRAFFIC_LIGHT_STOP_STATE:
		str = "Light Stop";
		break;
	case PlannerHNS::TRAFFIC_LIGHT_WAIT_STATE:
		str = "Light Wait";
		break;
	case PlannerHNS::STOP_SIGN_STOP_STATE:
		str = "Sign Stop";
		break;
	case PlannerHNS::STOP_SIGN_WAIT_STATE:
		str = "Sign Wait";
		break;

	case PlannerHNS::GOAL_STATE:
		str = "Goal Achieved";
		break;
	case PlannerHNS::YIELDING_STATE:
		str = "Yielding";
		break;
	case PlannerHNS::BRANCH_LEFT_STATE:
		str = "Turning Left";
		break;
	case PlannerHNS::BRANCH_RIGHT_STATE:
		str = "Turning Right";
		break;
	default:
		str = "Unknown";
		break;
	}

	return str;
}

void ROSHelpers::VisualizeBehaviorState(const PlannerHNS::WayPoint& currState, const PlannerHNS::BehaviorState& beh, const bool& bGreenLight, const int& avoidDirection, visualization_msgs::Marker& behaviorMarker, std::string ns,double size_factor)
{
	behaviorMarker.header.frame_id = "map";
	behaviorMarker.header.stamp = ros::Time();
	behaviorMarker.ns = ns;
	behaviorMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	behaviorMarker.scale.z = 1.0*size_factor;
	//behaviorMarker.scale.x = 1.0*size_factor;
	//behaviorMarker.scale.y = 1.0*size_factor;
	behaviorMarker.color.a = 0.9;
	behaviorMarker.frame_locked = false;
	if(bGreenLight)
	{
		behaviorMarker.color.r = 0.1;//trackedObstacles.at(i).center.v/16.0;
		behaviorMarker.color.g = 1;// - trackedObstacles.at(i).center.v/16.0;
		behaviorMarker.color.b = 0.1;
	}
	else
	{
		behaviorMarker.color.r = 1;//trackedObstacles.at(i).center.v/16.0;
		behaviorMarker.color.g = 0.1;// - trackedObstacles.at(i).center.v/16.0;
		behaviorMarker.color.b = 0.1;
	}

	behaviorMarker.id = 0;

	geometry_msgs::Point point;

	point.x = currState.pos.x;
	point.y = currState.pos.y;
	point.z = currState.pos.z+2.0;

	behaviorMarker.pose.position = point;

	std::ostringstream str_out;

	//str_out << "(" << (int)(beh.followDistance * 100) / 100 <<")";
	str_out << "(" << (int)(beh.stopDistance * 100.0) / 100.0 <<")";

	if(avoidDirection == -1)
		str_out << "<< ";

	str_out << GetBehaviorNameFromCode(beh.state);
	if(avoidDirection == 1)
		str_out << " >>";
	behaviorMarker.text = str_out.str();
}

void ROSHelpers::VisualizeIntentionState(const PlannerHNS::WayPoint& currState, const PlannerHNS::BEH_STATE_TYPE& beh, visualization_msgs::Marker& behaviorMarker, std::string ns,double size_factor)
{
	behaviorMarker.header.frame_id = "map";
	behaviorMarker.header.stamp = ros::Time();
	behaviorMarker.ns = ns;
	behaviorMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	behaviorMarker.scale.z = 1.0*size_factor;
//	behaviorMarker.scale.x = 1.0*size_factor;
//	behaviorMarker.scale.y = 1.0*size_factor;
	behaviorMarker.color.a = 0.9;
	behaviorMarker.frame_locked = false;

	behaviorMarker.color.r = 1;
	behaviorMarker.color.g = 0.5;
	behaviorMarker.color.b = 0.1;


	behaviorMarker.id = 0;

	geometry_msgs::Point point;

	point.x = currState.pos.x;
	point.y = currState.pos.y;
	point.z = currState.pos.z+3.0;

	behaviorMarker.pose.position = point;

	std::ostringstream str_out;

	if(beh == BEH_STOPPING_STATE)
		str_out << "Stopping";
	else if(beh == BEH_FORWARD_STATE)
		str_out << "Forward";
	else if(beh == BEH_YIELDING_STATE)
		str_out << "Yielding";
	else if(beh == BEH_PARKING_STATE)
		str_out << "Parking";
	else
		str_out << "Unknown!";

	behaviorMarker.text = str_out.str();
}

void ROSHelpers::ConvertFromAutowareBoundingBoxObstaclesToPlannerH(const jsk_recognition_msgs::BoundingBoxArray& detectedObstacles,
		std::vector<PlannerHNS::DetectedObject>& obstacles_list)
{
	obstacles_list.clear();
	for(unsigned int i =0; i < detectedObstacles.boxes.size(); i++)
	{
		PlannerHNS::DetectedObject obj;
		obj.center = PlannerHNS::WayPoint(detectedObstacles.boxes.at(i).pose.position.x,
				detectedObstacles.boxes.at(i).pose.position.y,
				0,
				0);
		obj.w = detectedObstacles.boxes.at(i).dimensions.y;
		obj.l = detectedObstacles.boxes.at(i).dimensions.x;
		obj.h = detectedObstacles.boxes.at(i).dimensions.z;
		//double objSize = obj.w*obj.l;
		//double d = hypot(m_State.state.pos.y - obj.center.pos.y, m_State.state.pos.x - obj.center.pos.x);
		//std::cout << ", Distance of  : " << d;
		//if(d < 7)
		{

			double l2 = obj.l/2.0;
			double w2 = obj.w/2.0;

			obj.contour.push_back(PlannerHNS::GPSPoint(-w2, -l2, 0,0));
			obj.contour.push_back(PlannerHNS::GPSPoint(w2, -l2, 0,0));
			obj.contour.push_back(PlannerHNS::GPSPoint(w2, l2, 0,0));
			obj.contour.push_back(PlannerHNS::GPSPoint(-w2, l2, 0,0));
			obstacles_list.push_back(obj);
		}
	}
}

void ROSHelpers::ConvertFromAutowareCloudClusterObstaclesToPlannerH(const PlannerHNS::WayPoint& currState, const double& car_width,
		const double& car_length, const autoware_msgs::CloudClusterArray& clusters, vector<PlannerHNS::DetectedObject>& obstacles_list,
		const double max_obj_size, const double& min_obj_size, const double& detection_radius,
		const int& n_poly_quarters,const double& poly_resolution, int& nOriginalPoints, int& nContourPoints)
{
	PlannerHNS::Mat3 rotationMat(-currState.pos.a);
	PlannerHNS::Mat3 translationMat(-currState.pos.x, -currState.pos.y);

	int nPoints = 0;
	int nOrPoints = 0;
	double object_size = 0;
	PlannerHNS::GPSPoint relative_point;
	PlannerHNS::GPSPoint avg_center;
	PolygonGenerator polyGen(n_poly_quarters);
	PlannerHNS::DetectedObject obj;

	for(unsigned int i =0; i < clusters.clusters.size(); i++)
	{
		obj.id = clusters.clusters.at(i).id;
		obj.label = clusters.clusters.at(i).label;

		obj.center.pos.x = clusters.clusters.at(i).centroid_point.point.x;
		obj.center.pos.y = clusters.clusters.at(i).centroid_point.point.y;
		obj.center.pos.z = clusters.clusters.at(i).centroid_point.point.z;
		obj.center.pos.a = 0;
		obj.center.v = 0;
		obj.actual_yaw = clusters.clusters.at(i).estimated_angle;

		obj.w = clusters.clusters.at(i).dimensions.x;
		obj.l = clusters.clusters.at(i).dimensions.y;
		obj.h = clusters.clusters.at(i).dimensions.z;

		pcl::PointCloud<pcl::PointXYZ> point_cloud;
		pcl::fromROSMsg(clusters.clusters.at(i).cloud, point_cloud);


		obj.contour = polyGen.EstimateClusterPolygon(point_cloud ,obj.center.pos,avg_center, poly_resolution);

		obj.distance_to_center = hypot(obj.center.pos.y-currState.pos.y, obj.center.pos.x-currState.pos.x);

		object_size = hypot(obj.w, obj.l);

		if(obj.distance_to_center > detection_radius || object_size < min_obj_size || object_size > max_obj_size)
			continue;

		relative_point = translationMat*obj.center.pos;
		relative_point = rotationMat*relative_point;

		double distance_x = fabs(relative_point.x - car_length/3.0);
		double distance_y = fabs(relative_point.y);

		if(distance_x  <= car_length*0.5 && distance_y <= car_width*0.5) // don't detect yourself
			continue;

		//obj.center.pos = avg_center;
		nOrPoints += point_cloud.points.size();
		nPoints += obj.contour.size();
		//std::cout << " Distance_X: " << distance_x << ", " << " Distance_Y: " << distance_y << ", " << " Size: " << object_size << std::endl;
		obstacles_list.push_back(obj);
	}

	nOriginalPoints = nOrPoints;
	nContourPoints =  nPoints;
}

//PlannerHNS::SHIFT_POS ROSHelpers::ConvertShiftFromAutowareToPlannerH(const PlannerHNS::AUTOWARE_SHIFT_POS& shift)
//{
//	if(shift == PlannerHNS::AW_SHIFT_POS_DD)
//		return PlannerHNS::SHIFT_POS_DD;
//	else if(shift == PlannerHNS::AW_SHIFT_POS_RR)
//		return PlannerHNS::SHIFT_POS_RR;
//	else if(shift == PlannerHNS::AW_SHIFT_POS_NN)
//		return PlannerHNS::SHIFT_POS_NN;
//	else if(shift == PlannerHNS::AW_SHIFT_POS_PP)
//		return PlannerHNS::SHIFT_POS_PP;
//	else if(shift == PlannerHNS::AW_SHIFT_POS_BB)
//		return PlannerHNS::SHIFT_POS_BB;
//	else if(shift == PlannerHNS::AW_SHIFT_POS_SS)
//		return PlannerHNS::SHIFT_POS_SS;
//	else
//		return PlannerHNS::SHIFT_POS_UU;
//}
//
//PlannerHNS::AUTOWARE_SHIFT_POS ROSHelpers::ConvertShiftFromPlannerHToAutoware(const PlannerHNS::SHIFT_POS& shift)
//{
//	if(shift == PlannerHNS::SHIFT_POS_DD)
//		return PlannerHNS::AW_SHIFT_POS_DD;
//	else if(shift == PlannerHNS::SHIFT_POS_RR)
//		return PlannerHNS::AW_SHIFT_POS_RR;
//	else if(shift == PlannerHNS::SHIFT_POS_NN)
//		return PlannerHNS::AW_SHIFT_POS_NN;
//	else if(shift == PlannerHNS::SHIFT_POS_PP)
//		return PlannerHNS::AW_SHIFT_POS_PP;
//	else if(shift == PlannerHNS::SHIFT_POS_BB)
//		return PlannerHNS::AW_SHIFT_POS_BB;
//	else if(shift == PlannerHNS::SHIFT_POS_SS)
//		return PlannerHNS::AW_SHIFT_POS_SS;
//	else
//		return PlannerHNS::AW_SHIFT_POS_UU;
//}

//PlannerHNS::AutowareBehaviorState ROSHelpers::ConvertBehaviorStateFromPlannerHToAutoware(const PlannerHNS::BehaviorState& beh)
//{
//	PlannerHNS::AutowareBehaviorState arw_state;
//	arw_state.followDistance = beh.followDistance;
//	arw_state.followVelocity = beh.followVelocity;
//	arw_state.maxVelocity = beh.maxVelocity;
//	arw_state.minVelocity = beh.minVelocity;
//	arw_state.stopDistance = beh.stopDistance;
//
//	if(beh.indicator == PlannerHNS::LIGHT_INDICATOR::INDICATOR_LEFT)
//		arw_state.indicator = PlannerHNS::AW_INDICATOR_LEFT;
//	else if(beh.indicator == PlannerHNS::LIGHT_INDICATOR::INDICATOR_RIGHT)
//		arw_state.indicator = PlannerHNS::AW_INDICATOR_RIGHT;
//	else if(beh.indicator == PlannerHNS::LIGHT_INDICATOR::INDICATOR_BOTH)
//		arw_state.indicator = PlannerHNS::AW_INDICATOR_BOTH;
//	else if(beh.indicator == PlannerHNS::LIGHT_INDICATOR::INDICATOR_NONE)
//		arw_state.indicator = PlannerHNS::AW_INDICATOR_NONE;
//
//	if(beh.state == PlannerHNS::INITIAL_STATE)
//		arw_state.state = PlannerHNS::AW_INITIAL_STATE;
//	else if(beh.state == PlannerHNS::WAITING_STATE)
//		arw_state.state = PlannerHNS::AW_WAITING_STATE;
//	else if(beh.state == PlannerHNS::FORWARD_STATE)
//		arw_state.state = PlannerHNS::AW_FORWARD_STATE;
//	else if(beh.state == PlannerHNS::STOPPING_STATE)
//		arw_state.state = PlannerHNS::AW_STOPPING_STATE;
//	else if(beh.state == PlannerHNS::EMERGENCY_STATE)
//		arw_state.state = PlannerHNS::AW_EMERGENCY_STATE;
//	else if(beh.state == PlannerHNS::TRAFFIC_LIGHT_STOP_STATE)
//		arw_state.state = PlannerHNS::AW_TRAFFIC_LIGHT_STOP_STATE;
//	else if(beh.state == PlannerHNS::STOP_SIGN_STOP_STATE)
//		arw_state.state = PlannerHNS::AW_STOP_SIGN_STOP_STATE;
//	else if(beh.state == PlannerHNS::FOLLOW_STATE)
//		arw_state.state = PlannerHNS::AW_FOLLOW_STATE;
//	else if(beh.state == PlannerHNS::LANE_CHANGE_STATE)
//		arw_state.state = PlannerHNS::AW_LANE_CHANGE_STATE;
//	else if(beh.state == PlannerHNS::OBSTACLE_AVOIDANCE_STATE)
//		arw_state.state = PlannerHNS::AW_OBSTACLE_AVOIDANCE_STATE;
//	else if(beh.state == PlannerHNS::FINISH_STATE)
//		arw_state.state = PlannerHNS::AW_FINISH_STATE;
//
//
//	return arw_state;
//
//}

//PlannerHNS::BehaviorState ROSHelpers::ConvertBehaviorStateFromAutowareToPlannerH(const geometry_msgs::TwistStampedConstPtr& msg)
//{
//	PlannerHNS::BehaviorState behavior;
//	behavior.followDistance = msg->twist.linear.x;
//	behavior.stopDistance = msg->twist.linear.y;
//	behavior.followVelocity = msg->twist.angular.x;
//	behavior.maxVelocity = msg->twist.angular.y;
//
//
//	if(msg->twist.linear.z == PlannerHNS::LIGHT_INDICATOR::INDICATOR_LEFT)
//		behavior.indicator = PlannerHNS::LIGHT_INDICATOR::INDICATOR_LEFT;
//	else if(msg->twist.linear.z == PlannerHNS::LIGHT_INDICATOR::INDICATOR_RIGHT)
//		behavior.indicator = PlannerHNS::LIGHT_INDICATOR::INDICATOR_RIGHT;
//	else if(msg->twist.linear.z == PlannerHNS::LIGHT_INDICATOR::INDICATOR_BOTH)
//		behavior.indicator = PlannerHNS::LIGHT_INDICATOR::INDICATOR_BOTH;
//	else if(msg->twist.linear.z == PlannerHNS::LIGHT_INDICATOR::INDICATOR_NONE)
//		behavior.indicator = PlannerHNS::LIGHT_INDICATOR::INDICATOR_NONE;
//
//	if(msg->twist.angular.z == PlannerHNS::INITIAL_STATE)
//		behavior.state = PlannerHNS::INITIAL_STATE;
//	else if(msg->twist.angular.z == PlannerHNS::WAITING_STATE)
//		behavior.state = PlannerHNS::WAITING_STATE;
//	else if(msg->twist.angular.z == PlannerHNS::FORWARD_STATE)
//		behavior.state = PlannerHNS::FORWARD_STATE;
//	else if(msg->twist.angular.z == PlannerHNS::STOPPING_STATE)
//		behavior.state = PlannerHNS::STOPPING_STATE;
//	else if(msg->twist.angular.z == PlannerHNS::EMERGENCY_STATE)
//		behavior.state = PlannerHNS::EMERGENCY_STATE;
//	else if(msg->twist.angular.z == PlannerHNS::TRAFFIC_LIGHT_STOP_STATE)
//		behavior.state = PlannerHNS::TRAFFIC_LIGHT_STOP_STATE;
//	else if(msg->twist.angular.z == PlannerHNS::STOP_SIGN_STOP_STATE)
//		behavior.state = PlannerHNS::STOP_SIGN_STOP_STATE;
//	else if(msg->twist.angular.z == PlannerHNS::STOP_SIGN_WAIT_STATE)
//		behavior.state = PlannerHNS::STOP_SIGN_WAIT_STATE;
//	else if(msg->twist.angular.z == PlannerHNS::FOLLOW_STATE)
//		behavior.state = PlannerHNS::FOLLOW_STATE;
//	else if(msg->twist.angular.z == PlannerHNS::LANE_CHANGE_STATE)
//		behavior.state = PlannerHNS::LANE_CHANGE_STATE;
//	else if(msg->twist.angular.z == PlannerHNS::OBSTACLE_AVOIDANCE_STATE)
//		behavior.state = PlannerHNS::OBSTACLE_AVOIDANCE_STATE;
//	else if(msg->twist.angular.z == PlannerHNS::FINISH_STATE)
//		behavior.state = PlannerHNS::FINISH_STATE;
//
//
//	return behavior;
//
//}

void ROSHelpers::ConvertFromLocalLaneToAutowareLane(const std::vector<PlannerHNS::WayPoint>& path, autoware_msgs::Lane& trajectory , const unsigned int& iStart)
{
	trajectory.waypoints.clear();

	for(unsigned int i = iStart; i < path.size(); i++)
	{
		autoware_msgs::Waypoint wp;

		wp.pose.pose.position.x = path.at(i).pos.x;
		wp.pose.pose.position.y = path.at(i).pos.y;
		wp.pose.pose.position.z = path.at(i).pos.z;
		wp.pose.pose.orientation = tf::createQuaternionMsgFromYaw(UtilityHNS::UtilityH::SplitPositiveAngle(path.at(i).pos.a));

		wp.twist.twist.linear.x = path.at(i).v;
		wp.lane_id = path.at(i).laneId;
		wp.stop_line_id = path.at(i).stopLineID;
		wp.left_lane_id = path.at(i).LeftPointId;
		wp.right_lane_id = path.at(i).RightPointId;
		wp.time_cost = path.at(i).timeCost;
		wp.change_flag = ceil(path.at(i).laneChangeCost);
		wp.gid = path.at(i).gid;
		wp.wpstate.event_state = path.at(i).custom_type;
		wp.cost = 0;
		if(path.at(i).actionCost.size()>0)
		{
			wp.direction = path.at(i).actionCost.at(0).first;
			wp.cost += path.at(i).actionCost.at(0).second;
		}

		trajectory.waypoints.push_back(wp);
	}
}

void ROSHelpers::ConvertFromLocalLaneToAutowareLane(const std::vector<PlannerHNS::GPSPoint>& path, autoware_msgs::Lane& trajectory)
{
	trajectory.waypoints.clear();

	for(unsigned int i=0; i < path.size(); i++)
	{
		autoware_msgs::Waypoint wp;
		wp.pose.pose.position.x = path.at(i).x;
		wp.pose.pose.position.y = path.at(i).y;
		wp.pose.pose.position.z = path.at(i).z;
		wp.pose.pose.orientation = tf::createQuaternionMsgFromYaw(UtilityHNS::UtilityH::SplitPositiveAngle(path.at(i).a));

		trajectory.waypoints.push_back(wp);
	}
}

PlannerHNS::PID_CONST ROSHelpers::GetPIDValues(const std::string& str_param)
{
	PlannerHNS::PID_CONST pid;

	std::vector<std::string> pid_list = PlannerHNS::MappingHelpers::SplitString(str_param, ",");
	if(pid_list.size() == 3)
	{
		pid.kP = atof(pid_list.at(0).c_str());
		pid.kI = atof(pid_list.at(1).c_str());
		pid.kD = atof(pid_list.at(2).c_str());
	}

	return pid;
}

void ROSHelpers::ConvertFromAutowareLaneToLocalLane(const autoware_msgs::Lane& trajectory, std::vector<PlannerHNS::WayPoint>& path)
{
	path.clear();

	for(unsigned int i=0; i < trajectory.waypoints.size(); i++)
	{
		PlannerHNS::WayPoint wp;
		wp.pos.x = trajectory.waypoints.at(i).pose.pose.position.x;
		wp.pos.y = trajectory.waypoints.at(i).pose.pose.position.y;
		wp.pos.z = trajectory.waypoints.at(i).pose.pose.position.z;
		try
		{
			wp.pos.a = tf::getYaw(trajectory.waypoints.at(i).pose.pose.orientation);
		}
		catch (exception& ex)
		{
			ROS_ERROR("%s", ex.what());
		}

		wp.v = trajectory.waypoints.at(i).twist.twist.linear.x;

		wp.gid = trajectory.waypoints.at(i).gid;
		wp.laneId = trajectory.waypoints.at(i).lane_id;
		wp.stopLineID = trajectory.waypoints.at(i).stop_line_id;
		wp.LeftPointId = trajectory.waypoints.at(i).left_lane_id;
		wp.RightPointId = trajectory.waypoints.at(i).right_lane_id;
		wp.timeCost = trajectory.waypoints.at(i).time_cost;
		wp.laneChangeCost = trajectory.waypoints.at(i).change_flag;

		if(trajectory.waypoints.at(i).wpstate.event_state == 0)
		{
			wp.custom_type = CUSTOM_AVOIDANCE_DISABLED;
		}
		else if(trajectory.waypoints.at(i).wpstate.event_state == 1)
		{
			wp.custom_type = CUSTOM_AVOIDANCE_ENABLED;
		}

		if(trajectory.waypoints.at(i).direction == 0)
			wp.bDir = PlannerHNS::FORWARD_DIR;
		else if(trajectory.waypoints.at(i).direction == 1)
			wp.bDir = PlannerHNS::FORWARD_LEFT_DIR;
		else if(trajectory.waypoints.at(i).direction == 2)
			wp.bDir = PlannerHNS::FORWARD_RIGHT_DIR;
		else if(trajectory.waypoints.at(i).direction == 3)
			wp.bDir = PlannerHNS::BACKWARD_DIR;
		else if(trajectory.waypoints.at(i).direction == 4)
			wp.bDir = PlannerHNS::BACKWARD_LEFT_DIR;
		else if(trajectory.waypoints.at(i).direction == 5)
			wp.bDir = PlannerHNS::BACKWARD_RIGHT_DIR;
		else if(trajectory.waypoints.at(i).direction == 6)
			wp.bDir = PlannerHNS::STANDSTILL_DIR;

		wp.cost = trajectory.waypoints.at(i).cost;

		path.push_back(wp);
	}
}

void ROSHelpers::createGlobalLaneArrayMarker(std_msgs::ColorRGBA color,
		const autoware_msgs::LaneArray &lane_waypoints_array, visualization_msgs::MarkerArray& markerArray)
{
  visualization_msgs::Marker lane_waypoint_marker;
  lane_waypoint_marker.header.frame_id = "map";
  lane_waypoint_marker.header.stamp = ros::Time();
  lane_waypoint_marker.ns = "global_lane_array_marker";
  lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
  lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
  lane_waypoint_marker.scale.x = 0.75;
  //lane_waypoint_marker.scale.y = 0.75;
  lane_waypoint_marker.color = color;
  lane_waypoint_marker.frame_locked = false;
  lane_waypoint_marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);

  int count = 0;
  for (unsigned int i=0; i<  lane_waypoints_array.lanes.size(); i++)
  {
    lane_waypoint_marker.points.clear();
    lane_waypoint_marker.id = count;

    for (unsigned int j=0; j < lane_waypoints_array.lanes.at(i).waypoints.size(); j++)
    {
      geometry_msgs::Point point;
      point = lane_waypoints_array.lanes.at(i).waypoints.at(j).pose.pose.position;
      lane_waypoint_marker.points.push_back(point);
    }
    markerArray.markers.push_back(lane_waypoint_marker);
    count++;
  }

}

void ROSHelpers::createGlobalLaneArrayVelocityMarker(const autoware_msgs::LaneArray &lane_waypoints_array
		, visualization_msgs::MarkerArray& markerArray)
{
  visualization_msgs::MarkerArray tmp_marker_array;
  // display by markers the velocity of each waypoint.
  visualization_msgs::Marker velocity_marker;
  velocity_marker.header.frame_id = "map";
  velocity_marker.header.stamp = ros::Time();
  velocity_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  velocity_marker.action = visualization_msgs::Marker::ADD;
  //velocity_marker.scale.x = 0.4;
  //velocity_marker.scale.y = 0.4;
  velocity_marker.scale.z = 0.4;
  velocity_marker.color.a = 0.9;
  velocity_marker.color.r = 1;
  velocity_marker.color.g = 1;
  velocity_marker.color.b = 1;
  velocity_marker.frame_locked = false;
  velocity_marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);

  int count = 1;
  for (unsigned int i=0; i<  lane_waypoints_array.lanes.size(); i++)
  {

	  std::ostringstream str_count;
	  str_count << count;
    velocity_marker.ns = "global_velocity_lane_" + str_count.str();
    for (unsigned int j=0; j < lane_waypoints_array.lanes.at(i).waypoints.size(); j++)
    {
      //std::cout << _waypoints[i].GetX() << " " << _waypoints[i].GetY() << " " << _waypoints[i].GetZ() << " " << _waypoints[i].GetVelocity_kmh() << std::endl;
      velocity_marker.id = j;
      geometry_msgs::Point relative_p;
      relative_p.y = 0.5;
      velocity_marker.pose.position = calcAbsoluteCoordinate(relative_p, lane_waypoints_array.lanes.at(i).waypoints.at(j).pose.pose);
      velocity_marker.pose.position.z += 0.2;

      // double to string
      std::ostringstream str_out;
      //str_out << lane_waypoints_array.lanes.at(i).waypoints.at(j).twist.twist.linear.x;
      str_out << j;
      //std::string vel = str_out.str();
      velocity_marker.text = str_out.str();//vel.erase(vel.find_first_of(".") + 2);

      tmp_marker_array.markers.push_back(velocity_marker);
    }
    count++;
  }

  markerArray.markers.insert(markerArray.markers.end(), tmp_marker_array.markers.begin(),
                                       tmp_marker_array.markers.end());
}

void ROSHelpers::createGlobalLaneArrayOrientationMarker(const autoware_msgs::LaneArray &lane_waypoints_array
		, visualization_msgs::MarkerArray& markerArray)
{
  visualization_msgs::MarkerArray tmp_marker_array;
  visualization_msgs::Marker lane_waypoint_marker;
  lane_waypoint_marker.header.frame_id = "map";
  lane_waypoint_marker.header.stamp = ros::Time();
  lane_waypoint_marker.type = visualization_msgs::Marker::ARROW;
  lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
  lane_waypoint_marker.scale.x = 0.6;
  lane_waypoint_marker.scale.y = 0.2;
  lane_waypoint_marker.scale.z = 0.1;
  lane_waypoint_marker.color.r = 1.0;
  lane_waypoint_marker.color.a = 1.0;
  //lane_waypoint_marker.frame_locked = false;

  lane_waypoint_marker.ns = "global_lane_waypoint_orientation_marker";

  int count = 1;
  for (unsigned int i=0; i<  lane_waypoints_array.lanes.size(); i++)
  {
//	  std::ostringstream str_ns;
//	  str_ns << "global_lane_waypoint_orientation_marker_";
//	  str_ns << i;
//	 lane_waypoint_marker.ns = str_ns.str();

    for (unsigned int j=0; j < lane_waypoints_array.lanes.at(i).waypoints.size(); j++)
    {
    	lane_waypoint_marker.id = count;
    	lane_waypoint_marker.pose = lane_waypoints_array.lanes.at(i).waypoints.at(j).pose.pose;

    	if(lane_waypoints_array.lanes.at(i).waypoints.at(j).dtlane.dir == 1)
    	{
    		lane_waypoint_marker.color.r = 0.0;
    		lane_waypoint_marker.color.g = 1.0;
    		lane_waypoint_marker.color.b = 0.0;
    		tmp_marker_array.markers.push_back(lane_waypoint_marker);
    	}
    	else if(lane_waypoints_array.lanes.at(i).waypoints.at(j).dtlane.dir == 2)
    	{
    		lane_waypoint_marker.color.r = 0.0;
			lane_waypoint_marker.color.g = 0.0;
			lane_waypoint_marker.color.b = 1.0;
			tmp_marker_array.markers.push_back(lane_waypoint_marker);
    	}
    	else
    	{

    		if(lane_waypoints_array.lanes.at(i).waypoints.at(j).cost >= 100)
    		{
    			lane_waypoint_marker.color.r = 1.0;
				lane_waypoint_marker.color.g = 0.0;
				lane_waypoint_marker.color.b = 0.0;
				tmp_marker_array.markers.push_back(lane_waypoint_marker);
    		}
    		else
    		{
				lane_waypoint_marker.color.r = 0.0;
				lane_waypoint_marker.color.g = 0.8;
				lane_waypoint_marker.color.b = 0.0;
				tmp_marker_array.markers.push_back(lane_waypoint_marker);
    		}
    	}


      count++;
    }
  }

  markerArray.markers.insert(markerArray.markers.end(), tmp_marker_array.markers.begin(),
										   tmp_marker_array.markers.end());
}

void ROSHelpers::GetTrafficLightForVisualization(std::vector<PlannerHNS::TrafficLight>& lights, visualization_msgs::MarkerArray& markerArray)
{
	markerArray.markers.clear();
	for(unsigned int i=0; i<lights.size(); i++)
	{
		if(lights.at(i).lightType == RED_LIGHT)
		{
			visualization_msgs::Marker mkr = CreateGenMarker(lights.at(i).pose.pos.x,lights.at(i).pose.pos.y,lights.at(i).pose.pos.z,0,1,0,0,3,i,"traffic_light_visualize", visualization_msgs::Marker::SPHERE);
			markerArray.markers.push_back(mkr);
		}
		else if(lights.at(i).lightType == GREEN_LIGHT)
		{
			visualization_msgs::Marker mkr = CreateGenMarker(lights.at(i).pose.pos.x,lights.at(i).pose.pos.y,lights.at(i).pose.pos.z,0,0,1,0,3,i,"traffic_light_visualize", visualization_msgs::Marker::SPHERE);
			markerArray.markers.push_back(mkr);
		}
	}
}

void ROSHelpers::ConvertFromAutowareDetectedObjectToOpenPlannerDetectedObject(const autoware_msgs::DetectedObject& det_obj, PlannerHNS::DetectedObject& obj)
{
	obj.id = det_obj.id;
	obj.label = det_obj.label;
	obj.l = det_obj.dimensions.y;
	obj.w = det_obj.dimensions.x;
	obj.h = det_obj.dimensions.z;

	obj.center.pos.x = det_obj.pose.position.x;
	obj.center.pos.y = det_obj.pose.position.y;
	obj.center.pos.z = det_obj.pose.position.z;
	try
	{
		obj.center.pos.a = tf::getYaw(det_obj.pose.orientation);
	}
	catch (exception& ex)
	{
		ROS_ERROR("%s", ex.what());
	}

	obj.center.v = det_obj.velocity.linear.x;
	obj.acceleration_raw = det_obj.velocity.linear.y;
	obj.acceleration_desc = det_obj.velocity.linear.z;
	obj.bVelocity = det_obj.velocity_reliable;
	obj.bDirection = det_obj.pose_reliable;

	if(det_obj.indicator_state == 0)
		obj.indicator_state = PlannerHNS::INDICATOR_LEFT;
	else if(det_obj.indicator_state == 1)
		obj.indicator_state = PlannerHNS::INDICATOR_RIGHT;
	else if(det_obj.indicator_state == 2)
		obj.indicator_state = PlannerHNS::INDICATOR_BOTH;
	else if(det_obj.indicator_state == 3)
		obj.indicator_state = PlannerHNS::INDICATOR_NONE;

	PlannerHNS::GPSPoint p;
	obj.contour.clear();

	for(unsigned int j=0; j < det_obj.convex_hull.polygon.points.size(); j++)
	{

		p.x = det_obj.convex_hull.polygon.points.at(j).x;
		p.y = det_obj.convex_hull.polygon.points.at(j).y;
		p.z = det_obj.convex_hull.polygon.points.at(j).z;
		obj.contour.push_back(p);
	}

	obj.predTrajectories.clear();

	for(unsigned int j = 0 ; j < det_obj.candidate_trajectories.lanes.size(); j++)
	{
		std::vector<PlannerHNS::WayPoint> _traj;
		ConvertFromAutowareLaneToLocalLane(det_obj.candidate_trajectories.lanes.at(j), _traj);
		for(unsigned int k=0; k < _traj.size(); k++)
		{
			_traj.at(k).collisionCost = det_obj.candidate_trajectories.lanes.at(j).cost;
		}

		obj.predTrajectories.push_back(_traj);
	}
}

void ROSHelpers::ConvertFromOpenPlannerDetectedObjectToAutowareDetectedObject(const PlannerHNS::DetectedObject& det_obj, const bool& bSimulationMode, autoware_msgs::DetectedObject& obj)
{

	if(bSimulationMode)
		obj.id = det_obj.originalID;
	else
		obj.id = det_obj.id;

	obj.valid = true;
	obj.label = det_obj.label;
	obj.indicator_state = det_obj.indicator_state;
	obj.dimensions.x = det_obj.w;
	obj.dimensions.y = det_obj.l;
	obj.dimensions.z = det_obj.h;

	obj.pose.position.x = det_obj.center.pos.x;
	obj.pose.position.y = det_obj.center.pos.y;
	obj.pose.position.z = det_obj.center.pos.z;
	obj.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, UtilityHNS::UtilityH::SplitPositiveAngle(det_obj.center.pos.a));
	obj.pose_reliable = true;

	obj.acceleration.linear.x = det_obj.acceleration_raw;
	obj.acceleration_reliable = true;

	obj.velocity.linear.x = det_obj.center.v;
	//obj.velocity.linear.y = det_obj.acceleration_raw;
	//obj.velocity.linear.z = det_obj.acceleration_desc;
	//obj.velocity_reliable = det_obj.bVelocity;
	//obj.pose_reliable = det_obj.bDirection;
	obj.velocity_reliable = true;


	geometry_msgs::Point32 p;
	obj.convex_hull.polygon.points.clear();

	for(unsigned int j=0; j < det_obj.contour.size(); j++)
	{
		p.x = det_obj.contour.at(j).x;
		p.y = det_obj.contour.at(j).y;
		p.z = det_obj.contour.at(j).z;
		obj.convex_hull.polygon.points.push_back(p);
	}


	obj.candidate_trajectories.lanes.clear();
	for(unsigned int j = 0 ; j < det_obj.predTrajectories.size(); j++)
	{
		autoware_msgs::Lane pred_traj;
		PlannerHNS::ROSHelpers::ConvertFromLocalLaneToAutowareLane(det_obj.predTrajectories.at(j), pred_traj);
		if(det_obj.predTrajectories.at(j).size() > 0)
		{
			pred_traj.cost = det_obj.predTrajectories.at(j).at(0).collisionCost;
		}
		pred_traj.lane_index = 0;
		obj.candidate_trajectories.lanes.push_back(pred_traj);
	}
}

autoware_msgs::Waypoint ROSHelpers::ConvertBehaviorStateToAutowareWaypoint(const PlannerHNS::BehaviorState& beh)
{
	autoware_msgs::Waypoint wp;
	wp.change_flag = beh.bNewPlan;
	wp.lid = beh.iTrajectory;
	wp.gid = beh.iLane;
	wp.cost = beh.followDistance;
	wp.time_cost = beh.followVelocity;
	wp.twist.twist.linear.x = beh.maxVelocity;
	wp.twist.twist.linear.y = beh.minVelocity;
	wp.pose.pose.position.x = beh.stopDistance;
	wp.direction = beh.indicator;
	wp.wpstate.event_state = beh.state;

	return wp;
}

PlannerHNS::BehaviorState ROSHelpers::ConvertAutowareWaypointToBehaviorState(const autoware_msgs::Waypoint& state_point)
{
	PlannerHNS::BehaviorState behavior;
	behavior.bNewPlan = state_point.change_flag;
	behavior.iTrajectory = state_point.lid;
	behavior.iLane = state_point.gid;
	behavior.followDistance = state_point.cost;
	behavior.followVelocity = state_point.time_cost;
	behavior.maxVelocity = state_point.twist.twist.linear.x;
	behavior.minVelocity = state_point.twist.twist.linear.y;
	behavior.stopDistance = state_point.pose.pose.position.x;

	if(state_point.direction == PlannerHNS::LIGHT_INDICATOR::INDICATOR_LEFT)
		behavior.indicator = PlannerHNS::LIGHT_INDICATOR::INDICATOR_LEFT;
	else if(state_point.direction == PlannerHNS::LIGHT_INDICATOR::INDICATOR_RIGHT)
		behavior.indicator = PlannerHNS::LIGHT_INDICATOR::INDICATOR_RIGHT;
	else if(state_point.direction == PlannerHNS::LIGHT_INDICATOR::INDICATOR_BOTH)
		behavior.indicator = PlannerHNS::LIGHT_INDICATOR::INDICATOR_BOTH;
	else if(state_point.direction == PlannerHNS::LIGHT_INDICATOR::INDICATOR_NONE)
		behavior.indicator = PlannerHNS::LIGHT_INDICATOR::INDICATOR_NONE;

	if(state_point.wpstate.event_state == PlannerHNS::INITIAL_STATE)
		behavior.state = PlannerHNS::INITIAL_STATE;
	else if(state_point.wpstate.event_state == PlannerHNS::WAITING_STATE)
		behavior.state = PlannerHNS::WAITING_STATE;
	else if(state_point.wpstate.event_state == PlannerHNS::FORWARD_STATE)
		behavior.state = PlannerHNS::FORWARD_STATE;
	else if(state_point.wpstate.event_state == PlannerHNS::STOPPING_STATE)
		behavior.state = PlannerHNS::STOPPING_STATE;
	else if(state_point.wpstate.event_state == PlannerHNS::EMERGENCY_STATE)
		behavior.state = PlannerHNS::EMERGENCY_STATE;
	else if(state_point.wpstate.event_state == PlannerHNS::TRAFFIC_LIGHT_STOP_STATE)
		behavior.state = PlannerHNS::TRAFFIC_LIGHT_STOP_STATE;
	else if(state_point.wpstate.event_state == PlannerHNS::STOP_SIGN_STOP_STATE)
		behavior.state = PlannerHNS::STOP_SIGN_STOP_STATE;
	else if(state_point.wpstate.event_state == PlannerHNS::STOP_SIGN_WAIT_STATE)
		behavior.state = PlannerHNS::STOP_SIGN_WAIT_STATE;
	else if(state_point.wpstate.event_state == PlannerHNS::FOLLOW_STATE)
		behavior.state = PlannerHNS::FOLLOW_STATE;
	else if(state_point.wpstate.event_state == PlannerHNS::LANE_CHANGE_STATE)
		behavior.state = PlannerHNS::LANE_CHANGE_STATE;
	else if(state_point.wpstate.event_state == PlannerHNS::OBSTACLE_AVOIDANCE_STATE)
		behavior.state = PlannerHNS::OBSTACLE_AVOIDANCE_STATE;
	else if(state_point.wpstate.event_state == PlannerHNS::FINISH_STATE)
		behavior.state = PlannerHNS::FINISH_STATE;


	return behavior;
}

void ROSHelpers::GetIndicatorArrows(const PlannerHNS::WayPoint& center, const double& width,const double& length, const PlannerHNS::LIGHT_INDICATOR& indicator, const int& id, visualization_msgs::MarkerArray& markerArray)
{
	double critical_lateral_distance =  width/2.0 + 0.2;
	//double critical_long_front_distance =  carInfo.length/2.0 ;
	PlannerHNS::GPSPoint top_right(critical_lateral_distance, length, center.pos.z, 0);
	PlannerHNS::GPSPoint top_left(-critical_lateral_distance, length, center.pos.z, 0);

	PlannerHNS::Mat3 invRotationMat(center.pos.a-M_PI_2);
	PlannerHNS::Mat3 invTranslationMat(center.pos.x, center.pos.y);

	top_right = invRotationMat*top_right;
	top_right = invTranslationMat*top_right;
	top_left = invRotationMat*top_left;
	top_left = invTranslationMat*top_left;

	top_right.a = center.pos.a - M_PI_2;
	top_left.a = center.pos.a + M_PI_2;

	std_msgs::ColorRGBA color_l, color_r;
	color_l.r = 1; color_l.g = 1;color_l.b = 1;
	color_r.r = 1; color_r.g = 1;color_r.b = 1;

	if(indicator == PlannerHNS::INDICATOR_LEFT)
	{
		color_l.b = 0;
	}
	else if(indicator == PlannerHNS::INDICATOR_RIGHT )
	{
		color_r.b = 0;
	}
	else if(indicator == PlannerHNS::INDICATOR_BOTH)
	{
		color_l.b = 0;
		color_r.b = 0;
	}

	visualization_msgs::Marker mkr_l = PlannerHNS::ROSHelpers::CreateGenMarker(top_left.x,top_left.y,top_left.z,top_left.a,color_l.r,color_l.g,color_l.b,1.0, id,"simu_car_indicator_left", visualization_msgs::Marker::ARROW);
	mkr_l.scale.y = 0.4;
	mkr_l.scale.z = 0.4;
	visualization_msgs::Marker mkr_r = PlannerHNS::ROSHelpers::CreateGenMarker(top_right.x,top_right.y,top_right.z,top_right.a,color_r.r,color_r.g,color_r.b,1.0, id,"simu_car_indicator_right", visualization_msgs::Marker::ARROW);
	mkr_r.scale.y = 0.4;
	mkr_r.scale.z = 0.4;
	markerArray.markers.push_back(mkr_l);
	markerArray.markers.push_back(mkr_r);
}

void ROSHelpers::TTC_PathRviz(const std::vector<PlannerHNS::WayPoint>& path, visualization_msgs::MarkerArray& markerArray)
{
	visualization_msgs::Marker lane_waypoint_marker;
	lane_waypoint_marker.header.frame_id = "map";
	lane_waypoint_marker.header.stamp = ros::Time();
	lane_waypoint_marker.ns = "ttc_path";
	lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
	lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
	lane_waypoint_marker.scale.x = 1;
	lane_waypoint_marker.scale.y = 1;
	lane_waypoint_marker.scale.z = 1;
	lane_waypoint_marker.frame_locked = false;
	std_msgs::ColorRGBA  total_color, curr_color;

	lane_waypoint_marker.color.a = 0.9;
	lane_waypoint_marker.color.r = 0.5;
	lane_waypoint_marker.color.g = 1.0;
	lane_waypoint_marker.color.b = 0.0;

	lane_waypoint_marker.id = 1;
	for (unsigned int i = 0; i < path.size(); i++)
	{
		geometry_msgs::Point point;
		point.x = path.at(i).pos.x;
		point.y = path.at(i).pos.y;
		point.z = path.at(i).pos.z;

		lane_waypoint_marker.points.push_back(point);

		markerArray.markers.push_back(lane_waypoint_marker);
	}
}

void ROSHelpers::CreateNextPlanningTreeLevelMarker(std::vector<PlannerHNS::WayPoint*>& level, visualization_msgs::MarkerArray& markerArray, PlannerHNS::WayPoint* pCurrGoal, double max_cost)
{
	if(level.size() == 0 || pCurrGoal == nullptr)
		return;

	std::vector<PlannerHNS::WayPoint*> newlevel;

	//lane_waypoint_marker.frame_locked = false;

	for(unsigned int i = 0; i < level.size(); i++)
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
		lane_waypoint_marker.color.b = 1;
		lane_waypoint_marker.color.r = 1;
		lane_waypoint_marker.color.g = 0;

//		float norm_cost = level.at(i)->cost / max_cost * 2.0;
//		if(norm_cost <= 1.0)
//		{
//			lane_waypoint_marker.color.r = 1-norm_cost;
//			lane_waypoint_marker.color.g = 1-1.0;
//		}
//		else if(norm_cost > 1.0)
//		{
//			lane_waypoint_marker.color.r = 1-1.0;
//			lane_waypoint_marker.color.g = 1- (2.0 - norm_cost);
//		}

		if(markerArray.markers.size() == 0)
			lane_waypoint_marker.id = 0;
		else
			lane_waypoint_marker.id = markerArray.markers.at(markerArray.markers.size()-1).id + 1;

		lane_waypoint_marker.pose.position.x = level.at(i)->pos.x;
		lane_waypoint_marker.pose.position.y = level.at(i)->pos.y;
		lane_waypoint_marker.pose.position.z = level.at(i)->pos.z;
		double a = UtilityHNS::UtilityH::SplitPositiveAngle(level.at(i)->pos.a);
		lane_waypoint_marker.pose.orientation = tf::createQuaternionMsgFromYaw(a);
		markerArray.markers.push_back(lane_waypoint_marker);

		if(level.at(i)->pLeft)
		{
			lane_waypoint_marker.pose.orientation = tf::createQuaternionMsgFromYaw(a + M_PI_2);
			newlevel.push_back(level.at(i)->pLeft);
			lane_waypoint_marker.id = markerArray.markers.at(markerArray.markers.size()-1).id + 1;
			markerArray.markers.push_back(lane_waypoint_marker);
		}
		if(level.at(i)->pRight)
		{
			newlevel.push_back(level.at(i)->pRight);
			lane_waypoint_marker.pose.orientation = tf::createQuaternionMsgFromYaw(a - M_PI_2);
			lane_waypoint_marker.id = markerArray.markers.at(markerArray.markers.size()-1).id + 1;
			markerArray.markers.push_back(lane_waypoint_marker);
		}

		for(unsigned int j = 0; j < level.at(i)->pFronts.size(); j++)
			if(level.at(i)->pFronts.at(j))
				newlevel.push_back(level.at(i)->pFronts.at(j));

		if(hypot(pCurrGoal->pos.y - level.at(i)->pos.y, pCurrGoal->pos.x - level.at(i)->pos.x) < 0.5)
		{
			newlevel.clear();
			break;
		}

		//std::cout << "Levels: " <<  lane_waypoint_marker.id << ", pLeft:" << level.at(i)->pLeft << ", pRight:" << level.at(i)->pRight << ", nFront:" << level.at(i)->pFronts.size() << ", Cost: "<< norm_cost<< std::endl;
	}

	level = newlevel;

	//std::cout << "Levels: " <<  level.size() << std::endl;
}

}
