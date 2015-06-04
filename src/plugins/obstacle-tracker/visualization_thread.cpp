
/***************************************************************************
 *  visualization_thread.cpp - Visualization for obstacle-tracker via rviz
 *
 *  Created: Sun May 30 12:12:17 2015
 *  based on Template from Tim Niemueller
 *  Copyright  2015  Sebastian Reuter
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "visualization_thread.h"

#include <ros/ros.h>

using namespace fawkes;

/** @class ObstacleTrackerVisualizationThread "visualization_thread.h"
 * Send Marker messages to rviz to show obstacl-tracker info.
 * @author Sebastian Reuter
 */

/** Constructor. */
ObstacleTrackerVisualizationThread::ObstacleTrackerVisualizationThread()
  : fawkes::Thread("ObstacleTrackerVisualizationThread", Thread::OPMODE_WAITFORWAKEUP)
{
  set_coalesce_wakeups(true);
}


void
ObstacleTrackerVisualizationThread::init()
{
  try{
	  cfg_global_frame_  = config->get_string("/frames/fixed");
  }
  catch(Exception &e){
	  logger->log_warn(name(), "No fixed frame set - wil take '/map' as fixed frame");
	  cfg_global_frame_ = "/map";
  }

  vispub_ =
    rosnode->advertise<visualization_msgs::MarkerArray>
    ("obstacle_tracker_marker_array", 100, /* latching */ true);

  last_id_num_ = 0;

}

void
ObstacleTrackerVisualizationThread::finalize()
{
  visualization_msgs::MarkerArray m;

  for (size_t i = 0; i < last_id_num_; ++i) {
    visualization_msgs::Marker delop;
    delop.header.frame_id = "/map";
    delop.header.stamp = ros::Time::now();
    delop.ns = "obstacle-tracker-kalman";
    delop.id = i;
    delop.action = visualization_msgs::Marker::DELETE;
    m.markers.push_back(delop);
  }
  vispub_.publish(m);
  usleep(500000); // needs some time to actually send
  vispub_.shutdown();

  logger->log_info(name(), "Obstacle Tracker Visualization - finalize");
}


void
ObstacleTrackerVisualizationThread::loop()
{
   visualization_msgs::MarkerArray m;
   unsigned int idnum = 0;

   for (auto &o : objects_) {
      visualization_msgs::Marker text;
      text.header.frame_id = cfg_global_frame_;
      text.header.stamp = ros::Time::now();
      text.ns = "obstacle-tracker-kalman";
      text.id = idnum++;
      text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::Marker::ADD;
      text.pose.position.x = o.getX();
      text.pose.position.y = o.getY();
      text.pose.position.z = .35;
      text.pose.orientation.w = 1.;
      text.scale.z = 0.15;
      text.color.r = text.color.g = text.color.b = 1.0f;
      text.color.a = 1.0;
      text.lifetime = ros::Duration(0, 0);
      text.text = "Object";
      m.markers.push_back(text);
//
      visualization_msgs::Marker sphere;
      sphere.header.frame_id = cfg_global_frame_;
      sphere.header.stamp = ros::Time::now();
      sphere.ns = "obstacle-tracker-kalman";
      sphere.id = idnum++;
      sphere.type = visualization_msgs::Marker::SPHERE;
      sphere.action = visualization_msgs::Marker::ADD;
      sphere.pose.position.x = o.getX();
      sphere.pose.position.y = o.getY();
      sphere.pose.position.y = o.getY();
      sphere.pose.position.z = 0.15;
      sphere.pose.orientation.w = 1.;
      sphere.scale.x = 0.1;
      sphere.scale.y = 0.1;
      sphere.scale.z = 0.1;
      sphere.color.r = 1.0;
      sphere.color.g = .27;
      sphere.color.b = 0.;
      sphere.color.a = 1.0;
      sphere.lifetime = ros::Duration(0, 0);
      m.markers.push_back(sphere);
   }

   for (auto &meas : measurements_) {
      visualization_msgs::Marker text;
      text.header.frame_id = cfg_global_frame_;
      text.header.stamp = ros::Time::now();
      text.ns = "obstacle-tracker-kalman";
      text.id = idnum++;
      text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::Marker::ADD;
      text.pose.position.x = meas.getX();
      text.pose.position.y = meas.getY();
      text.pose.position.z = .25;
      text.pose.orientation.w = 1.;
      text.scale.z = 0.15;
      text.color.r = text.color.g = text.color.b = 1.0f;
      text.color.a = 1.0;
      text.lifetime = ros::Duration(0, 0);
      text.text = "Measurement";
      m.markers.push_back(text);
//
      visualization_msgs::Marker sphere;
      sphere.header.frame_id = cfg_global_frame_;
      sphere.header.stamp = ros::Time::now();
      sphere.ns = "obstacle-tracker-kalman";
      sphere.id = idnum++;
      sphere.type = visualization_msgs::Marker::SPHERE;
      sphere.action = visualization_msgs::Marker::ADD;
      sphere.pose.position.x = meas.getX();
      sphere.pose.position.y = meas.getY();
      sphere.pose.position.z = 0.05;
      sphere.pose.orientation.w = 1.;
      sphere.scale.x = 0.1;
      sphere.scale.y = 0.1;
      sphere.scale.z = 0.1;
      sphere.color.b = .0;
      sphere.color.r = sphere.color.g = 1.0;
      sphere.color.a = 1.0;
      sphere.lifetime = ros::Duration(0, 0);
      m.markers.push_back(sphere);

   }


  for (size_t i = idnum; i < last_id_num_; ++i) {
    visualization_msgs::Marker delop;
    delop.header.frame_id = cfg_global_frame_;
    delop.header.stamp = ros::Time::now();
    delop.ns = "obstacle-tracker-kalman";
    delop.id = i;
    delop.action = visualization_msgs::Marker::DELETE;
    m.markers.push_back(delop);
  }
    last_id_num_ = idnum;

    //logger->log_info(name(), "VisualizationThread id num was %d last_id_num was %d.", idnum, last_id_num_);

    vispub_.publish(m);

}


/** Trigger publishing of visualization.
 * @param obstacles obstacles used for graph generation
 * @param map_obstacles obstacles generated from map
 * @param pois points of interest
 */
void
ObstacleTrackerVisualizationThread::publish(std::vector<fawkes::tf::Point> measurements, std::vector<fawkes::tf::Point> objects)
{
  measurements_ = measurements;
  objects_ = objects;

  wakeup();
}
