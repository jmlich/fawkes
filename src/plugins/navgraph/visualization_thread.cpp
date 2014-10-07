
/***************************************************************************
 *  visualization_thread.cpp - Visualization pathplan via rviz
 *
 *  Created: Fri Nov 11 21:25:46 2011
 *  Copyright  2011-2012  Tim Niemueller [www.niemueller.de]
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

#include <utils/graph/topological_map_graph.h>
#include <plugins/navgraph/constraints/constraint_repo.h>
#include <tf/types.h>
#include <utils/math/angle.h>
#include <utils/math/coord.h>

#include <ros/ros.h>

using namespace fawkes;

/** @class NavGraphVisualizationThread "visualization_thread.h"
 * Send Marker messages to rviz to show navgraph info.
 * @author Tim Niemueller
 */

typedef std::multimap<std::string, std::string> ConnMap;

/** Constructor. */
NavGraphVisualizationThread::NavGraphVisualizationThread()
  : fawkes::Thread("NavGraphVisualizationThread", Thread::OPMODE_WAITFORWAKEUP)
{
  graph_ = NULL;
  crepo_ = NULL;
}


void
NavGraphVisualizationThread::init()
{
  vispub_ = rosnode->advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100, /* latching */ true);

  cfg_cost_scale_max_ = config->get_float("/plugins/navgraph/visualization/cost_scale_max");
  if (cfg_cost_scale_max_ < 1.0) {
    throw Exception("Visualization cost max scale must greater or equal to 1.0");
  }

  // subtract one because 1.0 is the minimum value where we want the
  // resulting value to be zero.
  cfg_cost_scale_max_ -= 1.0;

  last_id_num_ = constraints_last_id_num_ = 0;
  publish();
}

void
NavGraphVisualizationThread::finalize()
{
  visualization_msgs::MarkerArray m;

  for (size_t i = 0; i < last_id_num_; ++i) {
    visualization_msgs::Marker delop;
    delop.header.frame_id = "/map";
    delop.header.stamp = ros::Time::now();
    delop.ns = "navgraph";
    delop.id = i;
    delop.action = visualization_msgs::Marker::DELETE;
    m.markers.push_back(delop);
  }
  for (size_t i = 0; i < constraints_last_id_num_; ++i) {
    visualization_msgs::Marker delop;
    delop.header.frame_id = "/map";
    delop.header.stamp = ros::Time::now();
    delop.ns = "navgraph-constraints";
    delop.id = i;
    delop.action = visualization_msgs::Marker::DELETE;
    m.markers.push_back(delop);
  }
  vispub_.publish(m);
  usleep(500000); // needs some time to actually send
  vispub_.shutdown();
}


/** Set the graph.
 * @param graph graph to use
 */
void
NavGraphVisualizationThread::set_graph(fawkes::LockPtr<TopologicalMapGraph> &graph)
{
  graph_ = graph;
  plan_.clear();
  plan_to_ = plan_from_ = "";
  wakeup();
}

/** Set the constraint repo.
 * @param crepo constraint repo
 */
void
NavGraphVisualizationThread::set_constraint_repo(fawkes::LockPtr<ConstraintRepo> &crepo)
{
  crepo_ = crepo;
}

/** Set current plan.
 * @param plan current plan
 */
void
NavGraphVisualizationThread::set_plan(std::vector<fawkes::TopologicalMapNode> plan)
{
  plan_ = plan;
  plan_to_ = plan_from_ = "";
  wakeup();
}

/** Reset the current plan. */
void
NavGraphVisualizationThread::reset_plan()
{
  plan_.clear();
  plan_to_ = plan_from_ = "";
  wakeup();
}


/** Set the currently executed edge of the plan.
 * @param from node name of the originating node
 * @param to node name of the target node
 */
void
NavGraphVisualizationThread::set_current_edge(std::string from, std::string to)
{
  if (plan_from_ != from || plan_to_ != to) {
    plan_from_ = from;
    plan_to_ = to;
    wakeup();
  }
}


void
NavGraphVisualizationThread::loop()
{
  publish();
}


float
NavGraphVisualizationThread::edge_cost_factor(
  std::list<std::tuple<std::string, std::string, std::string, float>> &costs,
  const std::string &from, const std::string &to, std::string &constraint_name)
{
  for (const std::tuple<std::string, std::string, std::string, float> &c : costs) {
    if ((std::get<0>(c) == from && std::get<1>(c) == to) ||
	(std::get<0>(c) == to   && std::get<1>(c) == from))
    {
      constraint_name = std::get<2>(c);
      return std::get<3>(c);
    }
  }

  return 0.;
}

void
NavGraphVisualizationThread::add_circle_markers(visualization_msgs::MarkerArray &m, size_t &id_num,
						float center_x, float center_y, float radius,
						unsigned int arc_length,
						float r, float g, float b, float alpha, float line_width)
{
  for (unsigned int a = 0; a < 360; a += 2 * arc_length) {
    visualization_msgs::Marker arc;
    arc.header.frame_id = "/map";
    arc.header.stamp = ros::Time::now();
    arc.ns = "navgraph";
    arc.id = id_num++;
    arc.type = visualization_msgs::Marker::LINE_STRIP;
    arc.action = visualization_msgs::Marker::ADD;
    arc.scale.x = arc.scale.y = arc.scale.z = line_width;
    arc.color.r = r;
    arc.color.g = g;
    arc.color.b = b;
    arc.color.a = alpha;
    arc.lifetime = ros::Duration(0, 0);
    arc.points.resize(arc_length);
    for (unsigned int j = 0; j < arc_length; ++j) {
      float circ_x = 0, circ_y = 0;
      polar2cart2d(deg2rad(a + j), radius, &circ_x, &circ_y);
      arc.points[j].x = center_x + circ_x;
      arc.points[j].y = center_y + circ_y;
      arc.points[j].z = 0.;
    }
    m.markers.push_back(arc);
  }
}

void
NavGraphVisualizationThread::publish()
{
  if (! graph_) return;

  std::vector<fawkes::TopologicalMapNode> nodes = graph_->nodes();
  std::vector<fawkes::TopologicalMapEdge> edges = graph_->edges();

  crepo_.lock();
  std::map<std::string, std::string> bl_nodes = crepo_->blocks(nodes);
  std::map<std::pair<std::string, std::string>, std::string> bl_edges =
    crepo_->blocks(edges);
  std::list<std::tuple<std::string, std::string, std::string, float>> edge_cfs =
    crepo_->cost_factor(edges);
  crepo_.unlock();

  size_t id_num = 0;
  size_t constraints_id_num = 0;

  visualization_msgs::MarkerArray m;
  visualization_msgs::Marker lines;
  lines.header.frame_id = "/map";
  lines.header.stamp = ros::Time::now();
  lines.ns = "navgraph";
  lines.id = id_num++;
  lines.type = visualization_msgs::Marker::LINE_LIST;
  lines.action = visualization_msgs::Marker::ADD;
  lines.color.r = 0.5;
  lines.color.g = lines.color.b = 0.f;
  lines.color.a = 1.0;
  lines.scale.x = 0.02;
  lines.lifetime = ros::Duration(0, 0);

  visualization_msgs::Marker plan_lines;
  plan_lines.header.frame_id = "/map";
  plan_lines.header.stamp = ros::Time::now();
  plan_lines.ns = "navgraph";
  plan_lines.id = id_num++;
  plan_lines.type = visualization_msgs::Marker::LINE_LIST;
  plan_lines.action = visualization_msgs::Marker::ADD;
  plan_lines.color.r = 1.0;
  plan_lines.color.g = plan_lines.color.b = 0.f;
  plan_lines.color.a = 1.0;
  plan_lines.scale.x = 0.035;
  plan_lines.lifetime = ros::Duration(0, 0);

  visualization_msgs::Marker blocked_lines;
  blocked_lines.header.frame_id = "/map";
  blocked_lines.header.stamp = ros::Time::now();
  blocked_lines.ns = "navgraph";
  blocked_lines.id = id_num++;
  blocked_lines.type = visualization_msgs::Marker::LINE_LIST;
  blocked_lines.action = visualization_msgs::Marker::ADD;
  blocked_lines.color.r = blocked_lines.color.g = blocked_lines.color.b = 0.5;
  blocked_lines.color.a = 1.0;
  blocked_lines.scale.x = 0.02;
  blocked_lines.lifetime = ros::Duration(0, 0);

  visualization_msgs::Marker cur_line;
  cur_line.header.frame_id = "/map";
  cur_line.header.stamp = ros::Time::now();
  cur_line.ns = "navgraph";
  cur_line.id = id_num++;
  cur_line.type = visualization_msgs::Marker::LINE_LIST;
  cur_line.action = visualization_msgs::Marker::ADD;
  cur_line.color.g = 1.f;
  cur_line.color.r = cur_line.color.b = 0.f;
  cur_line.color.a = 1.0;
  cur_line.scale.x = 0.05;
  cur_line.lifetime = ros::Duration(0, 0);


  for (size_t i = 0; i < nodes.size(); ++i) {
    bool is_in_plan = (std::find(plan_.begin(), plan_.end(), nodes[i]) != plan_.end());
    bool is_last    = (plan_.size() >= 1) && (plan_.back().name() == nodes[i].name());
    //bool is_next    = (plan_.size() >= 2) && (plan_[1].name() == nodes[i].name());
    bool is_active  = (plan_to_ == nodes[i].name());

    visualization_msgs::Marker sphere;
    sphere.header.frame_id = "/map";
    sphere.header.stamp = ros::Time::now();
    sphere.ns = "navgraph";
    sphere.id = id_num++;
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.pose.position.x =  nodes[i].x();
    sphere.pose.position.y =  nodes[i].y();
    sphere.pose.position.z = 0.;
    sphere.pose.orientation.w = 1.;
    sphere.scale.y = 0.05;
    sphere.scale.z = 0.05;
    if (is_in_plan) {
      sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.1;
      if (is_last) {
	sphere.color.r = 0.f;
        sphere.color.g = 1.f;
      } else {
        sphere.color.r = 1.f;
        sphere.color.g = 0.f;
      }
      sphere.color.b = 0.f;
    } else if (bl_nodes.find(nodes[i].name()) != bl_nodes.end()) {
      sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.05;
      sphere.color.r = sphere.color.g = sphere.color.b = 0.5;

      visualization_msgs::Marker text;
      text.header.frame_id = "/map";
      text.header.stamp = ros::Time::now();
      text.ns = "navgraph-constraints";
      text.id = constraints_id_num++;
      text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::Marker::ADD;
      text.pose.position.x =  nodes[i].x();
      text.pose.position.y =  nodes[i].y();
      text.pose.position.z = 0.3;
      text.pose.orientation.w = 1.;
      text.scale.z = 0.12;
      text.color.r = 1.0;
      text.color.g = text.color.b = 0.f;
      text.color.a = 1.0;
      text.lifetime = ros::Duration(0, 0);
      text.text = bl_nodes[nodes[i].name()];
      m.markers.push_back(text);

    } else {
      sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.05;
      sphere.color.r = 0.5;
      sphere.color.b = 0.f;
    }
    sphere.color.a = 1.0;
    sphere.lifetime = ros::Duration(0, 0);
    m.markers.push_back(sphere);

    if (is_last) {
      float target_tolerance = 0.;
      if (nodes[i].has_property("target_tolerance")) {
	target_tolerance = nodes[i].property_as_float("target_tolerance");
      } else if (graph_->has_default_property("target_tolerance")) {
	target_tolerance = graph_->default_property_as_float("target_tolerance");
      }
      if (target_tolerance > 0.) {
	add_circle_markers(m, id_num, nodes[i].x(), nodes[i].y(), target_tolerance, 5,
			   sphere.color.r, sphere.color.g, sphere.color.b,
			   is_active ? sphere.color.a : 0.4);
      }
    } else if (is_active) {
      float travel_tolerance = 0.;
      if (nodes[i].has_property("travel_tolerance")) {
	travel_tolerance = nodes[i].property_as_float("travel_tolerance");
      } else if (graph_->has_default_property("travel_tolerance")) {
	travel_tolerance = graph_->default_property_as_float("travel_tolerance");
      }
      if (travel_tolerance > 0.) {
	add_circle_markers(m, id_num, nodes[i].x(), nodes[i].y(), travel_tolerance, 10,
			   sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a);
      }
    }

    if (is_in_plan) {
      float shortcut_tolerance = 0.;
      if (nodes[i].has_property("shortcut_tolerance")) {
	shortcut_tolerance = nodes[i].property_as_float("shortcut_tolerance");
      } else if (graph_->has_default_property("shortcut_tolerance")) {
	shortcut_tolerance = graph_->default_property_as_float("shortcut_tolerance");
      }
      if (shortcut_tolerance > 0.) {
	add_circle_markers(m, id_num, nodes[i].x(), nodes[i].y(), shortcut_tolerance, 30,
			   sphere.color.r, sphere.color.g, sphere.color.b, 0.3);
      }
    }

    if (nodes[i].has_property("orientation")) {
      float ori = nodes[i].property_as_float("orientation");
      //logger->log_debug(name(), "Node %s has orientation %f", nodes[i].name().c_str(), ori);
      visualization_msgs::Marker arrow;
      arrow.header.frame_id = "/map";
      arrow.header.stamp = ros::Time::now();
      arrow.ns = "navgraph";
      arrow.id = id_num++;
      arrow.type = visualization_msgs::Marker::ARROW;
      arrow.action = visualization_msgs::Marker::ADD;
      arrow.pose.position.x =  nodes[i].x();
      arrow.pose.position.y =  nodes[i].y();
      arrow.pose.position.z = 0.;
      tf::Quaternion q = tf::create_quaternion_from_yaw(ori);
      arrow.pose.orientation.x = q.x();
      arrow.pose.orientation.y = q.y();
      arrow.pose.orientation.z = q.z();
      arrow.pose.orientation.w = q.w();
      arrow.scale.x = 0.1;
      arrow.scale.y = 0.5;
      arrow.scale.z = 0.10;
      if (is_in_plan) {
	if (is_last) {
	  arrow.color.r = arrow.color.g = 1.f;
	} else {
	  arrow.color.r = 1.f;
	  arrow.color.g = 0.f;
	}
      } else {
	arrow.color.r = 0.5;
      }
      arrow.color.b = 0.f;
      arrow.color.a = 1.0;
      arrow.lifetime = ros::Duration(0, 0);
      m.markers.push_back(arrow);
    }


    visualization_msgs::Marker text;
    text.header.frame_id = "/map";
    text.header.stamp = ros::Time::now();
    text.ns = "navgraph";
    text.id = id_num++;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    text.pose.position.x =  nodes[i].x();
    text.pose.position.y =  nodes[i].y();
    text.pose.position.z = 0.08;
    text.pose.orientation.w = 1.;
    text.scale.z = 0.15; // 15cm high
    text.color.r = text.color.g = text.color.b = 1.0f;
    text.color.a = 1.0;
    text.lifetime = ros::Duration(0, 0);
    text.text = nodes[i].name();
    m.markers.push_back(text);

  }

  if (! plan_.empty() && plan_.back().name() == "free-target") {
    TopologicalMapNode &target_node = plan_[plan_.size() - 1];

    // we are traveling to a free target
    visualization_msgs::Marker sphere;
    sphere.header.frame_id = "/map";
    sphere.header.stamp = ros::Time::now();
    sphere.ns = "navgraph";
    sphere.id = id_num++;
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.pose.position.x =  target_node.x();
    sphere.pose.position.y =  target_node.y();
    sphere.pose.position.z = 0.;
    sphere.pose.orientation.w = 1.;
    sphere.scale.y = 0.05;
    sphere.scale.z = 0.05;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.1;
    sphere.color.r = 1.;
    sphere.color.g = 0.5f;
    sphere.color.b = 0.f;
    sphere.color.a = 1.0;
    sphere.lifetime = ros::Duration(0, 0);
    m.markers.push_back(sphere);

    visualization_msgs::Marker text;
    text.header.frame_id = "/map";
    text.header.stamp = ros::Time::now();
    text.ns = "navgraph";
    text.id = id_num++;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    text.pose.position.x =  target_node.x();
    text.pose.position.y =  target_node.y();
    text.pose.position.z = 0.08;
    text.pose.orientation.w = 1.;
    text.scale.z = 0.15; // 15cm high
    text.color.r = text.color.g = text.color.b = 1.0f;
    text.color.a = 1.0;
    text.lifetime = ros::Duration(0, 0);
    text.text = "Free Target";
    m.markers.push_back(text);

    if (target_node.has_property("orientation")) {
      float ori = target_node.property_as_float("orientation");
      visualization_msgs::Marker ori_arrow;
      ori_arrow.header.frame_id = "/map";
      ori_arrow.header.stamp = ros::Time::now();
      ori_arrow.ns = "navgraph";
      ori_arrow.id = id_num++;
      ori_arrow.type = visualization_msgs::Marker::ARROW;
      ori_arrow.action = visualization_msgs::Marker::ADD;
      ori_arrow.pose.position.x =  target_node.x();
      ori_arrow.pose.position.y =  target_node.y();
      ori_arrow.pose.position.z = 0.;
      tf::Quaternion q = tf::create_quaternion_from_yaw(ori);
      ori_arrow.pose.orientation.x = q.x();
      ori_arrow.pose.orientation.y = q.y();
      ori_arrow.pose.orientation.z = q.z();
      ori_arrow.pose.orientation.w = q.w();
      ori_arrow.scale.x = 0.1;
      ori_arrow.scale.y = 0.5;
      ori_arrow.scale.z = 0.10;
      ori_arrow.color.r = 1.f;
      ori_arrow.color.g = 0.5f;
      ori_arrow.color.b = 0.f;
      ori_arrow.color.a = 1.0;
      ori_arrow.lifetime = ros::Duration(0, 0);
      m.markers.push_back(ori_arrow);
    }

    float target_tolerance = 0.;
    if (plan_.back().has_property("target_tolerance")) {
      target_tolerance = plan_.back().property_as_float("target_tolerance");
    } else if (graph_->has_default_property("target_tolerance")) {
      target_tolerance = graph_->default_property_as_float("target_tolerance");
    }
    if (target_tolerance > 0.) {
      add_circle_markers(m, id_num, plan_.back().x(), plan_.back().y(), target_tolerance, 10,
			 sphere.color.r, sphere.color.g, sphere.color.b,
			 (plan_.size() == 1) ? sphere.color.a : 0.5);
    }

    float shortcut_tolerance = 0.;
    if (plan_.back().has_property("shortcut_tolerance")) {
      shortcut_tolerance = plan_.back().property_as_float("shortcut_tolerance");
    } else if (graph_->has_default_property("shortcut_tolerance")) {
      shortcut_tolerance = graph_->default_property_as_float("shortcut_tolerance");
    }
    if (shortcut_tolerance > 0.) {
      add_circle_markers(m, id_num, plan_.back().x(), plan_.back().y(), shortcut_tolerance, 30,
			 sphere.color.r, sphere.color.g, sphere.color.b, 0.3);
    }

    if (plan_.size() >= 2) {
      TopologicalMapNode &last_graph_node = plan_[plan_.size() - 2];
      
      geometry_msgs::Point p1;
      p1.x =  last_graph_node.x();
      p1.y =  last_graph_node.y();
      p1.z = 0.;

      geometry_msgs::Point p2;
      p2.x =  target_node.x();
      p2.y =  target_node.y();
      p2.z = 0.;

      visualization_msgs::Marker arrow;
      arrow.header.frame_id = "/map";
      arrow.header.stamp = ros::Time::now();
      arrow.ns = "navgraph";
      arrow.id = id_num++;
      arrow.type = visualization_msgs::Marker::ARROW;
      arrow.action = visualization_msgs::Marker::ADD;
      arrow.color.a = 1.0;
      arrow.lifetime = ros::Duration(0, 0);
      arrow.points.push_back(p1);
      arrow.points.push_back(p2);

      
      if (plan_to_ == target_node.name())
      {
        // it's the current line
        arrow.color.r = arrow.color.g = 1.f;
        arrow.color.b = 0.f;
        arrow.scale.x = 0.1; // shaft radius
        arrow.scale.y = 0.3; // head radius
      } else {
        // it's in the current plan
        arrow.color.r = 1.0;
        arrow.color.g = 0.5f;
        arrow.color.b = 0.f;
        arrow.scale.x = 0.07; // shaft radius
        arrow.scale.y = 0.2; // head radius
      }
      m.markers.push_back(arrow);
    }
  }
    

  for (size_t i = 0; i < edges.size(); ++i) {
    TopologicalMapEdge &edge = edges[i];
    if (graph_->node_exists(edge.from()) && graph_->node_exists(edge.to())) {
      TopologicalMapNode from = graph_->node(edge.from());
      TopologicalMapNode to   = graph_->node(edge.to());

      geometry_msgs::Point p1;
      p1.x =  from.x();
      p1.y =  from.y();
      p1.z = 0.;

      geometry_msgs::Point p2;
      p2.x =  to.x();
      p2.y =  to.y();
      p2.z = 0.;

      std::string cost_cstr_name;
      float cost_factor = edge_cost_factor(edge_cfs, from.name(), to.name(), cost_cstr_name);

      if (edge.is_directed()) {
        visualization_msgs::Marker arrow;
        arrow.header.frame_id = "/map";
        arrow.header.stamp = ros::Time::now();
        arrow.ns = "navgraph";
        arrow.id = id_num++;
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.action = visualization_msgs::Marker::ADD;
        arrow.color.a = 1.0;
        arrow.lifetime = ros::Duration(0, 0);
        arrow.points.push_back(p1);
        arrow.points.push_back(p2);

        if (plan_from_ == from.name() && plan_to_ == to.name())
        {
          // it's the current line
          arrow.color.g = 1.f;
          arrow.color.r = arrow.color.b = 0.f;
          arrow.scale.x = 0.1; // shaft radius
          arrow.scale.y = 0.3; // head radius
        } else {
          bool in_plan = false;
          for (size_t p = 0; p < plan_.size(); ++p) {
            if (plan_[p] == from && (p < plan_.size() - 1 && plan_[p+1] == to)) {
              in_plan = true;
              break;
            }
          }

          if (in_plan) {
            // it's in the current plan
            arrow.color.r = 1.0;
	    if (cost_factor >= 1.00001) {
	      arrow.color.g = std::min(1.0, (cost_factor - 1.0) / cfg_cost_scale_max_);
	    } else {
	      arrow.color.g = 0.f;
	    }
	    arrow.color.b = 0.f;
            arrow.scale.x = 0.07; // shaft radius
            arrow.scale.y = 0.2; // head radius
	  } else if (bl_nodes.find(from.name()) != bl_nodes.end() ||
		     bl_nodes.find(to.name()) != bl_nodes.end() ||
		     bl_edges.find(std::make_pair(to.name(), from.name())) != bl_edges.end() ||
		     bl_edges.find(std::make_pair(from.name(), to.name())) != bl_edges.end())
	  {
            arrow.color.r = arrow.color.g = arrow.color.b = 0.5;
            arrow.scale.x = 0.04; // shaft radius
            arrow.scale.y = 0.15; // head radius
          } else {
            // regular
            arrow.color.r = 0.66666;
	    if (cost_factor >= 1.00001) {
	      arrow.color.g =
		std::min(1.0, (cost_factor - 1.0) / cfg_cost_scale_max_) * 0.66666;
	    } else {
	      arrow.color.g = 0.f;
	    }
	    arrow.color.b = 0.f;
            arrow.scale.x = 0.04; // shaft radius
            arrow.scale.y = 0.15; // head radius
          }
        }
        m.markers.push_back(arrow);
      } else {
        if ( (plan_to_   == from.name() && plan_from_ == to.name()) ||
             (plan_from_ == from.name() && plan_to_   == to.name()) )
        {
          // it's the current line
          cur_line.points.push_back(p1);
          cur_line.points.push_back(p2);
        } else {
          bool in_plan = false;
          for (size_t p = 0; p < plan_.size(); ++p) {
            if (plan_[p] == from &&
                ((p > 0 && plan_[p-1] == to) || (p < plan_.size() - 1 && plan_[p+1] == to)))
            {
              in_plan = true;
              break;
            }
          }

          if (in_plan) {
            // it's in the current plan
	    if (cost_factor >= 1.00001) {
	      visualization_msgs::Marker line;
	      line.header.frame_id = "/map";
	      line.header.stamp = ros::Time::now();
	      line.ns = "navgraph";
	      line.id = id_num++;
	      line.type = visualization_msgs::Marker::LINE_STRIP;
	      line.action = visualization_msgs::Marker::ADD;
	      line.color.a = 1.0;
	      line.lifetime = ros::Duration(0, 0);
	      line.points.push_back(p1);
	      line.points.push_back(p2);
	      line.color.r = 1.f;
	      line.color.g = std::min(1.0, (cost_factor - 1.0) / cfg_cost_scale_max_);
	      line.color.b = 0.f;
	      line.scale.x = 0.035;
	      m.markers.push_back(line);
	    } else {
	      plan_lines.points.push_back(p1);
	      plan_lines.points.push_back(p2);
	    }
	  } else if (bl_nodes.find(from.name()) != bl_nodes.end() ||
		     bl_nodes.find(to.name()) != bl_nodes.end())
	  {
            blocked_lines.points.push_back(p1);
            blocked_lines.points.push_back(p2);

	  } else if (bl_edges.find(std::make_pair(to.name(), from.name())) != bl_edges.end() ||
		     bl_edges.find(std::make_pair(from.name(), to.name())) != bl_edges.end())
	  {
            blocked_lines.points.push_back(p1);
            blocked_lines.points.push_back(p2);

	    tf::Vector3 p1v(p1.x, p1.y, p1.z);
	    tf::Vector3 p2v(p2.x, p2.y, p2.z);

	    tf::Vector3 p = p1v + (p2v - p1v) * 0.5;

	    std::string text_s = "";

	    std::map<std::pair<std::string, std::string>, std::string>::iterator e =
	      bl_edges.find(std::make_pair(to.name(), from.name()));
	    if (e != bl_edges.end()) {
	      text_s = e->second;
	    } else {
	      e = bl_edges.find(std::make_pair(from.name(), to.name()));
	      if (e != bl_edges.end()) {
		text_s = e->second;
	      }
	    }

	    visualization_msgs::Marker text;
	    text.header.frame_id = "/map";
	    text.header.stamp = ros::Time::now();
	    text.ns = "navgraph-constraints";
	    text.id = constraints_id_num++;
	    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	    text.action = visualization_msgs::Marker::ADD;
	    text.pose.position.x =  p[0];
	    text.pose.position.y =  p[1];
	    text.pose.position.z = 0.3;
	    text.pose.orientation.w = 1.;
	    text.scale.z = 0.12;
	    text.color.r = 1.0;
	    text.color.g = text.color.b = 0.f;
	    text.color.a = 1.0;
	    text.lifetime = ros::Duration(0, 0);
	    text.text = text_s;
	    m.markers.push_back(text);

          } else {
	    if (cost_factor >= 1.00001) {
	      visualization_msgs::Marker line;
	      line.header.frame_id = "/map";
	      line.header.stamp = ros::Time::now();
	      line.ns = "navgraph";
	      line.id = id_num++;
	      line.type = visualization_msgs::Marker::LINE_STRIP;
	      line.action = visualization_msgs::Marker::ADD;
	      line.color.a = 1.0;
	      line.lifetime = ros::Duration(0, 0);
	      line.points.push_back(p1);
	      line.points.push_back(p2);
	      line.color.r = 0.66666;
	      line.color.g =
		std::min(1.0, (cost_factor - 1.0) / cfg_cost_scale_max_) * 0.66666;
	      line.color.b = 0.f;
	      line.scale.x = 0.02;
	      m.markers.push_back(line);
	    } else {
	      lines.points.push_back(p1);
	      lines.points.push_back(p2);
	    }
          }
        }
      }
    }
  }

  m.markers.push_back(lines);
  m.markers.push_back(plan_lines);
  m.markers.push_back(blocked_lines);
  m.markers.push_back(cur_line);

  for (size_t i = id_num; i < last_id_num_; ++i) {
    visualization_msgs::Marker delop;
    delop.header.frame_id = "/map";
    delop.header.stamp = ros::Time::now();
    delop.ns = "navgraph";
    delop.id = i;
    delop.action = visualization_msgs::Marker::DELETE;
    m.markers.push_back(delop);
  }

  for (size_t i = constraints_id_num; i < constraints_last_id_num_; ++i) {
    visualization_msgs::Marker delop;
    delop.header.frame_id = "/map";
    delop.header.stamp = ros::Time::now();
    delop.ns = "navgraph-constraints";
    delop.id = i;
    delop.action = visualization_msgs::Marker::DELETE;
    m.markers.push_back(delop);
  }

  last_id_num_ = id_num;
  constraints_last_id_num_ = constraints_id_num;

  vispub_.publish(m);
}
