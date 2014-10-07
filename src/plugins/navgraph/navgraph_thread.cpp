/***************************************************************************
 *  navgraph_thread.cpp - Graph-based global path planning
 *
 *  Created: Tue Sep 18 16:00:34 2012
 *  Copyright  2012-2014  Tim Niemueller [www.niemueller.de]
 *                  2014  Tobias Neumann
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

#include "navgraph_thread.h"

#include <utils/graph/yaml_navgraph.h>
#include <utils/search/astar.h>
#include <utils/math/angle.h>
#include <tf/utils.h>
#include <core/utils/lockptr.h>

#include "search_state.h"

#include <fstream>

using namespace fawkes;

/** @class NavGraphThread "navgraph_thread.h"
 * Thread to perform graph-based path planning.
 * @author Tim Niemueller
 */

/** Constructor. */
NavGraphThread::NavGraphThread()
  : Thread("NavGraphThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT),
    AspectProviderAspect(&navgraph_aspect_inifin_)
{
#ifdef HAVE_VISUALIZATION
  vt_ = NULL;
#endif
}

#ifdef HAVE_VISUALIZATION
/** Constructor. */
NavGraphThread::NavGraphThread(NavGraphVisualizationThread *vt)
  : Thread("NavGraphThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT),
    AspectProviderAspect(&navgraph_aspect_inifin_)
{
  vt_ = vt;
}
#endif

/** Destructor. */
NavGraphThread::~NavGraphThread()
{
}

void
NavGraphThread::init()
{
  cfg_graph_file_      = config->get_string("/plugins/navgraph/graph_file");
  cfg_base_frame_      = config->get_string("/plugins/navgraph/base_frame");
  cfg_global_frame_    = config->get_string("/plugins/navgraph/global_frame");
  cfg_nav_if_id_       = config->get_string("/plugins/navgraph/navigator_interface_id");
  cfg_travel_tolerance_ = config->get_float("/plugins/navgraph/travel_tolerance");
  cfg_target_tolerance_ = config->get_float("/plugins/navgraph/target_tolerance");
  cfg_orientation_tolerance_ = config->get_float("/plugins/navgraph/orientation_tolerance");
  cfg_shortcut_tolerance_ = config->get_float("/plugins/navgraph/shortcut_tolerance");
  cfg_resend_interval_ = config->get_float("/plugins/navgraph/resend_interval");
  cfg_replan_interval_ = config->get_float("/plugins/navgraph/replan_interval");
  cfg_replan_factor_   = config->get_float("/plugins/navgraph/replan_cost_factor");
  cfg_target_time_     = config->get_float("/plugins/navgraph/target_time");
  cfg_log_graph_       = config->get_bool("/plugins/navgraph/log_graph");
  cfg_abort_on_error_  = config->get_bool("/plugins/navgraph/abort_on_error");
#ifdef HAVE_VISUALIZATION
  cfg_visual_interval_ = config->get_float("/plugins/navgraph/visualization_interval");
#endif
  cfg_monitor_file_ = false;
  try {
    cfg_monitor_file_ = config->get_bool("/plugins/navgraph/monitor_file");
  } catch (Exception &e) {} // ignored

  pp_nav_if_ = blackboard->open_for_writing<NavigatorInterface>("Pathplan");
  nav_if_    = blackboard->open_for_reading<NavigatorInterface>(cfg_nav_if_id_.c_str());
  path_if_ = blackboard->open_for_writing<NavPathInterface>("NavPath");


  if (cfg_graph_file_[0] != '/') {
    cfg_graph_file_ = std::string(CONFDIR) + "/" + cfg_graph_file_;
  }

  graph_ = load_graph(cfg_graph_file_);

  if (graph_->has_default_property("travel_tolerance")) {
    cfg_travel_tolerance_ = graph_->default_property_as_float("travel_tolerance");
    logger->log_info(name(), "Using travel tolerance %f from graph file", cfg_travel_tolerance_);
  }
  if (graph_->has_default_property("target_tolerance")) {
    cfg_target_tolerance_ = graph_->default_property_as_float("target_tolerance");
    logger->log_info(name(), "Using target tolerance %f from graph file", cfg_target_tolerance_);
  }
  if (graph_->has_default_property("orientation_tolerance")) {
    cfg_orientation_tolerance_ = graph_->default_property_as_float("orientation_tolerance");
    logger->log_info(name(), "Using orientation tolerance %f from graph file", cfg_orientation_tolerance_);
  }
  if (graph_->has_default_property("shortcut_tolerance")) {
    cfg_shortcut_tolerance_ = graph_->default_property_as_float("shortcut_tolerance");
    logger->log_info(name(), "Using shortcut tolerance %f from graph file", cfg_shortcut_tolerance_);
  }
  if (graph_->has_default_property("target_time")) {
    cfg_target_time_ = graph_->default_property_as_float("target_time");
    logger->log_info(name(), "Using target time %f from graph file", cfg_target_time_);
  }

  navgraph_aspect_inifin_.set_navgraph(graph_);
  if (cfg_log_graph_) {
    log_graph();
  }
  astar_ = new AStar();

  if (cfg_monitor_file_) {
    logger->log_info(name(), "Enabling graph file monitoring");
    fam_ = new FileAlterationMonitor();
    fam_->watch_file(cfg_graph_file_.c_str());
    fam_->add_listener(this);
  }

  exec_active_       = false;
  target_reached_    = false;
  last_node_         = "";
  error_reason_      = "";
  constrained_plan_  = false;
  cmd_sent_at_       = new Time(clock);
  path_planned_at_   = new Time(clock);
  target_reached_at_ = new Time(clock);
  error_at_          = new Time(clock);
#ifdef HAVE_VISUALIZATION
  visualized_at_     = new Time(clock);
#endif

  constraint_repo_   = new ConstraintRepo(logger);
  navgraph_aspect_inifin_.set_constraint_repo(constraint_repo_);
}

void
NavGraphThread::finalize()
{
  delete cmd_sent_at_;
  delete path_planned_at_;
  delete astar_;
  delete target_reached_at_;
  delete error_at_;
#ifdef HAVE_VISUALIZATION
  delete visualized_at_;
#endif
  graph_.clear();
  blackboard->close(pp_nav_if_);
  blackboard->close(nav_if_);
}

void
NavGraphThread::once()
{
#ifdef HAVE_VISUALIZATION
  if (vt_) {
    vt_->set_constraint_repo(constraint_repo_);
    vt_->set_graph(graph_);
  }
#endif
}

void
NavGraphThread::loop()
{
  // process messages
  bool needs_write = false;
  while (! pp_nav_if_->msgq_empty()) {
    needs_write = true;

    if (pp_nav_if_->msgq_first_is<NavigatorInterface::StopMessage>()) {

      stop_motion();
      exec_active_ = false;

    } else if (pp_nav_if_->msgq_first_is<NavigatorInterface::CartesianGotoMessage>()) {
      NavigatorInterface::CartesianGotoMessage *msg = pp_nav_if_->msgq_first(msg);
      logger->log_info(name(), "cartesian goto (x,y,ori) = (%f,%f,%f)",
		       msg->x(), msg->y(), msg->orientation());

      pp_nav_if_->set_msgid(msg->id());
      generate_plan(msg->x(), msg->y(), msg->orientation());
      optimize_plan();
      start_plan();

    } else if (pp_nav_if_->msgq_first_is<NavigatorInterface::PlaceGotoMessage>()) {
      NavigatorInterface::PlaceGotoMessage *msg = pp_nav_if_->msgq_first(msg);
      logger->log_info(name(), "goto '%s'", msg->place());

      pp_nav_if_->set_msgid(msg->id());
      generate_plan(msg->place());
      optimize_plan();
      start_plan();
    }

    pp_nav_if_->msgq_pop();
  }

  if (cfg_monitor_file_) {
    fam_->process_events();
  }

  if (exec_active_) {
    // check if current was target reached
    size_t shortcut_to;

    if (! tf_listener->transform_origin(cfg_base_frame_, cfg_global_frame_, pose_)) {
      logger->log_warn(name(), "Cannot get pose info, skipping loop");

    } else if (target_reached_) {
      // reached the target, check if colli/navi/local planner is final
      nav_if_->read();
      fawkes::Time now(clock);
      if (nav_if_->is_final()) {
	pp_nav_if_->set_final(true);
	needs_write = true;
      } else if ((now - target_reached_at_) >= target_time_) {
	stop_motion();
	needs_write = true;
      }
    } else if (node_reached()) {
      logger->log_info(name(), "Node '%s' has been reached", plan_[0].name().c_str());
      last_node_ = plan_[0].name();
      if (plan_.size() == 1) {
	target_time_ = 0;
	if (plan_[0].has_property("target-time")) {
	  target_time_ = plan_[0].property_as_float("target-time");
	}
	if (target_time_ == 0)  target_time_ = cfg_target_time_;

	target_reached_ = true;
	target_reached_at_->stamp();
      }
      plan_.erase(plan_.begin());
      publish_path(plan_);

      if (! plan_.empty()) {
        try {
          logger->log_info(name(), "Sending next goal %s after node reached",
			   plan_[0].name().c_str());
          send_next_goal();
        } catch (Exception &e) {
          logger->log_warn(name(), "Failed to send next goal (node reached)");
          logger->log_warn(name(), e);
        }
      }

    } else if ((shortcut_to = shortcut_possible()) > 0) {
      logger->log_info(name(), "Shortcut posible, jumping from '%s' to '%s'",
		       plan_[0].name().c_str(), plan_[shortcut_to].name().c_str());

      plan_.erase(plan_.begin(), plan_.begin() + shortcut_to);

      if (! plan_.empty()) {
        try {
          logger->log_info(name(), "Sending next goal after taking a shortcut");
          send_next_goal();
        } catch (Exception &e) {
          logger->log_warn(name(), "Failed to send next goal (shortcut)");
          logger->log_warn(name(), e);
        }
      }

    } else {
      fawkes::Time now(clock);
      bool new_plan = false;

      if (plan_.size() > 2 && (now - path_planned_at_) > cfg_replan_interval_)
      {
	*path_planned_at_ = now;
	constraint_repo_.lock();
	if (constraint_repo_->compute() || constraint_repo_->modified(/* reset */ true)) {
	  TopologicalMapNode goal = plan_.back();

	  if (replan(plan_[0], goal)) {
	    // do not optimize here, we know that we do want to travel
	    // to the first node, we are already on the way...
	    //optimize_plan();
	    start_plan();
	    new_plan = true;
	  }
	}
	constraint_repo_.unlock();
      }

      if (! new_plan && (now - cmd_sent_at_) > cfg_resend_interval_) {
        try {
          //logger->log_info(name(), "Re-sending goal");
	  send_next_goal();
        } catch (Exception &e) {
          logger->log_warn(name(), "Failed to send next goal (resending)");
          logger->log_warn(name(), e);
        }
      }
    }
  }

#ifdef HAVE_VISUALIZATION
  if (vt_) {
    fawkes::Time now(clock);
    if (now - visualized_at_ >= cfg_visual_interval_) {
      *visualized_at_ = now;
      constraint_repo_.lock();
      if (constraint_repo_->compute() || constraint_repo_->modified(/* reset */ false)) {
	vt_->wakeup();
      }
      constraint_repo_.unlock();
    }
  }
#endif

  if (needs_write) {
    pp_nav_if_->write();
  }
}

fawkes::LockPtr<fawkes::TopologicalMapGraph>
NavGraphThread::load_graph(std::string filename)
{
  std::ifstream inf(filename);
  std::string firstword;
  inf >> firstword;
  inf.close();

  if (firstword == "%YAML") {
    logger->log_info(name(), "Loading YAML graph from %s", filename.c_str());
    return fawkes::LockPtr<TopologicalMapGraph>(load_yaml_navgraph(filename));
  } else if (firstword == "<Graph>") {
    logger->log_info(name(), "Loading RCSoft graph from %s", filename.c_str());
    return fawkes::LockPtr<TopologicalMapGraph>(load_rcsoft_graph(filename));
  } else {
    throw Exception("Unknown graph format");
  }
}

void
NavGraphThread::generate_plan(std::string goal_name)
{
  if (! tf_listener->transform_origin(cfg_base_frame_, cfg_global_frame_, pose_)) {
    logger->log_warn(name(),
		     "Failed to compute pose, cannot generate plan");
    return;
  }

  TopologicalMapNode init =
    graph_->closest_node(pose_.getOrigin().x(), pose_.getOrigin().y());
  TopologicalMapNode goal = graph_->node(goal_name);


  logger->log_debug(name(), "Starting at (%f,%f), closest node is '%s'",
		    pose_.getOrigin().x(), pose_.getOrigin().y(), init.name().c_str());

  plan_.clear();

  std::vector<AStarState *> a_star_solution;

  constraint_repo_.lock();
  if (constraint_repo_->has_constraints()) {
    constraint_repo_->compute();

    NavGraphSearchState *initial_state =
      new NavGraphSearchState(init, goal, *graph_, *constraint_repo_);
    a_star_solution =  astar_->solve(initial_state);
  }
  constraint_repo_.unlock();
  
  if (! a_star_solution.empty()) {
    constrained_plan_ = true;
  } else {
    constrained_plan_ = false;
    logger->log_warn(name(), "Failed to generate plan, will try without constraints");
    NavGraphSearchState *initial_state =
      new NavGraphSearchState(init, goal, *graph_);
    a_star_solution =  astar_->solve(initial_state);
  }

  NavGraphSearchState *solstate;
  for (unsigned int i = 0; i < a_star_solution.size(); ++i ) {
    solstate = dynamic_cast<NavGraphSearchState *>(a_star_solution[i]);
    plan_.push_back(solstate->node());
  }

  if (plan_.empty()) {
    logger->log_error(name(), "Failed to generate plan to travel to '%s'",
		      goal_name.c_str());
  }
}

void
NavGraphThread::generate_plan(float x, float y, float ori)
{
  TopologicalMapNode close_to_goal = graph_->closest_node(x, y);
  generate_plan(close_to_goal.name());

  TopologicalMapNode n("free-target", x, y);
  n.set_property("orientation", ori);
  plan_.push_back(n);
}


bool
NavGraphThread::replan(const TopologicalMapNode &start, const TopologicalMapNode &goal)
{
  logger->log_debug(name(), "Starting at node '%s'", start.name().c_str());

  TopologicalMapNode act_goal = goal;

  TopologicalMapNode close_to_goal;
  if (goal.name() == "free-target") {
    close_to_goal = graph_->closest_node(goal.x(), goal.y());
    act_goal = close_to_goal;
  }

  NavGraphSearchState *initial_state =
    new NavGraphSearchState(start, act_goal, *graph_, *constraint_repo_);
  std::vector<AStarState *> a_star_solution =  astar_->solve(initial_state);
  
  if (! a_star_solution.empty()) {
    // get cost of current plan
    TopologicalMapNode pose("current-pose", pose_.getOrigin().x(), pose_.getOrigin().y());
    TopologicalMapNode prev(pose);
    float old_cost = 0.;
    for (const TopologicalMapNode &n : plan_) {
      old_cost += NavGraphSearchState::cost(prev, n);
      prev = n;
    }

    NavGraphSearchState *ngss = dynamic_cast<NavGraphSearchState *>(a_star_solution[0]);

    float new_cost =
      NavGraphSearchState::cost(pose, ngss->node())
      + a_star_solution[a_star_solution.size() - 1]->total_estimated_cost;

    if (new_cost <= old_cost * cfg_replan_factor_) {
      constrained_plan_ = true;
      plan_.clear();
      NavGraphSearchState *solstate;
      for (unsigned int i = 0; i < a_star_solution.size(); ++i ) {
	solstate = dynamic_cast<NavGraphSearchState *>(a_star_solution[i]);
	plan_.push_back(solstate->node());
      }
      if (goal.name() == "free-target") {
	// add free target node again
	plan_.push_back(goal);
      }
      logger->log_info(name(), "Executing after re-planning from '%s' to '%s', "
		       "old cost: %f  new cost: %f (%f * %f)",
		       plan_[0].name().c_str(), goal.name().c_str(),
		       old_cost, new_cost * cfg_replan_factor_, new_cost, cfg_replan_factor_);
      return true;
    } else {
      logger->log_warn(name(), "Re-planning from '%s' to '%s' resulted in "
		       "more expensive plan: %f > %f (%f * %f), keeping old",
		       start.name().c_str(), goal.name().c_str(),
		       new_cost, old_cost * cfg_replan_factor_, old_cost, cfg_replan_factor_);
      return false;
    }
  } else {
    logger->log_error(name(), "Failed to re-plan from '%s' to '%s'",
		      start.name().c_str(), goal.name().c_str());
    return false;
  }
}


/** Optimize the current plan.
 * Note that after generating a plan, the robot first needs to
 * travel to the first actual node from a free position within
 * the environment. It can happen, that this closest node lies
 * in the opposite direction of the second node, hence the robot
 * needs to "go back" first, and only then starts following
 * the path. We can optimize this by removing the first node,
 * so that the robot directly travels to the second node which
 * "lies on the way".
 */
void
NavGraphThread::optimize_plan()
{
  if (plan_.size() > 1) {
    // get current position of robot in map frame
    double sqr_dist_a = ( pow(pose_.getOrigin().x() - plan_[0].x(), 2) +
                          pow(pose_.getOrigin().y() - plan_[0].y(), 2) );
    double sqr_dist_b = ( pow(plan_[0].x() - plan_[1].x(), 2) +
                          pow(plan_[0].y() - plan_[1].y(), 2) );
    double sqr_dist_c = ( pow(pose_.getOrigin().x() - plan_[1].x(), 2) +
                          pow(pose_.getOrigin().y() - plan_[1].y(), 2) );

    if (sqr_dist_a + sqr_dist_b >= sqr_dist_c){
      plan_.erase(plan_.begin());
    }
  }
}


void
NavGraphThread::start_plan()
{
  path_planned_at_->stamp();

  target_reached_ = false;
  if (plan_.empty()) {
    exec_active_ = false;
    pp_nav_if_->set_final(true);
    pp_nav_if_->set_error_code(NavigatorInterface::ERROR_UNKNOWN_PLACE);
    logger->log_warn(name(), "Cannot start empty plan.");

#ifdef HAVE_VISUALIZATION
    if (vt_) {
      vt_->reset_plan();
      visualized_at_->stamp();
    }
#endif

  } else {    

    std::string m = plan_[0].name();
    for (unsigned int i = 1; i < plan_.size(); ++i) {
      m += " - " + plan_[i].name();
    }
    logger->log_info(name(), "Starting route: %s", m.c_str());
#ifdef HAVE_VISUALIZATION
    if (vt_) {
      vt_->set_plan(plan_);
      visualized_at_->stamp();
    }
#endif

    exec_active_ = true;

    TopologicalMapNode &final_target = plan_.back();

    pp_nav_if_->set_error_code(NavigatorInterface::ERROR_NONE);
    pp_nav_if_->set_final(false);
    pp_nav_if_->set_dest_x(final_target.x());
    pp_nav_if_->set_dest_y(final_target.y());

    try {
      logger->log_info(name(), "Sending next goal on plan start");
      send_next_goal();
    } catch (Exception &e) {
      logger->log_warn(name(), "Failed to send next goal (start plan)");
      logger->log_warn(name(), e);
    }
  }

  publish_path(plan_);
}


void
NavGraphThread::stop_motion()
{
  NavigatorInterface::StopMessage *stop = new NavigatorInterface::StopMessage();
  try {
    nav_if_->msgq_enqueue(stop);
  } catch (Exception &e) {
    logger->log_warn(name(), "Failed to stop motion, exception follows");
    logger->log_warn(name(), e);
  }
  last_node_ = "";
  exec_active_ = false;
  target_reached_ = false;
  pp_nav_if_->set_final(true);

#ifdef HAVE_VISUALIZATION
  if (vt_) {
    vt_->reset_plan();
    visualized_at_->stamp();
  }
#endif

}


void
NavGraphThread::send_next_goal()
{
  bool stop_at_target   = false;
  bool orient_at_target = false;

  if (plan_.empty()) {
    throw Exception("Cannot send next goal if plan is empty");
  }

  TopologicalMapNode &next_target = plan_.front();

  if (plan_.size() == 1) {
    stop_at_target = true;
  } else {
    stop_at_target = false;
  }

  float ori = 0.;
  if ( plan_.size() == 1 ) {
    if ( next_target.has_property("orientation") ) {
      orient_at_target  = true;

      // take the given orientation for the final node
      ori = next_target.property_as_float("orientation");
    } else {
      orient_at_target  = false;
      ori = NAN;
    }
  } else {
    orient_at_target  = false;

    // set direction facing from next_target (what is the actual point to drive to) to next point to drive to.
    // So orientation is the direction from next_target to the target after that

    TopologicalMapNode &next_next_target = plan_[1];//*(++plan_.begin());

    ori = atan2f( next_next_target.y() - next_target.y(),
                  next_next_target.x() - next_target.x());
  }

  // get target position in map frame
  tf::Stamped<tf::Pose> tpose;
  tf::Stamped<tf::Pose>
    tposeglob(tf::Transform(tf::create_quaternion_from_yaw(ori),
			    tf::Vector3(next_target.x(), next_target.y(), 0)),
	      Time(0,0), cfg_global_frame_);
  try {
    tf_listener->transform_pose(cfg_base_frame_, tposeglob, tpose);
  } catch (Exception &e) {
    logger->log_warn(name(),
		     "Failed to compute pose, cannot generate plan", e.what());
    throw;
  }

  NavigatorInterface::CartesianGotoMessage *gotomsg =
    new NavigatorInterface::CartesianGotoMessage(tpose.getOrigin().x(),
						 tpose.getOrigin().y(),
						 tf::get_yaw(tpose.getRotation()));

  NavigatorInterface::SetStopAtTargetMessage* stop_at_target_msg      = new NavigatorInterface::SetStopAtTargetMessage(stop_at_target);
  NavigatorInterface::SetOrientationModeMessage* orient_mode_msg;
  if ( orient_at_target ) {
    orient_mode_msg = new NavigatorInterface::SetOrientationModeMessage(
        fawkes::NavigatorInterface::OrientationMode::OrientAtTarget );
  } else {
    orient_mode_msg = new NavigatorInterface::SetOrientationModeMessage(
            fawkes::NavigatorInterface::OrientationMode::OrientDuringTravel );
  }

  try {
#ifdef HAVE_VISUALIZATION
    if (vt_)  vt_->set_current_edge(last_node_, next_target.name());
#endif

    if (! nav_if_->has_writer()) {
      throw Exception("No writer for navigator interface");
    }

    logger->log_debug(name(), "Sending goto(x=%f,y=%f,ori=%f) for node '%s'",
		      tpose.getOrigin().x(), tpose.getOrigin().y(),
		      tf::get_yaw(tpose.getRotation()), next_target.name().c_str());

    nav_if_->msgq_enqueue(stop_at_target_msg);
    nav_if_->msgq_enqueue(orient_mode_msg);

    nav_if_->msgq_enqueue(gotomsg);
    cmd_sent_at_->stamp();

    error_at_->stamp();
    error_reason_ = "";

  } catch (Exception &e) {
    if (cfg_abort_on_error_) {
      logger->log_warn(name(), "Failed to send cartesian goto for "
		       "next goal, exception follows");
      logger->log_warn(name(), e);
      exec_active_ = false;
      pp_nav_if_->set_final(true);
      pp_nav_if_->set_error_code(NavigatorInterface::ERROR_OBSTRUCTION);
      pp_nav_if_->write();
#ifdef HAVE_VISUALIZATION
      if (vt_)  vt_->reset_plan();
#endif
    } else {
      fawkes::Time now(clock);
      if (error_reason_ != e.what_no_backtrace() || (now - error_at_) > 4.0) {
	error_reason_ = e.what_no_backtrace();
	*error_at_ = now;
	logger->log_warn(name(), "Failed to send cartesian goto for "
			 "next goal, exception follows");
	logger->log_warn(name(), e);
	logger->log_warn(name(), "*** NOT aborting goal (as per config)");
      }
    }
  }
}


bool
NavGraphThread::node_reached()
{
  if (plan_.empty()) {
    logger->log_error(name(), "Cannot check node reached if plan is empty");
    return true;
  }

  TopologicalMapNode &cur_target = plan_.front();

  // get current position of robot in map frame
  float dist = sqrt(pow(pose_.getOrigin().x() - cur_target.x(), 2) +
		    pow( pose_.getOrigin().y() - cur_target.y(), 2));

  float tolerance = 0.;
  if (cur_target.has_property("travel_tolerance")) {
    tolerance = cur_target.property_as_float("travel_tolerance");
  }
  float default_tolerance = cfg_travel_tolerance_;
  // use a different tolerance for the final node
  if (plan_.size() == 1) {
    default_tolerance = cfg_target_tolerance_;
    if (cur_target.has_property("target_tolerance")) {
      tolerance = cur_target.property_as_float("target_tolerance");
    }
    if (cur_target.has_property("orientation")) {
      float ori_tolerance = cfg_orientation_tolerance_;
      //cur_target.property_as_float("orientation_tolerance");
      float ori_diff  =
	fabs( angle_distance( normalize_rad(tf::get_yaw(pose_.getRotation())),
			      normalize_rad(cur_target.property_as_float("orientation"))));
      
      if (tolerance == 0.)  tolerance = default_tolerance;
      
      //logger->log_info(name(), "Ori=%f Rot=%f Diff=%f Tol=%f Dist=%f Tol=%f", cur_target.property_as_float("orientation"), tf::get_yaw(pose_.getRotation() ), ori_diff, ori_tolerance, dist, tolerance);
      return (dist <= tolerance) && (ori_diff <= ori_tolerance);
    }
  }


  // can be no or invalid tolerance
  if (tolerance == 0.)  tolerance = default_tolerance;

  return (dist <= tolerance);
}



size_t
NavGraphThread::shortcut_possible()
{
  if (plan_.size() < 1) {
    logger->log_debug(name(), "Cannot shortcut if plan empty");
    return 0;
  }

  for (ssize_t i = plan_.size() - 1; i > 0; --i) {
    TopologicalMapNode &node = plan_[i];

    float dist = sqrt(pow(pose_.getOrigin().x() - node.x(), 2) +
		      pow(pose_.getOrigin().y() - node.y(), 2));

    float tolerance = cfg_shortcut_tolerance_;
    if (node.has_property("shortcut_tolerance")) {
      tolerance = node.property_as_float("shortcut_tolerance");
    }

    if (tolerance == 0.0)  return 0;
    if (dist <= tolerance) return i;
  }

  return 0;
}


void
NavGraphThread::fam_event(const char *filename, unsigned int mask)
{
  // The file will be ignored from now onwards, re-register
  if (mask & FAM_IGNORED) {
    fam_->watch_file(cfg_graph_file_.c_str());
  }

  if (mask & (FAM_MODIFY | FAM_IGNORED)) {
    logger->log_info(name(), "Graph changed on disk, reloading");

    try {
      LockPtr<TopologicalMapGraph> new_graph = load_graph(cfg_graph_file_);
      **graph_ = **new_graph;
    } catch (Exception &e) {
      logger->log_warn(name(), "Loading new graph failed, exception follows");
      logger->log_warn(name(), e);
      return;
    }

#ifdef HAVE_VISUALIZATION
    if (vt_) {
      vt_->set_graph(graph_);
      visualized_at_->stamp();
    }
#endif

    if (exec_active_) {
      // store the goal and restart it after the graph has been reloaded

      stop_motion();
      TopologicalMapNode goal = plan_.back();

      if (goal.name() == "free-target") {
	generate_plan(goal.x(), goal.y(), goal.property_as_float("orientation"));
	optimize_plan();
      } else {
	generate_plan(goal.name());
	optimize_plan();
      }

      start_plan();
    }
  }
}


void
NavGraphThread::log_graph()
{
  const std::vector<TopologicalMapNode> & nodes = graph_->nodes();
  std::vector<TopologicalMapNode>::const_iterator n;
  for (n = nodes.begin(); n != nodes.end(); ++n) {
    logger->log_info(name(), "Node %s @ (%f,%f)%s",
		     n->name().c_str(), n->x(), n->y(),
		     n->unconnected() ? " UNCONNECTED" : "");

    const std::map<std::string, std::string> &props = n->properties();
    std::map<std::string, std::string>::const_iterator p;
    for (p = props.begin(); p != props.end(); ++p) {
      logger->log_info(name(), "  - %s: %s", p->first.c_str(), p->second.c_str());
    }
  }

  std::vector<TopologicalMapEdge> edges = graph_->edges();
  std::vector<TopologicalMapEdge>::iterator e;
  for (e = edges.begin(); e != edges.end(); ++e) {
    logger->log_info(name(), "Edge %10s --%s %s",
		     e->from().c_str(), e->is_directed() ? ">" : "-", e->to().c_str());

    const std::map<std::string, std::string> &props = e->properties();
    std::map<std::string, std::string>::const_iterator p;
    for (p = props.begin(); p != props.end(); ++p) {
      logger->log_info(name(), "  - %s: %s", p->first.c_str(), p->second.c_str());
    }
  }
}

void
NavGraphThread::publish_path(std::vector<fawkes::TopologicalMapNode> path)
{
  std::vector<std::string> vpath(40, "");

  for (unsigned int i = 0; i < path.size() && i < vpath.size(); ++i) {
    vpath[i] = path[i].name();
  }

  path_if_->set_path_node_1(vpath[0].c_str());
  path_if_->set_path_node_2(vpath[1].c_str());
  path_if_->set_path_node_3(vpath[2].c_str());
  path_if_->set_path_node_4(vpath[3].c_str());
  path_if_->set_path_node_5(vpath[4].c_str());
  path_if_->set_path_node_6(vpath[5].c_str());
  path_if_->set_path_node_7(vpath[6].c_str());
  path_if_->set_path_node_8(vpath[7].c_str());
  path_if_->set_path_node_9(vpath[8].c_str());
  path_if_->set_path_node_10(vpath[9].c_str());
  path_if_->set_path_node_11(vpath[10].c_str());
  path_if_->set_path_node_12(vpath[11].c_str());
  path_if_->set_path_node_13(vpath[12].c_str());
  path_if_->set_path_node_14(vpath[13].c_str());
  path_if_->set_path_node_15(vpath[14].c_str());
  path_if_->set_path_node_16(vpath[15].c_str());
  path_if_->set_path_node_17(vpath[16].c_str());
  path_if_->set_path_node_18(vpath[17].c_str());
  path_if_->set_path_node_19(vpath[18].c_str());
  path_if_->set_path_node_20(vpath[19].c_str());
  path_if_->set_path_node_21(vpath[20].c_str());
  path_if_->set_path_node_22(vpath[21].c_str());
  path_if_->set_path_node_23(vpath[22].c_str());
  path_if_->set_path_node_24(vpath[23].c_str());
  path_if_->set_path_node_25(vpath[24].c_str());
  path_if_->set_path_node_26(vpath[25].c_str());
  path_if_->set_path_node_27(vpath[26].c_str());
  path_if_->set_path_node_28(vpath[27].c_str());
  path_if_->set_path_node_29(vpath[28].c_str());
  path_if_->set_path_node_30(vpath[29].c_str());
  path_if_->set_path_node_31(vpath[30].c_str());
  path_if_->set_path_node_32(vpath[31].c_str());
  path_if_->set_path_node_33(vpath[32].c_str());
  path_if_->set_path_node_34(vpath[33].c_str());
  path_if_->set_path_node_35(vpath[34].c_str());
  path_if_->set_path_node_36(vpath[35].c_str());
  path_if_->set_path_node_37(vpath[36].c_str());
  path_if_->set_path_node_38(vpath[37].c_str());
  path_if_->set_path_node_39(vpath[38].c_str());
  path_if_->set_path_node_40(vpath[39].c_str());
  path_if_->set_path_length(path.size());
  path_if_->write();
}
