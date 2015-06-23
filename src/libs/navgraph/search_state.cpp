/***************************************************************************
 *  search_state.cpp - Graph-based global path planning - A-Star search state
 *
 *  Created: Tue Sep 18 18:21:56 2012
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
 *             2002  Stefan Jacobs
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

#include <navgraph/search_state.h>

#include <functional>
#include <cmath>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class NavGraphSearchState <navgraph/search_state.h>
 * Graph-based path planner A* search state.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param path graph path this search state represents
 * @param goal graph node of the goal
 * @param cost_sofar the cost until to this node from the start
 * @param parent parent search state
 * @param map_graph map graph
 * @param estimate_func function to estimate the cost from any node to the goal.
 * Note that the estimate function must be admissible for optimal A* search. That
 * means that for no query may the calculated estimate be higher than the actual
 * cost.
 * @param cost_func function to calculate the cost from a node to another adjacent
 * node. Note that the cost function is directly related to the estimate function.
 * For example, the cost can be calculated in terms of distance between nodes, or in
 * time that it takes to travel from one node to the other. The estimate function must
 * match the cost function to be admissible.
 * @param constraint_repo constraint repository, null to plan only without constraints
 */
NavGraphSearchState::NavGraphSearchState(NavGraphPath path, NavGraphNode goal,
					 double cost_sofar, NavGraphSearchState *parent,
					 NavGraph *map_graph,
					 navgraph::EstimateFunction estimate_func,
					 navgraph::CostFunction cost_func,
					 fawkes::NavGraphConstraintRepo *constraint_repo)
  : AStarState(cost_sofar, parent), estimate_func_(estimate_func), cost_func_(cost_func)
{
  path_ = path;
  goal_ = goal;
  map_graph_ = map_graph;

  total_estimated_cost = path_cost + estimate();

  std::hash<std::string> h;
  key_ = h(path_.name());

  constraint_repo_ = constraint_repo;
}


/** Constructor.
 * @param path graph path this search state represents
 * @param goal graph node of the goal
 * @param map_graph map graph
 * @param constraint_repo constraint repository, null to plan only without constraints
 */
NavGraphSearchState::NavGraphSearchState(NavGraphPath path, NavGraphNode goal,
					 NavGraph *map_graph,
					 fawkes::NavGraphConstraintRepo *constraint_repo)
  : AStarState(0, NULL)
{
  path_ = path;
  goal_ = goal;
  map_graph_ = map_graph;

  estimate_func_ = straight_line_estimate;
  cost_func_ = euclidean_cost;

  total_estimated_cost = path_cost + estimate();

  std::hash<std::string> h;
  key_ = h(path_.name());

  constraint_repo_ = constraint_repo;
}


/** Constructor.
 * @param path graph path this search state represents
 * @param goal graph node of the goal
 * @param map_graph map graph
 * @param estimate_func function to estimate the cost from any node to the goal.
 * Note that the estimate function must be admissible for optimal A* search. That
 * means that for no query may the calculated estimate be higher than the actual
 * cost.
 * @param cost_func function to calculate the cost from a node to another adjacent
 * node. Note that the cost function is directly related to the estimate function.
 * For example, the cost can be calculated in terms of distance between nodes, or in
 * time that it takes to travel from one node to the other. The estimate function must
 * match the cost function to be admissible.
 * @param constraint_repo constraint repository, null to plan only without constraints
 */
NavGraphSearchState::NavGraphSearchState(NavGraphPath path, NavGraphNode goal,
					 NavGraph *map_graph,
					 navgraph::EstimateFunction estimate_func,
					 navgraph::CostFunction cost_func,
					 fawkes::NavGraphConstraintRepo *constraint_repo)
  : AStarState(0, NULL), estimate_func_(estimate_func), cost_func_(cost_func)
{
  path_ = path;
  goal_ = goal;
  map_graph_ = map_graph;

  total_estimated_cost = path_cost + estimate();

  std::hash<std::string> h;
  key_ = h(path_.name());

  constraint_repo_ = constraint_repo;
}


/** Destructor. */
NavGraphSearchState::~NavGraphSearchState()
{
}


/** Get graph node corresponding to this search state.
 * @return graph node corresponding to this search state
 */
fawkes::NavGraphPath &
NavGraphSearchState::path()
{
  return path_;
}


float
NavGraphSearchState::estimate()
{
  return estimate_func_(path_, goal_);
}


bool
NavGraphSearchState::is_goal()
{
  return (path_.nodes().back().name() == goal_.name());
}


std::vector<AStarState *>
NavGraphSearchState::children()
{
  std::vector< AStarState * > children;
  children.clear();

  std::vector<std::string> descendants = path_.nodes().back().reachable_nodes();

  for (unsigned int i = 0; i < descendants.size(); ++i) {
    NavGraphNode d = map_graph_->node(descendants[i]);

    bool expand = true;
    if (constraint_repo_) {
      if (constraint_repo_->blocks(d)) {
	expand = false;
      } else if (constraint_repo_->blocks(path_.back(), d)) {
	expand = false;
      }
    }

    if (expand) {
      float d_cost = cost_func_(path_, d);

      if (constraint_repo_) {
	float cost_factor = 0.;
	if (constraint_repo_->increases_cost(path_.back(), d, cost_factor)) {
	  d_cost *= cost_factor;
	}
      }

      std::vector<NavGraphNode> new_nodes(path_.nodes());
      NavGraphPath new_path(map_graph_, new_nodes);
      new_path.add_node(d);
      children.push_back(new NavGraphSearchState(new_path, goal_, path_cost + d_cost, this,
						 map_graph_, estimate_func_, cost_func_,
						 constraint_repo_));
    }
  }

  return children;
}

} // end of namespace fawkes
