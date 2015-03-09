
/***************************************************************************
 *  astar_search.h - A colli-specific A* search implementation
 *
 *  Created: Fri Oct 18 15:16:23 2013
 *  Copyright  2002  Stefan Jacobs
 *             2013  Bahram Maleki-Fard
 *             2015  Tobias Neumann
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

#ifndef __PLUGINS_COLLI_SEARCH_ASTAR_SPEED_SEARCH_H_
#define __PLUGINS_COLLI_SEARCH_ASTAR_SPEED_SEARCH_H_

#include "abstract_search.h"

#include <vector>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class LaserOccupancyGrid;
class AStarSpeed;
class Logger;
class Configuration;

typedef struct point_struct point_t;

/** This is the plan class.
 *  Here the plan from A* is managed and cut into small pieces.
 *    Also usable methods for managing the plan are implemented here.
 */
class SearchAStarSpeed: public AbstractSearch
{
 public:
  SearchAStarSpeed( LaserOccupancyGrid * occ_grid , Logger* logger, Configuration* config);
  virtual ~SearchAStarSpeed();

  ///\brief update complete plan things
  void update( int robo_x, int robo_y, int target_x, int target_y );

 private:

  /** Returns the current, modified waypoint to drive to. */
  point_t calculate_local_target();

  /** Adjust the waypoint if it is not the final point. */
  point_t adjust_waypoint( const point_t &local_target );

  /** Returns the current trajectory point to drive to. */
  point_t calculate_local_trajec_point( );

  /** Method for checking if an obstacle is between two points. */
  bool is_obstacle_between( const point_t &a, const point_t &b, const int maxcount );

  AStarSpeed * astar_;              /**< the A* search algorithm */

  int cfg_search_line_allowed_cost_max_; /**< the config value for the max allowed costs on the line search on the a-star result */

  fawkes::Logger* logger_;
};

} // namespace fawkes

#endif
