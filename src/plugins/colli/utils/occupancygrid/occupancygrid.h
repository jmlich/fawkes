
/***************************************************************************
 *  occupancygrid.h - An occupancy-grid
 *
 *  Created: Fri Oct 18 15:16:23 2013
 *  Copyright  2002  AllemaniACs
 *             2013-2014  Bahram Maleki-Fard
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

#ifndef __PLUGINS_COLLI_UTILS_OCCUPANCYGRID_OCCUPANCYGRID_H_
#define __PLUGINS_COLLI_UTILS_OCCUPANCYGRID_OCCUPANCYGRID_H_

#include "probability.h"
#include "../../common/types.h"

#include <utils/math/types.h>
#include <vector>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class RoboShapeColli;
class ColliObstacleMap;

class Logger;
class Configuration;

/** Occupancy threshold. */
const float OCCUPANCY_THRESHOLD = 0.45f;

class OccupancyGrid
{
 public:
  OccupancyGrid(Logger* logger, Configuration* config,int width, int height, int cell_width=5, int cell_height=5);
  virtual ~OccupancyGrid();

  ///\brief Get the cell width (in cm)
  int get_cell_width();

   ///\brief Get the cell height (in cm)
  int get_cell_height();

  ///\brief Get the width of the grid
  int get_width();

  ///\brief Get the height of the grid
  int get_height();

  ///\brief Resets the cell width (in cm)
  void set_cell_width(int cell_width);

  ///\brief Resets the cell height (in cm)
  void set_cell_height(int cell_height);

  ///\brief Resets the width of the grid and constructs a new empty grid
  void set_width(int width);

  ///\brief Resets the height of the grid and constructs a new empty grid
  void set_height(int height);

  ///\brief Reset the occupancy probability of a cell
  virtual void set_prob(int x, int y, Probability prob);

  ///\brief Resets all occupancy probabilities
  void fill(Probability prob);

  ///\brief Get the occupancy probability of a cell
  Probability get_prob(int x, int y);

  ///\brief Get the occupancy probability of a cell
  Probability& operator () (const int x, const int y);

  ///\brief Init a new empty grid with the predefined parameters */
  void init_grid();

  ///\brief Set the offset of base_link from laser
  void set_base_offset(float x, float y);

  ///\brief Put the laser readings in the occupancy grid
  void update_occ_grid( int mid_x, int mid_y, float inc );

  /// The occupancy probability of the cells in a 2D array
  std::vector<std::vector<Probability> > occupancy_probs_;

  ///\brief Get cell costs
  colli_cell_cost_t get_cell_costs() const;

 protected:

  /** Integrate a single obstacle
   * @param x x coordinate of obstacle center
   * @param y y coordinate of obstacle center
   * @param width total width of obstacle
   * @param height total height of obstacle
   */
  void integrate_obstacle( int x, int y, int width, int height );

  ///\brief Is called to put readings in the occupancy grid
  virtual void update_occ_grid_inner() = 0;

  bool cfg_write_spam_debug_;

  int cell_width_;   /**< Cell width in cm */
  int cell_height_;  /**< Cell height in cm */
  int width_;       /**< Width of the grid in # cells */
  int height_;      /**< Height of the grid in # cells */

  /** Costs for the cells in grid */
  colli_cell_cost_t cell_costs_;

  RoboShapeColli*    robo_shape_;   /**< my roboshape */
  ColliObstacleMap*  obstacle_map;  /**< fast obstacle map */

  point_t search_start_;      /**< the start point for the search in the grid */
  point_t offset_base_;       /**< in grid cells */

  /** Offsets to robot center */
  cart_coord_2d_t offset_laser_; /**< in meters */

  double  obstacle_increase_; /**< the actual factor of obstacle increasemend in this loop (if enabled) */

  bool cfg_obstacle_inc_ ;          /**< increasing obstacles or not */
  bool cfg_force_elipse_obstacle_;  /**< the used shape for obstacles */

  int cfg_emergency_stop_beams_used_;  /**< number of beams that are used to calculate the min distance to obstacle */
};

} // namespace fawkes

#endif
