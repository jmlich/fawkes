
/***************************************************************************
 *  og_laser.h - Occupancy grid for colli's A* search
 *
 *  Created: Fri Oct 18 15:16:23 2013
 *  Copyright  2002  Stefan Jacobs
 *             2013-2014  Bahram Maleki-Fard
 *             2014-2015  Tobias Neumann
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

#ifndef __PLUGINS_COLLI_SEARCH_OG_LASER_H_
#define __PLUGINS_COLLI_SEARCH_OG_LASER_H_

#include "../utils/occupancygrid/occupancygrid.h"

#include <utils/time/time.h>
#include <utils/math/types.h>
#include <string>
#include <list>
#include <utils/math/common.h>

#include <tf/transformer.h>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Laser360Interface;
class Velocity3DInterface;

class LaserOccupancyGrid : public OccupancyGrid
{
 public:
  LaserOccupancyGrid( Laser360Interface * laser, Logger* logger, Configuration* config, tf::Transformer* listener,
                      int width = 150, int height = 150, int cell_width = 5, int cell_height = 5);
  ~LaserOccupancyGrid();

  ///\brief Put the laser readings in the occupancy grid
  virtual void update_occ_grid_inner( );

  float obstacle_in_path_distance( float vx, float vy );

  ///\brief Reset all old readings and forget about the world state!
  void reset_old();

  ///\brief Get the laser's position in the grid
  point_t get_laser_position();

 private:
  class LaserPoint {
  public:
    cart_coord_2d_t coord;
    Time timestamp;

    LaserPoint() { }
//    LaserPoint(LaserPoint& src) {
//      coord     = src.coord;
//      timestamp = src.timestamp;
//    }
//    LaserPoint operator=(LaserPoint src) {
//      coord     = src.coord;
//      timestamp = src.timestamp;
//      return src;
//    }
  };

  void update_laser();

  void validate_old_laser_points(cart_coord_2d_t pos_robot, cart_coord_2d_t pos_new_laser_point);

  std::vector< LaserPoint >* transform_laser_points(std::vector< LaserPoint >& laser_points, tf::StampedTransform& transform);

  /** Integrate historical readings to the current occgrid. */
  void integrate_old_readings( int mid_x, int mid_y, float inc,
                               tf::StampedTransform& transform );

  /** Integrate the current readings to the current occgrid. */
  void integrate_new_readings( int mid_x, int mid_y, float inc,
                               tf::StampedTransform& transform );

  tf::Transformer* tf_listener_;
  std::string reference_frame_;
  std::string laser_frame_;

  Logger* logger_;
  Laser360Interface* if_laser_;

  std::vector< LaserPoint > new_readings_;
  std::vector< LaserPoint > old_readings_; /**< readings history */

  point_t laser_pos_; /**< the laser's position in the grid */

  /* interface buffer history */
  int if_buffer_size_;
  std::vector<bool> if_buffer_filled_;

  /** History concerned constants */
  float max_history_length_, min_history_length_;
  int initial_history_size_;

  /** Laser concerned settings */
  float min_laser_length_;
  float obstacle_distance_;

  bool  cfg_delete_invisible_old_obstacles_; /**< delete old invalid obstables or not */
  int   cfg_delete_invisible_old_obstacles_angle_min_ ;  /**< the min angle for old obstacles */
  int   cfg_delete_invisible_old_obstacles_angle_max_ ;  /**< the max angle for old obstacles */
  float angle_min_;   /**< the angle min in rad */
  float angle_range_; /**< the angle range from min - max */
};

} // namespace fawkes

#endif
