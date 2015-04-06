
/***************************************************************************
 *  occupancygrid.cpp - An occupancy-grid
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

#include <logging/logger.h>
#include <config/config.h>

#include "../rob/roboshape_colli.h"
#include "../../search/obstacle_map.h"

#include "occupancygrid.h"

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class OccupancyGrid <plugins/colli/utils/occupancygrid/occupancygrid.h>
 * Occupancy Grid class for general use. Many derivated classes
 * exist, which are usually used instead of this general class.
 * Note: the coord system is assumed to map x onto width an y onto
 * height, with x being the first coordinate !
 */

/** Constructs an empty occupancy grid
 *
 * @param width the width of the grid in # of cells
 * @param height the height of the cells in # of cells
 * @param cell_width the cell width in cm
 * @param cell_height the cell height in cm
 */
OccupancyGrid::OccupancyGrid(Logger* logger, Configuration* config, int width, int height, int cell_width, int cell_height)
{
  std::string cfg_prefix = "/plugins/colli/";
  cfg_write_spam_debug_  = config->get_bool((cfg_prefix + "write_spam_debug").c_str());

  cfg_obstacle_inc_          = config->get_bool((cfg_prefix + "obstacle_increasement").c_str());
  cfg_force_elipse_obstacle_ = config->get_bool((cfg_prefix + "laser_occupancy_grid/force_ellipse_obstacle").c_str());

  cell_costs_.occ  = config->get_int((cfg_prefix + "laser_occupancy_grid/cell_cost/occupied").c_str());
  cell_costs_.near = config->get_int((cfg_prefix + "laser_occupancy_grid/cell_cost/near").c_str());
  cell_costs_.mid  = config->get_int((cfg_prefix + "laser_occupancy_grid/cell_cost/mid").c_str());
  cell_costs_.far  = config->get_int((cfg_prefix + "laser_occupancy_grid/cell_cost/far").c_str());
  cell_costs_.free = config->get_int((cfg_prefix + "laser_occupancy_grid/cell_cost/free").c_str());

  robo_shape_ = new RoboShapeColli( (cfg_prefix + "roboshape/").c_str(), logger, config );

  logger->log_debug("OccupancyGrid", "Generating obstacle map");
  bool obstacle_shape = robo_shape_->is_angular_robot() && ! cfg_force_elipse_obstacle_;
  obstacle_map = new ColliObstacleMap(cell_costs_, obstacle_shape);
  logger->log_debug("OccupancyGrid", "Generating obstacle map done");

  // calculate laser offset from robot center
  offset_base_.x=0;
  offset_base_.y=0;
  offset_laser_.x = robo_shape_->get_complete_width_x()/2.f - robo_shape_->get_robot_length_for_deg(0);
  offset_laser_.y = robo_shape_->get_complete_width_y()/2.f - robo_shape_->get_robot_length_for_deg(90);
  logger->log_debug("LaserOccupancyGrid", "Laser (x,y) offset from robo-center is (%f, %f)",
                    offset_laser_.x, offset_laser_.y);


  width_ = width;
  height_ = height;
  cell_width_ = cell_width;
  cell_height_ = cell_height;

  init_grid();
}

/** Destructor */
OccupancyGrid::~OccupancyGrid()
{
  delete robo_shape_;
  occupancy_probs_.clear();
}

/** Get the cell width
 * @return the cell width in cm
 */
int
OccupancyGrid::get_cell_width()
{
  return cell_width_;
}

/** Get the cell height
 * @return the height of the cells in cm
 */
int
OccupancyGrid::get_cell_height()
{
  return cell_height_;
}

/** Get the width of the grid
 * @return the width of the grid in # of cells
 */
int
OccupancyGrid::get_width()
{
  return width_;
}

/** Get the height of the grid
 * @return the height of the grid in # cells
 */
int
OccupancyGrid::get_height()
{
  return height_;
}

/** Resets the cell width
 * @param width the width of the cells in cm
 */
void
OccupancyGrid::set_cell_width(int width)
{
  cell_width_ = width;
}

/** Resets the cell height
 * @param height the height of the cells in cm
 */
void
OccupancyGrid::set_cell_height(int height)
{
  cell_height_ = height;
}

/** Resets the width of the grid and constructs a new empty grid
 * @param width the cell width in cm
 */
void
OccupancyGrid::set_width(int width)
{
  width_ = width;
  init_grid();
}

/** Resets the height of the grid and constructs a new empty grid
 * @param height the height of the grid in # of cells
 */
void
OccupancyGrid::set_height(int height)
{
  height_ = height;
  init_grid();
}


/** Reset the occupancy probability of a cell
 * @param x the x-position of the cell
 * @param y the y-position of the cell
 * @param prob the occupancy probability of cell (x,y)
 */
void
OccupancyGrid::set_prob(int x, int y, Probability prob)
{
  if( (x < width_) && (y < height_) && ((isProb(prob)) || (prob == 2.f)) )
    occupancy_probs_[x][y] = prob;
}

/** Resets all occupancy probabilities
 * @param prob the occupancy probability the grid will become filled with
 */
void
OccupancyGrid::fill(Probability prob)
{
  if((isProb(prob)) || (prob == -1.f)) {
    for(int x = 0; x < width_; x++) {
      for(int y = 0; y < height_; y++) {
        occupancy_probs_[x][y] = prob;
      }
    }
  }
}

/** Get the occupancy probability of a cell
 * @param x the x-position of the cell
 * @param y the y-position of the cell
 * @return the occupancy probability of cell (x,y)
 */
Probability
OccupancyGrid::get_prob(int x, int y)
{
  if( (x >= 0) && (x < width_) && (y >= 0) && (y < height_) ) {
    return occupancy_probs_[x][y];
  } else {
    return 1;
  }
}

/** Operator (), get occupancy probability of a cell
 * @param x the x-position of the cell
 * @param y the y-position of the cell
 * @return the occupancy probability of cell (x,y)
 */
Probability&
OccupancyGrid::operator () (const int x, const int y)
{
  return occupancy_probs_[x][y];
}

/** Init a new empty grid with the predefined parameters */
void
OccupancyGrid::init_grid()
{
  occupancy_probs_.clear();
  std::vector<Probability> column;
  column.resize(height_, 0.f);
  occupancy_probs_.resize(width_, column);
  fill( 0.f );
}

/** Set the offset of base_link from laser.
 * @param x offset in x-direction (in meters)
 * @param y offset in y-direction (in meters)
 */
void
OccupancyGrid::set_base_offset(float x, float y)
{
  offset_base_.x = (int)round( (offset_laser_.x + x)*100.f/cell_height_ ); // # in grid-cells
  offset_base_.y = (int)round( (offset_laser_.y + y)*100.f/cell_width_  );
}


/** Get cell costs.
 * @return struct that contains all the cost values for the occgrid cells
 */
colli_cell_cost_t
OccupancyGrid::get_cell_costs() const
{
  return cell_costs_;
}

void
OccupancyGrid::integrate_obstacle( int x, int y, int width, int height )
{
  std::vector< int > fast_obstacle = obstacle_map->get_obstacle( width, height, cfg_obstacle_inc_ );

  int posX = 0;
  int posY = 0;

  // i = x offset, i+1 = y offset, i+2 is cost
  for( unsigned int i = 0; i < fast_obstacle.size(); i+=3 ) {
    /* On the laser-points, we draw obstacles based on base_link. The obstacle has the robot-shape,
     * which means that we need to rotate the shape 180° around base_link and move that rotation-
     * point onto the laser-point on the grid. That's the same as adding the center_to_base_offset
     * to the calculated position of the obstacle-center ("x + fast_obstacle[i]" and "y" respectively).
     */
    posX = x + fast_obstacle[i]   + offset_base_.x;
    posY = y + fast_obstacle[i+1] + offset_base_.y;

    if( (posX > 0) && (posX < height_)
     && (posY > 0) && (posY < width_)
     && (occupancy_probs_[posX][posY] < fast_obstacle[i+2]) )
      {
      occupancy_probs_[posX][posY] = fast_obstacle[i+2];
    }
  }
}

/** Put the laser readings in the occupancy grid
 *  Also, every reading gets a radius according to the relative direction
 *  of this reading to the robot.
 * @param midX is the current x position of the robot in the grid.
 * @param midY is the current y position of the robot in the grid.
 * @param inc is the current constant to increase the obstacles.
 * @param vx Translation x velocity of the motor
 * @param vy Translation y velocity of the motor
 * @return distance to next obstacle in pathdirection
 */
void
OccupancyGrid::update_occ_grid( int midX, int midY, float inc)
{
  for ( int y = 0; y < width_; ++y )
    for ( int x = 0; x < height_; ++x )
      occupancy_probs_[x][y] = cell_costs_.free;

  search_start_.x = midX;
  search_start_.y = midY;

  obstacle_increase_ = inc;

  update_occ_grid_inner();
}

} // namespace fawkes
