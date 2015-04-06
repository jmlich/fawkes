
/***************************************************************************
 *  og_robo_vel.cpp - Occupancy grid for colli's A* search
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

#include "og_robo_vel.h"
#include "obstacle_map.h"

#include "../utils/rob/roboshape_colli.h"

#include <utils/time/clock.h>
#include <utils/math/angle.h>
#include <utils/math/coord.h>

#include <logging/logger.h>
#include <config/config.h>

#include <interfaces/Velocity3DInterface.h>
#include <blackboard/interface_list_maintainer.h>

#include <cmath>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class LaserOccupancyGrid <plugins/colli/search/og_laser.h>
 *  This OccGrid is derived by the Occupancy Grid originally from Andreas Strack,
 *    but modified for speed purposes.
 */

/** Constructor.
 * @param ifs_velocity the BlackBoardInterfaceListMaintainer of the Velocity3DInterfaces
 * @param logger The fawkes logger
 * @param config The fawkes configuration
 * @param listener The tf::Transformer
 * @param width The width of the grid (in m)
 * @param height The height of the grid (in m)
 * @param cell_width The width of a cell (in cm)
 * @param cell_height The height of a cell (in cm)
 */
OccupancyGridRobotVelocity::OccupancyGridRobotVelocity( BlackBoardInterfaceListMaintainer *ifs_velocity,
                                        Logger* logger, Configuration* config, tf::Transformer* listener,
                                        int width, int height, int cell_width, int cell_height)
 : OccupancyGrid( logger, config, width, height, cell_width, cell_height ),
   tf_listener_(listener ),
   logger_( logger ),
   ifs_velocity_( ifs_velocity)
{
  logger->log_debug("LaserOccupancyGrid", "(Constructor): Entering");

  //read config
  std::string cfg_prefix = "/plugins/colli/";
  obstacle_distance_     = config->get_float((cfg_prefix + "laser_occupancy_grid/distance_account").c_str());
  initial_history_size_  = 3*config->get_int((cfg_prefix + "laser_occupancy_grid/history/initial_size").c_str());
  max_history_length_    = config->get_float((cfg_prefix + "laser_occupancy_grid/history/max_length").c_str());
  min_history_length_    = config->get_float((cfg_prefix + "laser_occupancy_grid/history/min_length").c_str());
  min_laser_length_      = config->get_float((cfg_prefix + "laser/min_reading_length").c_str());
  cfg_write_spam_debug_  = config->get_bool((cfg_prefix + "write_spam_debug").c_str());

  cfg_emergency_stop_beams_used_ = config->get_float((cfg_prefix + "emergency_stopping/beams_used").c_str());

  cfg_delete_invisible_old_obstacles_           = config->get_bool((cfg_prefix + "laser_occupancy_grid/history/delete_invisible_old_obstacles/enable").c_str());
  cfg_delete_invisible_old_obstacles_angle_min_ = config->get_int((cfg_prefix + "laser_occupancy_grid/history/delete_invisible_old_obstacles/angle_min").c_str());
  cfg_delete_invisible_old_obstacles_angle_max_ = config->get_int((cfg_prefix + "laser_occupancy_grid/history/delete_invisible_old_obstacles/angle_max").c_str());
  if ( cfg_delete_invisible_old_obstacles_angle_min_ >= 360 ) {
    logger_->log_warn("LaserOccupancyGrid", "Min angle out of bounce, use 0");
    cfg_delete_invisible_old_obstacles_angle_min_ = 0;
  }
  if ( cfg_delete_invisible_old_obstacles_angle_min_ >= 360 ) {
    logger_->log_warn("LaserOccupancyGrid", "Max angle out of bounce, use 360");
    cfg_delete_invisible_old_obstacles_angle_min_ = 360;
  }

  if (cfg_delete_invisible_old_obstacles_angle_max_ > cfg_delete_invisible_old_obstacles_angle_min_) {
    angle_range_ = deg2rad((unsigned int)abs(cfg_delete_invisible_old_obstacles_angle_max_ - cfg_delete_invisible_old_obstacles_angle_min_));
  } else {
    angle_range_ = deg2rad((360 - cfg_delete_invisible_old_obstacles_angle_min_) + cfg_delete_invisible_old_obstacles_angle_max_);
  }
  angle_min_ = deg2rad( cfg_delete_invisible_old_obstacles_angle_min_ );


  reference_frame_ = config->get_string((cfg_prefix + "frame/odometry").c_str());
  laser_frame_     = config->get_string((cfg_prefix + "frame/laser").c_str());       //TODO change to base_link => search in base_link instead base_laser

  cfg_obstacle_inc_          = config->get_bool((cfg_prefix + "obstacle_increasement").c_str());
  cfg_force_elipse_obstacle_ = config->get_bool((cfg_prefix + "laser_occupancy_grid/force_ellipse_obstacle").c_str());

  if_buffer_size_ = config->get_int((cfg_prefix + "laser_occupancy_grid/buffer_size").c_str());
  if_buffer_size_ = std::max(if_buffer_size_, 1); //needs to be >= 1, because the data is always wrote into the buffer (instead of read())

  cell_costs_.occ  = config->get_int((cfg_prefix + "laser_occupancy_grid/cell_cost/occupied").c_str());
  cell_costs_.near = config->get_int((cfg_prefix + "laser_occupancy_grid/cell_cost/near").c_str());
  cell_costs_.mid  = config->get_int((cfg_prefix + "laser_occupancy_grid/cell_cost/mid").c_str());
  cell_costs_.far  = config->get_int((cfg_prefix + "laser_occupancy_grid/cell_cost/far").c_str());
  cell_costs_.free = config->get_int((cfg_prefix + "laser_occupancy_grid/cell_cost/free").c_str());

  if_buffer_filled_.resize(if_buffer_size_);
  std::fill(if_buffer_filled_.begin(), if_buffer_filled_.end(), false);

  robo_shape_ = new RoboShapeColli( (cfg_prefix + "roboshape/").c_str(), logger, config );
  init_grid();

  logger->log_debug("LaserOccupancyGrid", "Generating obstacle map");
  bool obstacle_shape = robo_shape_->is_angular_robot() && ! cfg_force_elipse_obstacle_;
  obstacle_map = new ColliObstacleMap(cell_costs_, obstacle_shape);
  logger->log_debug("LaserOccupancyGrid", "Generating obstacle map done");

  laser_pos_ = point_t(0,0);

  // calculate laser offset from robot center
  offset_base_.x=0;
  offset_base_.y=0;
  offset_laser_.x = robo_shape_->get_complete_width_x()/2.f - robo_shape_->get_robot_length_for_deg(0);
  offset_laser_.y = robo_shape_->get_complete_width_y()/2.f - robo_shape_->get_robot_length_for_deg(90);
  logger->log_debug("LaserOccupancyGrid", "Laser (x,y) offset from robo-center is (%f, %f)",
                    offset_laser_.x, offset_laser_.y);


  logger->log_debug("LaserOccupancyGrid", "(Constructor): Exiting");
}

/** Descturctor. */
OccupancyGridRobotVelocity::~OccupancyGridRobotVelocity()
{
  delete robo_shape_;
}

void
OccupancyGridRobotVelocity::update_robot_velocitys()
{
  // get list of interfaces
  std::list<Velocity3DInterface *> ifs_vel = ifs_velocity_->lock_and_get_list<fawkes::Velocity3DInterface>();
  // copy data of interfaces (the interfaces itself are just to be used untill unlocked)
  velocitys_.clear();
  for(  std::list<Velocity3DInterface *>::iterator i = ifs_vel.begin();
        i != ifs_vel.end();
        ++i ) {
    RobotVelocity vel;
    vel.point.x     = (*i)->translation(0);
    vel.point.y     = (*i)->translation(1);
    vel.velocity.x  = (*i)->linear_velocity(0);
    vel.velocity.y  = (*i)->linear_velocity(1);
    vel.radius      = 0.2;

    velocitys_.push_back( vel );
  }
  // unlock list of interfaces
  ifs_velocity_->unlock_list();
}

/** Set the offset of base_link from laser.
 * @param x offset in x-direction (in meters)
 * @param y offset in y-direction (in meters)
 */
void
OccupancyGridRobotVelocity::set_base_offset(float x, float y)
{
  offset_base_.x = (int)round( (offset_laser_.x + x)*100.f/cell_height_ ); // # in grid-cells
  offset_base_.y = (int)round( (offset_laser_.y + y)*100.f/cell_width_  );
}

} // namespace fawkes
