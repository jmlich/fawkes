
/***************************************************************************
 *  visualization_thread.h - Visualization for obstacle-tracker via rviz
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

#ifndef __PLUGIN_OBSTACLE_TRACKER_KALMAN_VISUALIZATION_THREAD_H_
#define __PLUGIN_OBSTACLE_TRACKER_KALMAN_VISUALIZATION_THREAD_H_

#include "obstacle-tracker-kalman-thread.h"

#include <vector>

#include <core/threading/thread.h>
#include <core/threading/mutex.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <plugins/ros/aspect/ros.h>

#include <ros/publisher.h>
#include <visualization_msgs/MarkerArray.h>


class ObstacleTrackerVisualizationThread
: public fawkes::Thread,
  public fawkes::ConfigurableAspect,
  public fawkes::LoggingAspect,
  public fawkes::ROSAspect,
  public fawkes::TransformAspect
{
 public:
  ObstacleTrackerVisualizationThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  void publish(std::vector<fawkes::tf::Point> measurements, std::vector<fawkes::tf::Point> objects);

 private:
  std::string  cfg_global_frame_;
  std::vector<fawkes::tf::Point> measurements_;
  std::vector<fawkes::tf::Point> objects_;

  size_t last_id_num_;
  ros::Publisher vispub_;

};

#endif
