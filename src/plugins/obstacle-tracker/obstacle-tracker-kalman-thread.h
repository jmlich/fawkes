
/***************************************************************************
 *  obstacle-tracker-average-thread.h
 *
 *  Created: Sun Apr 21 01:17:09 2013
 *  Copyright  2015  Sebastian Reuter
 *
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

#ifndef __PLUGIN_OBSTACLE_TRACKER_KALMAN_THREAD_H_
#define __PLUGIN_OBSTACLE_TRACKER_KALMAN_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <core/utils/lock_list.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/tf.h>
#include <tuple>

#include <bfl/filter/extendedkalmanfilter.h>

#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>

#include <bfl/pdf/analyticconditionalgaussian.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>

#include <bfl/wrappers/matrix/matrix_wrapper.h>


namespace fawkes {
  class Position3DInterface;
  class Velocity3DInterface;
  class Time;
}

class ObstacleTrackerKalmanThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::TransformAspect
{
 private:

  /*
   *  map of dynamic obstacle detections
   */
  struct timed_translation{
			  double x,y,z;
			  fawkes::Time time;
  };

 public:
  ObstacleTrackerKalmanThread();
  virtual ~ObstacleTrackerKalmanThread();

  /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
  protected: virtual void run() { Thread::run();}

  virtual void init();
  virtual void once();
  virtual void loop();
  virtual void finalize();

 private:
  std::string  										cfg_laser_cluster_iface_prefix_;
  int  												cfg_min_vishistory_;
  fawkes::LockList<fawkes::Position3DInterface *>  	cluster_ifs_;

 private:

};

#endif
