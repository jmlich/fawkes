/***************************************************************************
 *  obstacle-tracker-kalman_thread.cpp
 *
 *  Created: Tue Sep 18 16:00:34 2012
 *  Copyright  2015  Sebastian Reuter
 *
* ****************************************************************************/

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

#include "obstacle-tracker-kalman-thread.h"

#include <tf/utils.h>
#include <core/utils/lockptr.h>
#include <interfaces/Position3DInterface.h>
#include <interfaces/Velocity3DInterface.h>
#include <cmath>

#include <fstream>

using namespace fawkes;
using namespace MatrixWrapper;
using namespace BFL;


/** @class ObstacleTrackerKalmanThread "obstacle-tracker-kalman-thread.h"
 * Thread to track moving obstacles.
 * @author Sebastian Reuter
 */

/** Constructor. */
ObstacleTrackerKalmanThread::ObstacleTrackerKalmanThread()
  : Thread("ObstacleTrackerKalmanThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
{
}

/** Destructor. */
ObstacleTrackerKalmanThread::~ObstacleTrackerKalmanThread()
{
}

void
ObstacleTrackerKalmanThread::init()
{

  /************ GET CONFIGS *******************************************************/
  cfg_laser_cluster_iface_prefix_    = config->get_string("/obstacle-tracker/laser-cluster-interface-prefix");

  cfg_min_vishistory_  = config->get_int("/obstacle-tracker/min-visibility-history");

  /************ OPEN INTERFACES *******************************************************/
  std::string pattern = cfg_laser_cluster_iface_prefix_ + "*";
  cluster_ifs_ = blackboard->open_multiple_for_reading<Position3DInterface>(pattern.c_str());


}

void
ObstacleTrackerKalmanThread::finalize()
{

  cluster_ifs_.clear();

}

void
ObstacleTrackerKalmanThread::once()
{
}

void
ObstacleTrackerKalmanThread::loop()
{

  for(auto& interface : cluster_ifs_){


      interface->read();

	  if (interface->visibility_history() >= cfg_min_vishistory_) {
	      	  // get translation of obstacle from interface
	      	  timed_translation cluster_trans;
	      	  cluster_trans.x = interface->translation(0);
	      	  cluster_trans.y = interface->translation(1);

	      	  // get current time
	      	  fawkes::Time now(clock);
	      	  cluster_trans.time = now;

	  }

  }// end for
}// end loop
