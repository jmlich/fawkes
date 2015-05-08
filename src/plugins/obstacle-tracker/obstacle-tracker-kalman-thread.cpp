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
#include "object-estimator.h"

#include <tf/utils.h>
#include <cmath>
using namespace fawkes;
using namespace MatrixWrapper;


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

  /************ CREATE FILTER FOR EVERY INTERFACE *************************************/
  unsigned int i = 0;
  for(auto& interface : cluster_ifs_){
	  i++;
	  if(i==1) {
		  logger->log_info(name(),"Create Filter for Interface %s", interface->id());
		  filter_ = new ObjectEstimator(logger, clock, "odom", "tracked_object_1");
	  }
  }
}


void
ObstacleTrackerKalmanThread::finalize()
{

  // clear list of interfaces
  cluster_ifs_.clear();
  delete filter_;

}

/*
 *
 * Initialize Cluster for testing
 * Has to be moved into loop
 * if new object is detected, initialize filter with new object
 *
 */
void
ObstacleTrackerKalmanThread::once()
{

  int i =0;
  for( auto& interface : cluster_ifs_){

	  interface->read();

	  if (i == 0) {

		 // INITIALIZATION OF FILTER -> GOES INTO LOOP LATER ON

		  // create stamped-transform from Quaternion and Euler Vektor at current time from Interface
		  tf::Quaternion q(interface->rotation(0),interface->rotation(1),
							interface->rotation(2),interface->rotation(3));
		  tf::Vector3 vtrans(interface->translation(0), interface->translation(1), interface->translation(2));
		  fawkes::tf::StampedTransform cluster_transform_stamped;

		  fawkes::Time now(clock);
		  cluster_transform_stamped = fawkes::tf::StampedTransform(fawkes::tf::Transform(q,vtrans), now, "odom", "tracked_object_1");

		  filter_->initialize(cluster_transform_stamped, now);
		  i++;
	  }
  }

}

/*
 *
 *
 *
 */
void
ObstacleTrackerKalmanThread::loop()
{

  /* If there is a new Object, initialize filter */
	// is currently in once()


  /* get measurements and add measurements to filter */

  unsigned int i=0;
  for( auto& interface : cluster_ifs_){

	  interface->read();

	  if (i == 0) {


		  // create stamped-transform from Quaternion and Euler Vektor at current time from Interface
		  tf::Quaternion q(interface->rotation(0),interface->rotation(1),
				            interface->rotation(2),interface->rotation(3));
		  tf::Vector3 vtrans(interface->translation(0), interface->translation(1), interface->translation(2));
		  fawkes::tf::StampedTransform cluster_transform_stamped;

		  fawkes::Time now(clock);
		  cluster_transform_stamped = fawkes::tf::StampedTransform(fawkes::tf::Transform(q,vtrans), now, "odom", "tracked_object_1");

		  fawkes::Time now_l(clock);
		  filter_->update(now_l,cluster_transform_stamped);

	  }

	  // update if value changes since last loop
	  i++;

  }// end for
}// end loop
