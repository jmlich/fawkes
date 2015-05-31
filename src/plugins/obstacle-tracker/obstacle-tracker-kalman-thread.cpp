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
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_POST_LOOP)
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
  std::string cfg_prefix = "/obstacle-tracker/";
  measurement_frame_id_ = config->get_string((cfg_prefix + "frame/measurement-frame-id").c_str());
  reference_frame_id_ = config->get_string((cfg_prefix + "frame/reference-frame-id").c_str());
  object_frame_id_prefix_ = config->get_string((cfg_prefix + "frame/object-frame-id-prefix").c_str());

  cfg_laser_cluster_iface_prefix_ = config->get_string((cfg_prefix + "laser-cluster-interface-prefix").c_str());
  cfg_min_vishistory_ = config->get_int((cfg_prefix + "min-visibility-history").c_str());
  std::string cfg_odom_if_id = config->get_string((cfg_prefix + "odom-interface").c_str());

  /************ OPEN INTERFACES *******************************************************/
  std::string pattern = cfg_laser_cluster_iface_prefix_ + "*";
  cluster_ifs_ = blackboard->open_multiple_for_reading<Position3DInterface>(pattern.c_str());
  odom_if_ = blackboard->open_for_reading<MotorInterface>( cfg_odom_if_id.c_str() );

  /************ CREATE FILTER FOR EVERY INTERFACE *************************************/
  unsigned int i = 0;
  for(auto& interface : cluster_ifs_){
	  i++;
	  if(i==1) {
		  logger->log_info(name(),"Create Filter for Interface %s", interface->id());
		  filter_ = new ObjectEstimator(logger, clock, config, tf_listener, odom_if_);
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

  unsigned int i =0;
  for( auto& interface : cluster_ifs_){

	  interface->read();

	  if (i == 0) {

		  logger->log_info(name(),"Initialize Filter for Interface %s", interface->id());
		 // INITIALIZATION OF FILTER -> GOES INTO LOOP LATER ON

		  tf::Point cluster_as_point(interface->translation(0), interface->translation(1), interface->translation(2));
		  const fawkes::Time cluster_time = interface->timestamp();
		  tf::Stamped<tf::Point> stamped_cluster_point(cluster_as_point, cluster_time, measurement_frame_id_);

		  filter_->initialize(stamped_cluster_point);
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

		  tf::Point cluster_as_point(interface->translation(0), interface->translation(1), interface->translation(2));
		  const fawkes::Time cluster_time = interface->timestamp();
		  tf::Stamped<tf::Point> stamped_cluster_point(cluster_as_point, cluster_time, measurement_frame_id_);

		  filter_->update(stamped_cluster_point);

	  }

	  // update if value changes since last loop
	  i++;

  }// end for
}// end loop
