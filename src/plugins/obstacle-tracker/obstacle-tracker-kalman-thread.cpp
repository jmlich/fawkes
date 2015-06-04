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
#include "visualization_thread.h"

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
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS)
{
   vt_ = NULL;
}


/** Constructor. */
ObstacleTrackerKalmanThread::ObstacleTrackerKalmanThread(ObstacleTrackerVisualizationThread *vt)
: Thread("ObstacleTrackerKalmanThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_POST_LOOP)
{
  vt_ = vt;
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

  cfg_visualization_ = false;
  try {
    cfg_visualization_ = config->get_bool(cfg_prefix + "visualization/enable");
  } catch (Exception &e) {} // ignore, use default

  if (cfg_visualization_) {
    logger->log_info(name(), "Visualization enabled");
  }
  else {
    logger->log_info(name(), "Visualization disabled");
  }
}


void
ObstacleTrackerKalmanThread::finalize()
{

  // clear list of interfaces
  cluster_ifs_.clear();
  objects_.clear();
  measurements_.clear();

  if(filter_) delete filter_;

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

  measurements_.clear();
  objects_.clear();


  unsigned int i=0;
  for( auto& interface : cluster_ifs_){

	  interface->read();


	  if (i == 0) {

		  tf::Point cluster_as_point(interface->translation(0), interface->translation(1), interface->translation(2));
		  const fawkes::Time cluster_time = interface->timestamp();
		  tf::Stamped<tf::Point> stamped_cluster_point(cluster_as_point, cluster_time, measurement_frame_id_);

		  /*
		   * Measurement is pushed into vector - has to be assigned to fliter later on
		   */
		  measurements_.push_back(stamped_cluster_point);

		  /*
		   *
		   * following update has to be moved down !
		   *
		   */
		  filter_->update(stamped_cluster_point);

		  tf::Stamped<tf::Point> positionEstimate = filter_->getPositionEstimate(fawkes::Time(clock));
		  objects_.push_back( positionEstimate );

		  logger->log_info(name(), "measurement: %f %f - object estimate: %f %f", stamped_cluster_point.getX(),stamped_cluster_point.getY(), positionEstimate.getX(), positionEstimate.getY());
	  }
	  i++;
  }// end for

  // find corresponding filter for measurement
  // Hungarian-Method

  // update filter with current measurements

  // get estimate for current position from filter

  // write estimate into Velocity3DInterface

  /*
   * visualize
   */
  if (cfg_visualization_){

	  publish_visualization();
  }

}// end loop()


void
ObstacleTrackerKalmanThread::publish_visualization()
{
  if (vt_) {

	std::vector<tf::Point> meas;
	std::vector<tf::Point> objs;

	// unpack points of stamped measurements
	for( fawkes::tf::Stamped<fawkes::tf::Point> &m : measurements_){

		fawkes::tf::Stamped<tf::Point> trans_m;

		// try to transform measurement into reference-frame
		try{
			tf_listener->transform_point(reference_frame_id_, m, trans_m);
		}
		catch(Exception &e){
			// try to transform measurement at timestamp 0,0
			m.stamp = fawkes::Time(0,0);
			try{
				tf_listener->transform_point(reference_frame_id_, m, trans_m);
			}
			catch(Exception &e){
			    //logger->log_warn(name(), "Transform of measurement at clock(0,0) didn't work - will take raw-measurement ");
			}
		}

		// cast transform into point
		fawkes::tf::Point transformed_point(trans_m.getX(),trans_m.getY(),trans_m.getZ());
		meas.push_back(transformed_point);

	}

	// unpack points of stamped objects
	for( fawkes::tf::Stamped<fawkes::tf::Point> &o : objects_){
		fawkes::tf::Point transformed_point(o.getX(),o.getY(),o.getZ());
		objs.push_back(transformed_point);
	}
    vt_->publish(meas, objs);
  }
  else{
	   logger->log_warn(name(),"VisualizationThread 'vt' is NULL - something went wrong !");
  }
}
