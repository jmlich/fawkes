/***************************************************************************
 *  ObstacleTracker_thread.cpp
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

#include "obstacle-tracker-average-thread.h"

#include <tf/utils.h>
#include <core/utils/lockptr.h>
#include <interfaces/Position3DInterface.h>
#include <interfaces/Velocity3DInterface.h>
#include <cmath>


#include <fstream>

using namespace fawkes;

/** @class ObstacleTrackerAverageThread "ObstacleTracker_thread.h"
 * Thread to track moving obstacles.
 * @author Sebastian Reuter
 */

/** Constructor. */
ObstacleTrackerAverageThread::ObstacleTrackerAverageThread()
  : Thread("ObstacleTrackerAverageThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
{
}

/** Destructor. */
ObstacleTrackerAverageThread::~ObstacleTrackerAverageThread()
{
}

void
ObstacleTrackerAverageThread::init()
{

  /************ GET CONFIGS *******************************************************/
  cfg_laser_cluster_iface_prefix_    = config->get_string("/obstacle-tracker/laser-cluster-interface-prefix");
  cfg_cluster_velocity_iface_prefix_    = config->get_string("/obstacle-tracker/cluster-velocity-interface-prefix");

  cfg_min_vishistory_  = config->get_int("/obstacle-tracker/min-visibility-history");
  cfg_moving_average_number_ = config->get_int("/obstacle-tracker/moving-average-number");

  /************ OPEN INTERFACES *******************************************************/
  std::string pattern = cfg_laser_cluster_iface_prefix_ + "*";
  cluster_ifs_ = blackboard->open_multiple_for_reading<Position3DInterface>(pattern.c_str());

  for (Position3DInterface *pif : cluster_ifs_) {
	  std::string cluster_name = pif->id();
	  std::string id = cluster_name.substr( cluster_name.find_last_of("/\\")+1 );

	  /************ OPEN VELOCITY INTERFACES *********************************************/
	  pattern = cfg_cluster_velocity_iface_prefix_ + id ;
	  fawkes::Velocity3DInterface* dyn_object;
	  dyn_object = blackboard->open_for_writing<Velocity3DInterface>(pattern.c_str());

	  /************ Put Position/Velocity INterface Pair in map****************************/
	  interface_pair i_pair;
	  i_pair.ClusterPosition3DInterface = pif;
	  i_pair.ClusterVelocity3DInterface = dyn_object;
	  dyn_object_ifs_[id] = i_pair;

	  /************ INITIALIZE OBSACTE_MAP *************************************************/
	  // map-key is id of interface
	  // capacity of circular buffer is specified by cfg-value 'cfg_moving_average_number_'
	  circular_translation_buffer new_circle_buffer{ cfg_moving_average_number_ };
	  obstacle_map_[id] = new_circle_buffer;
  }

}

void
ObstacleTrackerAverageThread::finalize()
{

  // unregister velocity interface
  for(auto& interface : dyn_object_ifs_){
      blackboard->close(interface.second.ClusterVelocity3DInterface);
  }

  dyn_object_ifs_.clear();
  cluster_ifs_.clear();
  obstacle_map_.clear();

}

void
ObstacleTrackerAverageThread::once()
{
}

void
ObstacleTrackerAverageThread::loop()
{

  for(auto& interface : dyn_object_ifs_){


	  fawkes::Position3DInterface* pif = interface.second.ClusterPosition3DInterface;
      fawkes::Velocity3DInterface* vif = interface.second.ClusterVelocity3DInterface;
      std::string id = interface.first;

      pif->read();

	  if (pif->visibility_history() >= cfg_min_vishistory_) {
	        try {

	      	  // get translation of obstacle from interface
	      	  timed_translation cluster_trans;
	      	  cluster_trans.x = pif->translation(0);
	      	  cluster_trans.y = pif->translation(1);
	      	  cluster_trans.z = pif->translation(2);

	      	  // get current time
	      	  fawkes::Time now(clock);
	      	  cluster_trans.time = now;

	      	  // push translation into circular-buffer
	      	  obstacle_map_[id].push_back( cluster_trans );

	      	  float average_x_velocity = 0;
	      	  float average_y_velocity = 0;
	      	  float average_absolute_velocity = 0;

	      	  timed_translation old_trans; // wil be used to keep track of last known position
	      	  int i = 0;
	      	  // calculate moving average
	      	  for( auto trans : obstacle_map_[id] ){

	      		  // calculate distances and average velocities
	      		  if(i>0){

	      			  // axial velocities and restulting velocity
	      			  float x_travel_distance =  trans.x - old_trans.x ;
	      			  float y_travel_distance =  trans.y - old_trans.y ;
	      			  float absolute_travel_distance = std::sqrt(  powf( trans.x - old_trans.x, 2 ) +
	      					  	  	  	  	  	  	  	  	  	   powf( trans.y - old_trans.y, 2 ) );

	      			  float travel_time = trans.time.in_sec() - old_trans.time.in_sec();

	      			  average_x_velocity += (x_travel_distance / travel_time) ; // [m/s]
	      			  average_y_velocity += (y_travel_distance / travel_time) ; // [m/s]
	      			  average_absolute_velocity += (absolute_travel_distance / travel_time) ; // [m/s]

	      			  /*
	      			  if(id=="0"){
	      				  logger->log_info(name(), "pos_x=%f pos_y=%f at time=%f", trans.x, trans.y, travel_time );
	      				  logger->log_info(name(), " Times new=%f old=%f diff=%f", trans.time.in_sec(), old_trans.time.in_sec(), travel_time );
	      				  logger->log_info(name(), "Time=%f",cluster_trans.time.in_sec() );
	      			  }
	      			  */

	      		  }
	      		  old_trans = trans;
	      		  i++;
	      	  }

	      	  average_x_velocity = average_x_velocity / obstacle_map_.size();
	      	  average_y_velocity = average_y_velocity / obstacle_map_.size();
	      	  average_absolute_velocity = average_absolute_velocity / obstacle_map_.size();

	      	  /*
	      	  if(id=="1"){
	      	    logger->log_info(name(), "Velocities =  abs%f x=%f y=%f", average_absolute_velocity, average_x_velocity, average_y_velocity );
	      	  }
	      	  */

	      	  // SET VELOCITY INTERFACE
	      	  vif->set_visibility_history(pif->visibility_history());
	      	  vif->set_translation(pif->translation());
	      	  vif->set_rotation(pif->rotation());
	      	  // angular velocity is not calculated yet !
	      	  // vif->set_angular_velocity();
	      	  double lin_velocity[3] = {average_x_velocity, average_y_velocity, 0};
	      	  vif->set_linear_velocity( lin_velocity );
	      	  vif->set_absolute_linear_velocity(average_absolute_velocity);
	      	  vif->write();

	        }
	        catch (Exception &e) {
	        	logger->log_info(name(), "Failed to track cluster %s, ignoring", pif->uid());
	        }
	  }

	  // if there are items within the buffer and visibility history is less than 5
	  else if (obstacle_map_[id].size()>0) {

		  // clear buffer - so that old values will not be used
		  obstacle_map_[id].clear();

		  // clear Interface
		  double zero_val_three[3] = {0, 0, 0};
		  double zero_val_four[4] = {0, 0, 0, 0};
      	  vif->set_visibility_history(pif->visibility_history());
      	  vif->set_translation(zero_val_three);
      	  vif->set_rotation(zero_val_four);
      	  vif->set_linear_velocity( zero_val_three );
      	  vif->set_absolute_linear_velocity(0);
      	  vif->write();

	  }

  }// end for
}// end loop
