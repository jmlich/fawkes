
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

#include <filter/extendedkalmanfilter.h>

#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>

#include <pdf/analyticconditionalgaussian.h>
#include <pdf/linearanalyticconditionalgaussian.h>

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


  struct interface_pair{
	  fawkes::Position3DInterface* ClusterPosition3DInterface;
	  fawkes::Velocity3DInterface* ClusterVelocity3DInterface;
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
  std::string										cfg_cluster_velocity_iface_prefix_;
  int  												cfg_min_vishistory_;
  fawkes::LockList<fawkes::Position3DInterface *>  	cluster_ifs_;
  fawkes::LockList<fawkes::Velocity3DInterface *>  	velocity_ifs_;
  std::map<std::string, interface_pair>  			dyn_object_ifs_;

 private:
  // pdf / model / filter
  BFL::AnalyticSystemModelGaussianUncertainty*            sys_model_;
  BFL::NonLinearAnalyticConditionalGaussianOdo*           sys_pdf_;
  BFL::LinearAnalyticConditionalGaussian*                 cluster_meas_pdf_;
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty* cluster_meas_model_;
  BFL::Gaussian*                                          prior_;
  BFL::ExtendedKalmanFilter*                              filter_;
  MatrixWrapper::SymmetricMatrix                          odom_covariance_, imu_covariance_, vo_covariance_, gps_covariance_;

  // vars
  MatrixWrapper::ColumnVector vel_desi_, filter_estimate_old_vec_;
  fawkes::tf::Transform filter_estimate_old_;
  fawkes::tf::StampedTransform cluster_meas_;
  fawkes::Time filter_time_old_;
  bool filter_initialized_, cluster_initialized_;

  std::string output_frame_;
  fawkes::tf::Transformer transformer_;




};

#endif
