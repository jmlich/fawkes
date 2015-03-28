/***************************************************************************
 *  amcl_thread.cpp - Thread to perform localization
 *
 *  Created: Wed May 16 16:04:41 2012
 *  Copyright  2012-2015  Tim Niemueller [www.niemueller.de]
 *             2012       Daniel Ewert
 *             2012       Kathrin Goffart (Robotino Hackathon 2012)
 *             2012       Kilian Hinterwaelder  (Robotino Hackathon 2012)
 *             2012       Marcel Prochnau (Robotino Hackathon 2012)
 *             2012       Jannik Altgen (Robotino Hackathon 2012)
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

/* Based on amcl_node.cpp from the ROS amcl package
 * Copyright (c) 2008, Willow Garage, Inc.
 */

#include "amcl_thread.h"
#include "amcl_utils.h"

#include <utils/math/angle.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <baseapp/run.h>
#include <cstdlib>
#include <cstdio>
#include <cstring>

#ifdef HAVE_ROS
#  include <ros/node_handle.h>
#  include <geometry_msgs/PoseArray.h>
#  ifdef USE_MAP_PUB
#    include <nav_msgs/OccupancyGrid.h>
#  endif
#endif

using namespace fawkes;

static double normalize(double z) {
  return atan2(sin(z), cos(z));
}

static double angle_diff(double a, double b) {
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a - b;
  d2 = 2 * M_PI - fabs(d1);
  if (d1 > 0)
    d2 *= -1.0;
  if (fabs(d1) < fabs(d2))
    return (d1);
  else
    return (d2);
}

/** @class AmclThread "amcl_thread.h"
 * Thread to perform Adaptive Monte Carlo Localization.
 * @author Tim Niemueller
 */

std::vector<std::pair<int,int> > AmclThread::free_space_indices;

/** Constructor. */
AmclThread::AmclThread()
  : Thread("AmclThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS),
    TransformAspect(TransformAspect::BOTH, "Pose"),
    BlackBoardInterfaceListener("AmclThread")
{
  map_ = NULL;
  conf_mutex_ = new Mutex();
}

/** Destructor. */
AmclThread::~AmclThread()
{
  delete conf_mutex_;
}

void AmclThread::init()
{
  map_ = NULL;

  fawkes::amcl::read_map_config(config, cfg_map_file_, cfg_resolution_, cfg_origin_x_,
				cfg_origin_y_, cfg_origin_theta_, cfg_occupied_thresh_,
				cfg_free_thresh_);

  cfg_laser_ifname_ = config->get_string(AMCL_CFG_PREFIX"laser_interface_id");
  cfg_pose_ifname_ = config->get_string(AMCL_CFG_PREFIX"pose_interface_id");

  map_ = fawkes::amcl::read_map(cfg_map_file_.c_str(),
				cfg_origin_x_, cfg_origin_y_, cfg_resolution_,
				cfg_occupied_thresh_, cfg_free_thresh_, free_space_indices);
  map_width_  = map_->size_x;
  map_height_ = map_->size_y;

  logger->log_info(name(), "Size: %ux%u (%zu of %u cells free, this are %.1f%%)",
                   map_width_, map_height_, free_space_indices.size(),
		   map_width_ * map_height_,
                   (float)free_space_indices.size() / (float)(map_width_ * map_height_) * 100.);

  save_pose_last_time.set_clock(clock);
  save_pose_last_time.stamp();

  sent_first_transform_ = false;
  latest_tf_valid_ = false;
  pf_ = NULL;
  resample_count_ = 0;
  odom_ = NULL;
  laser_ = NULL;
  // private_nh_="~";
  initial_pose_hyp_ = NULL;
  first_map_received_ = false;
  first_reconfigure_call_ = true;

  init_pose_[0] = 0.0;
  init_pose_[1] = 0.0;
  init_pose_[2] = 0.0;
  init_cov_[0] = 0.5 * 0.5;
  init_cov_[1] = 0.5 * 0.5;
  init_cov_[2] = (M_PI / 12.0) * (M_PI / 12.0);

  save_pose_period_ = config->get_float(AMCL_CFG_PREFIX"save_pose_period");
  laser_min_range_ = config->get_float(AMCL_CFG_PREFIX"laser_min_range");
  laser_max_range_ = config->get_float(AMCL_CFG_PREFIX"laser_max_range");
  pf_err_ = config->get_float(AMCL_CFG_PREFIX"kld_err");
  pf_z_ = config->get_float(AMCL_CFG_PREFIX"kld_z");
  alpha1_ = config->get_float(AMCL_CFG_PREFIX"alpha1");
  alpha2_ = config->get_float(AMCL_CFG_PREFIX"alpha2");
  alpha3_ = config->get_float(AMCL_CFG_PREFIX"alpha3");
  alpha4_ = config->get_float(AMCL_CFG_PREFIX"alpha4");
  alpha5_ = config->get_float(AMCL_CFG_PREFIX"alpha5");
  z_hit_ = config->get_float(AMCL_CFG_PREFIX"z_hit");
  z_short_ = config->get_float(AMCL_CFG_PREFIX"z_short");
  z_max_ = config->get_float(AMCL_CFG_PREFIX"z_max");
  z_rand_ = config->get_float(AMCL_CFG_PREFIX"z_rand");
  sigma_hit_ = config->get_float(AMCL_CFG_PREFIX"sigma_hit");
  lambda_short_ = config->get_float(AMCL_CFG_PREFIX"lambda_short");
  laser_likelihood_max_dist_ =
    config->get_float(AMCL_CFG_PREFIX"laser_likelihood_max_dist");
  d_thresh_ = config->get_float(AMCL_CFG_PREFIX"d_thresh");
  a_thresh_ = config->get_float(AMCL_CFG_PREFIX"a_thresh");
  t_thresh_ = config->get_float(AMCL_CFG_PREFIX"t_thresh");
  alpha_slow_ = config->get_float(AMCL_CFG_PREFIX"alpha_slow");
  alpha_fast_ = config->get_float(AMCL_CFG_PREFIX"alpha_fast");
  angle_increment_ = deg2rad(config->get_float(AMCL_CFG_PREFIX"angle_increment"));
  try {
    angle_min_idx_ = config->get_uint(AMCL_CFG_PREFIX"angle_min_idx");
    if (angle_min_idx_ > 359) {
      throw Exception("Angle min index out of bounds");
    }
  } catch (Exception &e) {
    angle_min_idx_ = 0;
  }
  try {
    angle_max_idx_ = config->get_uint(AMCL_CFG_PREFIX"angle_max_idx");
    if (angle_max_idx_ > 359) {
      throw Exception("Angle max index out of bounds");
    }
  } catch (Exception &e) {
    angle_max_idx_ = 359;
  }
  if (angle_max_idx_ > angle_min_idx_) {
    angle_range_ = (unsigned int)abs(angle_max_idx_ - angle_min_idx_);
  } else {
    angle_range_ = (360 - angle_min_idx_) + angle_max_idx_;
  }
  angle_min_ = deg2rad(angle_min_idx_);

  max_beams_ = config->get_uint(AMCL_CFG_PREFIX"max_beams");
  min_particles_ = config->get_uint(AMCL_CFG_PREFIX"min_particles");
  max_particles_ = config->get_uint(AMCL_CFG_PREFIX"max_particles");
  resample_interval_ = config->get_uint(AMCL_CFG_PREFIX"resample_interval");

  odom_frame_id_ = config->get_string(AMCL_CFG_PREFIX"odom_frame_id");
  base_frame_id_ = config->get_string(AMCL_CFG_PREFIX"base_frame_id");
  laser_frame_id_ = config->get_string(AMCL_CFG_PREFIX"laser_frame_id");
  global_frame_id_ = config->get_string(AMCL_CFG_PREFIX"global_frame_id");

  std::string tmp_model_type;
  tmp_model_type = config->get_string(AMCL_CFG_PREFIX"laser_model_type");

  if (tmp_model_type == "beam")
    laser_model_type_ = ::amcl::LASER_MODEL_BEAM;
  else if (tmp_model_type == "likelihood_field")
    laser_model_type_ = ::amcl::LASER_MODEL_LIKELIHOOD_FIELD;
  else {
    logger->log_warn(name(),
		     "Unknown laser model type \"%s\"; "
                     "defaulting to likelihood_field model",
		     tmp_model_type.c_str());
    laser_model_type_ = ::amcl::LASER_MODEL_LIKELIHOOD_FIELD;
  }

  tmp_model_type = config->get_string(AMCL_CFG_PREFIX"odom_model_type");
  if (tmp_model_type == "diff")
    odom_model_type_ = ::amcl::ODOM_MODEL_DIFF;
  else if (tmp_model_type == "omni")
    odom_model_type_ = ::amcl::ODOM_MODEL_OMNI;
  else {
    logger->log_warn(name(),
		     "Unknown odom model type \"%s\"; defaulting to diff model",
		     tmp_model_type.c_str());
    odom_model_type_ = ::amcl::ODOM_MODEL_DIFF;
  }

  try {
    init_pose_[0] = config->get_float(AMCL_CFG_PREFIX"init_pose_x");
  } catch (Exception &e) {} // ignored, use default

  try {
    init_pose_[1] = config->get_float(AMCL_CFG_PREFIX"init_pose_y");
  } catch (Exception &e) {} // ignored, use default

  try {
    init_pose_[2] = config->get_float(AMCL_CFG_PREFIX"init_pose_a");
  } catch (Exception &e) {} // ignored, use default

  cfg_read_init_cov_ = false;
  try {
    cfg_read_init_cov_ = config->get_bool(AMCL_CFG_PREFIX"read_init_cov");
  } catch (Exception &e) {} // ignored, use default

  if (cfg_read_init_cov_) {
    try {
      init_cov_[0] = config->get_float(AMCL_CFG_PREFIX"init_cov_xx");
    } catch (Exception &e) {} // ignored, use default

    try {
      init_cov_[1] = config->get_float(AMCL_CFG_PREFIX"init_cov_yy");
    } catch (Exception &e) {} // ignored, use default

    try {
      init_cov_[2] = config->get_float(AMCL_CFG_PREFIX"init_cov_aa");
    } catch (Exception &e) {} // ignored, use default
  } else {
    logger->log_debug(name(), "Reading initial covariance from config disabled");
  }

  transform_tolerance_ = config->get_float(AMCL_CFG_PREFIX"transform_tolerance");

  if (min_particles_ > max_particles_) {
    logger->log_warn(name(),
		     "You've set min_particles to be less than max particles, "
                     "this isn't allowed so they'll be set to be equal.");
    max_particles_ = min_particles_;
  }

  //logger->log_info(name(),"calling pf_alloc with %d min and %d max particles",
  //                 min_particles_, max_particles_);
  pf_ = pf_alloc(min_particles_, max_particles_, alpha_slow_, alpha_fast_,
		 (pf_init_model_fn_t) AmclThread::uniform_pose_generator,
		 (void *) map_);

  pf_init_model(pf_, (pf_init_model_fn_t)AmclThread::uniform_pose_generator,
		(void *)map_);

  pf_->pop_err = pf_err_;
  pf_->pop_z = pf_z_;

  // Initialize the filter

  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  //pf_init_pose_mean.v[0] = last_published_pose.pose.pose.position.x;
  //pf_init_pose_mean.v[1] = last_published_pose.pose.pose.position.y;
  //double *q_values = pos3d_if_->rotation();
  //tf::Quaternion q(q_values[0], q_values[1], q_values[2], q_values[3]);
  //btScalar unused_pitch, unused_roll, yaw;
  //btMatrix3x3(q).getRPY(unused_roll, unused_pitch, yaw);

  pf_init_pose_mean.v[0] = init_pose_[0];
  pf_init_pose_mean.v[1] = init_pose_[1];
  pf_init_pose_mean.v[2] = init_pose_[2];

  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = init_cov_[0]; //last_covariance_[6 * 0 + 0];
  pf_init_pose_cov.m[1][1] = init_cov_[1]; //last_covariance_[6 * 1 + 1];
  pf_init_pose_cov.m[2][2] = init_cov_[2]; //last_covariance_[6 * 5 + 5];
  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
  pf_init_ = false;

  initial_pose_hyp_ = new amcl_hyp_t();
  initial_pose_hyp_->pf_pose_mean = pf_init_pose_mean;
  initial_pose_hyp_->pf_pose_cov = pf_init_pose_cov;

  // Instantiate the sensor objects
  // Odometry
  odom_ = new ::amcl::AMCLOdom();

  if (odom_model_type_ == ::amcl::ODOM_MODEL_OMNI)
    odom_->SetModelOmni(alpha1_, alpha2_, alpha3_, alpha4_, alpha5_);
  else
    odom_->SetModelDiff(alpha1_, alpha2_, alpha3_, alpha4_);

  // Laser
  laser_ = new ::amcl::AMCLLaser(max_beams_, map_);

  if (laser_model_type_ == ::amcl::LASER_MODEL_BEAM) {
    laser_->SetModelBeam(z_hit_, z_short_, z_max_, z_rand_, sigma_hit_,
			 lambda_short_, 0.0);
  } else {
    logger->log_info(name(),
		     "Initializing likelihood field model; "
                     "this can take some time on large maps...");
    laser_->SetModelLikelihoodField(z_hit_, z_rand_, sigma_hit_,
				    laser_likelihood_max_dist_);
    logger->log_info(name(), "Done initializing likelihood field model.");
  }

  laser_pose_set_ = set_laser_pose();

  last_move_time_ = new Time(clock);
  last_move_time_->stamp();

#ifdef HAVE_ROS
  pose_pub_ =
    rosnode->advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 2);
  particlecloud_pub_ =
    rosnode->advertise<geometry_msgs::PoseArray>("particlecloud", 2);
  initial_pose_sub_ =
    rosnode->subscribe("initialpose", 2,
		       &AmclThread::initial_pose_received, this);
#  ifdef USE_MAP_PUB
  map_pub_ = rosnode->advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  publish_map();
#  endif
#endif

  laser_if_ =
    blackboard->open_for_reading<Laser360Interface>(cfg_laser_ifname_.c_str());
  pos3d_if_ =
    blackboard->open_for_writing<Position3DInterface>(cfg_pose_ifname_.c_str());
  loc_if_ =
    blackboard->open_for_writing<LocalizationInterface>("AMCL");

  bbil_add_message_interface(loc_if_);
  blackboard->register_listener(this, BlackBoard::BBIL_FLAG_MESSAGES);

  cfg_buffer_enable_ = true;
  try {
    cfg_buffer_enable_ = config->get_bool(AMCL_CFG_PREFIX"buffering/enable");
  } catch (Exception &e) {} // ignored, use default

  cfg_buffer_debug_ = true;
  try {
    cfg_buffer_debug_ = config->get_bool(AMCL_CFG_PREFIX"buffering/debug");
  } catch (Exception &e) {} // ignored, use default

  laser_buffered_ = false;

  if (cfg_buffer_enable_) {
    laser_if_->resize_buffers(1);
  }

  pos3d_if_->set_frame(global_frame_id_.c_str());
  pos3d_if_->write();

  char *map_file = strdup(cfg_map_file_.c_str());
  std::string map_name = basename(map_file);
  free(map_file);
  std::string::size_type pos;
  if (((pos = map_name.rfind(".")) != std::string::npos) && (pos > 0)) {
    map_name = map_name.substr(0, pos-1);
  }

  loc_if_->set_map(map_name.c_str());
  loc_if_->write();

  apply_initial_pose();
}


void
AmclThread::loop()
{
  if (!laser_pose_set_) {
    if (set_laser_pose()) {
      laser_pose_set_ = true;
      apply_initial_pose();
    } else {
      if (fawkes::runtime::uptime() >= tf_listener->get_cache_time()) {
	logger->log_warn(name(), "Could not determine laser pose, skipping loop");
      }
      return;
    }
  }

  laser_if_->read();
  //if (! laser_if_->changed() && ! laser_buffered_) {
  //  logger->log_warn(name(), "Laser data unchanged, skipping loop");
  //  return;
  //}

  MutexLocker lock(conf_mutex_);

  // Where was the robot when this scan was taken?
  tf::Stamped<tf::Pose> odom_pose;
  pf_vector_t pose;

  if (laser_if_->changed()) {
    if (!get_odom_pose(odom_pose, pose.v[0], pose.v[1], pose.v[2],
                       laser_if_->timestamp(), base_frame_id_))
    {
      if (cfg_buffer_debug_) {
	logger->log_warn(name(), "Couldn't determine robot's pose "
			 "associated with current laser scan");
      }
      if (laser_buffered_) {
	Time buffer_timestamp(laser_if_->buffer_timestamp(0));
	if (!get_odom_pose(odom_pose, pose.v[0], pose.v[1], pose.v[2],
			   &buffer_timestamp, base_frame_id_))
	{
	  fawkes::Time zero_time(0, 0);
	  if (! get_odom_pose(odom_pose, pose.v[0], pose.v[1], pose.v[2],
			      &zero_time, base_frame_id_))
	  {
	    // could not even use the buffered scan, buffer current one
	    // and try that one next time, always warn, this is bad
	    logger->log_warn(name(), "Couldn't determine robot's pose "
			     "associated with buffered laser scan nor at "
			     "current time, re-buffering");
	    laser_if_->copy_private_to_buffer(0);
	    return;
	  } else {
	    // we got a transform at some time, it is by far not as good
	    // as the correct value, but will at least allow us to go on
	    laser_buffered_ = false;
	  }
	} else {
	  // yay, that worked, use that one, re-buffer current data
	  if (cfg_buffer_debug_) {
	    logger->log_warn(name(), "Using buffered laser data, re-buffering current");
	  }
	  laser_if_->read_from_buffer(0);
	  laser_if_->copy_shared_to_buffer(0);
	}
      } else if (cfg_buffer_enable_) {
	if (cfg_buffer_debug_) {
	  logger->log_warn(name(), "Buffering current data for next loop");
	}
	laser_if_->copy_private_to_buffer(0);
	laser_buffered_ = true;
	return;
      } else {
	return;
      }
    } else {
      //logger->log_info(name(), "Fresh data is good, using that");
      laser_buffered_ = false;
    }
  } else if (laser_buffered_) {
    // either data is good to use now or there is no fresh we can buffer
    laser_buffered_ = false;

    Time buffer_timestamp(laser_if_->buffer_timestamp(0));
    if (get_odom_pose(odom_pose, pose.v[0], pose.v[1], pose.v[2],
		       &buffer_timestamp, base_frame_id_))
    {
      // yay, that worked, use that one
      if (cfg_buffer_debug_) {
	logger->log_info(name(), "Using buffered laser data (no changed data)");
      }
      laser_if_->read_from_buffer(0);
    } else {
      if (cfg_buffer_debug_) {
	logger->log_error(name(), "Couldn't determine robot's pose "
			  "associated with buffered laser scan (2)");
      }
      return;
    }
  } else {
    //logger->log_error(name(), "Neither changed nor buffered data, skipping loop");
    return;
  }

  float* laser_distances = laser_if_->distances();

  pf_vector_t delta = pf_vector_zero();

  if (pf_init_) {
    // Compute change in pose
    //delta = pf_vector_coord_sub(pose, pf_odom_pose_);
    delta.v[0] = pose.v[0] - pf_odom_pose_.v[0];
    delta.v[1] = pose.v[1] - pf_odom_pose_.v[1];
    delta.v[2] = angle_diff(pose.v[2], pf_odom_pose_.v[2]);

    fawkes::Time now(clock);

    // See if we should update the filter
    bool update =
      fabs(delta.v[0]) > d_thresh_ ||
      fabs(delta.v[1]) > d_thresh_ ||
      fabs(delta.v[2]) > a_thresh_;
      
    if (update) {
      last_move_time_->stamp();
      /*
      logger->log_info(name(), "(%f,%f,%f) vs. (%f,%f,%f)  diff (%f|%i,%f|%i,%f|%i)",
                       pose.v[0], pose.v[1], pose.v[2],
                       pf_odom_pose_.v[0], pf_odom_pose_.v[1], pf_odom_pose_.v[2],
                       fabs(delta.v[0]), fabs(delta.v[0]) > d_thresh_,
                       fabs(delta.v[1]), fabs(delta.v[1]) > d_thresh_,
                       fabs(delta.v[2]), fabs(delta.v[2]) > a_thresh_);
      */

      laser_update_ = true;
    } else if ((now - last_move_time_) <= t_thresh_) {
      laser_update_ = true;
    }
  }

  bool force_publication = false;
  if (!pf_init_) {
    //logger->log_debug(name(), "! PF init");
    // Pose at last filter update
    pf_odom_pose_ = pose;

    // Filter is now initialized
    pf_init_ = true;

    // Should update sensor data
    laser_update_ = true;
    force_publication = true;

    resample_count_ = 0;
  } else if (pf_init_ && laser_update_) {
    //logger->log_debug(name(), "PF init && laser update");
    //printf("pose\n");
    //pf_vector_fprintf(pose, stdout, "%.3f");

    ::amcl::AMCLOdomData odata;
    odata.pose = pose;
    // HACK
    // Modify the delta in the action data so the filter gets
    // updated correctly
    odata.delta = delta;

    // Use the action data to update the filter
    //logger->log_debug(name(), "Updating Odometry");
    odom_->UpdateAction(pf_, (::amcl::AMCLSensorData*) &odata);

    // Pose at last filter update
    //this->pf_odom_pose = pose;
  }

  bool resampled = false;
  // If the robot has moved, update the filter
  if (laser_update_) {
    //logger->log_warn(name(), "laser update");

    ::amcl::AMCLLaserData ldata;
    ldata.sensor = laser_;
    ldata.range_count = angle_range_ + 1;

    // To account for lasers that are mounted upside-down, we determine the
    // min, max, and increment angles of the laser in the base frame.
    //
    // Construct min and max angles of laser, in the base_link frame.
    Time latest(0, 0);
    tf::Quaternion q;
    q.setEulerZYX(angle_min_, 0.0, 0.0);
    tf::Stamped<tf::Quaternion> min_q(q, latest, laser_frame_id_);
    q.setEulerZYX(angle_min_ + angle_increment_, 0.0, 0.0);
    tf::Stamped<tf::Quaternion> inc_q(q, latest, laser_frame_id_);
    try
    {
      tf_listener->transform_quaternion(base_frame_id_, min_q, min_q);
      tf_listener->transform_quaternion(base_frame_id_, inc_q, inc_q);
    }
    catch(Exception &e)
    {
      logger->log_warn(name(), "Unable to transform min/max laser angles "
                       "into base frame");
      logger->log_warn(name(), e);
      return;
    }

    double angle_min = tf::get_yaw(min_q);
    double angle_increment = tf::get_yaw(inc_q) - angle_min;

    // wrapping angle to [-pi .. pi]
    angle_increment = fmod(angle_increment + 5 * M_PI, 2 * M_PI) - M_PI;

    // Apply range min/max thresholds, if the user supplied them
    if (laser_max_range_ > 0.0)
      ldata.range_max = (float) laser_max_range_;
    else
      ldata.range_max = HUGE;
    double range_min;
    if (laser_min_range_ > 0.0)
      range_min = (float) laser_min_range_;
    else
      range_min = 0.0;
    // The AMCLLaserData destructor will free this memory
    ldata.ranges = new double[ldata.range_count][2];
    
    const unsigned int maxlen_dist = laser_if_->maxlenof_distances();
    for (int i = 0; i < ldata.range_count; ++i) {
      unsigned int idx = (angle_min_idx_ + i) % maxlen_dist;
      // amcl doesn't (yet) have a concept of min range.  So we'll map short
      // readings to max range.
      if (laser_distances[idx] <= range_min)
        ldata.ranges[i][0] = ldata.range_max;
      else
        ldata.ranges[i][0] = laser_distances[idx];

      // Compute bearing
      ldata.ranges[i][1] = fmod(angle_min_ + (i * angle_increment), 2 * M_PI);
    }

    try {
      laser_->UpdateSensor(pf_, (::amcl::AMCLSensorData*) &ldata);
    } catch (Exception &e) {
      logger->log_warn(name(), "Failed to update laser sensor data, "
                       "exception follows");
      logger->log_warn(name(), e);
    }

    laser_update_ = false;

    pf_odom_pose_ = pose;

    // Resample the particles
    if (!(++resample_count_ % resample_interval_)) {
      //logger->log_info(name(), "Resample!");
      pf_update_resample(pf_);
      resampled = true;
    }

#ifdef HAVE_ROS
    pf_sample_set_t* set = (pf_->sets) + pf_->current_set;
    //logger->log_debug(name(), "Num samples: %d", set->sample_count);

    // Publish the resulting cloud
    // TODO: set maximum rate for publishing
    geometry_msgs::PoseArray cloud_msg;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = global_frame_id_;
    cloud_msg.poses.resize(set->sample_count);
    for (int i = 0; i < set->sample_count; i++) {
      tf::Quaternion q(tf::create_quaternion_from_yaw(set->samples[i].pose.v[2]));
      cloud_msg.poses[i].position.x = set->samples[i].pose.v[0];
      cloud_msg.poses[i].position.y = set->samples[i].pose.v[1];
      cloud_msg.poses[i].position.z = 0.;
      cloud_msg.poses[i].orientation.x   = q.x();
      cloud_msg.poses[i].orientation.y   = q.y();
      cloud_msg.poses[i].orientation.z   = q.z();
      cloud_msg.poses[i].orientation.w   = q.w();
    }

    particlecloud_pub_.publish(cloud_msg);
#endif
  }

  if (resampled || force_publication) {
    // Read out the current hypotheses
    double max_weight = 0.0;
    int max_weight_hyp = -1;
    std::vector<amcl_hyp_t> hyps;
    hyps.resize(pf_->sets[pf_->current_set].cluster_count);
    for (int hyp_count = 0;
	 hyp_count < pf_->sets[pf_->current_set].cluster_count;
	 hyp_count++) {
      double weight;
      pf_vector_t pose_mean;
      pf_matrix_t pose_cov;
      if (!pf_get_cluster_stats(pf_, hyp_count, &weight, &pose_mean,
				&pose_cov)) {
	logger->log_error(name(), "Couldn't get stats on cluster %d",
			  hyp_count);
	break;
      }

      hyps[hyp_count].weight = weight;
      hyps[hyp_count].pf_pose_mean = pose_mean;
      hyps[hyp_count].pf_pose_cov = pose_cov;

      if (hyps[hyp_count].weight > max_weight) {
	max_weight = hyps[hyp_count].weight;
	max_weight_hyp = hyp_count;
      }
    }

    if (max_weight > 0.0) {
      /*
      logger->log_debug(name(), "Max weight pose: %.3f %.3f %.3f (weight: %f)",
			hyps[max_weight_hyp].pf_pose_mean.v[0],
			hyps[max_weight_hyp].pf_pose_mean.v[1],
			hyps[max_weight_hyp].pf_pose_mean.v[2], max_weight);

	puts("");
	pf_matrix_fprintf(hyps[max_weight_hyp].pf_pose_cov, stdout, "%6.3f");
	puts("");
      */

      // Copy in the covariance, converting from 3-D to 6-D
      pf_sample_set_t* set = pf_->sets + pf_->current_set;
      for (int i = 0; i < 2; i++) {
	for (int j = 0; j < 2; j++) {
	  // Report the overall filter covariance, rather than the
	  // covariance for the highest-weight cluster
	  //last_covariance_[6 * i + j] = hyps[max_weight_hyp].pf_pose_cov.m[i][j];
	  last_covariance_[6 * i + j] = set->cov.m[i][j];
	}
      }

      // Report the overall filter covariance, rather than the
      // covariance for the highest-weight cluster
      //last_covariance_[6 * 5 + 5] = hyps[max_weight_hyp].pf_pose_cov.m[2][2];
      last_covariance_[6 * 5 + 5] = set->cov.m[2][2];

#ifdef HAVE_ROS
      geometry_msgs::PoseWithCovarianceStamped p;
      // Fill in the header
      p.header.frame_id = global_frame_id_;
      p.header.stamp = ros::Time();
      // Copy in the pose
      p.pose.pose.position.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
      p.pose.pose.position.y = hyps[max_weight_hyp].pf_pose_mean.v[1];
      tf::Quaternion q(tf::Vector3(0,0,1),
		       hyps[max_weight_hyp].pf_pose_mean.v[2]);
      p.pose.pose.orientation.x = q.x();
      p.pose.pose.orientation.y = q.y();
      p.pose.pose.orientation.z = q.z();
      p.pose.pose.orientation.w = q.w();

      // Copy in the covariance
      for (int i = 0; i < 2; i++) {
	for (int j = 0; j < 2; j++) {
	  // Report the overall filter covariance, rather than the
	  // covariance for the highest-weight cluster
	  p.pose.covariance[6*i+j] = last_covariance_[6 * i + j];
	}
      }
      p.pose.covariance[6 * 5 + 5] = last_covariance_[6 * 5 + 5];

      pose_pub_.publish(p);
#endif
      //last_published_pose = p;
      /*
      logger->log_debug(name(), "New pose: %6.3f %6.3f %6.3f",
      		hyps[max_weight_hyp].pf_pose_mean.v[0],
      		hyps[max_weight_hyp].pf_pose_mean.v[1],
      		hyps[max_weight_hyp].pf_pose_mean.v[2]);
      */

      // subtracting base to odom from map to base and send map to odom instead
      tf::Stamped<tf::Pose> odom_to_map;

      try {
	tf::Transform
          tmp_tf(tf::create_quaternion_from_yaw(hyps[max_weight_hyp].pf_pose_mean.v[2]),
                 tf::Vector3(hyps[max_weight_hyp].pf_pose_mean.v[0],
                             hyps[max_weight_hyp].pf_pose_mean.v[1], 0.0));
        Time latest(0, 0);
	tf::Stamped<tf::Pose> tmp_tf_stamped(tmp_tf.inverse(),
                                             latest, base_frame_id_);
	tf_listener->transform_pose(odom_frame_id_,
                                    tmp_tf_stamped, odom_to_map);
      } catch (Exception &e) {
	logger->log_warn(name(),
			  "Failed to subtract base to odom transform");
	return;
      }

      latest_tf_ =
        tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                      tf::Point(odom_to_map.getOrigin()));
      latest_tf_valid_ = true;

      // We want to send a transform that is good up until a
      // tolerance time so that odom can be used
      Time transform_expiration =
        (*laser_if_->timestamp() + transform_tolerance_);
      tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
					  transform_expiration,
					  global_frame_id_, odom_frame_id_);
      tf_publisher->send_transform(tmp_tf_stamped);


      // We need to apply the last transform to the latest odom pose to get
      // the latest map pose to store.  We'll take the covariance from
      // last_published_pose.
      tf::Pose map_pose = latest_tf_.inverse() * odom_pose;
      tf::Quaternion map_att = map_pose.getRotation();

      double trans[3] = {map_pose.getOrigin().x(), map_pose.getOrigin().y(), 0};
      double rot[4] = { map_att.x(), map_att.y(), map_att.z(), map_att.w() };
    
      if (pos3d_if_->visibility_history() >= 0) {
	pos3d_if_->set_visibility_history(pos3d_if_->visibility_history() + 1);
      } else {
	pos3d_if_->set_visibility_history(1);
      }
      pos3d_if_->set_translation(trans);
      pos3d_if_->set_rotation(rot);
      pos3d_if_->write();

      sent_first_transform_ = true;
    } else {
      logger->log_error(name(), "No pose!");
    }
  } else if (latest_tf_valid_) {
    // Nothing changed, so we'll just republish the last transform, to keep
    // everybody happy.
    Time transform_expiration =
      (*laser_if_->timestamp() + transform_tolerance_);
    tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
					transform_expiration,
                                        global_frame_id_, odom_frame_id_);
    tf_publisher->send_transform(tmp_tf_stamped);

    // We need to apply the last transform to the latest odom pose to get
    // the latest map pose to store.  We'll take the covariance from
    // last_published_pose.
    tf::Pose map_pose = latest_tf_.inverse() * odom_pose;
    tf::Quaternion map_att = map_pose.getRotation();

    double trans[3] = {map_pose.getOrigin().x(), map_pose.getOrigin().y(), 0};
    double rot[4] = { map_att.x(), map_att.y(), map_att.z(), map_att.w() };
    
    if (pos3d_if_->visibility_history() >= 0) {
      pos3d_if_->set_visibility_history(pos3d_if_->visibility_history() + 1);
    } else {
      pos3d_if_->set_visibility_history(1);
    }
    pos3d_if_->set_translation(trans);
    pos3d_if_->set_rotation(rot);
    pos3d_if_->write();

    // Is it time to save our last pose to the config
    Time now(clock);
    if ((save_pose_period_ > 0.0) &&
	(now - save_pose_last_time) >= save_pose_period_)
    {
      double yaw, pitch, roll;
      map_pose.getBasis().getEulerYPR(yaw, pitch, roll);

      //logger->log_debug(name(), "Saving pose (%f,%f,%f) as initial pose to host config",
      //		map_pose.getOrigin().x(), map_pose.getOrigin().y(), yaw);

      // Make sure we write the config only once by locking/unlocking it
      config->lock();
      try {
	config->set_float(AMCL_CFG_PREFIX"init_pose_x", map_pose.getOrigin().x());
	config->set_float(AMCL_CFG_PREFIX"init_pose_y", map_pose.getOrigin().y());
	config->set_float(AMCL_CFG_PREFIX"init_pose_a", yaw);
	config->set_float(AMCL_CFG_PREFIX"init_cov_xx", last_covariance_[6 * 0 + 0]);
	config->set_float(AMCL_CFG_PREFIX"init_cov_yy", last_covariance_[6 * 1 + 1]);
	config->set_float(AMCL_CFG_PREFIX"init_cov_aa", last_covariance_[6 * 5 + 5]);
      } catch (Exception &e) {
	logger->log_warn(name(), "Failed to save pose, exception follows, disabling saving");
	logger->log_warn(name(), e);
	save_pose_period_ = 0.0; 
      }
      config->unlock();
      save_pose_last_time = now;
    }
  } else {
    if (pos3d_if_->visibility_history() <= 0) {
      pos3d_if_->set_visibility_history(pos3d_if_->visibility_history() - 1);
    } else {
      pos3d_if_->set_visibility_history(-1);
    }
    pos3d_if_->write();
  }
}

void AmclThread::finalize()
{
  blackboard->unregister_listener(this);
  bbil_remove_message_interface(loc_if_);

  if (map_) {
    map_free(map_);
    map_ = NULL;
  }
  delete initial_pose_hyp_;
  initial_pose_hyp_ = NULL;

  delete last_move_time_;

  blackboard->close(laser_if_);
  blackboard->close(pos3d_if_);
  blackboard->close(loc_if_);

#ifdef HAVE_ROS
  pose_pub_.shutdown();
  particlecloud_pub_.shutdown();
  initial_pose_sub_.shutdown();
  map_pub_.shutdown();
#endif
}

bool
AmclThread::get_odom_pose(tf::Stamped<tf::Pose>& odom_pose, double& x,
                          double& y, double& yaw,
                          const fawkes::Time* t, const std::string& f)
{
  // Get the robot's pose
  tf::Stamped<tf::Pose>
    ident(tf::Transform(tf::Quaternion(0, 0, 0, 1),
			tf::Vector3(0, 0, 0)), t, f);
  try {
    tf_listener->transform_pose(odom_frame_id_, ident, odom_pose);
  } catch (Exception &e) {
    if (cfg_buffer_debug_) {
      logger->log_warn(name(), "Failed to compute odom pose (%s)",
		       e.what_no_backtrace());
    }
    return false;
  }
  x = odom_pose.getOrigin().x();
  y = odom_pose.getOrigin().y();
  double pitch, roll;
  odom_pose.getBasis().getEulerZYX(yaw, pitch, roll);

  //logger->log_info(name(), "Odom pose: (%f, %f, %f)", x, y, yaw);

  return true;
}


#ifdef HAVE_ROS
#  ifdef USE_MAP_PUB
void
AmclThread::publish_map()
{
  nav_msgs::OccupancyGrid msg;
  msg.info.map_load_time = ros::Time::now();
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";

  msg.info.width  = map_width_;
  msg.info.height = map_height_;
  msg.info.resolution = cfg_resolution_;
  msg.info.origin.position.x = cfg_origin_x_;
  msg.info.origin.position.y = cfg_origin_y_;
  msg.info.origin.position.z = 0.0;
  tf::Quaternion q(tf::create_quaternion_from_yaw(cfg_origin_theta_));
  msg.info.origin.orientation.x = q.x();
  msg.info.origin.orientation.y = q.y();
  msg.info.origin.orientation.z = q.z();
  msg.info.origin.orientation.w = q.w();

  // Allocate space to hold the data
  msg.data.resize(msg.info.width * msg.info.height);

  for (unsigned int i = 0; i < msg.info.width * msg.info.height; ++i) {
    if (map_->cells[i].occ_state == +1) {
      msg.data[i] = +100;
    } else if (map_->cells[i].occ_state == -1) {
      msg.data[i] =    0;
    } else {
      msg.data[i] =   -1;
    }
  }

  map_pub_.publish(msg);
}
#  endif
#endif


bool
AmclThread::set_laser_pose()
{
  //logger->log_debug(name(), "Transform 1");
  fawkes::Time now(clock);
  tf::Stamped<tf::Pose>
    ident(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0)),
          &now, laser_frame_id_);
  tf::Stamped<tf::Pose> laser_pose;
  try {
    tf_listener->transform_pose(base_frame_id_, ident, laser_pose);
  } catch (fawkes::tf::LookupException& e) {
    //logger->log_error(name(), "Failed to lookup transform from %s to %s.",
    //                  laser_frame_id_.c_str(), base_frame_id_.c_str());
    //logger->log_error(name(), e);
    return false;
  } catch (fawkes::tf::TransformException& e) {
    //logger->log_error(name(), "Transform error from %s to %s, exception follows.",
    //                  laser_frame_id_.c_str(), base_frame_id_.c_str());
    //logger->log_error(name(), e);
    return false;
  } catch (fawkes::Exception& e) {
    if (fawkes::runtime::uptime() >= tf_listener->get_cache_time()) {
      logger->log_error(name(), "Generic exception for transform from %s to %s.",
			laser_frame_id_.c_str(), base_frame_id_.c_str());
      logger->log_error(name(), e);
    }
    return false;
  }

  /*
  tf::Stamped<tf::Pose>
    ident(tf::Transform(tf::Quaternion(0, 0, 0, 1),
                        tf::Vector3(0, 0, 0)), Time(), laser_frame_id_);
  tf::Stamped<tf::Pose> laser_pose;

  try {
    tf_listener->transform_pose(base_frame_id_, ident, laser_pose);
  } catch (tf::TransformException& e) {
    logger->log_error(name(), "Couldn't transform from %s to %s, "
                      "even though the message notifier is in use",
                      laser_frame_id_.c_str(), base_frame_id_.c_str());
    logger->log_error(name(), e);
    return;
  }
  */

  pf_vector_t laser_pose_v;
  laser_pose_v.v[0] = laser_pose.getOrigin().x();
  laser_pose_v.v[1] = laser_pose.getOrigin().y();

  // laser mounting angle gets computed later -> set to 0 here!
  laser_pose_v.v[2] = tf::get_yaw(laser_pose.getRotation());
  laser_->SetLaserPose(laser_pose_v);
  logger->log_debug(name(),
  		      "Received laser's pose wrt robot: %.3f %.3f %.3f",
  		      laser_pose_v.v[0], laser_pose_v.v[1], laser_pose_v.v[2]);

  return true;
}

void
AmclThread::apply_initial_pose()
{
  if (initial_pose_hyp_ != NULL && map_ != NULL) {
    logger->log_info(name(), "Applying pose: %.3f %.3f %.3f "
		   "(cov: %.3f %.3f %.3f, %.3f %.3f %.3f, %.3f %.3f %.3f)",
		     initial_pose_hyp_->pf_pose_mean.v[0],
		     initial_pose_hyp_->pf_pose_mean.v[1],
		     initial_pose_hyp_->pf_pose_mean.v[2],
		     initial_pose_hyp_->pf_pose_cov.m[0][0],
		     initial_pose_hyp_->pf_pose_cov.m[0][1],
		     initial_pose_hyp_->pf_pose_cov.m[0][2],
		     initial_pose_hyp_->pf_pose_cov.m[1][0],
		     initial_pose_hyp_->pf_pose_cov.m[1][1],
		     initial_pose_hyp_->pf_pose_cov.m[1][2],
		     initial_pose_hyp_->pf_pose_cov.m[2][0],
		     initial_pose_hyp_->pf_pose_cov.m[2][1],
		     initial_pose_hyp_->pf_pose_cov.m[2][2]);
    pf_init(pf_, initial_pose_hyp_->pf_pose_mean,
	    initial_pose_hyp_->pf_pose_cov);
    pf_init_ = false;
  } else {
    logger->log_warn(name(), "Called apply initial pose but no pose to apply");
  }
}

pf_vector_t
AmclThread::uniform_pose_generator(void* arg)
{
  map_t* map = (map_t*) arg;
#if NEW_UNIFORM_SAMPLING
  unsigned int rand_index = drand48() * free_space_indices.size();
  std::pair<int,int> free_point = free_space_indices[rand_index];
  pf_vector_t p;
  p.v[0] = MAP_WXGX(map, free_point.first);
  p.v[1] = MAP_WYGY(map, free_point.second);
  p.v[2] = drand48() * 2 * M_PI - M_PI;
#else
  double min_x, max_x, min_y, max_y;
  min_x = (map->size_x * map->scale) / 2.0 - map->origin_x;
  max_x = (map->size_x * map->scale) / 2.0 + map->origin_x;
  min_y = (map->size_y * map->scale) / 2.0 - map->origin_y;
  max_y = (map->size_y * map->scale) / 2.0 + map->origin_y;

  pf_vector_t p;
  for (;;) {
    p.v[0] = min_x + drand48() * (max_x - min_x);
    p.v[1] = min_y + drand48() * (max_y - min_y);
    p.v[2] = drand48() * 2 * M_PI - M_PI;
    // Check that it's a free cell
    int i, j;
    i = MAP_GXWX(map, p.v[0]);
    j = MAP_GYWY(map, p.v[1]);
    if (MAP_VALID(map,i,j) && (map->cells[MAP_INDEX(map,i,j)].occ_state == -1))
      break;
  }
#endif
  return p;
}

void
AmclThread::set_initial_pose(const std::string &frame_id, const fawkes::Time &msg_time,
			     const tf::Pose &pose, const double *covariance)
{
  MutexLocker lock(conf_mutex_);
  if(frame_id == "") {
    // This should be removed at some point
    logger->log_warn(name(), "Received initial pose with empty frame_id. "
		     "You should always supply a frame_id.");
  } else if (tf_listener->resolve(frame_id) != tf_listener->resolve(global_frame_id_))
  {
    // We only accept initial pose estimates in the global frame, #5148.
    logger->log_warn(name(),"Ignoring initial pose in frame \"%s\"; "
		     "initial poses must be in the global frame, \"%s\"",
		     frame_id.c_str(), global_frame_id_.c_str());
    return;
  }

  fawkes::Time latest(0, 0);
  
  // In case the client sent us a pose estimate in the past, integrate the
  // intervening odometric change.
  tf::StampedTransform tx_odom;
  try {
    tf_listener->lookup_transform(base_frame_id_, latest,
				  base_frame_id_, msg_time,
				  global_frame_id_, tx_odom);
  } catch(tf::TransformException &e) {
    // If we've never sent a transform, then this is normal, because the
    // global_frame_id_ frame doesn't exist.  We only care about in-time
    // transformation for on-the-move pose-setting, so ignoring this
    // startup condition doesn't really cost us anything.
    if(sent_first_transform_)
      logger->log_warn(name(), "Failed to transform initial pose "
		       "in time (%s)", e.what_no_backtrace());
    tx_odom.setIdentity();
  } catch (Exception &e) {
    logger->log_warn(name(), "Failed to transform initial pose in time");
    logger->log_warn(name(), e);
  }

  tf::Pose pose_new;
  pose_new = tx_odom.inverse() * pose;

  // Transform into the global frame

  logger->log_info(name(), "Setting pose: %.3f %.3f %.3f",
		   pose_new.getOrigin().x(),
		   pose_new.getOrigin().y(),
		   tf::get_yaw(pose_new));
  // Re-initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = pose_new.getOrigin().x();
  pf_init_pose_mean.v[1] = pose_new.getOrigin().y();
  pf_init_pose_mean.v[2] = tf::get_yaw(pose_new);
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  // Copy in the covariance, converting from 6-D to 3-D
  for(int i=0; i<2; i++) {
    for(int j=0; j<2; j++) {
      pf_init_pose_cov.m[i][j] = covariance[6*i+j];
    }
  }
  pf_init_pose_cov.m[2][2] = covariance[6*5+5];

  delete initial_pose_hyp_;
  initial_pose_hyp_ = new amcl_hyp_t();
  initial_pose_hyp_->pf_pose_mean = pf_init_pose_mean;
  initial_pose_hyp_->pf_pose_cov = pf_init_pose_cov;
  apply_initial_pose();

  last_move_time_->stamp();

}


#ifdef HAVE_ROS
void
AmclThread::initial_pose_received(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  fawkes::Time msg_time(msg->header.stamp.sec,
			msg->header.stamp.nsec / 1000);
  
  tf::Pose pose =
    tf::Transform(tf::Quaternion(msg->pose.pose.orientation.x,
				 msg->pose.pose.orientation.y,
				 msg->pose.pose.orientation.z,
				 msg->pose.pose.orientation.w), 
		  tf::Vector3(msg->pose.pose.position.x,
			      msg->pose.pose.position.y,
			      msg->pose.pose.position.z));


  const double *covariance = msg->pose.covariance.data();
  set_initial_pose(msg->header.frame_id, msg_time, pose, covariance);
}
#endif


bool
AmclThread::bb_interface_message_received(Interface *interface,
					  Message *message) throw()
{
  LocalizationInterface::SetInitialPoseMessage *ipm =
    dynamic_cast<LocalizationInterface::SetInitialPoseMessage *>(message);
  if (ipm) {
    fawkes::Time msg_time(ipm->time_enqueued());

    tf::Pose pose =
      tf::Transform(tf::Quaternion(ipm->rotation(0), ipm->rotation(1),
				   ipm->rotation(2), ipm->rotation(3)),
		    tf::Vector3(ipm->translation(0), ipm->translation(1),
				ipm->translation(2)));


    const double *covariance = ipm->covariance();
    set_initial_pose(ipm->frame(), msg_time, pose, covariance);
  }
  return false;
}
