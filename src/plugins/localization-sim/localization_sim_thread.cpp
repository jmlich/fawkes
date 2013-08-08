/***************************************************************************
 *  localization_sim_thread.h - Thread provides 
 *     the simulated position of a robot in Gazeo
 *
 *  Created: Thu Aug 08 17:45:31 2013
 *  Copyright  2013  Frederik Zwilling
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

#include "localization_sim_thread.h"

#include <tf/types.h>
#include <stdio.h>
#include <math.h>
#include <utils/math/angle.h>

#include <interfaces/Position3DInterface.h>
#include <tf/transform_publisher.h>

#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <aspect/logging.h>

using namespace fawkes;
using namespace gazebo;

/** @class LocalizationSimThread "localization_sim_thread.h"
 * Thread simulates the Localization in Gazebo
 * @author Frederik Zwilling
 */

/** Constructor. */
LocalizationSimThread::LocalizationSimThread()
  : Thread("LocalizationSimThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE),
    TransformAspect(TransformAspect::ONLY_PUBLISHER, "Localization Transform??????")
{
}

void LocalizationSimThread::init()
{
  logger->log_debug(name(), "Initializing Simulation of the Localization");

  //open interface
  localization_if_ = blackboard->open_for_writing<Position3DInterface>("Pose");

  //subscribing to gazebo publisher
  localization_sub_ = gazebonode->Subscribe(std::string("~/RobotinoSim/Gps/"), &LocalizationSimThread::on_localization_msg, this);
}

void LocalizationSimThread::finalize()
{
  blackboard->close(localization_if_);
}

void LocalizationSimThread::loop()
{
}

void LocalizationSimThread::on_localization_msg(ConstVector3dPtr &msg)
{
  //logger->log_info(name(), "Got new Localization data.\n");

  //read data from message
  double x = msg->x();
  double y = msg->y();
  double z = msg->z();

  //write interface
  localization_if_->set_translation(0, x);
  localization_if_->set_translation(1, y);
  localization_if_->set_translation(2, z);
  //TODO: set orientation
  localization_if_->write();
}
