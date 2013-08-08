/***************************************************************************
 *  localization_sim_thread.h - Thread provides 
 *     the simulated position of a robot in Gazeo
 *
 *  Created: Thu Aug 08 17:40:10 2013
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

#ifndef __PLUGINS_LOCALIZATION_SIM_THREAD_H_
#define __PLUGINS_LOCALIZATION_SIM_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <plugins/gazebo/aspect/gazebo.h>
#include <aspect/tf.h>

//from Gazebo
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/transport/transport.hh>


namespace fawkes {
  class Position3DInterface;
}

class LocalizationSimThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::TransformAspect,
  public fawkes::GazeboAspect
{
 public:
  LocalizationSimThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 private:
  //Subscriber to receive localization data from gazebo
  gazebo::transport::SubscriberPtr localization_sub_;

  //provided interface
  fawkes::Position3DInterface *localization_if_;

  //handler function for incoming localization data messages
  void on_localization_msg(ConstVector3dPtr &msg);
};

#endif
