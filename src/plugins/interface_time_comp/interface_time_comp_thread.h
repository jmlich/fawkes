
/***************************************************************************
 *  interface_time_comp_thread.h
 *
 *  Created: Mo 22. Jun 18:44:00 CEST 2015
 *  Copyright  2015 Tobias Neumann
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

#ifndef __PLUGINS_INTERFACE_TIME_COMP_THREAD_H_
#define __PLUGINS_INTERFACE_TIME_COMP_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>

#include <string>

namespace fawkes {
  class Interface;
}

class InterfaceTimeCompThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect
{
 private:
  std::vector<fawkes::Interface*> interfaces_;

 public:
  InterfaceTimeCompThread();

  virtual void init();
  virtual void loop();
  virtual bool prepare_finalize_user();
  virtual void finalize();

 protected: virtual void run() { Thread::run(); }

 private:
  void check_interfaces();
  
};


#endif
