/***************************************************************************
 *  interface_time_comp_thread.cpp
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

#include "interface_time_comp_thread.h"

#include <cmath>
#include <interface/interface.h>

using namespace fawkes;

/** @class InterfaceTimeCompThread "interface_time_comp_thread.h"
 * Class to display timediferences between interfaces and the actual time
 * @author Tobias Neumann
 */

/** Constructor. */
InterfaceTimeCompThread::InterfaceTimeCompThread() :
		Thread("InterfaceTimeCompThread", Thread::OPMODE_WAITFORWAKEUP), BlockedTimingAspect(
				BlockedTimingAspect::WAKEUP_HOOK_ACT) {
}

void InterfaceTimeCompThread::init()
{
	logger->log_info(name(), "Plugin \"Interface Time compare\" starts up");
	// open cfg for interfaces
	std::string cfg_prefix = "plugins/interface_time_comp/";
	std::vector<std::string> if_types = config->get_strings((cfg_prefix + "if_types").c_str());
	std::vector<std::string> if_ids   = config->get_strings((cfg_prefix + "if_ids").c_str());

	if ( if_types.size() != if_ids.size() ) {
	  logger->log_error(name(), "Different size of cfg if_type and if_ids");
	} else {
	  // open all interfaces
	  int size = if_types.size();
	  for (int i = 0; i < size; ++i) {
      std::string if_type, if_id;
      if_type = if_types[i];
      if_id   = if_ids[i];
      try {
        Interface* if_tmp = blackboard->open_for_reading(if_type.c_str(), if_id.c_str());
        interfaces_.push_back(if_tmp);
      } catch (Exception e) {
        logger->log_error(name(), "Can't open interface %s::%s\n%s", if_type.c_str(), if_id.c_str(), e.what());
      }
	  }
	}
}

bool InterfaceTimeCompThread::prepare_finalize_user()
{
	return true;
}

void InterfaceTimeCompThread::finalize()
{
  // close all interfaces
  Interface* if_tmp = NULL;
  while ( ! interfaces_.empty()) {
    if_tmp = interfaces_.back();
    interfaces_.pop_back();
    blackboard->close( if_tmp );
  }
}

void InterfaceTimeCompThread::loop()
{
  check_interfaces();
}

void InterfaceTimeCompThread::check_interfaces()
{
  // get time
  Time now(clock);
  // for all interfaces
  for ( std::vector<Interface*>::iterator it = interfaces_.begin();
        it != interfaces_.end();
        it++) {
    // read
    Interface* if_tmp = *it;
    if_tmp->read();

    if (if_tmp->has_writer()) {
      logger->log_info(name(), "%s::%s\ttime: %lf\tage: %f", if_tmp->type(), if_tmp->id(), if_tmp->timestamp()->in_sec(), now - if_tmp->timestamp());
    } else {
      logger->log_warn(name(), "%s::%s\thas no writer", if_tmp->type(), if_tmp->id());
    }
  }
}
