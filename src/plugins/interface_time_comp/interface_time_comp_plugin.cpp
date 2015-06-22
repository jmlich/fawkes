
/***************************************************************************
 *  interface_time_comp_plugin.cpp
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

#include <core/plugin.h>

#include "interface_time_comp_thread.h"

using namespace fawkes;

/** Displayes the time of different interfaces compared to the actual time
 * @author Tobias Neumann
 */
class InterfaceTimeCompPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  InterfaceTimeCompPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new InterfaceTimeCompThread());
  }
};

PLUGIN_DESCRIPTION("Interface Time compare plugin")
EXPORT_PLUGIN(InterfaceTimeCompPlugin)
