
/***************************************************************************
 *  obstacle-tracker-average-plugin.cpp
 *
 *  Created: Sun Apr 21 01:15:54 2013
 *  Copyright  2015 Sebastian Reuter
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
#include "obstacle-tracker-average-thread.h"

using namespace fawkes;

/** Plugin to provide ObstacleTracker platform support for Fawkes.
 * @author Sebastian Reuter
 */
class ObstacleTrackerAveragePlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  ObstacleTrackerAveragePlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new ObstacleTrackerAverageThread());
  }
};

PLUGIN_DESCRIPTION("Plugin for tracking dynamic obstacles with moving average")
EXPORT_PLUGIN(ObstacleTrackerAveragePlugin)
