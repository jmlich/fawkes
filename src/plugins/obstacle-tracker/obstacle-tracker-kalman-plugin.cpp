
/***************************************************************************
 *  obstacle-tracker-plugin.cpp
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
#include "obstacle-tracker-kalman-thread.h"
#include "visualization_thread.h"

using namespace fawkes;

/** Plugin to provide ObstacleTracker platform support for Fawkes.
 * @author Sebastian Reuter
 */
class ObstacleTrackerKalmanPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  ObstacleTrackerKalmanPlugin(Configuration *config)
    : Plugin(config)
  {
	bool use_vis = false;
	try {
	      use_vis = config->get_bool("/obstacle-tracker/visualization/enable");
	} catch (Exception &e) {} // ignored, use default
	if (use_vis) {
	  ObstacleTrackerVisualizationThread *vt = new ObstacleTrackerVisualizationThread();
	  thread_list.push_back(new ObstacleTrackerKalmanThread(vt));
	  thread_list.push_back(vt);
	} else {
	  thread_list.push_back(new ObstacleTrackerKalmanThread());
	}
  }
};

PLUGIN_DESCRIPTION("Plugin for tracking dynamic obstacles")
EXPORT_PLUGIN(ObstacleTrackerKalmanPlugin)
