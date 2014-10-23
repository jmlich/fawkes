
/***************************************************************************
 *  webview-agent-plugin.cpp - Show agent information in webview
 *
 *  Created: Thu Oct 23 13:32:42 2014
 *  Copyright  2014 Till Hofmann
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

#include "webview-agent-thread.h"
#include <core/plugin.h>

using namespace fawkes;

/** Agent webview plugin.
 * @author Till Hofmann
 */
class WebviewAgentPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  WebviewAgentPlugin(Configuration *config) : Plugin(config)
  {
    thread_list.push_back(new WebviewAgentThread());
  }
};


PLUGIN_DESCRIPTION("Show information about the running agent in webview")
EXPORT_PLUGIN(WebviewAgentPlugin)
