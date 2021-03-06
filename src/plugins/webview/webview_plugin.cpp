
/***************************************************************************
 *  webview_plugin.h - Fawkes Webview Plugin
 *
 *  Created: Mon Oct 13 17:46:57 2008 (I5 Developer's Day)
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include <plugins/webview/webview_plugin.h>
#include <core/exception.h>

#include "webview_thread.h"

using namespace fawkes;

/** @class WebviewPlugin <plugins/webview/webview_plugin.h>
 * Webview plugin for Fawkes.
 * This provides an extensible web interface for Fawkes.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
WebviewPlugin::WebviewPlugin(Configuration *config)
  : Plugin(config)
{
  thread_list.push_back(new WebviewThread());
}


PLUGIN_DESCRIPTION("Web interface for Fawkes")
EXPORT_PLUGIN(WebviewPlugin)
