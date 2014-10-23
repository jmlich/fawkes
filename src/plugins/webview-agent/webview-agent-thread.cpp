
/***************************************************************************
 *  webview-agent-thread.cpp - Show agent information in webview
 *
 *  Created: Thu Oct 23 12:12:42 2014
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
#include "webview-agent-processor.h"

#include <webview/url_manager.h>
#include <webview/nav_manager.h>
#include <webview/request_manager.h>
#include <utils/time/time.h>
#include <utils/time/wait.h>

#include <interfaces/AgentInterface.h>

using namespace fawkes;

#define AGENT_URL_PREFIX "/agent"

/** @class WebviewAgentThread "webview-agent-thread.h"
 * Show agent information in webview.
 * @author Till Hofmann
 */

/** Constructor. */
WebviewAgentThread::WebviewAgentThread()
  : Thread("WebviewAgentThread", Thread::OPMODE_CONTINUOUS)
{
  //set_prepfin_conc_loop(true);
}


/** Destructor. */
WebviewAgentThread::~WebviewAgentThread()
{
}


void
WebviewAgentThread::init()
{
  std::string agent_id   = "Agent";
  try {
    agent_id = config->get_string("/webview/agent/agent-id");
  } catch (Exception &e) {} // ignored, use default

  std::string nav_entry = "Agent";
  try {
    nav_entry = config->get_string("/webview/agent/nav-entry");
  } catch (Exception &e) {} // ignored, use default

  web_proc_  = new WebviewAgentRequestProcessor(AGENT_URL_PREFIX,
						 agent_id, blackboard, logger);
  webview_url_manager->register_baseurl(AGENT_URL_PREFIX, web_proc_);
  webview_nav_manager->add_nav_entry(AGENT_URL_PREFIX, nav_entry.c_str());

  //agent_if_ = blackboard->open_for_

}


void
WebviewAgentThread::finalize()
{
  webview_url_manager->unregister_baseurl(AGENT_URL_PREFIX);
  webview_nav_manager->remove_nav_entry(AGENT_URL_PREFIX);
  delete web_proc_;
}

