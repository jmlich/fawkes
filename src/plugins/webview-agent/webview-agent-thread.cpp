
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
#include "webview-agent-worker-thread.h"

#include <webview/url_manager.h>
#include <webview/nav_manager.h>
#include <webview/request_manager.h>
#include <utils/time/time.h>
#include <utils/time/wait.h>

#include <interfaces/AgentInterface.h>


using namespace fawkes;
using namespace std;

#define AGENT_URL_PREFIX "/agent"
#define CFG_PREFIX "/webview/agent/"

/** @class WebviewAgentThread "webview-agent-thread.h"
 * Show agent information in webview.
 * @author Till Hofmann
 */

/** Constructor. */
WebviewAgentThread::WebviewAgentThread()
  : Thread("WebviewAgentThread", Thread::OPMODE_WAITFORWAKEUP),
    BlackBoardInterfaceListener("WebviewAgentThread"), next_buffer_(0)
{
}


/** Destructor. */
WebviewAgentThread::~WebviewAgentThread()
{
}


void
WebviewAgentThread::init()
{
  string agent_id   = "Agent";
  try {
    agent_id = config->get_string(CFG_PREFIX"agent_id");
  } catch (Exception &e) {} // ignored, use default

  string nav_entry = "Agent";
  try {
    nav_entry = config->get_string(CFG_PREFIX"nav_entry");
  } catch (Exception &e) {} // ignored, use default

  cfg_buffer_size_ = 20;
  try {
    cfg_buffer_size_ = config->get_uint(CFG_PREFIX"buffer_size");
  } catch (Exception &e) {} // ignored, use default

  cfg_save_images_ = false;
  try {
    cfg_save_images_ = config->get_bool(CFG_PREFIX"save_images");
  } catch (Exception &e) {} // ignored, use default

  cfg_image_path_ = "./";
  try {
    cfg_image_path_ = config->get_string(CFG_PREFIX"image_path");
  } catch (Exception &e) {} // ignored, use default
  if (cfg_image_path_ == "") {
    throw Exception("The empty path is not valid as image_path config value!");
  }
  if (cfg_image_path_.back() != '/') {
    cfg_image_path_.push_back('/');
  }

  agent_if_ = blackboard->open_for_reading<AgentInterface>(agent_id.c_str());
  agent_if_->resize_buffers(cfg_buffer_size_);

  worker_thread_ = new WebviewAgentWorkerThread(agent_if_);
  thread_collector->add(worker_thread_);
  web_proc_  = new WebviewAgentRequestProcessor(AGENT_URL_PREFIX,
						 worker_thread_, logger);
  webview_url_manager->register_baseurl(AGENT_URL_PREFIX, web_proc_);
  webview_nav_manager->add_nav_entry(AGENT_URL_PREFIX, nav_entry.c_str());

  // InterfaceListener
  if (cfg_save_images_) {
    // we are only interested in interface changes if we save the generated images
    bbil_add_data_interface(agent_if_);
    blackboard->register_listener(this);
  }
}


void
WebviewAgentThread::finalize()
{

  thread_collector->remove(worker_thread_);
  webview_url_manager->unregister_baseurl(AGENT_URL_PREFIX);
  webview_nav_manager->remove_nav_entry(AGENT_URL_PREFIX);
  delete web_proc_;

  bbil_remove_data_interface(agent_if_);
  blackboard->unregister_listener(this);

  blackboard->close(agent_if_);
}


void
WebviewAgentThread::bb_interface_data_changed(fawkes::Interface *interface) throw()
{
  if (!cfg_save_images_) {
    // we are not interested in interface changes because we don't save the generated images
    return;
  }

  AgentInterface *iface = dynamic_cast<AgentInterface *>(interface);
  if (!iface) {
    // not our interface type
    return;
  }

  agent_if_->copy_private_to_buffer(next_buffer_);
  agent_if_->copy_shared_to_buffer(next_buffer_);
  char * tmp;
  if (asprintf(&tmp, "agent-%u.png", static_cast<uint>(Time().in_usec()))) {
    string filename = tmp;
    free(tmp);
    string filepath = cfg_image_path_ + filename;
    FILE * f = fopen(filename.c_str(), "w");
    worker_thread_->add_request_to_queue(f, next_buffer_, true);
    worker_thread_->wakeup();
    next_buffer_ = (next_buffer_ + 1) % cfg_buffer_size_;
  } else {
    logger->log_error(name(), "Could not initialize filename, skipping blackboard update.");
  }
}

