
/***************************************************************************
 *  webview-agent-thread.h - Show agent information in webview
 *
 *  Created: Thu Oct 23 12:12:42 2014
 *  Copyright  2014 Till Hofmann
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

#ifndef __PLUGINS_WEBVIEW_AGENT_WEBVIEW_AGENT_THREAD_H_
#define __PLUGINS_WEBVIEW_AGENT_WEBVIEW_AGENT_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/clock.h>
#include <aspect/blackboard.h>
#include <blackboard/interface_listener.h>
#include <aspect/webview.h>
#include <aspect/configurable.h>
#include <aspect/thread_producer.h>

namespace fawkes {
  class AgentInterface;
}

class WebviewAgentRequestProcessor;
class WebviewAgentWorkerThread;

class WebviewAgentThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ClockAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlackBoardInterfaceListener,
  public fawkes::WebviewAspect,
  public fawkes::ThreadProducerAspect
{
 public:
  WebviewAgentThread();
  virtual ~WebviewAgentThread();

  virtual void init();
  virtual void finalize();

  // InterfaceListener
  virtual void bb_interface_data_changed(fawkes::Interface *interface) throw();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  WebviewAgentRequestProcessor *web_proc_;
  WebviewAgentWorkerThread *worker_thread_;
  fawkes::AgentInterface   *agent_if_;
  uint next_buffer_;

  uint cfg_buffer_size_;
  bool cfg_save_images_;
  std::string cfg_image_path_;
};

#endif
