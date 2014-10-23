
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
#include <aspect/webview.h>
#include <aspect/configurable.h>

namespace fawkes {
  class AgentInterface;
}

class WebviewAgentRequestProcessor;

class WebviewAgentThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ClockAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::WebviewAspect
{
 public:
  WebviewAgentThread();
  virtual ~WebviewAgentThread();

  virtual void init();
//  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  WebviewAgentRequestProcessor *web_proc_;

  // TODO needed ? 
  // fawkes::TimeWait *time_wait_;
  fawkes::AgentInterface   *agent_if_;

};

#endif
