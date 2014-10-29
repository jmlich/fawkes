
/***************************************************************************
 *  webview-agent-worker-thread.h - Process agent information
 *
 *  Created: Thu Oct 27 13:49:42 2014
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

#ifndef __PLUGINS_WEBVIEW_AGENT_WEBVIEW_AGENT_WORKER_THREAD_H_
#define __PLUGINS_WEBVIEW_AGENT_WEBVIEW_AGENT_WORKER_THREAD_H_

#include <core/threading/thread.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>

#include <queue>

namespace fawkes {
  class AgentInterface;
}

/** A request which can be sent to the Agent Worker
 *  @author Till Hofmann
 */
class AgentWorkerRequest {
 public:
  /** Constructor.
   *  @param output the file the output shall be written to
   *  @param buf_num the buffer of the interface which shall be visualized
   *  @param close_on_exit true if the file shall be closed on exit
   */
  AgentWorkerRequest(FILE * output, int buf_num, bool close_on_exit)
  : output_(output), buf_num_(buf_num), close_on_exit_(close_on_exit) {}

  /** Constructor. The current values (no buffer) of the interface shall be used.
   *  The file shall not be closed on exit.
   *  @param output the file the output shall be written to
   */
  AgentWorkerRequest(FILE * output) : AgentWorkerRequest(output, -1, false) {}

 public:
  /** @return get a pointer to the output file */
  FILE * output() { return output_; }
  /** @return get the buffer number of the request */
  int buf_num() { return buf_num_; }
  /** @return whether the file should be closed on exit */
  bool close_on_exit() { return close_on_exit_; }
 private:
  FILE * output_;
  int buf_num_;
  bool close_on_exit_;
};

class WebviewAgentWorkerThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect
{

 public:
  WebviewAgentWorkerThread(fawkes::AgentInterface * agent_if);
  virtual ~WebviewAgentWorkerThread();

  void add_request_to_queue(FILE * output, int buf_num = -1, bool close_on_exit = false);
  void process_request(AgentWorkerRequest);

 protected:
  virtual void init();
  virtual void loop();

 private:
  std::string generate_graph_string();
  void string_to_graph(std::string graph_string, FILE *output);
  std::vector<std::string> action_string_to_list(std::string action_string, std::string delimiter = ";");
  bool ignore_action(std::string action);

 private:
  fawkes::Mutex *queue_mutex_;
  fawkes::Mutex *process_mutex_;
  std::queue<AgentWorkerRequest> request_queue_;
  fawkes::AgentInterface * agent_if_;

  std::vector<std::string> cfg_ignored_actions_;
};

#endif
