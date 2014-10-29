
/***************************************************************************
 *  webview-agent-worker-thread.cpp - Process agent information
 *
 *  Created: Thu Oct 27 14:30:42 2014
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

#include "webview-agent-worker-thread.h"

#include <interfaces/AgentInterface.h>

#include <sstream>

#include <gvc.h>
#include <gvcjob.h>

#include <fnmatch.h>

using namespace fawkes;
using namespace std;

#define CFG_PREFIX "/webview/agent/"

/** @class WebviewAgentWorkerThread "webview-agent-worker-thread.h"
 * Worker thread used by the webview-agent plugin to generate dot graphs
 * @author Till Hofmann
 */

/** Constructor.
 *  @param agent_if The agent interface the data is read from
 */
WebviewAgentWorkerThread::WebviewAgentWorkerThread(AgentInterface *agent_if)
: Thread("WebviewAgentWorkerThread", Thread::OPMODE_WAITFORWAKEUP),
  agent_if_(agent_if)
{
  queue_mutex_ = new Mutex();
  process_mutex_ = new Mutex();
}

void
WebviewAgentWorkerThread::init()
{
  set_coalesce_wakeups(false);

  cfg_ignored_actions_ = config->get_strings(CFG_PREFIX"ignored_action");
}

WebviewAgentWorkerThread::~WebviewAgentWorkerThread()
{
  delete queue_mutex_;
  delete process_mutex_;
}

/** Add a request to the queue.
 *  @param output_file The file the dot graph is written to
 *  @param buf_num The buffer number of the interface the data is read from
 *  @param close_on_exit true if the file will be closed after generating the dot graph
 */
void
WebviewAgentWorkerThread::add_request_to_queue(FILE * output_file, int buf_num /* = -1 */, bool close_on_exit /* = false */)
{
  MutexLocker ml(queue_mutex_);
  request_queue_.push(AgentWorkerRequest(output_file, buf_num, close_on_exit));
}

void
WebviewAgentWorkerThread::loop()
{
  queue_mutex_->lock();
  if (request_queue_.empty()) {
    queue_mutex_->unlock();
    return;
  }

  AgentWorkerRequest next_request = request_queue_.front();
  request_queue_.pop();
  queue_mutex_->unlock();
  process_request(next_request);
}

/** Directly process a request.
 *  This can be used by an external component to directly add a request without
 *  adding the request to the queue.  This is used by the
 *  WebviewAgentRequestProcessor.
 *  @param request The request which will be processed.
 */
void
WebviewAgentWorkerThread::process_request(AgentWorkerRequest request)
{
  MutexLocker ml(process_mutex_);
  if (request.buf_num() < 0) {
    agent_if_->read();
  } else {
  agent_if_->read_from_buffer(request.buf_num());
  }
  string graph_string = generate_graph_string();
  string_to_graph(graph_string, request.output());
  if (request.close_on_exit()) {
    fclose(request.output());
  }
}

void
WebviewAgentWorkerThread::string_to_graph(string graph, FILE * output)
{
  GVC_t* gvc = gvContext();
  Agraph_t* G = agmemread((char *)graph.c_str());
  gvLayout(gvc, G, (char *)"dot");
  gvRender(gvc, G, "png", output);
  gvFreeLayout(gvc, G);
  agclose(G);
  gvFreeContext(gvc);
}

string
WebviewAgentWorkerThread::generate_graph_string()
{
  // Do NOT read the interface here, the interface is already updated in process_request
  stringstream gstream;
  gstream << "digraph { graph [fontsize=14];";
  gstream << "node [fontsize=12]; edge [fontsize=12]; " << endl;

  if (! agent_if_->has_writer()) {
    gstream << "\"No writer for agent interface. No agent running\"";
    gstream << "}";
    return gstream.str();
  }

  string history = agent_if_->history();
  vector<string> action_vector = action_string_to_list(history);
  uint node_id = 1;
  for (vector<string>::const_iterator it = action_vector.begin(); it != action_vector.end(); it++) {
    if (ignore_action(*it)) {
      // this action is ignored, i.e. not shown in the graph
      continue;
    }
    gstream << node_id << " " << "[label=" << '"' << *it << '"' << ",color=green];" << endl;
    if (node_id != 1) {
      // node is not the first one, add an edge from the predecessor to the node
      gstream << node_id - 1 << " -> " << node_id << endl;
    }
    node_id++;
  }

  string plan = agent_if_->plan();
  vector<string> plan_vector = action_string_to_list(plan);
  for (vector<string>::const_iterator it = plan_vector.begin(); it != plan_vector.end(); it++) {
    if (ignore_action(*it)) {
      // this action is ignored, i.e. not shown in the graph
      continue;
    }
    gstream << node_id << " " << "[label=" << '"' << *it << '"' << ",color=blue];" << endl;
    if (node_id != 1) {
      // node is not the first one, add an edge from the predecessor to the node
      gstream << node_id - 1 << " -> " << node_id << endl;
    }
    node_id++;
  }

  gstream << "}";
  return gstream.str();
}

vector<string>
WebviewAgentWorkerThread::action_string_to_list(string action_string, string delimiter /* = ";" */)
{
  vector<string> action_vector;
  while (size_t match_pos = action_string.find(delimiter)) {
    if (match_pos == string::npos) break;
    action_vector.push_back(action_string.substr(0, match_pos));
    action_string = action_string.substr(match_pos+1, string::npos);
  }
  return action_vector;
}

bool
WebviewAgentWorkerThread::ignore_action(string action)
{
  for (vector<string>::const_iterator it = cfg_ignored_actions_.begin(); it != cfg_ignored_actions_.end(); it++) {
    if (!fnmatch(it->c_str(), action.c_str(), 0)) {
      // match, therefore ignore the action
      return true;
    }
  }
  return false;
}
