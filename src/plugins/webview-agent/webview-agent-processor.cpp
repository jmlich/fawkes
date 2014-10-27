
/***************************************************************************
 *  webview-agent-processor.h - Process agent information from the blackboard
 *
 *  Created: Wed Oct 22 20:21:42 2014
 *  Copyright  2014  Till Hofmann
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

#include "webview-agent-processor.h"

#include <core/exception.h>
#include <core/threading/mutex_locker.h>
#include <logging/logger.h>
#include <webview/page_reply.h>
#include <webview/error_reply.h>
#include <webview/file_reply.h>
#include <webview/redirect_reply.h>
#include <interfaces/AgentInterface.h>

#include <sstream>

#include <cstring>
#include <cerrno>

#include <gvc.h>
#include <gvcjob.h>

#include <unistd.h>

using namespace fawkes;
using namespace std;

/** @class WebviewAgentRequestProcessor "webview-agent-processor.h"
 * Agent request processor.
 * @author Till Hofmann
 */


/** Constructor.
 * @param base_url base URL of the webview agent web request processor.
 * @param agent_if AgentInterface
 * @param logger logger to report problems
 */
WebviewAgentRequestProcessor::WebviewAgentRequestProcessor(
  string base_url, fawkes::AgentInterface *agent_if, fawkes::Logger *logger)
: baseurl_(base_url), agent_if_(agent_if), logger_(logger)
{
}


/** Destructor. */
WebviewAgentRequestProcessor::~WebviewAgentRequestProcessor()
{
}


WebReply *
WebviewAgentRequestProcessor::process_request(const fawkes::WebRequest *request)
{
  if (request->url().compare(0, baseurl_.length(), baseurl_) == 0) {
    // It is in our URL prefix range
    string subpath = request->url().substr(baseurl_.length());

    if (subpath == "/graph.png") {
      string graph = generate_graph_string();

      //logger_->log_debug("WebviewAgentProcessor", "graph string is %s", graph.c_str());
      FILE *f = tmpfile();
      if (NULL == f) {
        return new WebErrorPageReply(WebReply::HTTP_INTERNAL_SERVER_ERROR,
            "Cannot open temp file: %s", strerror(errno));
      }

      string_to_graph(graph, f);

      try {
        DynamicFileWebReply *freply = new DynamicFileWebReply(f);
        return freply;
      } catch (fawkes::Exception &e) {
        return new WebErrorPageReply(WebReply::HTTP_INTERNAL_SERVER_ERROR, *(e.begin()));
      }

    } else {
      WebPageReply *r = new WebPageReply("Agent");
      r->append_body("<p><img src=\"%s/graph.png\" /></p>", baseurl_.c_str());
      //      r->append_body("<pre>%s</pre>", graph.c_str());
      return r;
    }
  } else {
    return NULL;
  }
}

void
WebviewAgentRequestProcessor::string_to_graph(string graph, FILE * output)
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
WebviewAgentRequestProcessor::generate_graph_string()
{
  stringstream gstream;
  agent_if_->read();
  gstream << "digraph { graph [fontsize=14];";
  gstream << "node [fontsize=12]; edge [fontsize=12]; " << endl;

  if (! agent_if_->has_writer()) {
    gstream << "\"No writer for agent interface. No agent running\"";
    gstream << "}";
    return gstream.str();
  }

  string history = agent_if_->history();
  logger_->log_debug("WebviewAgentRequestProcessor", "History is %s", history.c_str());
  vector<string> action_vector = action_string_to_list(history);
  uint node_id = 1;
  for (vector<string>::const_iterator it = action_vector.begin(); it != action_vector.end(); it++, node_id++) {
    gstream << node_id << " " << "[label=" << '"' << *it << '"' << ",color=green];" << endl;
    if (node_id != 1) {
      // node is not the first one, add an edge from the predecessor to the node
      gstream << node_id - 1 << " -> " << node_id << endl;
    }
  }
  
  string plan = agent_if_->plan();
  logger_->log_debug("WebviewAgentRequestProcessor", "Plan is %s", plan.c_str());
  vector<string> plan_vector = action_string_to_list(plan);
  for (vector<string>::const_iterator it = plan_vector.begin(); it != plan_vector.end(); it++, node_id++) {
    gstream << node_id << " " << "[label=" << '"' << *it << '"' << ",color=blue];" << endl;
    if (node_id != 1) {
      // node is not the first one, add an edge from the predecessor to the node
      gstream << node_id - 1 << " -> " << node_id << endl;
    }
  }

  gstream << "}";
  return gstream.str();
}

vector<string>
WebviewAgentRequestProcessor::action_string_to_list(string action_string, string delimiter /* = ";" */)
{
  vector<string> action_vector;
  while (size_t match_pos = action_string.find(delimiter)) {
    if (match_pos == string::npos) break;
    action_vector.push_back(action_string.substr(0, match_pos));
    action_string = action_string.substr(match_pos+1, string::npos);
  }
  return action_vector;
}
