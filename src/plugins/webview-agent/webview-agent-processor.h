
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

#ifndef __PLUGINS_WEBVIEW_AGENT_WEBVIEW_AGENT_PROCESSOR_H_
#define __PLUGINS_WEBVIEW_AGENT_WEBVIEW_AGENT_PROCESSOR_H_

#include <webview/request_processor.h>

#include <vector>

namespace fawkes {
  class Logger;
  class BlackBoard;
  class AgentInterface;
}

class WebviewAgentRequestProcessor : public fawkes::WebRequestProcessor
{
 public:
  WebviewAgentRequestProcessor(std::string base_url,
				fawkes::AgentInterface *agent_if, fawkes::Logger *logger);

  virtual ~WebviewAgentRequestProcessor();

  virtual fawkes::WebReply * process_request(const fawkes::WebRequest *request);

  virtual std::string generate_graph_string(); 
  virtual void string_to_graph(std::string graph_string, FILE *output);

 private:
  std::vector<std::string> action_string_to_list(std::string action_string, std::string delimiter = ";");

 private:
  const std::string      baseurl_;
  fawkes::AgentInterface *agent_if_;
  fawkes::Logger         *logger_;
};

#endif
