
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

#include "webview-agent-worker-thread.h"

#include <cstring>
#include <cerrno>

#include <unistd.h>

using namespace fawkes;
using namespace std;

/** @class WebviewAgentRequestProcessor "webview-agent-processor.h"
 * Agent request processor.
 * @author Till Hofmann
 */


/** Constructor.
 * @param base_url base URL of the webview agent web request processor.
 * @param worker_thread The worker thread which processes the request.
 * @param logger logger to report problems
 */
WebviewAgentRequestProcessor::WebviewAgentRequestProcessor(
  string base_url, WebviewAgentWorkerThread * worker_thread, fawkes::Logger *logger)
: baseurl_(base_url), worker_thread_(worker_thread), logger_(logger)
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

      //logger_->log_debug("WebviewAgentProcessor", "graph string is %s", graph.c_str());
      FILE *f = tmpfile();
      if (NULL == f) {
        return new WebErrorPageReply(WebReply::HTTP_INTERNAL_SERVER_ERROR,
            "Cannot open temp file: %s", strerror(errno));
      }

      worker_thread_->process_request(AgentWorkerRequest(f));

      try {
        DynamicFileWebReply *freply = new DynamicFileWebReply(f);
        return freply;
      } catch (fawkes::Exception &e) {
        return new WebErrorPageReply(WebReply::HTTP_INTERNAL_SERVER_ERROR, *(e.begin()));
      }

    } else {
      WebPageReply *r = new WebPageReply("Agent");
      r->append_body("<p><img src=\"%s/graph.png\" /></p>", baseurl_.c_str());
      return r;
    }
  } else {
    return NULL;
  }
}

