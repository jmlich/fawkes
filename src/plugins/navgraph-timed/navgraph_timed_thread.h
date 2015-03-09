/***************************************************************************
 *  navgraph_timed_thread.h - Thread to set the cost function in the
 *                            navgraph
 *  
 *  Created: Thu Sep 27 14:31:11 2012
 *  Copyright  2015  Matthias Loebach
 ***************************************************************************/

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

#ifndef __PLUGINS_NAVGRAPH_TIMED_NAVGRAPH_TIMED_THREAD_H_
#define __PLUGINS_NAVGRAPH_TIMED_NAVGRAPH_TIMED_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <navgraph/aspect/navgraph.h>
#include <navgraph/navgraph.h>

class NavGraphTimedThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::NavGraphAspect
{
   public:
     NavGraphTimedThread();

     virtual void init();
     virtual void loop();
     virtual void finalize();
   private:
     static float
       cost(const fawkes::NavGraphPath &from,
           const fawkes::NavGraphNode &to)
      {
        return sqrtf(powf(to.x() - from.nodes().back().x(), 2) +
             powf(to.y() - from.nodes().back().y(), 2) );
      }

     static float
       estimate(const fawkes::NavGraphPath &path,
           const fawkes::NavGraphNode &goal)
       {
         return sqrtf(powf(goal.x() - path.nodes().back().x(), 2) +
             powf(goal.y() - path.nodes().back().y(), 2) );
       }
};

#endif
