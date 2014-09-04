
/***************************************************************************
 *  openrave_thread.cpp - Kinova Jaco plugin OpenRAVE base Thread
 *
 *  Created: Tue Jun 04 13:13:20 2013
 *  Copyright  2013  Bahram Maleki-Fard
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

#include "openrave_base_thread.h"

#include <interfaces/JacoInterface.h>
#include <core/threading/mutex.h>

#include <cmath>
#include <stdio.h>
#include <cstring>

#ifdef HAVE_OPENRAVE
 #include <plugins/openrave/environment.h>
 #include <plugins/openrave/robot.h>
 #include <plugins/openrave/manipulator.h>
 #include <plugins/openrave/manipulators/kinova_jaco.h>
 using namespace OpenRAVE;
#endif

using namespace fawkes;
using namespace std;

/** @class JacoOpenraveBaseThread "openrave_base_thread.h"
 * Base Jaco Arm thread, integrating OpenRAVE
 *
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param thread_name thread name
 */
JacoOpenraveBaseThread::JacoOpenraveBaseThread(const char *name)
  : Thread(name, Thread::OPMODE_CONTINUOUS)
{
#ifdef HAVE_OPENRAVE
  __viewer_env.env   = NULL;
  __viewer_env.robot = NULL;
  __viewer_env.manip = NULL;

  __cfg_OR_auto_load_ik = false;
#endif
}


/** Destructor. */
JacoOpenraveBaseThread::~JacoOpenraveBaseThread()
{
#ifdef HAVE_OPENRAVE
  __viewer_env.env   = NULL;
  __viewer_env.robot = NULL;
  __viewer_env.manip = NULL;
#endif
}

void
JacoOpenraveBaseThread::init()
{
  __planning_mutex = new Mutex();

#ifdef HAVE_OPENRAVE
  __cfg_OR_use_viewer    = config->get_bool("/hardware/jaco/openrave/use_viewer");
  __cfg_OR_auto_load_ik  = config->get_bool("/hardware/jaco/openrave/auto_load_ik");

  // perform other initialization stuff (for child classes, that do not want to overload "init()")
  _init();

  __viewer_env.env = openrave->get_environment();
  __viewer_env.env->enable_debug();

  // load robot
  _load_robot();

  if( __cfg_OR_use_viewer )
    openrave->start_viewer();
#endif
}

void
JacoOpenraveBaseThread::finalize()
{
  unregister_arms();

  delete __planning_mutex;
  __planning_mutex = NULL;

#ifdef HAVE_OPENRAVE
  __viewer_env.robot = NULL;
  __viewer_env.manip = NULL;
  __viewer_env.env = NULL;
#endif
}