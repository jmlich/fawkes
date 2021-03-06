
/***************************************************************************
 *  environment.h - Fawkes to OpenRAVE Environment
 *
 *  Created: Sun Sep 19 14:50:34 2010
 *  Copyright  2010  Bahram Maleki-Fard, AllemaniACs RoboCup Team
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

#ifndef __PLUGINS_OPENRAVE_ENVIRONMENT_H_
#define __PLUGINS_OPENRAVE_ENVIRONMENT_H_

#include <openrave/openrave.h>
#include <string>

namespace OpenRAVE {
  class EnvironmentBase;
  class RobotBase;
}

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

void SetViewer(OpenRAVE::EnvironmentBasePtr penv, const std::string& viewername);

class Logger;
class OpenRaveRobot;

/** OpenRaveEnvironment class */
class OpenRaveEnvironment
{
 public:
  OpenRaveEnvironment(fawkes::Logger* logger = 0);
  ~OpenRaveEnvironment();

  virtual void create();
  virtual void destroy();
  virtual void lock();

  virtual void enable_debug(OpenRAVE::DebugLevel level=OpenRAVE::Level_Debug);
  virtual void disable_debug();

  virtual void start_viewer();
  virtual void load_IK_solver(OpenRaveRobot* robot, OpenRAVE::IkParameterizationType iktype=OpenRAVE::IKP_Transform6D);
  virtual void run_planner(OpenRaveRobot* robot, float sampling=0.01f);
  virtual void run_graspplanning(const std::string& target_name, OpenRaveRobot* robot, float sampling=0.01f);

  virtual void add_robot(const std::string& filename);
  virtual void add_robot(OpenRAVE::RobotBasePtr robot);
  virtual void add_robot(OpenRaveRobot* robot);

  virtual bool add_object(const std::string& name, const std::string& filename);
  virtual bool delete_object(const std::string& name);
  virtual bool rename_object(const std::string& name, const std::string& new_name);
  virtual bool move_object(const std::string& name, float trans_x, float trans_y, float trans_z, OpenRaveRobot* robot=NULL);
  virtual bool rotate_object(const std::string& name, float quat_x, float quat_y, float quat_z, float quat_w);
  virtual bool rotate_object(const std::string& name, float rot_x, float rot_y, float rot_z);

  //virtual RobotBasePtr getRobot() const;
  virtual OpenRAVE::EnvironmentBasePtr get_env_ptr() const;

 private:
  fawkes::Logger*	__logger;

  OpenRAVE::EnvironmentBasePtr	__env;
  OpenRAVE::PlannerBasePtr      __planner;
  OpenRAVE::ModuleBasePtr       __mod_ikfast;

  std::vector<OpenRAVE::GraphHandlePtr> __graph_handle;

  bool  __viewer_enabled;
};
} // end of namespace fawkes

#endif
