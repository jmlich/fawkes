
/***************************************************************************
 *  clips_agent_thread.h - CLIPS-based agent plugin
 *
 *  Created: Sat Jun 16 14:38:21 2012 (Mexico City)
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_CLIPS_AGENT_CLIPS_AGENT_THREAD_H_
#define __PLUGINS_CLIPS_AGENT_CLIPS_AGENT_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <plugins/clips/aspect/clips.h>
#include <utils/time/time.h>

#include <clipsmm.h>

#include <map>
#include <string>

namespace fawkes {
  class SkillerInterface;
  class SwitchInterface;
}

class ClipsAgentThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
  public fawkes::CLIPSAspect
{
 public:
  ClipsAgentThread();
  virtual ~ClipsAgentThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  CLIPS::Values  clips_get_clips_dirs();
  CLIPS::Values  clips_now();
  void           clips_call_skill(std::string skill_name, CLIPS::Values args);
  void           clips_load_config(std::string cfg_prefix);

 private:
  std::string cfg_clips_dir_;
  bool        cfg_clips_debug_;
  bool        cfg_use_sim_;
  bool        cfg_sim_randomize_;
  float       cfg_skill_sim_time_;

  fawkes::SkillerInterface *skiller_if_;
  fawkes::SwitchInterface *switch_if_;

  bool          ctrl_recheck_;

  typedef struct {
    fawkes::Time             start_time;
    std::string              skill_string;
    std::vector<std::string> args;
  } SkillExecInfo;

  std::map<std::string, SkillExecInfo> active_skills_;
  bool          started_;
  bool          skill_started_;
  fawkes::Time *skill_start_time_;

  bool          worldmodel_changed_;

};

#endif
