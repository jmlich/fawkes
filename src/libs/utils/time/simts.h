
/***************************************************************************
 *  simts.h - Simulator time source
 *
 *  Created: Mon Feb 25 15:44:00 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __UTILS_TIME_SIMTS_H_
#define __UTILS_TIME_SIMTS_H_

#include <utils/time/clock.h>
#include <utils/time/timesource.h>

class SimulatorTimeSource : public TimeSource
{
 public:
  SimulatorTimeSource();
  virtual ~SimulatorTimeSource();

  virtual void get_time(timeval* tv) const;
  virtual timeval conv_to_realtime(const timeval* tv) const;

  void set_start(float initial_offset);
  void set_sim_offset(float sim_offset);

 private:
  Clock *clock;
  Time start_time; // sim AND realtime
  Time current_simtime;
  Time current_realtime;
  float start_simoffset;
  float current_simoffset;
};

#endif
