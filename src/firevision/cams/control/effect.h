
/***************************************************************************
 *  effect.h - Abstract class defining a camera effect controller
 *
 *  Created: Wed Apr 22 11:01:18 2009
 *  Copyright  2009      Tobias Kellner
 *             2005-2009 Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __FIREVISION_CAMS_CONTROL_EFFECT_H_
#define __FIREVISION_CAMS_CONTROL_EFFECT_H_

#include <cams/control/control.h>

class CameraControlEffect : virtual public CameraControl
{
 public:
  static const unsigned int EFFECT_NONE;

  virtual ~CameraControlEffect();

  virtual bool         supports_effect(unsigned int effect)             = 0;
  virtual void         set_effect(unsigned int effect)                  = 0;
  virtual unsigned int effect()                                         = 0;
  virtual void         reset_effect()                                   = 0;
};

#endif // __FIREVISION_CAMS_CONTROL_EFFECT_H_
