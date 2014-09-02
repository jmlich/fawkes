
/***************************************************************************
 *  arm.h - Abstract arm class for a kinova arm
 *
 *  Created: Tue Jul 29 14:58:32 2014
 *  Copyright  2014  Bahram Maleki-Fard
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

#ifndef __PLUGINS_KINOVA_ARM_H_
#define __PLUGINS_KINOVA_ARM_H_

#include <string>
#include <vector>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class KinovaArm <plugins/kinova/arm.h>
 * Abstract class for a Kinova Arm that we want to control.
 * @author Bahram Maleki-Fard
 */
class KinovaArm
{
 public:
  /** Virtual empty destructor. */
  virtual ~KinovaArm() {}

  /** Initialize the arm. */
  virtual void initialize() = 0;



  // status checking
  /** Check if movement is final.
   * @return is movement final?
   */
  virtual bool final() = 0;

  /** Check if arm is initialized.
   * @return is arm initialized?
   */
  virtual bool initialized() = 0;



  // getters
  /** Get the joint angles of the arm
   * @param to vector to be filled with angle values for active joints.
   */
  virtual void get_joints(std::vector<float> &to) const = 0;

  /** Get the cartesian coordinates of the arm
   * @param to vector to be filled with coordinates.
   */
  virtual void get_coords(std::vector<float> &to) const = 0;

  /** Get the position values of the fingers
   * @param to vector to be filled with finger positions.
   */
  virtual void get_fingers(std::vector<float> &to) const = 0;



  // commands
  /** Stop the current movement. */
  virtual void stop() = 0;

  /** Simulate a push of a button on the joystick of the Kinova arm.
   * @param button the id of the joystick button (from 0 to 15).
   */
  virtual void push_joystick(unsigned int button) = 0;

  /** Simulate releasing the joystick of the Kinova arm. */
  virtual void release_joystick() = 0;

  /** Move the arm to given configuration.
   * @param joints target joint angles
   * @param fingers target finger positions
   */
  virtual void goto_joints(std::vector<float> &joints, std::vector<float> &fingers) = 0;

  /** Move the arm to given configuration.
   * @param coords target fingertip coordinations
   * @param fingers target finger positions
   */
  virtual void goto_coords(std::vector<float> &coords, std::vector<float> &fingers) = 0;

  /** Move the arm to READY position. */
  virtual void goto_ready() = 0;

  /** Move the arm to RETRACT position. */
  virtual void goto_retract() = 0;


  // non-abstract methods
  /** Get the name of the arm.
   * @return the name
   */
  std::string get_name() const;

 protected:
  std::string __name;
  bool        __initialized;
};

inline
std::string
KinovaArm::get_name() const
{
  return __name;
}

} // end of namespace fawkes

#endif