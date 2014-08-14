
/***************************************************************************
 *  goto_thread.cpp - Kinova plugin Jaco movement thread
 *
 *  Created: Thu Jun 20 15:04:20 2013
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

#include "goto_thread.h"
#include "openrave_base_thread.h"
#include "arm.h"

#include <interfaces/JacoInterface.h>
#include <utils/math/angle.h>
#include <core/threading/mutex.h>

#include <unistd.h>

using namespace fawkes;

/** @class KinovaGotoThread "goto_thread.h"
 * Jaco Arm movement thread.
 * This thread handles the movement of the arm.
 *
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param thread_name thread name
 */
KinovaGotoThread::KinovaGotoThread(const char name[])
  : Thread(name, Thread::OPMODE_CONTINUOUS)
{
  __arm = NULL;
  __final_mutex = NULL;

  __new_target = false;
  __final = true;

  __wait_status_check = 0; //wait loops to check for jaco_retract_mode_t again
}


/** Destructor. */
KinovaGotoThread::~KinovaGotoThread()
{
}

void
KinovaGotoThread::init()
{
  __final_mutex = new Mutex();
}

void
KinovaGotoThread::finalize()
{
  delete __final_mutex;
  __final_mutex = NULL;
}

bool
KinovaGotoThread::final()
{
  check_final();
  __final_mutex->lock();
  bool final = __final;
  __final_mutex->unlock();
  return final;
}

void
KinovaGotoThread::register_arm(fawkes::jaco_arm_t *arm)
{
  __arm = arm;
}

void
KinovaGotoThread::unregister_arm()
{
  __arm = NULL;
}

void
KinovaGotoThread::set_target(float x, float y, float z,
                             float e1, float e2, float e3,
                             float f1, float f2, float f3)
{
  __coords.clear();
  __coords.push_back(x);
  __coords.push_back(y);
  __coords.push_back(z);
  __coords.push_back(e1);
  __coords.push_back(e2);
  __coords.push_back(e3);

  __fingers.clear();
  if( f1 > 0.f && f2 > 0.f && f3 > 0.f ) {
    __fingers.push_back(f1);
    __fingers.push_back(f2);
    __fingers.push_back(f3);
  } else {
    __fingers.push_back(__arm->iface->finger1());
    __fingers.push_back(__arm->iface->finger2());
    __fingers.push_back(__arm->iface->finger3());
  }

  __new_target = true;
  __target_type = TARGET_CARTESIAN;
}

void
KinovaGotoThread::set_target_ang(float j1, float j2, float j3,
                                 float j4, float j5, float j6,
                                 float f1, float f2, float f3)
{
  __joints.clear();
  __joints.push_back(j1);
  __joints.push_back(j2);
  __joints.push_back(j3);
  __joints.push_back(j4);
  __joints.push_back(j5);
  __joints.push_back(j6);

  __fingers.clear();
  if( f1 > 0.f && f2 > 0.f && f3 > 0.f ) {
    __fingers.push_back(f1);
    __fingers.push_back(f2);
    __fingers.push_back(f3);
  } else {
    __fingers.push_back(__arm->iface->finger1());
    __fingers.push_back(__arm->iface->finger2());
    __fingers.push_back(__arm->iface->finger3());
  }

  __new_target = true;
  __target_type = TARGET_ANGULAR;
}

void
KinovaGotoThread::pos_ready()
{
  __new_target = true;
  __target_type = TARGET_READY;
}

void
KinovaGotoThread::pos_retract()
{
  __new_target = true;
  __target_type = TARGET_RETRACT;
}


void
KinovaGotoThread::move_gripper(float f1, float f2 ,float f3)
{
  __joints.clear();
  __joints.push_back(__arm->iface->joints(0));
  __joints.push_back(__arm->iface->joints(1));
  __joints.push_back(__arm->iface->joints(2));
  __joints.push_back(__arm->iface->joints(3));
  __joints.push_back(__arm->iface->joints(4));
  __joints.push_back(__arm->iface->joints(5));

  __fingers.clear();
  __fingers.push_back(f1);
  __fingers.push_back(f2);
  __fingers.push_back(f3);

  __new_target = true;
  __target_type = TARGET_ANGULAR;
}


void
KinovaGotoThread::stop()
{
  try {
    __arm->arm->stop();

    __final_mutex->lock();
    __final = true;
    __final_mutex->unlock();
    __new_target = false;

  } catch( Exception &e ) {
    logger->log_warn(name(), "Error sending stop command to arm. Ex:%s", e.what());
  }
}

void
KinovaGotoThread::check_final()
{
  bool check_fingers = false;
  bool final = true;

  //logger->log_debug(name(), "check final");
  switch( __target_type ) {

    case TARGET_READY:
    case TARGET_RETRACT:
      if( __wait_status_check == 0 ) {
        //logger->log_debug(name(), "check final for TARGET_MODE");
        __final_mutex->lock();
        __final = __arm->arm->final();
        __final_mutex->unlock();
      } else if( __wait_status_check >= 10 ) {
          __wait_status_check = 0;
      } else {
          ++__wait_status_check;
      }
      break;

/*
    default: //TARGET_ANGULAR, TARGET_CARTESIAN
      __final = __arm->arm->final();
//*/
//*
    case TARGET_ANGULAR:
      //logger->log_debug(name(), "check final for TARGET ANGULAR");
      //final = __arm->arm->final();
      for( unsigned int i=0; i<6; ++i ) {
        final &= (std::abs(normalize_mirror_rad(deg2rad(__joints.at(i) - __arm->iface->joints(i)))) < 0.01);
      }
      __final_mutex->lock();
      __final = final;
      __final_mutex->unlock();
      check_fingers = true;
      break;

    default: //TARGET_CARTESIAN
      //logger->log_debug(name(), "check final for TARGET CARTESIAN");
      //final = __arm->arm->final();
      final &= (std::abs(angle_distance(__coords.at(0) , __arm->iface->x())) < 0.01);
      final &= (std::abs(angle_distance(__coords.at(1) , __arm->iface->y())) < 0.01);
      final &= (std::abs(angle_distance(__coords.at(2) , __arm->iface->z())) < 0.01);
      final &= (std::abs(angle_distance(__coords.at(3) , __arm->iface->euler1())) < 0.1);
      final &= (std::abs(angle_distance(__coords.at(4) , __arm->iface->euler2())) < 0.1);
      final &= (std::abs(angle_distance(__coords.at(5) , __arm->iface->euler3())) < 0.1);
      __final_mutex->lock();
      __final = final;
      __final_mutex->unlock();

      check_fingers = true;
      break;
//*/

  }

  //logger->log_debug(name(), "check final: %u", __final);

  if( check_fingers && __final ) {
    //logger->log_debug(name(), "check fingeres for final");

    // also check fingeres
    if( __finger_last[0] == __arm->iface->finger1() &&
        __finger_last[1] == __arm->iface->finger2() &&
        __finger_last[2] == __arm->iface->finger3() ) {
      __finger_last[3] += 1;
    } else {
      __finger_last[0] = __arm->iface->finger1();
      __finger_last[1] = __arm->iface->finger2();
      __finger_last[2] = __arm->iface->finger3();
      __finger_last[3] = 0; // counter
    }
    __final_mutex->lock();
    __final &= __finger_last[3] > 10;
    __final_mutex->unlock();
  }
}

void
KinovaGotoThread::loop()
{
  if(__arm == NULL) {
    usleep(30e3);
    return;
  }

  if( __new_target ) {
    logger->log_debug(name(), "rcvd new target. skip old one, set this. using current finger positions");
    _goto_target();

  } else if( __final) {
    // all current trajectories have been processed. check for new ones
    __arm->trajec_mutex->lock();
    bool new_trajec = !__arm->trajec_queue->empty();
    __arm->trajec_mutex->unlock();

    if( new_trajec ) {
      logger->log_debug(name(), "new trajectory ready! processing now...");
      // get RefPtr to first trajectory in queue
      __arm->trajec_mutex->lock();
      RefPtr<jaco_trajec_t> trajec = __arm->trajec_queue->front();
      __arm->trajec_mutex->unlock();

      // process trajectory if it is "valid"
      if( trajec && !trajec->empty() ) {
        _exec_trajec(*trajec);
      }

      // trajectory has been processed. remove from queue
      __arm->trajec_mutex->lock();
      __arm->trajec_queue->pop_front();
      __arm->trajec_mutex->unlock();
    }

  } else {
    usleep(30e3);
  }

//  __arm->iface->set_final(__final);
}



void
KinovaGotoThread::_goto_target()
{

  __finger_last[0] = __arm->iface->finger1();
  __finger_last[1] = __arm->iface->finger2();
  __finger_last[2] = __arm->iface->finger3();
  __finger_last[3] = 0; // counter

  __final = false;
  __new_target = false;

  // process new target
  try {
    __arm->arm->stop(); // stop old movement
    //__arm->arm->start_api_ctrl();

    switch( __target_type ) {
      case TARGET_ANGULAR:
        logger->log_debug(name(), "target_type: TARGET_ANGULAR");
        __arm->arm->goto_joints(__joints, __fingers);
        break;

      case TARGET_READY:
        logger->log_debug(name(), "loop: target_type: TARGET_READY");
        __wait_status_check = 0;
        __arm->arm->goto_ready();
        break;

      case TARGET_RETRACT:
        logger->log_debug(name(), "target_type: TARGET_RETRACT");
        __wait_status_check = 0;
        __arm->arm->goto_retract();
        break;

      default: //TARGET_CARTESIAN
        logger->log_debug(name(), "target_type: TARGET_CARTESIAN");
        __arm->arm->goto_coords(__coords, __fingers);
        break;
    }

  } catch( Exception &e ) {
    logger->log_warn(name(), "Error sending command to arm. Ex:%s", e.what_no_backtrace());
  }
}

void
KinovaGotoThread::_exec_trajec(jaco_trajec_t* trajec)
{
  // set last trajec point as TARGET_ANGULAR, need for final-check
  std::vector<float> target = trajec->back();
  set_target_ang(target.at(0), target.at(1), target.at(2),
                 target.at(3), target.at(4), target.at(5));
  __new_target = false;

  try {
     // stop old movement
    __arm->arm->stop();

    logger->log_debug(name(), "exec traj: send traj commands...");
    // execute the trajectory
    for( unsigned int i=0; i<trajec->size(); ++i ) {
      __arm->arm->goto_joints(trajec->at(i), __fingers);
      usleep(10e3);
    }
    logger->log_debug(name(), "exec traj: ... DONE");

  } catch( Exception &e ) {
    logger->log_warn(name(), "Error executing trajectory. Ex:%s", e.what_no_backtrace());
  }
}
