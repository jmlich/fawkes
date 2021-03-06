/***************************************************************************
 *  time_cache.h - Fawkes tf time cache (based on ROS tf)
 *
 *  Created: Thu Oct 20 11:09:58 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
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

/* This code is based on ROS tf with the following copyright and license:
 *
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __LIBS_TF_TIME_CACHE_H_
#define __LIBS_TF_TIME_CACHE_H_

#include <tf/types.h>

#include <LinearMath/btTransform.h>
#include <list>
#include <stdint.h>

namespace fawkes {
  namespace tf {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

enum ExtrapolationMode {
  ONE_VALUE,
  INTERPOLATE,
  EXTRAPOLATE_BACK,
  EXTRAPOLATE_FORWARD
};

typedef std::pair<fawkes::Time, CompactFrameID> P_TimeAndFrameID;

class TransformStorage
{
 public:
  TransformStorage();
  TransformStorage(const StampedTransform& data, CompactFrameID frame_id,
                   CompactFrameID child_frame_id);
  TransformStorage(const TransformStorage& rhs);

  TransformStorage& operator=(const TransformStorage& rhs)
  {
    rotation = rhs.rotation;
    translation = rhs.translation;
    stamp = rhs.stamp;
    frame_id = rhs.frame_id;
    child_frame_id = rhs.child_frame_id;
    return *this;
  }

  btQuaternion rotation;	///< rotation quaternio
  btVector3 translation;	///< translation vector
  fawkes::Time stamp;		///< time stamp
  CompactFrameID frame_id;	///< parent/reference frame number
  CompactFrameID child_frame_id;	///< child frame number
};



class TimeCache
{
 public:
  /** List of stored transforms. */
  typedef std::list<TransformStorage> L_TransformStorage;

  /// Number of nano-seconds to not interpolate below.
  static const int MIN_INTERPOLATION_DISTANCE = 5;
  /// Maximum length of linked list, to make sure not to be able to use unlimited memory.
  static const unsigned int MAX_LENGTH_LINKED_LIST = 1000000;

  TimeCache(float max_storage_time = 10.0);
  TimeCache(const TimeCache &t);
  TimeCache(const TimeCache *t);
  TimeCache(const TimeCache *t, fawkes::Time &look_back_until);

  bool get_data(fawkes::Time time, TransformStorage & data_out,
                std::string* error_str = 0);
  bool insert_data(const TransformStorage& new_data);
  void clear_list();
  CompactFrameID get_parent(fawkes::Time time, std::string* error_str);
  P_TimeAndFrameID get_latest_time_and_parent() const;

  const L_TransformStorage & get_storage() const;
  L_TransformStorage         get_storage_copy() const;

  /// Debugging information methods
  unsigned int get_list_length() const;
  fawkes::Time get_latest_timestamp() const;
  fawkes::Time get_oldest_timestamp() const;

 private:
  L_TransformStorage storage_;

  float max_storage_time_;


  inline uint8_t find_closest(TransformStorage*& one, TransformStorage*& two,
                              fawkes::Time target_time, std::string* error_str);

  inline void interpolate(const TransformStorage& one, const TransformStorage& two,
                          fawkes::Time time, TransformStorage& output);

  void prune_list();
};


} // end namespace tf
} // end namespace fawkes

#endif
