
/***************************************************************************
 *  tf.h - Transform aspect for Fawkes
 *
 *  Created: Tue Oct 25 21:33:21 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __ASPECT_TF_H_
#define __ASPECT_TF_H_

#ifndef HAVE_TF
#  error TF not available. Forgot to add CFLAGS_TF?
#endif

#include <aspect/aspect.h>
#include <tf/transformer.h>
#include <tf/transform_publisher.h>

namespace fawkes {

  class BlackBoard;

#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class TransformAspect : public virtual Aspect
{
 public:
  /** Enumeration describing the desired mode of operation. */
  typedef enum {
    ONLY_LISTENER,	///< only create a transform listener
    ONLY_PUBLISHER,	///< only create a transform publisher
    DEFER_PUBLISHER,	/**< Create neither listener or publisher, but allow late
			 * enabling of a publisher using tf_enable_publisher() in init().
			 * Note that this requires to pass a valid (unique) tf_bb_iface_id
			 * to the constructor. */
    BOTH,		///< create both, transform listener and publisher
    BOTH_DEFER_PUBLISHER /**< create transform listener but defer creation of publisher,
			  * cf. DEFER_PUBLISHER mode documentation above for details. */
  } Mode;

  TransformAspect(Mode mode = ONLY_LISTENER, const char *tf_bb_iface_id = 0);
  virtual ~TransformAspect();

  void init_TransformAspect(BlackBoard *blackboard, tf::Transformer *transformer);
  void finalize_TransformAspect();

 protected: // methods
  void tf_enable_publisher();

 protected: // members
  tf::Transformer         * tf_listener;
  tf::TransformPublisher  * tf_publisher;

 private:
  Mode  __tf_aspect_mode;
  char *__tf_aspect_bb_iface_id;
  BlackBoard *__tf_aspect_blackboard;
  bool  __tf_aspect_own_listener;
};

} // end namespace fawkes

#endif
