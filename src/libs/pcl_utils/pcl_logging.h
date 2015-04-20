
/***************************************************************************
 *  pcl_logging.h - Force PCL logging through logger or to be quiet
 *
 *  Created: Mon Apr 20 12:32:51 2015
 *  Copyright  2015  Tim Niemueller [www.niemueller.de]
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

#ifndef __LIBS_PCL_UTILS_PCL_LOGGING_H_
#define __LIBS_PCL_UTILS_PCL_LOGGING_H_

#ifdef PCL_MAJOR_VERSION
#  error "pcl_logging.h must be included before any other PCL header"
#endif

#include <pcl/console/print.h>

#ifdef PCL_FORCE_QUIET
#  undef PCL_ALWAYS
#  undef PCL_ERROR
#  undef PCL_WARN
#  undef PCL_INFO
#  undef PCL_DEBUG
#  undef PCL_VERBOSE
#  define PCL_ALWAYS(...)
#  define PCL_ERROR(...)
#  define PCL_WARN(...)
#  define PCL_INFO(...)
#  define PCL_DEBUG(...)
#  define PCL_VERBOSE(...)
#elif defined(PCL_LOG_FAWKES)
#  include <logging/liblogger.h>
#  undef PCL_ALWAYS
#  undef PCL_ERROR
#  undef PCL_WARN
#  undef PCL_INFO
#  undef PCL_DEBUG
#  undef PCL_VERBOSE
#  define PCL_ALWAYS(...)  fawkes::LibLogger::log_info("PCL", __VA_ARGS__)
#  define PCL_ERROR(...)   fawkes::LibLogger::log_error("PCL", __VA_ARGS__)
#  define PCL_WARN(...)    fawkes::LibLogger::log_warn("PCL", __VA_ARGS__)
#  define PCL_INFO(...)    fawkes::LibLogger::log_info("PCL", __VA_ARGS__)
#  define PCL_DEBUG(...)   fawkes::LibLogger::log_debug("PCL", __VA_ARGS__)
#  define PCL_VERBOSE(...) fawkes::LibLogger::log_debug("PCL", __VA_ARGS__)
#endif

#endif
