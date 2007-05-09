
/***************************************************************************
 *  logger.cpp - Fawkes logging interface
 *
 *  Created: Tue Jan 16 20:40:15 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#include <utils/logging/logger.h>


/** @class Logger logging/logger.h
 * Interface for logging.
 * This interface facilitates a way to collect all output, be it debugging
 * output, informational output, warning or error messages.
 *
 * There should be no need no more for usage of printf in the code but
 * rather a logger should be used instead.
 *
 * The LoggingAspect should be added to a Thread that has to log something
 * (which is likely to be the case).
 * For library use QuickLog is the recommended way of logging. Do NOT use
 * these in plugins as it hides a dependency of this plugin.
 *
 * A special note to logging hackers: A logger may never ever bounce. This
 * means that not printing a message is ok in case of an internal error in
 * the logger, but it may never indicate that error with an exception!
 * If a logger cannot deliver the messages as it should be (like a network
 * logger that cannot send because the connection is dead) it should at
 * least dump it to stderr!
 *
 * Loggers have to be fast - damn fast. If a lengthy operations is needed
 * (like a network connection that may block) messages shall be enqueued
 * and processed later (for example in a separate thread). This is because
 * everywhere in the software (even in libraries like the utils) a logger
 * may be used to log an error that occured (but which is not that critical
 * that the application should die). In that case a logger which takes to
 * long is absolutely the wrong thing because this would influence the
 * performance of the whole software at unpredicted times - while if the
 * operations are carried out at a specified time or in a separate thread
 * they do not harm the performance.
 *
 * Caution: The line between log_* methods and vlog_* methods is very thin.
 * You can actually call log_info() with a va_list as the only variadic
 * parameter in some cases. The call is syntactically correct, but the
 * result is not what you intended. Thus make sure that you always use the
 * vlog_* method if you pass along a va_list!
 *
 * @fn void Logger::log_debug(const char *component, const char *format, ...) = 0
 * Log debug message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 *
 * @fn void Logger::log_info(const char *component, const char *format, ...) = 0
 * Log informational message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 *
 * @fn void Logger::log_warn(const char *component, const char *format, ...) = 0
 * Log warning message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 *
 * @fn void Logger::log_error(const char *component, const char *format, ...) = 0
 * Log error message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 *
 * @fn void Logger::vlog_debug(const char *component, const char *format, va_list va) = 0
 * Log debug message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 * @param va variable argument list
 *
 * @fn void Logger::vlog_info(const char *component, const char *format, va_list va) = 0
 * Log informational message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 * @param va variable argument list
 *
 * @fn void Logger::vlog_warn(const char *component, const char *format, va_list va) = 0
 * Log warning message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 * @param va variable argument list
 *
 * @fn void Logger::vlog_error(const char *component, const char *format, va_list va) = 0
 * Log error message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 * @param va variable argument list
 *
 * @fn void Logger::log_debug(const char *component, Exception &e) = 0
 * Log debug message.
 * @param component component, used to distuinguish logged messages
 * @param e exception to log, exception messages will be logged
 *
 * @fn void Logger::log_info(const char *component, Exception &e) = 0
 * Log informational message.
 * @param component component, used to distuinguish logged messages
 * @param e exception to log, exception messages will be logged
 *
 * @fn void Logger::log_warn(const char *component, Exception &e) = 0
 * Log warning message.
 * @param component component, used to distuinguish logged messages
 * @param e exception to log, exception messages will be logged
 *
 * @fn void Logger::log_error(const char *component, Exception &e) = 0
 * Log error message.
 * @param component component, used to distuinguish logged messages
 * @param e exception to log, exception messages will be logged
 *
 */

/** Virtual empty destructor. */
Logger::~Logger()
{
}
