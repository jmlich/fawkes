#*****************************************************************************
#         Makefile Build System for Fawkes: Laser Cluster Plugin
#                            -------------------
#   Created on Sun Apr 21 01:13:36 2013
#   Copyright (C) 2011 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../../..

include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDSYSDIR)/ros.mk
include $(BUILDSYSDIR)/boost.mk
include $(BUILDCONFDIR)/tf/tf.mk
include $(BUILDSYSDIR)/bfl.mk

ifeq ($(CC),gcc)
  ifneq ($(call gcc_atleast_version,4,6),1)
    GCC_TOO_OLD=1
  endif
endif

CFLAGS += $(CFLAGS_CPP11) -Wno-unknown-pragmas

LIBS_obstacle_tracker_average = fawkescore fawkesutils fawkesaspects fvutils fawkesbaseapp \
			fawkestf fawkesinterface fawkesblackboard \
			Position3DInterface Velocity3DInterface
			
LIBS_obstacle_tracker_kalman = fawkescore fawkesutils fawkesaspects fvutils fawkesbaseapp \
			fawkestf fawkesinterface fawkesblackboard \
			Position3DInterface Velocity3DInterface

OBJS_obstacle_tracker_average = obstacle-tracker-average-plugin.o obstacle-tracker-average-thread.o 

OBJS_obstacle_tracker_kalman = obstacle-tracker-kalman-plugin.o obstacle-tracker-kalman-thread.o object-estimator.o

OBJS_all = $(OBJS_obstacle_tracker_average) $(OBJS_obstacle_tracker_kalman)

ifneq ($(GCC_TOO_OLD),1)
  CFLAGS += $(CFLAGS_TF) $(CFLAGS_BFL) $(CFLAGS_BOOST)
  LDFLAGS += $(LDFLAGS_TF) $(LDFLAGS_BFL) $(LDFLAGS_BOOST)

  $(warning l:$(LDFLAGS_BFL))
  PLUGINS_all += $(PLUGINDIR)/obstacle-tracker-average.$(SOEXT)
  ifeq ($(HAVE_BFL),1)
    PLUGINS_all += $(PLUGINDIR)/obstacle-tracker-kalman.$(SOEXT)
  else
     WARN_TARGETS += warning_no_bfl
  endif
else
  WARN_TARGETS += warning_old_gcc
endif

CFLAGS  += $(call boost-lib-cflags,thread)
LDFLAGS += $(call boost-lib-ldflags,thread)

# Required for weirdo PCL (on Fedora with ROS headers)
CFLAGS  += $(call boost-lib-cflags,system)
LDFLAGS += $(call boost-lib-ldflags,system)

ifeq ($(OBJSSUBMAKE),1)
  all: $(WARN_TARGETS)
endif

.PHONY: warning_no_bfl
	warning_no_bfl:
	    $(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Obstacle Tracker Kalman$(TNORMAL) (No orocos-bfl library found)"


include $(BUILDSYSDIR)/base.mk




