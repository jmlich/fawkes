#*****************************************************************************
#   Makefile Build System for Fawkes: NavGraphTimed Plugin
#   
#   Created on Mon Mar 09 14:43:00 2015
#   Copyright (C) 2015 by Matthias Loebach, Carologistics RoboCup Team
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

LIBS_navgraph_timed = stdc++ m fawkescore fawkesutils fawkesaspects fvutils \
											fawkesinterface fawkesblackboard fawkesnavgraphaspect \
											fawkesnavgraph
OBJS_navgraph_timed = navgraph_timed_plugin.o navgraph_timed_thread.o

OBJS_all            = $(OBJS_navgraph_timed)

CFLAGS += $(CFLAGS_CPP11)

PLUGINS_all = $(PLUGINDIR)/navgraph_timed.$(SOEXT)

include $(BUILDSYSDIR)/base.mk
