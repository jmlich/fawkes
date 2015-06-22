
#*****************************************************************************
#         Makefile Build System for Fawkes: Robotino Template Plugin
#                            -------------------
#   Created on Wed May 23 18:31:45 2012
#   Copyright (C) 2012 by Daniel Ewert, Carologistics RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************
# JUST REPLACE ALL OCCURENCES OF plugin-template AND REPLACE THEM ACCORDING TO YOUR NEEDS
#
#

BASEDIR = ../../..
include $(BASEDIR)/etc/buildsys/config.mk
#include $(BUILDCONFDIR)/tf/tf.mk
#The necessary libraries your plugin needs.
LIBS_plugin_template = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
		fawkesblackboard fawkesinterface
#binaries that are needed for the given plugin (all your cpp-files, but with an o-ending :-) ).
OBJS_interface_time_comp = interface_time_comp_plugin.o interface_time_comp_thread.o

#names of the plugins you want to build
PLUGINS_all = $(PLUGINDIR)/interface_time_comp.$(SOEXT)

OBJS_all = $(OBJS_interface_time_comp)

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_somewarning
warning_somewarning:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TBROWN)Omitting Interface Time Compare plugin$(TNORMAL) (reason)"
endif

include $(BUILDSYSDIR)/base.mk
