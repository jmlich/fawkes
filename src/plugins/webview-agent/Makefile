#*****************************************************************************
#          Makefile Build System for Fawkes: Webview PTZ Camera Plugin
#                            -------------------
#   Created on Wed Oct 22 20:10:42 2014
#   Copyright (C) 2014 by Till Hofmann, AllemaniACs RoboCup Team
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
include $(BUILDSYSDIR)/clips.mk
include $(BUILDCONFDIR)/webview/webview.mk

ifneq ($(PKGCONFIG),)
  HAVE_GRAPHVIZ = $(if $(shell $(PKGCONFIG) --exists 'libgvc'; echo $${?/1/}),1,0)
  ifeq ($(HAVE_GRAPHVIZ),1)
    GRAPHVIZ_HAVE_GRAPH  = $(if $(shell $(PKGCONFIG) --exists 'libgraph'; echo $${?/1/}),1,0)

    CFLAGS_GRAPHVIZ = -DHAVE_GRAPHVIZ $(shell $(PKGCONFIG) --cflags 'libgvc') -DWITH_CODEGENS
    ifeq ($(GRAPHVIZ_HAVE_GRAPH),1)
      LDFLAGS_GRAPHVIZ = $(shell $(PKGCONFIG) --libs 'libgvc')
    else
      # "--libs 'libgvc'" requires -lgraph, even if only cgraph is installed.
      # Not fixed in stable releases yet
      LDFLAGS_GRAPHVIZ = $(shell $(PKGCONFIG) --libs 'libcgraph') -lgvc
      # add WITH_CGRAPH, in case something is wrong with libgvc.
      # Should be provided by "/usr/include/graphviz/types.h"
      CFLAGS_GRAPHVIZ += -DWITH_CGRAPH
    endif
  endif
endif

CFLAGS += $(CFLAGS_CPP11)

LIBS_webview_agent = m \
	fawkescore fawkesutils fawkesaspects fawkeswebview fawkesblackboard \
	fawkesinterface AgentInterface
OBJS_webview_agent = webview-agent-plugin.o webview-agent-thread.o \
		     webview-agent-processor.o webview-agent-worker-thread.o

OBJS_all = $(OBJS_webview_agent)

ifeq ($(HAVE_WEBVIEW),1)
  ifeq ($(HAVE_GRAPHVIZ),1)
    CFLAGS  += $(CFLAGS_WEBVIEW) $(CFLAGS_GRAPHVIZ)
    LDFLAGS += $(LDFLAGS_WEBVIEW) $(LDFLAGS_GRAPHVIZ)
    PLUGINS_all = $(PLUGINDIR)/webview-agent.$(SOEXT)
  else
    WARN_TARGETS += warning_graphviz
  endif
else
  ifneq ($(HAVE_WEBVIEW),1)
    WARN_TARGETS += warning_webview
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_webview warning_graphviz
warning_webview:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build webview-agent plugin$(TNORMAL) (webview not available)"
warning_graphviz:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build webview-agent plugin$(TNORMAL) (graphviz not found)"
endif

include $(BUILDSYSDIR)/base.mk
