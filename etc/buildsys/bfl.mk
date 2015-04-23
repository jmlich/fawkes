#*****************************************************************************
#                 Makefile Build System for Fawkes: Eigen3 bits
#                            -------------------
#   Created on Sun Jul 13 16:09:54 2014
#   Copyright (C) 2011-2014 by Tim Niemueller, Carologistics RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

ifndef __buildsys_config_mk_
$(error config.mk must be included before pcl.mk)
endif

ifndef __buildsys_bfl_mk_
__buildsys_bfl_mk_ := 1

ifneq ($(PKGCONFIG),)
  HAVE_BFL = $(if $(shell $(PKGCONFIG) --exists 'bfl'; echo $${?/1/}),1,0)
endif

ifeq ($(HAVE_BFL),1)
  CFLAGS_BFL  = -DHAVE_BFL $(shell $(PKGCONFIG) --cflags 'orcos-bfl') 
  LDFLAGS_BFL = $(shell $(PKGCONFIG) --libs 'orocos-bfl')
endif

endif # __buildsys_bfl_mk_
