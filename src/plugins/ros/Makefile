#*****************************************************************************
#               Makefile Build System for Fawkes: ROS Plugin
#                            -------------------
#   Created on Thu May 05 18:23:31 2011
#   Copyright (C) 2006-2012 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDSYSDIR)/pcl.mk
include $(BUILDSYSDIR)/boost.mk
include $(BUILDCONFDIR)/tf/tf.mk

LIBS_ros = fawkescore fawkesutils fawkesaspects fawkesblackboard \
           fawkesinterface fawkesrosaspect
OBJS_ros = ros_plugin.o node_thread.o

LIBS_ros_talkerpub = fawkescore fawkesutils fawkesaspects fawkesblackboard \
		     fawkesinterface fawkesrosaspect
OBJS_ros_talkerpub = talkerpub_plugin.o talkerpub_thread.o

LIBS_ros_webview = fawkescore fawkesutils fawkesaspects fawkesblackboard \
		   fawkesinterface fawkesrosaspect fawkeswebview
OBJS_ros_webview = webview_plugin.o webview_thread.o webview_reqproc.o

LIBS_ros_tf = fawkescore fawkesutils fawkesaspects fawkesblackboard \
	      fawkesinterface fawkesrosaspect fawkestf TransformInterface
OBJS_ros_tf = tf_plugin.o tf_thread.o

LIBS_ros_pcl = fawkescore fawkesutils fawkesaspects fawkesblackboard \
	      fawkesinterface fawkesrosaspect fawkestf fawkespcl_utils
OBJS_ros_pcl = pcl_plugin.o pcl_thread.o

LIBS_ros_images = fawkescore fawkesutils fawkesaspects fawkesblackboard \
	          fawkesinterface fawkesrosaspect fvutils
OBJS_ros_images = image_plugin.o image_thread.o

LIBS_ros_laserscan = m fawkescore fawkesutils fawkesaspects fawkesblackboard \
	          fawkesinterface fawkesrosaspect fvutils Laser360Interface \
		  Laser720Interface Laser1080Interface
OBJS_ros_laserscan = laserscan_plugin.o laserscan_thread.o

LIBS_ros_odometry = m fawkescore fawkesutils fawkesaspects fawkesblackboard \
		    fawkesinterface fawkestf fawkesrosaspect MotorInterface
OBJS_ros_odometry = odometry_plugin.o odometry_thread.o

LIBS_ros_cmdvel = m fawkescore fawkesutils fawkesaspects fawkesblackboard \
		  fawkesinterface fawkesrosaspect MotorInterface
OBJS_ros_cmdvel = cmdvel_thread.o cmdvel_plugin.o

LIBS_ros_navigator = m pthread fawkescore fawkesutils fawkesaspects fawkesblackboard \
		     fawkesinterface fawkesrosaspect NavigatorInterface
OBJS_ros_navigator = navigator_plugin.o navigator_thread.o

LIBS_ros_joint = fawkescore fawkesutils fawkesaspects fawkesblackboard \
	      fawkesinterface fawkesrosaspect JointInterface
OBJS_ros_joint = joint_plugin.o joint_thread.o

LIBS_ros_robot_description = fawkescore fawkesutils fawkesaspects fawkesrosaspect
OBJS_ros_robot_description = robot_description_plugin.o robot_description_thread.o

LIBS_ros_move_base = fawkescore fawkesutils fawkesaspects fawkesblackboard \
	      fawkesinterface fawkestf fawkesrosaspect NavigatorInterface
OBJS_ros_move_base = move_base_plugin.o move_base_thread.o

LIBS_ros_skiller = fawkescore fawkesutils fawkesaspects fawkesblackboard \
	      fawkesinterface fawkesrosaspect SkillerInterface
OBJS_ros_skiller = skiller_plugin.o skiller_thread.o

LIBS_ros_position_3d = fawkescore fawkesutils fawkesaspects fawkesblackboard \
	      fawkesinterface fawkesrosaspect Position3DInterface
OBJS_ros_position_3d = position_3d_thread.o position_3d_plugin.o

OBJS_all = $(OBJS_ros) $(OBJS_ros_talkerpub) $(OBJS_ros_webview) $(OBJS_ros_tf) \
	   $(OBJS_ros_pcl) $(OBJS_ros_images) $(OBJS_ros_laserscan) $(OBJS_ros_odometry) \
	   $(OBJS_ros_cmdvel) $(OBJS_ros_navigator) $(OBJS_ros_joint) \
	   $(OBJS_ros_robot_description) $(OBJS_ros_move_base) \
	   $(OBJS_ros_skiller)

ifneq ($(PKGCONFIG),)
  HAVE_TINYXML = $(if $(shell $(PKGCONFIG) --exists 'tinyxml'; echo $${?/1/}),1,0)
endif

ifeq ($(HAVE_ROS),1)
  PRESUBDIRS += aspect

  CFLAGS  += $(CFLAGS_ROS) -Wno-unknown-pragmas -Wno-deprecated-declarations
  LDFLAGS += $(LDFLAGS_ROS)

  CFLAGS_talkerpub_thread += $(call ros-pkg-cflags,std_msgs) $(CFLAGS)
  LDFLAGS_ros_talkerpub += $(LDFLAGS) $(call ros-pkg-lflags,std_msgs)

  PLUGINS_all = $(PLUGINDIR)/ros.so \
		$(PLUGINDIR)/ros-talkerpub.so

  ifeq ($(call ros-have-pkg,webview_msgs),1)
    CFLAGS_webview_thread += $(call ros-pkg-cflags,webview_msgs) $(CFLAGS)
    CFLAGS_webview_plugin = $(CFLAGS_webview_thread)
    CFLAGS_webview_reqproc = $(CFLAGS_webview_thread)
    LDFLAGS_ros_webview += $(LDFLAGS) $(call ros-pkg-lflags,webview_msgs)
    PLUGINS_all += $(PLUGINDIR)/ros-webview.so
  else
    WARN_TARGETS += warning_webview_msgs
  endif

  ifeq ($(call ros-have-pkg,tf),1)
    ifeq ($(HAVE_TF),1)
      CFLAGS_tf_thread += $(CFLAGS) $(CFLAGS_TF) \
			  $(call ros-pkg-cflags,tf) -Wno-strict-aliasing
      CFLAGS_tf_plugin = $(CFLAGS_tf_thread)
      LDFLAGS_ros_tf += $(LDFLAGS_TF)  $(patsubst %bullet/lib,,$(call ros-pkg-lflags,tf))
      PLUGINS_all += $(PLUGINDIR)/ros-tf.so
    else
      WARN_TARGETS += warning_fawkestf
    endif
  else
    WARN_TARGETS += warning_rostf
  endif

  ifeq ($(call ros-have-pkg,sensor_msgs),1)
    ifeq ($(HAVE_PCL),1)
      CFLAGS_pcl_adapter += $(CFLAGS_PCL) $(CFLAGS_TF) $(CFLAGS)
      CFLAGS_pcl_thread += $(call ros-pkg-cflags,sensor_msgs) \
			   $(CFLAGS_TF) $(CFLAGS_PCL) $(CFLAGS)
      CFLAGS_pcl_plugin = $(CFLAGS_pcl_thread) $(CFLAGS)
      LDFLAGS_ros_pcl += $(LDFLAGS_PCL) $(LDFLAGS_TF) $(call ros-pkg-lflags,sensor_msgs)
      PLUGINS_all += $(PLUGINDIR)/ros-pcl.so
    else
      WARN_TARGETS += warning_pcl
    endif

    ifeq ($(call ros-have-pkg,image_transport),1)
      ifeq ($(HAVE_TINYXML),1)
        CFLAGS_image_thread += $(call ros-pkg-cflags,image_transport) $(CFLAGS)
        CFLAGS_image_plugin = $(CFLAGS_image_thread)
        LDFLAGS_ros_images += $(call ros-pkg-lflags,image_transport)
        PLUGINS_all += $(PLUGINDIR)/ros-images.so
      else
        WARN_TARGETS += warning_tinyxml
    endif
    else
      WARN_TARGETS += warning_images
    endif

    CFLAGS_laserscan_thread += $(call ros-pkg-cflags,sensor_msgs) $(CFLAGS)
    CFLAGS_laserscan_plugin = $(CFLAGS_laserscan_thread) $(CFLAGS)
    LDFLAGS_ros_laserscan += $(call ros-pkg-lflags,sensor_msgs)
    PLUGINS_all += $(PLUGINDIR)/ros-laserscan.so

  else
    WARN_TARGETS += warning_rospcl warning_images warning_laserscan
  endif

  ifeq ($(call ros-have-pkg,nav_msgs),1)
    ifeq ($(HAVE_TF),1)
      CFLAGS_odometry_thread += $(call ros-pkg-cflags,nav_msgs) $(CFLAGS) $(CFLAGS_TF)
      CFLAGS_odometry_plugin = $(CFLAGS_odometry_thread)
      LDFLAGS_ros_odometry += $(call ros-pkg-lflags,nav_msgs) $(LDFLAGS_TF)
      PLUGINS_all += $(PLUGINDIR)/ros-odometry.so
    else
      WARN_TARGETS += warning_odometry_tf
    endif
  else
    WARN_TARGETS += warning_nav_msgs
  endif

  ifeq ($(call ros-have-pkg,geometry_msgs),1)
    CFLAGS_cmdvel_thread += $(call ros-pkg-cflags,geometry_msgs) $(CFLAGS) $(CFLAGS_TF)
    CFLAGS_cmdvel_plugin = $(CFLAGS_cmdvel_thread)
    LDFLAGS_ros_cmdvel += $(call ros-pkg-lflags,geometry_msgs) $(LDFLAGS_TF)
    PLUGINS_all += $(PLUGINDIR)/ros-cmdvel.so
  else
    WARN_TARGETS += warning_geometry_msgs
  endif

  ifeq ($(call ros-have-pkg,move_base_msgs),1)
    ifeq ($(call ros-have-pkg,actionlib),1)
      ifeq ($(HAVE_TF),1)
        CFLAGS_navigator_thread += $(call ros-pkg-cflags,move_base_msgs) \
				   $(call ros-pkg-cflags,actionlib) \
				   $(call boost-lib-cflags,thread) \
				   $(CFLAGS) $(CFLAGS_TF)
        CFLAGS_navigator_plugin = $(CFLAGS_navigator_thread)
        LDFLAGS_ros_navigator += $(call ros-pkg-lflags,move_base_msgs) \
				 $(call ros-pkg-lflags,actionlib) \
				 $(call boost-lib-ldflags,thread) \
				 $(call boost-lib-ldflags,system) \
				 $(LDFLAGS_TF)

        CFLAGS_move_base_thread += $(call ros-pkg-cflags,move_base_msgs) \
				   $(call ros-pkg-cflags,actionlib) \
				   $(call boost-lib-cflags,thread) \
				   $(call boost-lib-cflags,system) \
				   $(CFLAGS) $(CFLAGS_TF)
        CFLAGS_move_base_plugin = $(CFLAGS_move_base_thread)
        LDFLAGS_ros_move_base += $(call ros-pkg-lflags,move_base_msgs) \
				 $(call ros-pkg-lflags,actionlib) \
				 $(call boost-lib-ldflags,thread) -lpthread \
				 $(call boost-lib-ldflags,system)  \
				 $(LDFLAGS_TF)

        PLUGINS_all += $(PLUGINDIR)/ros-navigator.so $(PLUGINDIR)/ros-move-base.so
      else
        WARN_TARGETS += warning_navigator_tf
      endif
    else
      WARN_TARGETS += warning_actionlib
    endif
  else
    WARN_TARGETS += warning_move_base_msgs
  endif
  ifeq ($(call ros-have-pkg,sensor_msgs),1)
    CFLAGS_joint_thread += $(call ros-pkg-cflags,sensor_msgs)
    CFLAGS_joint_plugin = $(CFLAGS_joint_thread)
    LDFLAGS_joint += $(call ros-pkg-lflags,sensor_msgs)
    PLUGINS_all += $(PLUGINDIR)/ros-joint.so
  else
    WARN_TARGETS += warning_sensor_msgs
  endif
  ifeq ($(call ros-have-pkg,fawkes_msgs),1)
    CFLAGS_position_3d_thread += $(call ros-pkg-cflags,fawkes_msgs)
    CFLAGS_position_3d_plugin = $(CFLAGS_position_3d_thread)
    LDFLAGS_position_3d += $(call ros-pkg-lflags,fawkes_msgs)
    PLUGINS_all += $(PLUGINDIR)/ros-position-3d.so
  else
    WARN_TARGETS += warning_fawkes_msgs
  endif

  PLUGINS_all += $(PLUGINDIR)/ros-robot-description.so

  ifeq ($(call ros-have-pkg,fawkes_msgs),1)
    ifeq ($(call ros-have-pkg,actionlib),1)
      CFLAGS_skiller_thread += $(call ros-pkg-cflags,fawkes_msgs) \
      $(call ros-pkg-cflags,actionlib) \
      $(call boost-lib-cflags,thread) \
      $(call boost-lib-cflags,system) \
      $(CFLAGS)
      CFLAGS_skiller_plugin += $(CFLAGS_skiller_thread)
      LDFLAGS_ros_skiller += $(call ros-pkg-lflags,fawkes_msgs) \
      $(call ros-pkg-lflags,actionlib) \
      $(call boost-lib-ldflags,thread) \
      $(call boost-lib-ldflags,system) \
      $(LDFLAGS) -lpthread
      PLUGINS_all += $(PLUGINDIR)/ros-skiller.so
    else
      WARN_TARGETS += warning_actionlib      
    endif
  else
    WARN_TARGETS += warning_fawkes_msgs
  endif
else
  WARN_TARGETS += warning_ros
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_ros warning_webview_msgs warning_fawkestf warning_rostf warning_pcl warning_rospcl
warning_ros:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS Plugins$(TNORMAL) (ROS not found)"
warning_webview_msgs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS Webview Plugin$(TNORMAL) (ROS package webview_msgs not found)"
warning_fawkestf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS tf plugin$(TNORMAL) (fawkestf not available)"
warning_rostf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS tf plugin$(TNORMAL) (ROS package tf not found)"
warning_pcl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS point clouds plugin$(TNORMAL) (pcl[-devel] not installed)"
warning_rospcl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS point clouds plugin$(TNORMAL) (ROS package pcl not found)"
warning_images:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS images plugin$(TNORMAL) (ROS package image_transport not found)"
warning_laserscan:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS laser scan plugin$(TNORMAL) (ROS package sensor_msgs not found)"
warning_nav_msgs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS odometry Plugin$(TNORMAL) (ROS package nav_msgs not found)"
warning_odometry:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS odometry Plugin$(TNORMAL) (Fawkes TF library not available)"
warning_geometry_msgs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS cmdvel Plugin$(TNORMAL) (ROS package geometry_msgs not found)"
warning_move_base_msgs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS navigator and move_base plugins$(TNORMAL) (ROS package move_base_msgs not found)"
warning_navigator_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS navigator and move_base plugins$(TNORMAL) (Fawkes TF library not available)"
warning_actionlib:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS navigator, move_base and skiller plugins$(TNORMAL) (ROS package actionlib not found)"
warning_sensor_msgs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS JointState Plugin$(TNORMAL) (ROS package sensor_msgs not found)"
warning_fawkes_msgs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS skiller plugin$(TNORMAL) (ROS package fawkes_msgs not found)"
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS Position 3D Plugin$(TNORMAL) (ROS package fawkes_msgs not found)"
warning_tinyxml:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS images plugin$(TNORMAL) (tinyxml[-devel] not installed)"
endif

include $(BUILDSYSDIR)/base.mk

