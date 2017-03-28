# Copyright (C) 2015 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

PROTO_CC := $(CARTOGRAPHER_TOP)/proto_cc

#build proto_cc
PROTO_SRC_FILES := \
	$(call all-cc-files-unnder ,$(PROTO_CC))

PROTO_INCLUDES := \
	$(PROTO_CC)
	
PROTO_EXPORT_INCLUDES := \
	$(PROTO_CC)

SRC_FILES := \
	$(CARTOGRAPHER_TOP)/cartographer_android/sensor_bridge.cc \
	$(CARTOGRAPHER_TOP)/cartographer_android/node_options.cc \
	$(CARTOGRAPHER_TOP)/cartographer_android/assets_writer.cc \
	$(CARTOGRAPHER_TOP)/cartographer_android/ros_log_sink.cc \
	$(CARTOGRAPHER_TOP)/cartographer_android/map_builder_bridge.cc \
	$(CARTOGRAPHER_TOP)/cartographer_android/node.cc \
	$(CARTOGRAPHER_TOP)/cartographer_android/node_main.cc
#	$(call all-test-files-under ,$(CARTOGRAPHER_TOP)/cartographer)
INCLUDES := \
	$(PROTO_INCLUDES) \
	$(CARTOGRAPHER_TOP) 

LIBRARIES := \
	libapac_cartographer libapac_gtest libapac_gmock libapac_protobuf libapac_ceres libapac_glog libapac_boost libapac_gflags

include $(CLEAR_VARS)

LOCAL_SRC_FILES := \
	$(SRC_FILES)
LOCAL_MODULE := apac_cartographer_android

LOCAL_CFLAGS += -std=c++11 -fexceptions -frtti -DANDROID_STD_TO_STRING -Wno-return-type -D_ANDROID_ -Wno-c++11-narrowing
LOCAL_SHARED_LIBRARIES := \
	$(LIBRARIES)
#LOCAL_STATIC_LIBRARIES := libapac_cartographer_static
#libapac_gtest_static
LOCAL_MODULE_TAGS := optional
LOCAL_LDLIBS += -latomic -llog
include $(BUILD_EXECUTABLE)
#include $(BUILD_SHARED_LIBRARY)
