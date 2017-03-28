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
LOCAL_PATH := $(call my-dir)
CARTOGRAPHER_TOP := $(LOCAL_PATH)
EXTERNAL := $(CARTOGRAPHER_TOP)/external
CARTOGRAPHER := $(CARTOGRAPHER_TOP)/cartographer
include $(CARTOGRAPHER_TOP)/functions.mk

#EXTERNAL_protobuf := $(EXTERNAL)/protobuf
#EXTERNAL_protobuf_SRC := $(EXTERNAL_protobuf)/src

#build external
include $(EXTERNAL)/Android.mk

#build cartographer
include $(CARTOGRAPHER_TOP)/Android-cartographer.mk

#build cartorgarpher test
include $(CARTOGRAPHER_TOP)/Android-cartographer-android.mk

