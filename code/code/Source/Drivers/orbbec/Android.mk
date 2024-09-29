# OpenNI 2.x Android makefile.
# Copyright (C) 2012 PrimeSense Ltd.
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

include $(CLEAR_VARS)

include $(LOCAL_PATH)/../../../ThirdParty/PSCommon/BuildSystem/CommonAndroid.mk

# Sources
MY_SRC_FILES := \
    $(LOCAL_PATH)/Core/*.cpp	\
    $(LOCAL_PATH)/DDK/*.cpp 	\
    $(LOCAL_PATH)/DriverImpl/*.cpp\
    $(LOCAL_PATH)/Formats/*.cpp	\
    $(LOCAL_PATH)/Include/*.cpp	\
    $(LOCAL_PATH)/Sensor/*.cpp

MY_SRC_FILE_EXPANDED := $(wildcard $(MY_SRC_FILES))
LOCAL_SRC_FILES := $(MY_SRC_FILE_EXPANDED:$(LOCAL_PATH)/%=%)

# C/CPP Flags
LOCAL_CPP_FEATURES := rtti  

LOCAL_CFLAGS += -std=c++11
# Includes
LOCAL_C_INCLUDES := \
    $(LOCAL_PATH)/. \
    $(LOCAL_PATH)/Include \
    $(LOCAL_PATH)/../../DepthUtils \
    $(LOCAL_PATH)/../../../Include \
    $(LOCAL_PATH)/../../../ThirdParty/Orbbec/multiDistanceCalibrationLib/Include \
	$(LOCAL_PATH)/../../../ThirdParty/Orbbec/Include \
    $(LOCAL_PATH)/../../../ThirdParty/ffmpeg/include \

# Dependencies
LOCAL_STATIC_LIBRARIES := XnLib DepthUtils multiDisCalibration postfilter
LOCAL_SHARED_LIBRARIES := liborbbecusb2 libavcodec libavutil libswscale libavdevice libavfilter libavformat libswresample libpostproc

ifdef PS_OS_BUILD
    LOCAL_SHARED_LIBRARIES += libjpeg
else
    LOCAL_LDLIBS += -llog
endif

# Output
LOCAL_MODULE:= liborbbec

include $(BUILD_SHARED_LIBRARY)

#include XnLib
include $(LOCAL_PATH)/../../../ThirdParty/PSCommon/XnLib/Source/Android.mk

#include multiDisCalibration
include $(LOCAL_PATH)/../../../ThirdParty/Orbbec/multiDistanceCalibrationLib/Android.mk

#include postfilter
include $(LOCAL_PATH)/../../../ThirdParty/Orbbec/filter/Android.mk

#include ffmpeg
include $(LOCAL_PATH)/../../../ThirdParty/ffmpeg/avutil/Android.mk  
include $(LOCAL_PATH)/../../../ThirdParty/ffmpeg/avcodec/Android.mk 
include $(LOCAL_PATH)/../../../ThirdParty/ffmpeg/swscale/Android.mk 
include $(LOCAL_PATH)/../../../ThirdParty/ffmpeg/avdevice/Android.mk
include $(LOCAL_PATH)/../../../ThirdParty/ffmpeg/avfilter/Android.mk
include $(LOCAL_PATH)/../../../ThirdParty/ffmpeg/avformat/Android.mk
include $(LOCAL_PATH)/../../../ThirdParty/ffmpeg/swresample/Android.mk
include $(LOCAL_PATH)/../../../ThirdParty/ffmpeg/postproc/Android.mk

