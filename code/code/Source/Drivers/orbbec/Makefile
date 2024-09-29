include ../../../ThirdParty/PSCommon/BuildSystem/CommonDefs.mak

BIN_DIR = ../../../Bin

INC_DIRS = \
    . \
    Include \
    ../../../Include \
    ../../../ThirdParty/PSCommon/XnLib/Include \
    ../../DepthUtils \
    ../../../ThirdParty/Orbbec/Include \
    ../../../ThirdParty/Orbbec/multiDistanceCalibrationLib/Include \
	../../../ThirdParty/ffmpeg/include

SRC_FILES = \
    Core/*.cpp	\
    DDK/*.cpp	\
    DriverImpl/*.cpp\
    Formats/*.cpp	\
    Include/*.cpp	\
    Sensor/*.cpp	\


ifeq ("$(OSTYPE)","Darwin")
    INC_DIRS += \
        /usr/local/include \
        /opt/local/include
    LIB_DIRS += \
        /usr/local/lib \
        /opt/local/lib
    LDFLAGS += \
        -framework CoreFoundation \
        -framework IOKit
endif

LIB_NAME = orbbec

LIB_DIRS += ../../../ThirdParty/PSCommon/XnLib/Bin/$(PLATFORM)-$(CFG)
LIB_DIRS += $(BIN_DIR)/$(PLATFORM)-$(CFG)
LIB_DIRS += ../../../ThirdParty/Orbbec/filter/$(PLATFORM)-$(CFG)
LIB_DIRS += ../../../ThirdParty/Orbbec/multiDistanceCalibrationLib/$(PLATFORM)-$(CFG)

#FFMPEG_PLATFORM = raspberry
#FFMPEG_PLATFORM = ubuntu16-x64
FFMPEG_PLATFORM = ubuntu18-x64
#FFMPEG_PLATFORM = ubuntu20-x64
LIB_DIRS += ../../../ThirdParty/ffmpeg/avcodec/$(FFMPEG_PLATFORM)
LIB_DIRS += ../../../ThirdParty/ffmpeg/avdevice/$(FFMPEG_PLATFORM)
LIB_DIRS += ../../../ThirdParty/ffmpeg/avfilter/$(FFMPEG_PLATFORM)
LIB_DIRS += ../../../ThirdParty/ffmpeg/avformat/$(FFMPEG_PLATFORM)
LIB_DIRS += ../../../ThirdParty/ffmpeg/avutil/$(FFMPEG_PLATFORM)
LIB_DIRS += ../../../ThirdParty/ffmpeg/postproc/$(FFMPEG_PLATFORM)
LIB_DIRS += ../../../ThirdParty/ffmpeg/swresample/$(FFMPEG_PLATFORM)
LIB_DIRS += ../../../ThirdParty/ffmpeg/swscale/$(FFMPEG_PLATFORM)

USED_LIBS = XnLib dl pthread DepthUtils
USED_LIBS += SoftFilter
USED_LIBS += bp_undistortion
USED_LIBS += postfilter
USED_LIBS += avcodec
USED_LIBS += avutil
USED_LIBS += swscale
ifneq ("$(OSTYPE)","Darwin")
    USED_LIBS += rt usb-1.0 udev
else
    USED_LIBS += usb-1.0.0
endif

CFLAGS += -Wall -Wno-unused-variable

OUT_DIR := $(OUT_DIR)/OpenNI2/Drivers

include ../../../ThirdParty/PSCommon/BuildSystem/CommonCppMakefile
