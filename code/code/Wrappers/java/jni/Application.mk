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

# Android should be >= v4.0
APP_PLATFORM := android-19

# Use ARM v7a instruction set
APP_ABI := armeabi-v7a
#APP_ABI := arm64-v8a
ARCH_ARM_HAVE_ARMV7A := true
ARCH_ARM_HAVE_NEON := true
APP_STL := stlport_static
#APP_STL := gnustl_static
APP_ALLOW_MISSING_DEPS=true
