/*****************************************************************************
*                                                                            *
*  OpenNI 2.x Alpha                                                          *
*  Copyright (C) 2012 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  Licensed under the Apache License, Version 2.0 (the "License");           *
*  you may not use this file except in compliance with the License.          *
*  You may obtain a copy of the License at                                   *
*                                                                            *
*      http://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
*  Unless required by applicable law or agreed to in writing, software       *
*  distributed under the License is distributed on an "AS IS" BASIS,         *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and       *
*  limitations under the License.                                            *
*                                                                            *
*****************************************************************************/
#include <jni.h>
#include "OniProperties.h"
#include "OniEnums.h"
#include "OniCAPI.h"
#include "org_openni_NativeMethods.h"
#include "PS1080.h"
#include <XnPlatform.h>
#include "XnLogTypes.h"

#ifdef ANDROID
#include <android/log.h>
#endif

#define DEBUG 1

#if DEBUG && defined(ANDROID)
#include <android/log.h>
#define  LOGD(x...)  __android_log_print(ANDROID_LOG_INFO,"OpenNIJNI",x)
#define  LOGE(x...)  __android_log_print(ANDROID_LOG_ERROR,"OpenNIJNI",x)
#else
#define  LOGD(...)
#define  LOGE(...)
#endif

using namespace openni;
JavaVM* g_pVM = NULL;
jclass g_videoStreamClass;
jclass g_openNIClass;
jclass g_deviceInfoClass;
jclass g_logcatClass;

class JNIEnvSupplier
{
public:
    JNIEnvSupplier() : m_pEnv(NULL), m_bShouldDetach(FALSE)
    {
        if (JNI_EDETACHED == g_pVM->GetEnv((void**)&m_pEnv, JNI_VERSION_1_2))
        {
            g_pVM->AttachCurrentThread((void**)&m_pEnv, NULL);
            m_bShouldDetach = TRUE;
        }
    }

    ~JNIEnvSupplier()
    {
        if (m_bShouldDetach)
        {
            g_pVM->DetachCurrentThread();
        }
    }

    JNIEnv* GetEnv() { return m_pEnv; }

private:
    JNIEnv* m_pEnv;
    XnBool m_bShouldDetach;
};


void SetOutArgObjectValue(JNIEnv*env, jobject p, jobject value)
{
    jclass cls = env->GetObjectClass(p);
    jfieldID fieldID = env->GetFieldID(cls, "mValue", "Ljava/lang/Object;");
    env->SetObjectField(p, fieldID, value);
}


void SetOutArgVideoModeValue(JNIEnv*env, jobject p, jobject value)
{
    SetOutArgObjectValue(env, p, value);
}

void SetOutArgDoubleValue(JNIEnv*env, jobject p, double value)
{
    jclass cls = env->FindClass("java/lang/Double");
    jmethodID ctor = env->GetMethodID(cls, "<init>", "(D)V");
    SetOutArgObjectValue(env, p, env->NewObject(cls, ctor, value));
}

void SetOutArgLongValue(JNIEnv*env, jobject p, XnUInt64 value)
{
    jclass cls = env->FindClass("java/lang/Long");
    jmethodID ctor = env->GetMethodID(cls, "<init>", "(J)V");
    SetOutArgObjectValue(env, p, env->NewObject(cls, ctor, value));
}

void SetOutArgIntValue(JNIEnv*env, jobject p, int value)
{
    jclass cls = env->FindClass("java/lang/Integer");
    jmethodID ctor = env->GetMethodID(cls, "<init>", "(I)V");
    SetOutArgObjectValue(env, p, env->NewObject(cls, ctor, value));
}

void SetOutArgShortValue(JNIEnv*env, jobject p, short value)
{
    jclass cls = env->FindClass("java/lang/Short");
    jmethodID ctor = env->GetMethodID(cls, "<init>", "(S)V");
    SetOutArgObjectValue(env, p, env->NewObject(cls, ctor, value));
}

void SetOutArgByteValue(JNIEnv*env, jobject p, XnUInt8 value)
{
    jclass cls = env->FindClass("java/lang/Byte");
    jmethodID ctor = env->GetMethodID(cls, "<init>", "(B)V");
    SetOutArgObjectValue(env, p, env->NewObject(cls, ctor, value));
}

void SetOutArgBoolValue(JNIEnv*env, jobject p, jboolean value)
{
    jclass cls = env->FindClass("java/lang/Boolean");
    jmethodID ctor = env->GetMethodID(cls, "<init>", "(Z)V");
    SetOutArgObjectValue(env, p, env->NewObject(cls, ctor, value));
}

void SetOutArgFloatValue(JNIEnv*env, jobject p, jfloat value)
{
    jclass cls = env->FindClass("java/lang/Float");
    jmethodID ctor = env->GetMethodID(cls, "<init>", "(F)V");
    SetOutArgObjectValue(env, p, env->NewObject(cls, ctor, value));
}

void SetOutArgStringValue(JNIEnv*env, jobject p, const XnChar* value)
{
    SetOutArgObjectValue(env, p, env->NewStringUTF(value));
}

void setOutArgFloatArrayValue(JNIEnv*env, jclass clazz, jobject obj, const char *name, const jfloat *buf, int length)
{
    jmethodID dIntrP = env->GetMethodID(clazz, name, "([F)V");
    jfloatArray out = env->NewFloatArray(length);
    env->SetFloatArrayRegion(out, 0, length, buf);
    env->CallVoidMethod(obj, dIntrP, out);
    env->DeleteLocalRef(out);
}

JNIEnv *g_env;
JNIEXPORT void JNICALL Java_org_openni_NativeMethods_oniFrameRelease(JNIEnv *, jclass, jlong frame)
{
    oniFrameRelease((OniFrame*)frame);
}

JNIEXPORT void JNICALL Java_org_openni_NativeMethods_oniFrameAddRef(JNIEnv *, jclass, jlong frame)
{
    oniFrameAddRef((OniFrame*)frame);
}

static void ONI_CALLBACK_TYPE callback(OniStreamHandle streamHandle, void*)
{
    JNIEnvSupplier suplier;
    jmethodID methodID = suplier.GetEnv()->GetStaticMethodID(g_videoStreamClass, "onFrameReady", "(J)V");
    jlong handle = (jlong)streamHandle;
    suplier.GetEnv()->CallStaticVoidMethod(g_videoStreamClass, methodID, handle);
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniDeviceCreateStream(
    JNIEnv *env,
    jclass,
    jlong device,
    jint sensorType,
    jobject videoStreamObj)
{
    OniStreamHandle* streamHandle;
    jint status = oniDeviceCreateStream((OniDeviceHandle)device, (OniSensorType)sensorType, (OniStreamHandle*)&streamHandle);
    if (status == ONI_STATUS_OK)
    {
        jclass videoStreamCls = env->FindClass("org/openni/VideoStream");
        jfieldID fieldID = env->GetFieldID(videoStreamCls, "mStreamHandle", "J");
        env->SetLongField(videoStreamObj, fieldID, (jlong)streamHandle);
        OniCallbackHandle handle = 0;
        status = oniStreamRegisterNewFrameCallback((OniStreamHandle)streamHandle, callback, videoStreamCls, &handle);
        fieldID = env->GetFieldID(videoStreamCls, "mCallbackHandle", "J");
        env->SetLongField(videoStreamObj, fieldID, (jlong)handle);
    }
    return status;
}

JNIEXPORT void JNICALL Java_org_openni_NativeMethods_oniStreamDestroy(
    JNIEnv *,
    jclass,
    jlong streamHandle,
    jlong callbackHandle)
{
    oniStreamUnregisterNewFrameCallback((OniStreamHandle)streamHandle, (OniCallbackHandle)callbackHandle);
    oniStreamDestroy((OniStreamHandle)streamHandle);
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniStreamStart(JNIEnv *, jclass, jlong streamHandle)
{
    return oniStreamStart((OniStreamHandle)streamHandle);
}

JNIEXPORT void JNICALL Java_org_openni_NativeMethods_oniStreamStop(JNIEnv *, jclass, jlong streamHandle)
{
    oniStreamStop((OniStreamHandle)streamHandle);
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniStreamReadFrame(
    JNIEnv *env,
    jclass,
    jlong streamHandle,
    jobject outArgObj)
{
    OniFrame* pOniFrame = NULL;
    int status = oniStreamReadFrame((OniStreamHandle)streamHandle, &pOniFrame);
    if (status == STATUS_OK && pOniFrame != NULL)
    {
        jclass videoFrameRefCls = env->FindClass("org/openni/VideoFrameRef");
        jmethodID videoFrameCtor = env->GetMethodID(videoFrameRefCls, "<init>", "(J)V");
        jobject videoFrameRefObj = env->NewObject(videoFrameRefCls, videoFrameCtor, (jlong)pOniFrame);

        jfieldID fieldID = env->GetFieldID(videoFrameRefCls, "mTimestamp", "J");
        env->SetLongField(videoFrameRefObj, fieldID, (jlong)pOniFrame->timestamp);

        fieldID = env->GetFieldID(videoFrameRefCls, "mIndex", "I");
        env->SetIntField(videoFrameRefObj, fieldID, pOniFrame->frameIndex);

        fieldID = env->GetFieldID(videoFrameRefCls, "mWidth", "I");
        env->SetIntField(videoFrameRefObj, fieldID, pOniFrame->width);

        fieldID = env->GetFieldID(videoFrameRefCls, "mHeight", "I");
        env->SetIntField(videoFrameRefObj, fieldID, pOniFrame->height);

        fieldID = env->GetFieldID(videoFrameRefCls, "mIsCropping", "Z");
        env->SetBooleanField(videoFrameRefObj, fieldID, (pOniFrame->croppingEnabled == TRUE));

        fieldID = env->GetFieldID(videoFrameRefCls, "mCropOrigX", "I");
        env->SetIntField(videoFrameRefObj, fieldID, pOniFrame->cropOriginX);

        fieldID = env->GetFieldID(videoFrameRefCls, "mCropOrigY", "I");
        env->SetIntField(videoFrameRefObj, fieldID, pOniFrame->cropOriginY);

        fieldID = env->GetFieldID(videoFrameRefCls, "mStride", "I");
        env->SetIntField(videoFrameRefObj, fieldID, pOniFrame->stride);

        jclass byteOrderCls = env->FindClass("java/nio/ByteOrder");
        jfieldID littleEndianField = env->GetStaticFieldID(byteOrderCls, "LITTLE_ENDIAN", "Ljava/nio/ByteOrder;");
        jobject littleEndian = env->GetStaticObjectField(byteOrderCls, littleEndianField);

        jclass byteBufferCls = env->FindClass("java/nio/ByteBuffer");
        jmethodID orderMethodId = env->GetMethodID(byteBufferCls, "order", "(Ljava/nio/ByteOrder;)Ljava/nio/ByteBuffer;");

        jobject buffer = env->NewDirectByteBuffer(
            (uint8_t*)pOniFrame->data + pOniFrame->stride * pOniFrame->extraLine,
            pOniFrame->dataSize - pOniFrame->stride * pOniFrame->extraLine
            );
        env->CallObjectMethod(buffer, orderMethodId, littleEndian);
        fieldID = env->GetFieldID(videoFrameRefCls, "mData", "Ljava/nio/ByteBuffer;");
        env->SetObjectField(videoFrameRefObj, fieldID, buffer);

        jclass sensorTypeCls = env->FindClass("org/openni/SensorType");
        jmethodID sensorFromNative = env->GetStaticMethodID(sensorTypeCls, "fromNative", "(I)Lorg/openni/SensorType;");
        jobject sensorTypeObj = env->CallStaticObjectMethod(sensorTypeCls, sensorFromNative, pOniFrame->sensorType);
        fieldID = env->GetFieldID(videoFrameRefCls, "mSensorType", "Lorg/openni/SensorType;");
        env->SetObjectField(videoFrameRefObj, fieldID, sensorTypeObj);

        jclass videoModeCls = env->FindClass("org/openni/VideoMode");
        jmethodID videoModeCtor = env->GetMethodID(videoModeCls, "<init>", "(IIII)V");
        jobject videoModeObj = env->NewObject(
            videoModeCls,
            videoModeCtor,
            (jint)pOniFrame->videoMode.resolutionX,
            (jint)pOniFrame->videoMode.resolutionY,
            (jint)pOniFrame->videoMode.fps,
            (jint)pOniFrame->videoMode.pixelFormat);
        fieldID = env->GetFieldID(videoFrameRefCls, "mVideoMode", "Lorg/openni/VideoMode;");
        env->SetObjectField(videoFrameRefObj, fieldID, videoModeObj);

        jobject metaBufferObj = env->NewDirectByteBuffer(pOniFrame->data, pOniFrame->stride * pOniFrame->extraLine);
        env->CallObjectMethod(metaBufferObj, orderMethodId, littleEndian);
        jclass metaDataCls = env->FindClass("org/openni/ExtraMetaData");
        jmethodID metaDataCtor = env->GetMethodID(metaDataCls, "<init>", "(IIILjava/nio/ByteBuffer;)V");
        jobject metaObj = env->NewObject(
            metaDataCls,
            metaDataCtor,
            (jint)pOniFrame->width,
            (jint)pOniFrame->extraLine,
            (jint)pOniFrame->stride,
            metaBufferObj);
        fieldID = env->GetFieldID(videoFrameRefCls, "mMetaData", "Lorg/openni/ExtraMetaData;");
        env->SetObjectField(videoFrameRefObj, fieldID, metaObj);

        SetOutArgObjectValue(env, outArgObj, videoFrameRefObj);

        // release this frame. The java object is its owner now.
        oniFrameRelease(pOniFrame);
    }
    else{
        status = STATUS_ERROR;
    }

    return status;
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniStreamReadFrameEx(
    JNIEnv *env,
    jclass,
    jlong streamHandle,
    jobject outArgObj)
{
    OniFrame* pFrame = NULL;
    int ret = oniStreamReadFrame((OniStreamHandle)streamHandle, &pFrame);
    if (STATUS_OK != ret || NULL == pFrame || NULL == pFrame->data || pFrame->dataSize <= 0)
        return ret;

    OniAIFrame* pAIFrame = (OniAIFrame*)pFrame->data;

    jclass jListCls = env->FindClass("java/util/ArrayList");
    jmethodID jListInit = env->GetMethodID(jListCls, "<init>", "()V");
    jmethodID jListAdd = env->GetMethodID(jListCls, "add", "(Ljava/lang/Object;)Z");

    jclass jIntCls = env->FindClass("java/lang/Integer");
    jmethodID jIntCtor = env->GetMethodID(jIntCls, "<init>", "(I)V");

    jclass jFloatCls = env->FindClass("java/lang/Float");
    jmethodID jFloatCtor = env->GetMethodID(jFloatCls, "<init>", "(F)V");

    jclass doubleCls = env->FindClass("java/lang/Double");
    jmethodID doubleCtor = env->GetMethodID(doubleCls, "<init>", "(D)V");

    jclass jPoint2DCls = env->FindClass("org/openni/Point2D");
    jmethodID jPoint2DCtor = env->GetMethodID(jPoint2DCls, "<init>", "(Ljava/lang/Object;Ljava/lang/Object;)V");

    jclass jPoint3DCls = env->FindClass("org/openni/Point3D");
    jmethodID jPoint3DCtor = env->GetMethodID(jPoint3DCls, "<init>", "(Ljava/lang/Object;Ljava/lang/Object;Ljava/lang/Object;)V");

    jclass jJointCls = env->FindClass("org/openni/Joint");
    jmethodID jJointCtor = env->GetMethodID(jJointCls, "<init>", "(IDLorg/openni/Point3D;)V");

    jclass jBodyShapeCls = env->FindClass("org/openni/BodyShape");
    jmethodID jBodyShapeCtor = env->GetMethodID(jBodyShapeCls, "<init>", "(IFFFFFFFLjava/util/ArrayList;)V");
    
    jclass jBodyCls = env->FindClass("org/openni/Body");
    jmethodID jBodyCtor = env->GetMethodID(jBodyCls, "<init>", "(IIIIIILjava/util/ArrayList;Lorg/openni/BodyShape;)V");
    jobject jBodyListObj = env->NewObject(jListCls, jListInit);

    /// Body List.
    for (uint32_t i = 0; i < pAIFrame->bodyList.num; ++i)
    {
        OniBody* pBody = &pAIFrame->bodyList.bodies[i];

        /// Joints.
        jobject jJointListObj = env->NewObject(jListCls, jListInit);
        for (uint32_t j = 0; j < ONI_JOINT_MAX; ++j)
        {
            jobject jobjX = env->NewObject(
                doubleCls,
                doubleCtor,
                (jdouble)pBody->joints[j].position.x);

            jobject jobjY = env->NewObject(
                doubleCls,
                doubleCtor,
                (jdouble)pBody->joints[j].position.y);

            jobject jobjZ = env->NewObject(
                doubleCls,
                doubleCtor,
                (jdouble)pBody->joints[j].position.z);

            jobject jPoint3DObj = env->NewObject(
                jPoint3DCls,
                jPoint3DCtor,
                jobjX,
                jobjY,
                jobjZ);

            jobject jJointObj = env->NewObject(
                jJointCls,
                jJointCtor,
                (jint)pBody->joints[j].type,
                (jdouble)pBody->joints[j].score,
                (jobject)jPoint3DObj);

            env->CallBooleanMethod(jJointListObj, jListAdd, jJointObj);
        }

        /// Body Shape.
        jobject jPointListObj = env->NewObject(jListCls, jListInit);
        for (uint32_t z = 0; z < ONI_BODY_POINT_NUM; ++z)
        {
            jobject jObjU = env->NewObject(
                jIntCls,
                jIntCtor,
                (jint)pBody->bodyShape.points[z].x);

            jobject jObjV = env->NewObject(
                jIntCls,
                jIntCtor,
                (jint)pBody->bodyShape.points[z].y);

            jobject jPoint2DObj = env->NewObject(
                jPoint2DCls,
                jPoint2DCtor,
                jObjU,
                jObjV);

            env->CallBooleanMethod(jPointListObj, jListAdd, jPoint2DObj);
        }

        jobject jBodyShapeObj = env->NewObject(
            jBodyShapeCls,
            jBodyShapeCtor,
            (jint)pBody->bodyShape.figure,
            (jfloat)pBody->bodyShape.ratio,
            (jfloat)pBody->bodyShape.height,
            (jfloat)pBody->bodyShape.waist,
            (jfloat)pBody->bodyShape.waistline,
            (jfloat)pBody->bodyShape.bust,
            (jfloat)pBody->bodyShape.hips,
            (jfloat)pBody->bodyShape.shoulder,
            (jobject)jPointListObj
            );

        jobject jBodyObj = env->NewObject(
            jBodyCls,
            jBodyCtor,
            (jint)pBody->id,
            (jint)pBody->jointWidth,
            (jint)pBody->jointHeight,
            (jint)pBody->jointFormat,
            (jint)pBody->jointStatus,
            (jint)pBody->bodyShapeStatus,
            jJointListObj,
            jBodyShapeObj);

        env->CallBooleanMethod(jBodyListObj, jListAdd, jBodyObj);
    }

    jclass jByteOrderCls = env->FindClass("java/nio/ByteOrder");
    jfieldID jLittleEndianField = env->GetStaticFieldID(jByteOrderCls, "LITTLE_ENDIAN", "Ljava/nio/ByteOrder;");
    jobject jLittleEndian = env->GetStaticObjectField(jByteOrderCls, jLittleEndianField);

    jclass jByteBufferCls = env->FindClass("java/nio/ByteBuffer");
    jmethodID jOrderMethodId = env->GetMethodID(jByteBufferCls, "order", "(Ljava/nio/ByteOrder;)Ljava/nio/ByteBuffer;");

    jclass jBitmapMaskCls = env->FindClass("org/openni/BitmapMask");
    jmethodID jBitmapMaskCtor = env->GetMethodID(jBitmapMaskCls, "<init>", "(IILjava/nio/ByteBuffer;)V");

    jclass jBodyMaskCls = env->FindClass("org/openni/BodyMask");
    jmethodID jBodyMaskCtor = env->GetMethodID(jBodyMaskCls, "<init>", "(ILorg/openni/BitmapMask;)V");

    /// Body Mask.
    jobject jBodyMaskBufferObj = env->NewDirectByteBuffer(
        (uint8_t*)pAIFrame->bodyMask.mask.address,
        (jlong)(pAIFrame->bodyMask.mask.width * pAIFrame->bodyMask.mask.height));
    env->CallObjectMethod(jBodyMaskBufferObj, jOrderMethodId, jLittleEndian);

    jobject jBodyBitmapMaskObj = env->NewObject(
        jBitmapMaskCls,
        jBitmapMaskCtor,
        (jint)pAIFrame->bodyMask.mask.width,
        (jint)pAIFrame->bodyMask.mask.height,
        jBodyMaskBufferObj);

    jobject jBodyMaskObj = env->NewObject(
        jBodyMaskCls,
        jBodyMaskCtor,
        (jint)pAIFrame->bodyMask.status,
        jBodyBitmapMaskObj);

    /// Floor Info.
    jclass jPlaneCls = env->FindClass("org/openni/Plane");
    jmethodID jPlaneCtor = env->GetMethodID(jPlaneCls, "<init>", "(Lorg/openni/Point3D;Lorg/openni/Point3D;)V");

    jclass jFloorInfoCls = env->FindClass("org/openni/FloorInfo");
    jmethodID jFloorInfoCtor = env->GetMethodID(jFloorInfoCls, "<init>", "(ILorg/openni/Plane;Lorg/openni/BitmapMask;)V");

    jobject jCentorObjX = env->NewObject(
        jFloatCls,
        jFloatCtor,
        (jfloat)pAIFrame->floorInfo.plane.center.x);

    jobject jCentorObjY = env->NewObject(
        jFloatCls,
        jFloatCtor,
        (jfloat)pAIFrame->floorInfo.plane.center.y);

    jobject jCentorObjZ = env->NewObject(
        jFloatCls,
        jFloatCtor,
        (jfloat)pAIFrame->floorInfo.plane.center.z);

    jobject jNormalObjX = env->NewObject(
        jFloatCls,
        jFloatCtor,
        (jfloat)pAIFrame->floorInfo.plane.normal.x);

    jobject jNormalObjY = env->NewObject(
        jFloatCls,
        jFloatCtor,
        (jfloat)pAIFrame->floorInfo.plane.normal.y);

    jobject jNormalObjZ = env->NewObject(
        jFloatCls,
        jFloatCtor,
        (jfloat)pAIFrame->floorInfo.plane.normal.z);

    jobject jPlaneCenterObj = env->NewObject(
        jPoint3DCls,
        jPoint3DCtor,
        jCentorObjX,
        jCentorObjY,
        jCentorObjZ);

    jobject jPlaneNormalObj = env->NewObject(
        jPoint3DCls,
        jPoint3DCtor,
        jNormalObjX,
        jNormalObjY,
        jNormalObjZ);

    jobject jPlaneObj = env->NewObject(
        jPlaneCls,
        jPlaneCtor,
        jPlaneCenterObj,
        jPlaneNormalObj);

    jobject jFloorMaskBufferObj = env->NewDirectByteBuffer(
        (uint8_t*)pAIFrame->floorInfo.mask.address,
        (jlong)(pAIFrame->floorInfo.mask.width * pAIFrame->floorInfo.mask.height));
    env->CallObjectMethod(jFloorMaskBufferObj, jOrderMethodId, jLittleEndian);

    jobject jFloorBitmapMaskObj = env->NewObject(
        jBitmapMaskCls,
        jBitmapMaskCtor,
        (jint)pAIFrame->floorInfo.mask.width,
        (jint)pAIFrame->floorInfo.mask.height,
        jFloorMaskBufferObj);

    jobject jFloorInfoObj = env->NewObject(
        jFloorInfoCls,
        jFloorInfoCtor,
        (jint)pAIFrame->floorInfo.status,
        jPlaneObj,
        jFloorBitmapMaskObj);

    jclass jAIFrameCls = env->FindClass("org/openni/AIFrameRef");
    jmethodID jAIFrameCtor = env->GetMethodID(jAIFrameCls, "<init>", "(IIJLjava/util/ArrayList;Lorg/openni/BodyMask;Lorg/openni/FloorInfo;)V");
    jobject jAIFrameObj = env->NewObject(
        jAIFrameCls,
        jAIFrameCtor,
        (jint)pFrame->frameIndex,
        (jint)pFrame->videoMode.pixelFormat,
        (jlong)pFrame->timestamp,
        jBodyListObj,
        jBodyMaskObj,
        jFloorInfoObj);

    SetOutArgObjectValue(env, outArgObj, jAIFrameObj);

    /// Release this frame. The java object is its owner now.
    oniFrameRelease(pFrame);

    /// All is good.
    return STATUS_OK;
}

JNIEXPORT void Java_org_openni_NativeMethods_oniStreamfilter(
    JNIEnv *env,
    jclass,
    jlong streamHandle,
    jobject buf,
    jint newVal,
    jint maxSpeckleSize,
    jint maxDiff)
{

    uint16_t * srcBuf = (uint16_t *)env->GetDirectBufferAddress(buf);
    if (srcBuf == NULL){
        LOGE(" %s:%d src buf is null", __func__, __LINE__);
        return;
    }

    oniStreamfilter((OniStreamHandle)streamHandle, srcBuf, newVal, maxSpeckleSize, maxDiff);

}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_getCropping(
    JNIEnv *env,
    jclass,
    jlong streamHandle,
    jobject origXOutArg,
    jobject origYOutArg,
    jobject widthOurArg,
    jobject heightOurArg)
{
    OniCropping cropping;
    int size = sizeof(cropping);
    int status = oniStreamGetProperty((OniStreamHandle)streamHandle, STREAM_PROPERTY_CROPPING, &cropping, &size);
    SetOutArgIntValue(env, origXOutArg, cropping.originX);
    SetOutArgIntValue(env, origYOutArg, cropping.originY);
    SetOutArgIntValue(env, widthOurArg, cropping.width);
    SetOutArgIntValue(env, heightOurArg, cropping.height);
    return status;
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_setCropping(
    JNIEnv *,
    jclass,
    jlong streamHandle,
    jint originX,
    jint originY,
    jint width,
    jint height)
{
    OniCropping cropping;
    cropping.enabled = true;
    cropping.originX = originX;
    cropping.originY = originY;
    cropping.width = width;
    cropping.height = height;
    return oniStreamSetProperty((OniStreamHandle)streamHandle, STREAM_PROPERTY_CROPPING, &cropping, sizeof(cropping));
}

JNIEXPORT jboolean JNICALL Java_org_openni_NativeMethods_isCroppingSupported(
    JNIEnv *,
    jclass,
    jlong streamHandle)
{
    return (oniStreamIsPropertySupported((OniStreamHandle)streamHandle, STREAM_PROPERTY_CROPPING) == TRUE);
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_resetCropping(JNIEnv *, jclass, jlong streamHandle)
{
    OniCropping cropping;
    cropping.enabled = FALSE;
    return oniStreamSetProperty((OniStreamHandle)streamHandle, STREAM_PROPERTY_CROPPING, &cropping, sizeof(cropping));
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_getVideoMode(
    JNIEnv *env,
    jclass,
    jlong streamHandle,
    jobject videoModeArgOutObj)
{
    jclass videoModeCls = env->FindClass("org/openni/VideoMode");
    jmethodID videoModeCtor = env->GetMethodID(videoModeCls, "<init>", "(IIII)V");
    OniVideoMode videoMode;
    int size = sizeof(OniVideoMode);
    jint status = oniStreamGetProperty((OniStreamHandle)streamHandle, STREAM_PROPERTY_VIDEO_MODE, &videoMode, &size);

    jobject videoModeObj = env->NewObject(videoModeCls, videoModeCtor, videoMode.resolutionX,
        videoMode.resolutionY, videoMode.fps, videoMode.pixelFormat);
    SetOutArgVideoModeValue(env, videoModeArgOutObj, videoModeObj);
    return status;
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_setVideoMode(
    JNIEnv *,
    jclass,
    jlong streamHandle,
    jint resX,
    jint resY,
    jint fps,
    jint pixelFormat)
{
    OniVideoMode videoMode;
    int size = sizeof(OniVideoMode);
    videoMode.resolutionX = resX;
    videoMode.resolutionY = resY;
    videoMode.fps = fps;
    videoMode.pixelFormat = (OniPixelFormat)pixelFormat;
    return oniStreamSetProperty((OniStreamHandle)streamHandle, STREAM_PROPERTY_VIDEO_MODE, &videoMode, size);
}

JNIEXPORT jobject JNICALL Java_org_openni_NativeMethods_oniStreamGetSensorInfo(JNIEnv *env, jclass, jlong streamHandle)
{
    jclass arrayListCls = (*env).FindClass("java/util/ArrayList");
    jobject vectorObj = (*env).NewObject(arrayListCls, (*env).GetMethodID(arrayListCls, "<init>", "()V"));
    jclass videoModeCls = (*env).FindClass("org/openni/VideoMode");
    jmethodID videoModeCtor = env->GetMethodID(videoModeCls, "<init>", "(IIII)V");
    const OniSensorInfo* sensorInfo = oniStreamGetSensorInfo((OniStreamHandle)streamHandle);
    int i = 0;
    while (i < sensorInfo->numSupportedVideoModes)
    {
        OniVideoMode& videoMode = sensorInfo->pSupportedVideoModes[i];
        jobject videoModeObj = env->NewObject(videoModeCls, videoModeCtor, videoMode.resolutionX,
            videoMode.resolutionY, videoMode.fps, (int)videoMode.pixelFormat);

        (*env).CallBooleanMethod(vectorObj, (*env).GetMethodID(arrayListCls, "add", "(Ljava/lang/Object;)Z"), videoModeObj);
        i++;
    }
    jclass sensorInfoCls = (*env).FindClass("org/openni/SensorInfo");
    jobject obj = (*env).NewObject(sensorInfoCls, (*env).GetMethodID(sensorInfoCls, "<init>", "(ILjava/util/List;)V"), sensorInfo->sensorType, vectorObj);
    return obj;
}

JNIEXPORT jboolean JNICALL Java_org_openni_NativeMethods_hasSensor(JNIEnv *, jclass, jlong deviceHandle, jint sensorType)
{
    const OniSensorInfo* pInfo = oniDeviceGetSensorInfo((OniDeviceHandle)deviceHandle, (OniSensorType)sensorType);

    if (pInfo == NULL)
    {
        return false;
    }
    return true;
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniStreamGetIntProperty(
    JNIEnv *env,
    jclass,
    jlong streamHandle,
    jint property,
    jobject argOutObj)
{
    int value = 0;
    int size = sizeof(value);
    int rc = oniStreamGetProperty((OniStreamHandle)streamHandle, property, &value, &size);
    SetOutArgIntValue(env, argOutObj, value);
    return rc;
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniStreamGetBoolProperty(
    JNIEnv *env,
    jclass,
    jlong streamHandle,
    jint property,
    jobject argOutObj)
{
    OniBool value = false;
    int size = sizeof(value);
    int rc = oniStreamGetProperty((OniStreamHandle)streamHandle, property, &value, &size);
    SetOutArgBoolValue(env, argOutObj, value == TRUE);
    return rc;
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniStreamGetFloatProperty(
    JNIEnv *env,
    jclass,
    jlong streamHandle,
    jint property,
    jobject argOutObj)
{
    float value = 0;
    int size = sizeof(value);
    int rc = oniStreamGetProperty((OniStreamHandle)streamHandle, property, &value, &size);
    SetOutArgFloatValue(env, argOutObj, value);
    return rc;
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniStreamSetProperty__JII(
    JNIEnv *,
    jclass,
    jlong streamHandle,
    jint property,
    jint value)
{
    int size = sizeof(value);
    return oniStreamSetProperty((OniStreamHandle)streamHandle, property, &value, size);
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniStreamSetProperty__JIZ(
    JNIEnv *,
    jclass,
    jlong streamHandle,
    jint property,
    jboolean value)
{
    OniBool val = value ? TRUE : FALSE;
    int size = sizeof(val);
    return oniStreamSetProperty((OniStreamHandle)streamHandle, property, &val, size);
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniStreamSetProperty__JIF(
    JNIEnv *,
    jclass,
    jlong streamHandle,
    jint property,
    jfloat value)
{
    int size = sizeof(value);
    return oniStreamSetProperty((OniStreamHandle)streamHandle, property, &value, size);
}

JNIEXPORT jboolean JNICALL Java_org_openni_NativeMethods_oniStreamIsPropertySupported(
    JNIEnv *,
    jclass,
    jlong streamHandle,
    jint property)
{
    return (oniStreamIsPropertySupported((OniStreamHandle)streamHandle, property) == TRUE);
}

JNIEXPORT jobject JNICALL Java_org_openni_NativeMethods_oniDeviceGetSensorInfo(
    JNIEnv *env,
    jclass,
    jlong deviceHandle,
    jint sensorType)
{
    jclass arrayListCls = (*env).FindClass("java/util/ArrayList");
    jobject vectorObj = (*env).NewObject(arrayListCls, (*env).GetMethodID(arrayListCls, "<init>", "()V"));
    jclass videoModeCls = (*env).FindClass("org/openni/VideoMode");
    jmethodID videoModeCtor = env->GetMethodID(videoModeCls, "<init>", "(IIII)V");
    const OniSensorInfo* sensorInfo = oniDeviceGetSensorInfo((OniDeviceHandle)deviceHandle, (OniSensorType)sensorType);
    if (sensorInfo == NULL)
        return NULL;

    int i = 0;
    while (i < sensorInfo->numSupportedVideoModes)
    {
        OniVideoMode& videoMode = sensorInfo->pSupportedVideoModes[i];
        jobject videoModeObj = env->NewObject(videoModeCls, videoModeCtor, videoMode.resolutionX,
            videoMode.resolutionY, videoMode.fps, (int)videoMode.pixelFormat);

        (*env).CallBooleanMethod(vectorObj, (*env).GetMethodID(arrayListCls, "add", "(Ljava/lang/Object;)Z"), videoModeObj);
        i++;
    }
    jclass sensorInfoCls = (*env).FindClass("org/openni/SensorInfo");
    jobject obj = (*env).NewObject(sensorInfoCls, (*env).GetMethodID(sensorInfoCls, "<init>", "(ILjava/util/List;)V"), sensorInfo->sensorType, vectorObj);
    return obj;

}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniDeviceEnableDepthColorSync(JNIEnv *, jclass, jlong deviceHandle)
{
    return oniDeviceEnableDepthColorSync((OniDeviceHandle)deviceHandle);
}

JNIEXPORT void JNICALL Java_org_openni_NativeMethods_oniDeviceDisableDepthColorSync(JNIEnv *, jclass, jlong deviceHandle)
{
    return oniDeviceDisableDepthColorSync((OniDeviceHandle)deviceHandle);
}

JNIEXPORT jboolean JNICALL Java_org_openni_NativeMethods_oniDeviceGetDepthColorSyncEnabled(JNIEnv *, jclass, jlong deviceHandle)
{
    return (jboolean)oniDeviceGetDepthColorSyncEnabled((OniDeviceHandle)deviceHandle);
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniDeviceEnableDepthOptimization(
    JNIEnv *,
    jclass,
    jlong deviceHandle,
    jboolean enable)
{
    OniBool oniValue = (enable == JNI_TRUE) ? TRUE : FALSE;
    return oniDeviceSetProperty((OniDeviceHandle)deviceHandle, XN_MODULE_PROPERTY_DEPTH_OPTIM_STATE, &oniValue, sizeof(oniValue));
}

JNIEXPORT jboolean JNICALL Java_org_openni_NativeMethods_oniDeviceGetDepthOptimizationEnabled(
    JNIEnv *env,
    jclass,
    jlong deviceHandle,
    jobject argOutObj)
{
    OniBool value;
    int size = sizeof(value);
    int rc = oniDeviceGetProperty((OniDeviceHandle)deviceHandle, XN_MODULE_PROPERTY_DEPTH_OPTIM_STATE, &value, &size);
    SetOutArgBoolValue(env, argOutObj, (value == TRUE) ? JNI_TRUE : JNI_FALSE);

    return (STATUS_OK == rc) ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_org_openni_NativeMethods_oniDeviceGetMultiDistanceCalibrationEnabled(
    JNIEnv *env,
    jclass,
    jlong deviceHandle,
    jobject argOutObj)
{
    OniBool value;
    int size = sizeof(value);
    int rc = oniDeviceGetProperty((OniDeviceHandle)deviceHandle, XN_MODULE_PROPERTY_DISTORTION_STATE, &value, &size);
    SetOutArgBoolValue(env, argOutObj, (value == TRUE) ? JNI_TRUE : JNI_FALSE);

    return (STATUS_OK == rc) ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniDeviceEnableMultiDistanceCalibration(
    JNIEnv* /*env*/,
    jclass,
    jlong deviceHandle,
    jboolean enable)
{
    OniBool oniValue = (enable == JNI_TRUE) ? TRUE : FALSE;
    return oniDeviceSetProperty((OniDeviceHandle)deviceHandle, XN_MODULE_PROPERTY_DISTORTION_STATE, &oniValue, sizeof(oniValue));
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniDeviceEnableLaser(
    JNIEnv* /*env*/,
    jclass,
    jlong deviceHandle,
    jboolean enable)
{
    OniBool oniValue = (enable == JNI_TRUE) ? TRUE : FALSE;
    return oniDeviceSetProperty((OniDeviceHandle)deviceHandle, openni::OBEXTENSION_ID_LASER_EN, &oniValue, sizeof(oniValue));
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_seek(
    JNIEnv *,
    jclass,
    jlong deviceHandle,
    jlong streamHandle,
    jint frameIndex)
{
    OniSeek seek;
    seek.frameIndex = frameIndex;
    seek.stream = (OniStreamHandle)streamHandle;
    return oniDeviceInvoke((OniDeviceHandle)deviceHandle, DEVICE_COMMAND_SEEK, &seek, sizeof(seek));
}

JNIEXPORT jboolean JNICALL Java_org_openni_NativeMethods_isImageRegistrationModeSupported(
    JNIEnv *,
    jclass,
    jlong deviceHandle,
    jint mode)
{
    return (oniDeviceIsImageRegistrationModeSupported((OniDeviceHandle)deviceHandle, (OniImageRegistrationMode)mode) == TRUE);
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_getImageRegistrationMode(
    JNIEnv *env,
    jclass,
    jlong deviceHandle,
    jobject argOutObj)
{
    ImageRegistrationMode mode;
    int size = sizeof(mode);
    int rc = oniDeviceGetProperty((OniDeviceHandle)deviceHandle, DEVICE_PROPERTY_IMAGE_REGISTRATION, &mode, &size);
    SetOutArgIntValue(env, argOutObj, mode);
    return rc;
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_setImageRegistrationMode(
    JNIEnv *,
    jclass,
    jlong deviceHandle,
    jint mode)
{
    return oniDeviceSetProperty((OniDeviceHandle)deviceHandle, DEVICE_PROPERTY_IMAGE_REGISTRATION, &mode, sizeof(mode));
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniGetDeviceUSBSpeed(
    JNIEnv *env,
    jclass,
    jlong deviceHandle,
    jobject argOutObj)
{

    int aValue = 0;
    int size = sizeof(aValue);
    int rc = oniDeviceGetProperty((OniDeviceHandle)deviceHandle, XN_MODULE_PROPERTY_USB_SPEED, &aValue, &size);
    SetOutArgIntValue(env, argOutObj, aValue);
    return rc;
}

JNIEXPORT jobject JNICALL Java_org_openni_NativeMethods_oniDeviceGetInfo(JNIEnv *env, jclass, jlong deviceHandle)
{
    OniDeviceInfo deviceInfo;
    oniDeviceGetInfo((OniDeviceHandle)deviceHandle, &deviceInfo);
    jobject nameObj = env->NewStringUTF(deviceInfo.name);
    jobject uriObj = env->NewStringUTF(deviceInfo.uri);
    jobject vendorObj = env->NewStringUTF(deviceInfo.vendor);
    jclass deviceInfoCls = env->FindClass("org/openni/DeviceInfo");
    return (*env).NewObject(deviceInfoCls, (*env).GetMethodID(deviceInfoCls, "<init>", "(Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;II)V"),
        uriObj, vendorObj, nameObj, deviceInfo.usbVendorId, deviceInfo.usbProductId);
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniRecorderStart(JNIEnv *, jclass, jlong recorderHandle)
{
    return oniRecorderStart((OniRecorderHandle)recorderHandle);
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniRecorderDestroy(JNIEnv *, jclass, jlong recorderHandle)
{
    return oniRecorderDestroy((OniRecorderHandle*)&recorderHandle);
}

JNIEXPORT void JNICALL Java_org_openni_NativeMethods_oniRecorderStop(JNIEnv *, jclass, jlong recorderHandle)
{
    oniRecorderStop((OniRecorderHandle)recorderHandle);
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniRecorderAttachStream(
    JNIEnv *,
    jclass,
    jlong recorderHandle,
    jlong streamHadle,
    jboolean allowLossy)
{
    return oniRecorderAttachStream((OniRecorderHandle)recorderHandle, (OniStreamHandle)streamHadle, allowLossy);
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniDeviceGetIntProperty(
    JNIEnv *env,
    jclass,
    jlong deviceHandle,
    jint propery,
    jobject argOutObj)
{
    int value;
    int size = sizeof(value);
    int rc = oniDeviceGetProperty((OniDeviceHandle)deviceHandle, propery, &value, &size);
    SetOutArgIntValue(env, argOutObj, value);
    return rc;
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniDeviceGetBoolProperty(
    JNIEnv *env,
    jclass,
    jlong deviceHandle,
    jint propery,
    jobject argOutObj)
{
    OniBool value;
    int size = sizeof(value);
    int rc = oniDeviceGetProperty((OniDeviceHandle)deviceHandle, propery, &value, &size);
    SetOutArgBoolValue(env, argOutObj, (value == TRUE) ? JNI_TRUE : JNI_FALSE);
    return rc;
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniDeviceGetFloatProperty(
    JNIEnv *env,
    jclass,
    jlong deviceHandle,
    jint propery,
    jobject argOutObj)
{
    float value;
    int size = sizeof(float);
    int rc = oniDeviceGetProperty((OniDeviceHandle)deviceHandle, propery, &value, &size);
    SetOutArgFloatValue(env, argOutObj, value);
    return rc;
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniDeviceGetDoubleProperty(
    JNIEnv *env,
    jclass,
    jlong deviceHandle,
    jint propery,
    jobject argOutObj)
{
    double value;
    int size = sizeof(double);
    int rc = oniDeviceGetProperty((OniDeviceHandle)deviceHandle, propery, &value, &size);
    SetOutArgDoubleValue(env, argOutObj, value);
    return rc;
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniDeviceSetProperty__JII(
    JNIEnv *,
    jclass,
    jlong deviceHandle,
    jint property,
    jint value)
{
    return oniDeviceSetProperty((OniDeviceHandle)deviceHandle, property, &value, sizeof(value));
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniDeviceSetProperty__JIZ(
    JNIEnv *,
    jclass,
    jlong deviceHandle,
    jint property,
    jboolean value)
{
    OniBool oniValue = (value == JNI_TRUE) ? TRUE : FALSE;
    return oniDeviceSetProperty((OniDeviceHandle)deviceHandle, property, &oniValue, sizeof(oniValue));
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniDeviceSetProperty__JIF(
    JNIEnv *,
    jclass,
    jlong deviceHandle,
    jint property,
    float value)
{
    return oniDeviceSetProperty((OniDeviceHandle)deviceHandle, property, &value, sizeof(value));
}

JNIEXPORT jboolean JNICALL Java_org_openni_NativeMethods_oniDeviceIsPropertySupported(
    JNIEnv *,
    jclass,
    jlong deviceHandle,
    jint property)
{
    return (oniDeviceIsPropertySupported((OniDeviceHandle)deviceHandle, property) == TRUE);
}

JNIEXPORT jboolean JNICALL Java_org_openni_NativeMethods_oniDeviceIsCommandSupported(
    JNIEnv *,
    jclass,
    jlong deviceHandle,
    jint command)
{
    return (oniDeviceIsCommandSupported((OniDeviceHandle)deviceHandle, command) == TRUE);
}

static void ONI_CALLBACK_TYPE deviceConnectedCallback(const OniDeviceInfo* pInfo, void*)
{
    JNIEnvSupplier suplier;
    JNIEnv *env = suplier.GetEnv();
    jmethodID methodID = env->GetStaticMethodID(g_openNIClass, "deviceConnected", "(Lorg/openni/DeviceInfo;)V");
    jobject nameObj = env->NewStringUTF(pInfo->name);
    jobject uriObj = env->NewStringUTF(pInfo->uri);
    jobject vendorObj = env->NewStringUTF(pInfo->vendor);
    jobject deviceObj = (*env).NewObject(g_deviceInfoClass, (*env).GetMethodID(g_deviceInfoClass, "<init>", "(Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;II)V"),
        uriObj, vendorObj, nameObj, pInfo->usbVendorId, pInfo->usbProductId);
    env->CallStaticVoidMethod(g_openNIClass, methodID, deviceObj);
}

static void ONI_CALLBACK_TYPE deviceDisconnectedCallback(const OniDeviceInfo* pInfo, void*)
{
    JNIEnvSupplier suplier;
    JNIEnv *env = suplier.GetEnv();
    jmethodID methodID = env->GetStaticMethodID(g_openNIClass, "deviceDisconnected", "(Lorg/openni/DeviceInfo;)V");
    jobject nameObj = env->NewStringUTF(pInfo->name);
    jobject uriObj = env->NewStringUTF(pInfo->uri);
    jobject vendorObj = env->NewStringUTF(pInfo->vendor);
    jobject deviceObj = (*env).NewObject(g_deviceInfoClass, (*env).GetMethodID(g_deviceInfoClass, "<init>", "(Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;II)V"),
        uriObj, vendorObj, nameObj, pInfo->usbVendorId, pInfo->usbProductId);
    env->CallStaticVoidMethod(g_openNIClass, methodID, deviceObj);
}

static void ONI_CALLBACK_TYPE deviceStateChangedCallback(const OniDeviceInfo* pInfo, OniDeviceState state, void*)
{
    JNIEnvSupplier suplier;
    JNIEnv *env = suplier.GetEnv();
    jmethodID methodID = env->GetStaticMethodID(g_openNIClass, "deviceStateChanged", "(Lorg/openni/DeviceInfo;I)V");
    jobject nameObj = env->NewStringUTF(pInfo->name);
    jobject uriObj = env->NewStringUTF(pInfo->uri);
    jobject vendorObj = env->NewStringUTF(pInfo->vendor);
    jobject deviceObj = (*env).NewObject(g_deviceInfoClass, (*env).GetMethodID(g_deviceInfoClass, "<init>", "(Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;II)V"),
        uriObj, vendorObj, nameObj, pInfo->usbVendorId, pInfo->usbProductId);
    env->CallStaticVoidMethod(g_openNIClass, methodID, deviceObj, state);

}

static OniCallbackHandle callbackHandle = 0;
JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniInitialize(JNIEnv *env, jclass)
{
    int status = oniInitialize(ONI_API_VERSION);
    if (status == ONI_STATUS_OK)
    {
        OniDeviceCallbacks callbacks;
        callbacks.deviceConnected = deviceConnectedCallback;
        callbacks.deviceDisconnected = deviceDisconnectedCallback;
        callbacks.deviceStateChanged = deviceStateChangedCallback;

        status = oniRegisterDeviceCallbacks(&callbacks, env, &callbackHandle);
    }
    return status;
}

#ifdef ANDROID
static void ONI_CALLBACK_TYPE OniLogRedirectCallback(const XnLogEntry* pEntry, void*)
{
    if(g_logcatClass == NULL)
    {
        return;
    }

    JNIEnvSupplier suplier;
    JNIEnv *env = suplier.GetEnv();

    XnLogSeverity logSeverity = pEntry->nSeverity;
    switch (logSeverity)
    {
    case XN_LOG_VERBOSE:  //ANDROID_LOG_VERBOSE
    {
        jmethodID logVMethodID = env->GetStaticMethodID(g_logcatClass, "v", "(Ljava/lang/String;Ljava/lang/String;)V");
        if(logVMethodID != NULL)
        {
            jstring jtag = env->NewStringUTF("OpenNI");
            jstring jmsg = env->NewStringUTF(pEntry->strMessage);
            env->CallStaticVoidMethod(g_logcatClass, logVMethodID, jtag, jmsg);
        }
        break;
    }
    case XN_LOG_INFO:     //ANDROID_LOG_INFO
    {
        jmethodID logIMethodID = env->GetStaticMethodID(g_logcatClass, "i", "(Ljava/lang/String;Ljava/lang/String;)V");
        if(logIMethodID != NULL)
        {
            jstring jtag = env->NewStringUTF("OpenNI");
            jstring jmsg = env->NewStringUTF(pEntry->strMessage);
            env->CallStaticVoidMethod(g_logcatClass, logIMethodID, jtag, jmsg);
        }
        break;
    }
    case XN_LOG_WARNING:  //ANDROID_LOG_WARN
    {
        jmethodID logWMethodID = env->GetStaticMethodID(g_logcatClass, "w", "(Ljava/lang/String;Ljava/lang/String;)V");
        if(logWMethodID != NULL)
        {
            jstring jtag = env->NewStringUTF("OpenNI");
            jstring jmsg = env->NewStringUTF(pEntry->strMessage);
            env->CallStaticVoidMethod(g_logcatClass, logWMethodID, jtag, jmsg);
        }

        break;
    }
    case XN_LOG_ERROR:    //ANDROID_LOG_ERROR
    {
        jmethodID logEMethodID = env->GetStaticMethodID(g_logcatClass, "e", "(Ljava/lang/String;Ljava/lang/String;)V");
        if(logEMethodID != NULL)
        {
            jstring jtag = env->NewStringUTF("OpenNI");
            jstring jmsg = env->NewStringUTF(pEntry->strMessage);
            env->CallStaticVoidMethod(g_logcatClass, logEMethodID, jtag, jmsg);
        }

        break;
    }
    default:              //ANDROID_LOG_VERBOSE
    {
        jmethodID logDMethodID = env->GetStaticMethodID(g_logcatClass, "v", "(Ljava/lang/String;Ljava/lang/String;)V");
        if(logDMethodID != NULL)
        {
            jstring jtag = env->NewStringUTF("OpenNI");
            jstring jmsg = env->NewStringUTF(pEntry->strMessage);
            env->CallStaticVoidMethod(g_logcatClass, logDMethodID, jtag, jmsg);
        }
        break;
    }

    }

}
#endif

#ifdef ANDROID
static OniCallbackHandle logRedirectCallbackHandle = 0;
#endif

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniSetLogAndroidOutputRedirect(JNIEnv* env, jclass, jboolean enabled)
{
    (void)(env);
    (void)(enabled);
#ifdef ANDROID
    jint status = ONI_STATUS_OK;
    if(enabled == JNI_TRUE)
    {
        status = oniSetLogAndroidOutputRedirect(TRUE);
        if(status == ONI_STATUS_OK)
        {
            status = oniRegisterAndroidLogRedirectCallback(OniLogRedirectCallback, env,&logRedirectCallbackHandle);
            LOGD("Register Android Log Redirect status %d", status);

        }
    }
    else
    {
        oniUnRegisterAndroidLogRedirectCallback(logRedirectCallbackHandle);
        status = oniSetLogAndroidOutputRedirect(FALSE);
        logRedirectCallbackHandle = 0;
    }

    return status;
#else
    return ONI_STATUS_NOT_SUPPORTED;
#endif
}


JNIEXPORT void JNICALL Java_org_openni_NativeMethods_oniShutdown(JNIEnv *, jclass)
{
    if (callbackHandle != 0)
    {
        oniUnregisterDeviceCallbacks(callbackHandle);
    }

#ifdef ANDROID

    if(logRedirectCallbackHandle != 0)
    {
        oniUnRegisterAndroidLogRedirectCallback(logRedirectCallbackHandle);
        oniSetLogAndroidOutputRedirect(FALSE);
        logRedirectCallbackHandle = 0;
    }

#endif

    return oniShutdown();
}

JNIEXPORT jobject JNICALL Java_org_openni_NativeMethods_oniGetVersion(JNIEnv *env, jclass)
{
    OniVersion version = oniGetVersion();

    jclass versionCls = env->FindClass("org/openni/Version");
    return (*env).NewObject(versionCls, (*env).GetMethodID(versionCls, "<init>", "(IIII)V"),
        version.major, version.minor, version.maintenance, version.build);
}

JNIEXPORT jstring JNICALL Java_org_openni_NativeMethods_oniGetExtendedError(JNIEnv *env, jclass)
{
    return  env->NewStringUTF(oniGetExtendedError());
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniGetDeviceList(JNIEnv *env, jclass, jobject deviceListObj)
{
    OniDeviceInfo* m_pDeviceInfos;
    int m_deviceInfoCount;
    jint status = oniGetDeviceList(&m_pDeviceInfos, &m_deviceInfoCount);
    if (status == 0)
    {
        for (int i = 0; i < m_deviceInfoCount; i++)
        {
            jobject nameObj = env->NewStringUTF(m_pDeviceInfos[i].name);
            jobject uriObj = env->NewStringUTF(m_pDeviceInfos[i].uri);
            jobject vendorObj = env->NewStringUTF(m_pDeviceInfos[i].vendor);
            jclass deviceInfoCls = env->FindClass("org/openni/DeviceInfo");
            jobject deviceInfObj = (*env).NewObject(deviceInfoCls, (*env).GetMethodID(deviceInfoCls, "<init>", "(Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;II)V"),
                uriObj, vendorObj, nameObj, m_pDeviceInfos[i].usbVendorId, m_pDeviceInfos[i].usbProductId);
            jclass vectorCls = (*env).FindClass("java/util/List");
            jmethodID methodId = (*env).GetMethodID(vectorCls, "add", "(Ljava/lang/Object;)Z");
            (*env).CallBooleanMethod(deviceListObj, methodId, deviceInfObj);
        }
    }
    return status;
}

JNIEXPORT jboolean JNICALL Java_org_openni_NativeMethods_oniWaitForAnyStream(
    JNIEnv *env,
    jclass,
    jlongArray streamsArray,
    jobject outArgObj,
    jint timeout)
{
    jlong *streams = env->GetLongArrayElements(streamsArray, JNI_FALSE);
    int size = env->GetArrayLength(streamsArray);
    int id = 0;
    int rc = oniWaitForAnyStream((OniStreamHandle*)streams, size, &id, timeout);
    env->ReleaseLongArrayElements(streamsArray, streams, JNI_ABORT);
    SetOutArgIntValue(env, outArgObj, id);
    return rc == ONI_STATUS_OK;
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniCoordinateConverterWorldToDepth(
    JNIEnv *env,
    jclass,
    jlong streamHandle,
    jfloat worldX,
    jfloat worldY,
    jfloat worldZ,
    jobject depthXOutArg,
    jobject depthYOutArg,
    jobject depthZOutArg)
{
    float x, y, z;
    int status = oniCoordinateConverterWorldToDepth((OniStreamHandle)streamHandle, worldX, worldY, worldZ, &x, &y, &z);
    SetOutArgFloatValue(env, depthXOutArg, x);
    SetOutArgFloatValue(env, depthYOutArg, y);
    SetOutArgFloatValue(env, depthZOutArg, z);
    return status;
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniCoordinateConverterDepthToWorld(
    JNIEnv *env,
    jclass,
    jlong streamHandle,
    jfloat depthX,
    jfloat depthY,
    jfloat depthZ,
    jobject colorXOutArg,
    jobject colorYOutArg,
    jobject colorZOutArg)
{
    float x, y, z;
    int status = oniCoordinateConverterDepthToWorld((OniStreamHandle)streamHandle, depthX, depthY, depthZ, &x, &y, &z);
    SetOutArgFloatValue(env, colorXOutArg, x);
    SetOutArgFloatValue(env, colorYOutArg, y);
    SetOutArgFloatValue(env, colorZOutArg, z);
    return status;
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniCoordinateConverterDepthToColor(
    JNIEnv *env,
    jclass,
    jlong depthHandle,
    jlong colorHandle,
    jint depthX,
    jint depthY,
    jshort depthZ,
    jobject colorXOutArg,
    jobject colorYOutArg)
{
    int x, y;
    int status = oniCoordinateConverterDepthToColor((OniStreamHandle)depthHandle, (OniStreamHandle)colorHandle, depthX, depthY, depthZ, &x, &y);
    SetOutArgIntValue(env, colorXOutArg, x);
    SetOutArgIntValue(env, colorYOutArg, y);
    return status;
}


JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniCoordinateConverterD2C(
    JNIEnv *env,
    jclass,
    jlong depthHandle,
    jint depthX,
    jint depthY,
    jshort depthZ,
    jobject colorXOutArg,
    jobject colorYOutArg)
{
    int x, y;
    int status = oniCoordinateConverterD2C((OniStreamHandle)depthHandle, depthX, depthY, depthZ, &x, &y);
    SetOutArgIntValue(env, colorXOutArg, x);
    SetOutArgIntValue(env, colorYOutArg, y);
    return status;
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniCoordinateConverterC2D(
    JNIEnv *env,
    jclass,
    jlong depthHandle,
    jint colorX,
    jint colorY,
    jshort depthZ,
    jobject depthXOutArg,
    jobject depthYOutArg)
{
    int x, y;
    int status = oniCoordinateConverterC2D((OniStreamHandle)depthHandle, colorX, colorY, depthZ, &x, &y);
    SetOutArgIntValue(env, depthXOutArg, x);
    SetOutArgIntValue(env, depthYOutArg, y);
    return status;
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniCreateRecorder(
    JNIEnv *env,
    jclass,
    jstring filename,
    jobject recorder)
{
    OniRecorderHandle handle;
    jclass recorderCls = env->FindClass("org/openni/Recorder");
    const char * str = env->GetStringUTFChars(filename, JNI_FALSE);
    int status = oniCreateRecorder(str, &handle);
    jfieldID fieldID = env->GetFieldID(recorderCls, "mRecorderHandle", "J");
    env->SetLongField(recorder, fieldID, (long)handle);
    return status;
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniDeviceOpen__Ljava_lang_String_2Lorg_openni_Device_2(
    JNIEnv *env,
    jclass,
    jstring uriStrObj,
    jobject device)
{
    OniDeviceHandle handle;
    const char * str = env->GetStringUTFChars(uriStrObj, JNI_FALSE);
    int status = oniDeviceOpen(str, &handle);
    jclass deviceCls = env->FindClass("org/openni/Device");
    jfieldID fieldID = env->GetFieldID(deviceCls, "mDeviceHandle", "J");
    env->SetLongField(device, fieldID, (long)handle);
    return status;
}
static const char* ANY_DEVICE = NULL;
JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniDeviceOpen__Lorg_openni_Device_2(JNIEnv *env, jclass, jobject device)
{
    OniDeviceHandle handle;
    int status = oniDeviceOpen(ANY_DEVICE, &handle);
    jclass deviceCls = env->FindClass("org/openni/Device");
    jfieldID fieldID = env->GetFieldID(deviceCls, "mDeviceHandle", "J");
    env->SetLongField(device, fieldID, (long)handle);
    return status;
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniDeviecSwitchIRCamera(
    JNIEnv* /*env*/,
    jclass,
    jlong deviceHandle,
    jint irType)
{

    return oniDeviceSetProperty((OniDeviceHandle)deviceHandle, XN_MODULE_PROPERTY_SWITCH_IR, &irType, sizeof(irType));
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniDeviceGetCameraParams(
    JNIEnv *env,
    jclass,
    jlong deviceHandle,
    jobject obj)
{
    OBCameraParams cameraParams;
    int size = sizeof(cameraParams);
    memset(&cameraParams, 0, size);
    int rc = oniDeviceGetProperty((OniDeviceHandle)deviceHandle, openni::OBEXTENSION_ID_CAM_PARAMS, (uint8_t *)&cameraParams, &size);
    if (rc == STATUS_OK){
        LOGD("oniDeviceGetCameraParams-oniDeviceGetProperty--result--STATUS_OK");
        jclass camPams = env->GetObjectClass(obj);
        if (camPams == NULL){
            LOGE("Java OBCameraParams get fail!");
            return STATUS_ERROR;
        }

        LOGD("GetCameraParams-setD_intr_p=%f,%f,%f,%f", cameraParams.l_intr_p[0], cameraParams.l_intr_p[1], cameraParams.l_intr_p[2], cameraParams.l_intr_p[3]);
        LOGD("CameraParams-setC_intr_p=%f,%f,%f,%f", cameraParams.r_intr_p[0], cameraParams.r_intr_p[1], cameraParams.r_intr_p[2], cameraParams.r_intr_p[3]);

        setOutArgFloatArrayValue(env, camPams, obj, "setD_intr_p", cameraParams.l_intr_p, 4);
        setOutArgFloatArrayValue(env, camPams, obj, "setC_intr_p", cameraParams.r_intr_p, 4);
        setOutArgFloatArrayValue(env, camPams, obj, "setD2c_r", cameraParams.r2l_r, 9);
        setOutArgFloatArrayValue(env, camPams, obj, "setD2c_t", cameraParams.r2l_t, 3);
        setOutArgFloatArrayValue(env, camPams, obj, "setD_k", cameraParams.l_k, 5);
        setOutArgFloatArrayValue(env, camPams, obj, "setC_k", cameraParams.r_k, 5);

        jfieldID fieldID = env->GetFieldID(camPams, "width", "I");
        env->SetIntField(obj, fieldID, (jint)cameraParams.width);

        fieldID = env->GetFieldID(camPams, "height", "I");
        env->SetIntField(obj, fieldID, (jint)cameraParams.height);

        env->DeleteLocalRef(camPams);
    }
    else{
        rc = STATUS_ERROR;
        LOGE("Get device camera params failed!");
    }

    return rc;
}

/*
 * Class:     org_openni_NativeMethods
 * Method:    oniDeviceGetSerialNumber
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniDeviceGetSerialNumber(
    JNIEnv *env,
    jclass,
    jlong deviceHandle,
    jint propery,
    jobject argOutObj)
{
    char value[16] = { 0 };
    int size = sizeof(value);
    int rc = oniDeviceGetProperty((OniDeviceHandle)deviceHandle, propery, &value, &size);
    SetOutArgStringValue(env, argOutObj, value);
    return rc;
}

/*
 * Class:     org_openni_NativeMethods
 * Method:    Get firmware version
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniDeviceGetStringProperty(
    JNIEnv *env,
    jclass,
    jlong deviceHandle,
    jint propery,
    jobject argOutObj)
{
    char value[200] = { 0 };
    int size = sizeof(value);
    int rc = oniDeviceGetProperty((OniDeviceHandle)deviceHandle, propery, &value, &size);
    SetOutArgStringValue(env, argOutObj, value);
    return rc;
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniDeviceClose(JNIEnv *, jclass, jlong deviceHandle)
{
    return oniDeviceClose((OniDeviceHandle)deviceHandle);
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniSetLogOutputFolder(JNIEnv * env, jclass, jstring path)
{
    const char * str = env->GetStringUTFChars(path, JNI_FALSE);
    return oniSetLogOutputFolder(str);
}

JNIEXPORT jstring JNICALL Java_org_openni_NativeMethods_oniGetLogFileName(JNIEnv *env, jclass)
{
    XnChar fileName[XN_FILE_MAX_PATH];
    oniGetLogFileName(fileName, sizeof(fileName));
    return env->NewStringUTF(fileName);
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniSetLogMinSeverity(JNIEnv *, jclass, jint minSeverity)
{
    return oniSetLogMinSeverity(minSeverity);
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniSetLogConsoleOutput(JNIEnv *, jclass, jboolean enabled)
{
    return oniSetLogConsoleOutput(enabled);
}

JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniSetLogFileOutput(JNIEnv *, jclass, jboolean enabled)
{
    return oniSetLogFileOutput(enabled);
}


JNIEXPORT jint JNICALL Java_org_openni_NativeMethods_oniSetLogAndroidOutput(JNIEnv *, jclass, jboolean enabled)
{
    (void)(enabled);
#ifdef ANDROID
    return oniSetLogAndroidOutput(enabled);
#else
    return ONI_STATUS_NOT_SUPPORTED;
#endif
}

#ifdef FREE_SELINUX
JNIEXPORT void JNICALL Java_org_openni_NativeMethods_oniSetUsbParam
(JNIEnv * env, jclass,jint fd, jstring usbpath)
{
    const char * path = env->GetStringUTFChars(usbpath, JNI_FALSE);
    oniSetUsbParam(fd,path);
}
#endif
