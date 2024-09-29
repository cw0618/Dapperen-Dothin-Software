#pragma once
#ifndef XN_DOTHIN_PHASE_STREAM_PROPERTIES
#define XN_DOTHIN_PHASE_STREAM_PROPERTIES

#include "XnDothinStreamProperties.h"
#include "DriverImpl/Sensor/XnTofSensor.h"
#include "DriverImpl/Sensor/XnPlecoTofSensor.h"
#include "DriverImpl/Sensor/XnMFTofSensor.h"
#include "DriverImpl/Sensor/XnGaeaSensor.h"

#define XN_MASK_DOTHIN_PHASE_STREAM_PPS "DothinPhaseStreamProperties"



class XnDothinPhaseStreamProperties : public  XnDothinStreamProperties
{
public:

    XnDothinPhaseStreamProperties(XnChar* strType, OniSensorType sensorType, XnUInt32 sensorId);

    virtual ~XnDothinPhaseStreamProperties();

public:

	virtual XnStatus Init(XnMx6xModulesHelper* m_pMudulesHelper, XnDothinConnectHelper* dtConnectHelper);

    virtual OniSensorType GetOniSensorType(){ return m_sensorType; };

    virtual XnChar* GetSensorTypeName(){ return (XnChar*)m_strType; };

    virtual OniPixelFormat GetOutPutFormat(){ return m_outPixelFormat; };

    virtual XnUInt32 GetXRes(){ return XRes; };

    virtual XnUInt32 GetYRes(){ return YRes; };

    virtual XnInt32 GetStride(){ return stride; };

    virtual XnUInt32 GetBytesPerPixel(){ return m_bytesPerPixel; };

    virtual XnUInt32 getRequiredFrameSize(){ return m_requiredFrameSize; };

    virtual XnUInt32 GetGrabBufferSize(){ return m_grabBufferSize; };

    virtual XnUInt32 GetSensorInputFormat(){ return m_dothinMipiPackBit; };

    virtual void SetSensorInputFormat(const XnUInt32 inputFormat);

    virtual XnStatus SetDefaultVideoMode(OniVideoMode* pVideoMode);

    virtual XnStatus ChangeVideoMode(OniVideoMode* pVideoMode);

    virtual XnStatus GetVideoMode(OniVideoMode* pVideoMode);

    virtual XnStatus StreamMode(obc_stream_type streamType);

    virtual XnStatus UpdateMode();

protected:

    /*mipi package size*/
    virtual void CalcGrabBufferSize(XnUInt32 XRes, XnUInt32 YRes, XnUInt32 InputFormat);

    /*image size*/
    virtual void CalcRequiredSize(XnUInt32 XRes, XnUInt32 YRes, XnUInt32 nBytesPerPixel);

    virtual void CalcStride(XnUInt32 XRes, XnUInt32 nBytesPerPixel);

    /*FPS changed*/
    virtual XnStatus SetFPS(XnInt* fps);

private:

    XnUInt32 m_sensorId;

};



#endif //XN_DOTHIN_PHASE_STREAM_PROPERTIES


