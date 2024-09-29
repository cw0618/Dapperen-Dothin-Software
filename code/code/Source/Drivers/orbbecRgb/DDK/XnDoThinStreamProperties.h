#pragma once
#ifndef XN_DOTHIN_STREAM_PROPERTIES
#define XN_DOTHIN_STREAM_PROPERTIES

#include "XnLog.h"
#include "XnOS.h"
#include "OniCProperties.h"
#include "OniCTypes.h"
#include "OniCEnums.h"
#include "XnDothinConnectHelper.h"
#include "XnMx6xModulesHepler.h"
#include "XnMx6xModules.h"
#include "DriverImpl/Sensor/XnTofSensor.h"
/*Default Raw Data Input Format 12 bit*/
#define  DEFAULT_INPUTFORMAT 12


class XnDothinStreamProperties
{
public:

	XnDothinStreamProperties(XnChar* strType, OniSensorType sensorType);

	virtual ~XnDothinStreamProperties();

	virtual XnStatus Init(XnMx6xModulesHelper* m_pMudulesHelper, XnDothinConnectHelper* dtConnectHelper);

    virtual OniSensorType GetOniSensorType(){ return m_sensorType; };

    virtual XnChar* GetSensorTypeName(){ return (XnChar*)m_strType; };

    virtual OniPixelFormat GetOutPutFormat(){ return m_outPixelFormat; };

    virtual XnUInt32 GetXRes(){ return XRes; };

    virtual XnUInt32 GetYRes(){ return YRes; };

    virtual XnInt32 GetStride(){ return stride; };

    virtual XnUInt32 GetBytesPerPixel(){ return m_bytesPerPixel; };

    virtual XnUInt32 GetRequiredFrameSize(){ return m_requiredFrameSize; };

    virtual XnUInt32 GetGrabBufferSize(){ return m_grabBufferSize; };

    virtual XnUInt32 GetSensorInputFormat(){ return m_dothinMipiPackBit; };

    virtual XnTofSensor* GetSensor(){ return m_pSensor; };

    virtual void SetSensorInputFormat(const XnUInt32 inputFormat);

    virtual XnStatus SetDefaultVideoMode(OniVideoMode* pVideoMode);

    virtual XnStatus ChangeVideoMode(OniVideoMode* pVideoMode);

    virtual XnStatus GetVideoMode(OniVideoMode* pVideoMode);

    virtual XnStatus StreamMode(obc_stream_type streamType);

    /* update sensor mode and property when start stream */
    virtual XnStatus UpdateMode();

protected:

    virtual XnStatus OnOutputFormatChanged(OniPixelFormat pixelFormat);

    /*mipi package size*/
    virtual void CalcGrabBufferSize(XnUInt32 XRes, XnUInt32 YRes, XnUInt32 InputFormat);

    /*image size*/
    virtual void CalcRequiredSize(XnUInt32 XRes, XnUInt32 YRes, XnUInt32 nBytesPerPixel);

    virtual void CalcStride(XnUInt32 XRes, XnUInt32 nBytesPerPixel);

    /* Get the data bit width supported by Dothin HW sensor, how many bits a pixel.*/
    virtual XnStatus GetSensorInputFormatFromDot();

    /*FPS changed*/
    virtual XnStatus SetFPS(XnInt* fps);

protected:
	const XnChar* XN_MASK_DOTHIN_STREAM_PPS = "DothinStreamProperties";
    XnMx6xModulesHelper *m_pMudulesHelper;
    XnDothinConnectHelper *m_pConnectHelper;

    /*For phase processor and depth calc, function to parse tof data*/
    XnTofSensor *m_pSensor;

    OniVideoMode m_curVideoMode;

    OniVideoMode m_defaultVideoMode;

    /*Sensor type*/
    OniSensorType m_sensorType;

    /*Sensor name */
    const XnChar* m_strType;

    /*Output data stream format after unpacking*/
    OniPixelFormat m_outPixelFormat = ONI_PIXEL_FORMAT_DEPTH_1_MM;

    /*Stream Data Width */
    XnUInt32 XRes;

    /*Stream Data Height */
    XnUInt32 YRes;

    /*stride*/
    XnInt32 stride;

    /*How many bits per pixel*/
    XnUInt32 m_bytesPerPixel;

    /*Unpack a frame of bit data to 16 bits requires cache size*/
    XnUInt32 m_requiredFrameSize;

    /*Request Dothin to return a frame size*/
    XnUInt32 m_grabBufferSize;

    /*Sensor input data bit width: 10 bit 11bit 12bit*/
    XnUInt32 m_dothinMipiPackBit = DEFAULT_INPUTFORMAT;

};



#endif //XN_DOTHIN_STREAM_PROPERTIES


