#ifndef _SOFTWARE_REGIST_
#define _SOFTWARE_REGIST_

#include <OBExtensionCommand.h>
#include <OniCTypes.h>
#include <XnVector3D.h>

#include "XnDeviceSensor.h"
#define LOG_TAG "OpenNIEx-Jni"
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG,LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR,LOG_TAG, __VA_ARGS__)

//#include "AstraIO.h"

#define XN_PARAMS_REGISTRATION_OFF              0
#define XN_PARAMS_REGISTRATION_DEPTH_TO_COLOR   1
#define XN_PARAMS_REGISTRATION_USE_DISTORTION   2


#define MAT_NUM          16

#define SXGA 1280
#define VGA   640
#define QVGA  320

typedef struct Point3D{
	float u;
	float v;
	float z;
}Point3D;

class SoftwareRegistrator
{
public:
    SoftwareRegistrator();
    ~SoftwareRegistrator();


	void Init(OBCameraParams& params);

	//void CalcConversionMatrix();
	// Convert depth from projective to world, in IR coordinate
	void ConvertProjectiveToWorld(int u, int v, int z, Point3D &world, Intrinsic* pintrinsic, bool use_distort_coef);
	
	// Convert depth from world to projective, in RGB coordinate
	void ConvertWorldToProjective(Point3D& world, Point3D& projectPoint, Intrinsic* pintrinsic, bool use_distort_coef);
	
	// Transform depth point cloud from IR coordinate to RGB coordinate
	void TransformPointToPoint(Point3D& dst, const Point3D& src, const RotateMatrix* pRotateMatrix, const Translate* pTranslate); 

	void MappingDepth2Color(OniFrame& frame, int u, int v, OniDepthPixel z, bool use_distort_coef);

	void CoordinateConverterColorToDepth(int u, int v, OniDepthPixel z, int &dx, int &dy, bool use_distort_coef);
	void CoordinateConverterDepthToColor(int u, int v, OniDepthPixel z, int &dx, int &dy, bool use_distort_coef);
    bool IsNaN(float& dat);

private:
	double  c2d[MAT_NUM]; // Color to depth conversion matrix
	double  d2c[MAT_NUM]; // Depth to color conversion matrix

    OBCameraParams m_params;

	Intrinsic m_depthIntrinsic;
	Intrinsic m_colorIntrinsic;

	RotateMatrix m_r2lRotate;
    Translate    m_r2lTrans;

	RotateMatrix m_InverseRotate;
    Translate    m_c2dTrans;

	Distortion m_l_k;
	Distortion m_r_k;
};
#endif
