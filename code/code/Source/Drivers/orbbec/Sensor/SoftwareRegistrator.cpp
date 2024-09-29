

#include <stdint.h>
#include <iostream>
#include <string.h>
#include <XnLog.h>
#include <XnMatrix3x3.h>
#include "SoftwareRegistrator.h"

SoftwareRegistrator::SoftwareRegistrator()
{
}

SoftwareRegistrator::~SoftwareRegistrator()
{

}

void SoftwareRegistrator::Init(OBCameraParams& params){
 memcpy(&m_params, &params, sizeof(params));
 memcpy(&m_depthIntrinsic, &params, sizeof(m_depthIntrinsic));
 memcpy(&m_colorIntrinsic, &(m_params.r_intr_p), sizeof(m_colorIntrinsic));
 memcpy(&m_r2lRotate, &(params.r2l_r), sizeof(m_r2lRotate));
 memcpy(&m_r2lTrans, &(params.r2l_t), sizeof(m_r2lTrans));
 memcpy(&m_l_k, &(params.l_k), sizeof(m_l_k));
 memcpy(&m_r_k, &(params.r_k), sizeof(m_r_k));
#if 1
 xnLogVerbose("SoftwareRegistrator","=================================================");
 xnLogVerbose("SoftwareRegistrator","depth [%f,%f,%f,%f], \ncolor[%f,%f,%f,%f]\n r2lrotate[%f,%f,%f, %f,%f,%f, %f,%f,%f]\n    \
		       r2lTrans[%f,%f,%f], l_k[%f,%f,%f,%f,%f], r_k[%f,%f,%f,%f,%f]",
		     m_depthIntrinsic.fx, m_depthIntrinsic.fy,m_depthIntrinsic.cx,m_depthIntrinsic.cy,
		     m_colorIntrinsic.fx, m_colorIntrinsic.fy, m_colorIntrinsic.cx, m_colorIntrinsic.cy,
			 m_r2lRotate.r00,m_r2lRotate.r01,m_r2lRotate.r02,
			 m_r2lRotate.r10,m_r2lRotate.r11,m_r2lRotate.r12,
			 m_r2lRotate.r20,m_r2lRotate.r21,m_r2lRotate.r22,
			 m_r2lTrans.t0,m_r2lTrans.t1,m_r2lTrans.t2,
			 m_l_k.k0, m_l_k.k1, m_l_k.k2, m_l_k.k3, m_l_k.k4,
			 m_r_k.k0, m_r_k.k1, m_r_k.k2, m_r_k.k3, m_r_k.k4
		 );
#endif

 xnl::Matrix3x3  matrix( 
		     m_r2lRotate.r00,m_r2lRotate.r01,m_r2lRotate.r02,
             m_r2lRotate.r10,m_r2lRotate.r11,m_r2lRotate.r12,
             m_r2lRotate.r20,m_r2lRotate.r21,m_r2lRotate.r22
		  );
 xnl::Matrix3x3  RMat = matrix.Inverse();
 xnl::Matrix3x3  RMat2 = RMat*(-1.0f);
 xnl::Vector3D vector3d(m_params.r2l_t[0], m_params.r2l_t[1], m_params.r2l_t[2]);
 xnl::Vector3D Tmat = RMat2*vector3d;
 m_c2dTrans.t0 = Tmat[0];
 m_c2dTrans.t1 = Tmat[1];
 m_c2dTrans.t2 = Tmat[2];

 memcpy(&m_InverseRotate, RMat.GetData(), sizeof(m_InverseRotate));
#if 0
 xnLogVerbose("SoftwareRegistrator", "Rmat -> -'Rmat [%f, %f, %f, %f, %f, %f, %f, %f, %f] -> [%f, %f, %f, %f, %f, %f, %f, %f, %f]",
		 m_r2lRotate.r00,m_r2lRotate.r01,m_r2lRotate.r02,
		 m_r2lRotate.r10,m_r2lRotate.r11,m_r2lRotate.r12,
		 m_r2lRotate.r20,m_r2lRotate.r21,m_r2lRotate.r22,
		 m_InverseRotate.r00,m_InverseRotate.r01,m_InverseRotate.r02,
		 m_InverseRotate.r10,m_InverseRotate.r11,m_InverseRotate.r12,
		 m_InverseRotate.r20,m_InverseRotate.r21,m_InverseRotate.r22
		 );

 xnLogVerbose("SoftwareRegistrator", "src t[%f,%f,%f], -> dst[%f, %f, %f]",m_params.r2l_t[0],m_params.r2l_t[1],m_params.r2l_t[2], m_c2dTrans.t0, m_c2dTrans.t1, m_c2dTrans.t2);
#endif
}

void SoftwareRegistrator::ConvertProjectiveToWorld(int u, int v, int z, Point3D& world, Intrinsic* pintrinsic, bool use_distort_coef)
{
	// x = (u * z - z * Cx) / fx
	// y = (v * z - z * Cy) / fy

	//x = (u * z - z * cxl) / fxl;
	//y = (v * z - z * cyl) / fyl;
	float ifx, ify;

#if 1
	ifx = 1. / pintrinsic->fx;
	ify = 1. / pintrinsic->fy;

	float tx = (u - pintrinsic->cx) * ifx;
	float ty = (v - pintrinsic->cy) * ify;
#else
	ifx = 1. / m_params.l_intr_p[0];
	ify = 1. / m_params.l_intr_p[1];

	float tx = (u - m_params.l_intr_p[2]) * ifx;
	float ty = (v - m_params.l_intr_p[3]) * ify;
#endif
	float x0 = tx;
	float y0 = ty;

	if (use_distort_coef)
	{
		for (int j = 0; j < 5; j++)
		{
			double r2 = tx * tx + ty * ty;
			double icdist = (1) / (1 + ((m_l_k.k4 * r2 + m_l_k.k1)*r2 + m_l_k.k0)*r2);
			double deltaX = 2 * m_l_k.k2 * tx*ty + m_l_k.k3 * (r2 + 2 * tx*tx);
			double deltaY = m_l_k.k2 * (r2 + 2 * ty*ty) + 2 * m_l_k.k3 * tx*ty;
			tx = (x0 - deltaX)*icdist;
			ty = (y0 - deltaY)*icdist;
		}
	}
	
	world.u = z * tx;
    world.v = z * ty;
	world.z = z;
}

void SoftwareRegistrator::ConvertWorldToProjective(Point3D &world, Point3D& projectPoint, Intrinsic* pintrinsic, bool use_distort_coef)
{
	float tx = world.u / world.z;
	float ty = world.v / world.z;
	if (use_distort_coef)
	{
		float r2 = tx*tx + ty*ty;
		float f = 1 + m_r_k.k0 * r2 + m_r_k.k1 * r2*r2 + m_r_k.k4 * r2*r2*r2;
		tx *= f;
		ty *= f;
		float dx = tx + 2 * m_r_k.k2 * tx*ty + m_r_k.k3 * (r2 + 2 * tx*tx);
		float dy = ty + 2 * m_r_k.k3 * tx*ty + m_r_k.k2 * (r2 + 2 * ty*ty);
		tx = dx;
		ty = dy;
	}
#if 1
	projectPoint.u = tx * pintrinsic->fx + pintrinsic->cx;
	projectPoint.v = ty * pintrinsic->fy + pintrinsic->cy;
#else
	projectPoint.u = tx * m_params.r_intr_p[0] + m_params.r_intr_p[2];
	projectPoint.v = ty * m_params.r_intr_p[1] + m_params.r_intr_p[3];
#endif
}

void SoftwareRegistrator::TransformPointToPoint(Point3D& dst, const Point3D& src, const RotateMatrix* pRotateMatrix, const Translate* pTranslate)
{
#if 1
		dst.u = pRotateMatrix->r00 * src.u + pRotateMatrix->r01 * src.v + pRotateMatrix->r02 * src.z + pTranslate->t0;
		dst.v = pRotateMatrix->r10 * src.u + pRotateMatrix->r11 * src.v + pRotateMatrix->r12 * src.z + pTranslate->t1;
		dst.z = pRotateMatrix->r20 * src.u + pRotateMatrix->r21 * src.v + pRotateMatrix->r22 * src.z + pTranslate->t2;
#else
		dst.u = m_params.r2l_r[0] * src.u + m_params.r2l_r[1] * src.v + m_params.r2l_r[2] * src.z + m_params.r2l_t[0];
		dst.v = m_params.r2l_r[3] * src.u + m_params.r2l_r[4] * src.v + m_params.r2l_r[5] * src.z + m_params.r2l_t[1];
		dst.z = m_params.r2l_r[6] * src.u + m_params.r2l_r[7] * src.v + m_params.r2l_r[8] * src.z + m_params.r2l_t[2];
#endif
}

void SoftwareRegistrator::MappingDepth2Color(OniFrame& frame, int u, int v, OniDepthPixel z, bool use_distort_coef)
{
	Point3D pixel3d;
	//float point[3]={0.0f,0.0f,0.0f}, to_point[3] = {0.0f,0.0f,0.0f};
	Point3D point;
	Point3D to_point;
	memset(&point, 0, sizeof(Point3D));
	memset(&to_point, 0, sizeof(Point3D));

    short u_rgb = 0, v_rgb = 0;

	const int width = frame.width;
#if (XN_PLATFORM == XN_PLATFORM_WIN32)
	OniDepthPixel* pDst = (OniDepthPixel *)frame.data;
#else
	typedef OniDepthPixel (*pFrameType)[width];
	pFrameType pDst = (pFrameType)frame.data;
#endif
	ConvertProjectiveToWorld(u, v, z, point, &m_depthIntrinsic, use_distort_coef);

	TransformPointToPoint(to_point, point, &m_r2lRotate, &m_r2lTrans);

	ConvertWorldToProjective(to_point, pixel3d, &m_colorIntrinsic, use_distort_coef);

	u_rgb = (uint16_t)(pixel3d.u);
	v_rgb = (uint16_t)(pixel3d.v);

	if (u_rgb < 0 || u_rgb >= frame.width || v_rgb < 0 || v_rgb >= frame.height) return;

#if (XN_PLATFORM == XN_PLATFORM_WIN32)
	pDst[v_rgb*width + u_rgb] = (OniDepthPixel)z;
#else
	pDst[v_rgb][u_rgb] = (OniDepthPixel)z;
#endif
}

void SoftwareRegistrator::CoordinateConverterColorToDepth(int u, int v, OniDepthPixel z, int &dx, int &dy, bool use_distort_coef)
{
	Point3D pixel3d;
	//float point[3]={0.0f,0.0f,0.0f}, to_point[3] = {0.0f,0.0f,0.0f};
	Point3D point;
	Point3D to_point;
	memset(&point, 0, sizeof(Point3D));
	memset(&to_point, 0, sizeof(Point3D));

	ConvertProjectiveToWorld(u, v, z, point, &m_colorIntrinsic, use_distort_coef);

	TransformPointToPoint(to_point, point, &m_InverseRotate, &m_c2dTrans);
	
	ConvertWorldToProjective(to_point, pixel3d, &m_depthIntrinsic, use_distort_coef);

	dx = (uint16_t)(pixel3d.u);
	dy = (uint16_t)(pixel3d.v);
	printf(" C 2 D [%d, %d, %d ]======[%d, %d] ==>\n", u, v, z, dx, dy);
}

void SoftwareRegistrator::CoordinateConverterDepthToColor(int u, int v, OniDepthPixel z, int &cx, int &cy, bool use_distort_coef)
{
	Point3D pixel3d;
	//float point[3]={0.0f,0.0f,0.0f}, to_point[3] = {0.0f,0.0f,0.0f};
	Point3D point;
	Point3D to_point;
	memset(&point, 0, sizeof(Point3D));
	memset(&to_point, 0, sizeof(Point3D));

    short u_rgb = 0, v_rgb = 0;

	ConvertProjectiveToWorld(u, v, z, point, &m_depthIntrinsic, use_distort_coef);

	TransformPointToPoint(to_point, point, &m_r2lRotate, &m_r2lTrans);

	ConvertWorldToProjective(to_point, pixel3d, &m_colorIntrinsic, use_distort_coef);

	cx = (uint16_t)(pixel3d.u);
	cy = (uint16_t)(pixel3d.v);
	printf(" D 2 C [%d, %d, %d ]======[%d, %d] ==>\n", u, v, z, cx, cy);
}

bool SoftwareRegistrator::IsNaN(float& dat){
	int& ref=*(int *)&dat;
	return (ref&0x7F800000) == 0x7F800000 && (ref&0x7FFFFF)!=0;
}

