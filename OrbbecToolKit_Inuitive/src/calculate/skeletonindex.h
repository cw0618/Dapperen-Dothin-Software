#pragma once
#include <QOpenGLBuffer>
#include <QMap>
#include <QDebug>
class SkeletonIndex
{
public:
	SkeletonIndex();
	~SkeletonIndex();
	const int COORDS_PER_VERTEX = 3;  // xyz
	int getLastPoint(int idx);
public:
	QOpenGLBuffer mIndexTrunk;        // Çû¸É 1-2-15
	QOpenGLBuffer mIndexShoulder;     // ¼ç°ò 3-2-6
	QOpenGLBuffer mIndexLeftArm;      // ×ó±Û 3-4-5
	QOpenGLBuffer mIndexRightArm;     // ÓÒ±Û 6-7-8
	QOpenGLBuffer mIndexThighbone;    // ¿è¹Ç 9-15-12
	QOpenGLBuffer mIndexLeftLeg;      // ×óÍÈ 9-10-11
	QOpenGLBuffer mIndexRightLeg;     // ÓÒÍÈ 12-13-14
	QOpenGLBuffer mIndexLeftS2T;      // ×ó¼çºÍ×ó¿è¹Ç 3-9
	QOpenGLBuffer mIndexRightS2T;     // ÓÒ¼çºÍÓÒ¿è¹Ç 4-10
	QOpenGLBuffer mIndexT2T;          // ÓÒ¼çºÍÓÒ¿è¹Ç 4-10
	QOpenGLBuffer mIndexOutLine;      // Íâ¿ò5¸öµã
	GLfloat ARRAY_TRUNK[3] = { 0, 1, 2 };           // Çû¸É
	GLfloat ARRAY_SHOULDER[3] = { 3, 1, 4 };        // ¼ç°ò
	GLfloat ARRAY_LEFT_ARM[3] = { 3, 5, 7 };        // ×ó±Û
	GLfloat ARRAY_RIGHT_ARM[3] = { 4, 6, 8 };       // ÓÒ±Û
	GLfloat ARRAY_THIGHBONE[3] = { 9, 2, 10 };      // ¿è¹Ç
	GLfloat ARRAY_LEFT_LEG[3] = { 9, 11, 13 };      // ×óÍÈ
	GLfloat ARRAY_RIGHT_LEG[3] = { 10, 12, 14 };    // ÓÒÍÈ
	GLfloat ARRAY_LEFT_S2T[3] = { 3, 9 };           // ×ó¼çºÍ×ó¿è¹Ç
	GLfloat ARRAY_RIGHT_S2T[3] = { 4, 10 };         // ÓÒ¼çºÍÓÒ¿è¹Ç
	GLfloat ARRAY_T2T[2] = { 9, 10 };               // ÓÒ¿è¹ÇºÍÓÒ¿è¹Ç
	GLfloat ARRAY_OUTLINE[5] = { 15, 16, 17, 18, 15 };  // Íâ¿ò
public:
	QOpenGLBuffer getIndexTrunk() {
		return mIndexTrunk;
	}

	QOpenGLBuffer getIndexShoulder() {
		return mIndexShoulder;
	}

	QOpenGLBuffer getIndexLeftArm() {
		return mIndexLeftArm;
	}

	QOpenGLBuffer getIndexRightArm() {
		return mIndexRightArm;
	}

	QOpenGLBuffer getIndexThighbone() {
		return mIndexThighbone;
	}

	QOpenGLBuffer getIndexLeftLeg() {
		return mIndexLeftLeg;
	}

	QOpenGLBuffer getIndexRightLeg() {
		return mIndexRightLeg;
	}

	QOpenGLBuffer getIndexLeftS2T() {
		return mIndexLeftS2T;
	}

	QOpenGLBuffer getIndexRightS2T() { return mIndexRightS2T; }

	QOpenGLBuffer getIndexT2T() { return mIndexT2T; }

	QOpenGLBuffer getIndexOutLine() { return mIndexOutLine; }
};
