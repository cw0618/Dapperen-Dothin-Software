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
	QOpenGLBuffer mIndexTrunk;        // ���� 1-2-15
	QOpenGLBuffer mIndexShoulder;     // ��� 3-2-6
	QOpenGLBuffer mIndexLeftArm;      // ��� 3-4-5
	QOpenGLBuffer mIndexRightArm;     // �ұ� 6-7-8
	QOpenGLBuffer mIndexThighbone;    // ��� 9-15-12
	QOpenGLBuffer mIndexLeftLeg;      // ���� 9-10-11
	QOpenGLBuffer mIndexRightLeg;     // ���� 12-13-14
	QOpenGLBuffer mIndexLeftS2T;      // �������� 3-9
	QOpenGLBuffer mIndexRightS2T;     // �Ҽ���ҿ�� 4-10
	QOpenGLBuffer mIndexT2T;          // �Ҽ���ҿ�� 4-10
	QOpenGLBuffer mIndexOutLine;      // ���5����
	GLfloat ARRAY_TRUNK[3] = { 0, 1, 2 };           // ����
	GLfloat ARRAY_SHOULDER[3] = { 3, 1, 4 };        // ���
	GLfloat ARRAY_LEFT_ARM[3] = { 3, 5, 7 };        // ���
	GLfloat ARRAY_RIGHT_ARM[3] = { 4, 6, 8 };       // �ұ�
	GLfloat ARRAY_THIGHBONE[3] = { 9, 2, 10 };      // ���
	GLfloat ARRAY_LEFT_LEG[3] = { 9, 11, 13 };      // ����
	GLfloat ARRAY_RIGHT_LEG[3] = { 10, 12, 14 };    // ����
	GLfloat ARRAY_LEFT_S2T[3] = { 3, 9 };           // ��������
	GLfloat ARRAY_RIGHT_S2T[3] = { 4, 10 };         // �Ҽ���ҿ��
	GLfloat ARRAY_T2T[2] = { 9, 10 };               // �ҿ�Ǻ��ҿ��
	GLfloat ARRAY_OUTLINE[5] = { 15, 16, 17, 18, 15 };  // ���
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
