#include "skeletonindex.h"

SkeletonIndex::SkeletonIndex()
{
	
	mIndexTrunk.create();
	mIndexTrunk.bind();
	mIndexTrunk.allocate(ARRAY_TRUNK, 3 * sizeof(GLfloat));

	mIndexShoulder.create();
	mIndexShoulder.bind();
	mIndexShoulder.allocate(ARRAY_SHOULDER, 3 * sizeof(GLfloat));

	mIndexLeftArm.create();
	mIndexLeftArm.bind();
	mIndexLeftArm.allocate(ARRAY_LEFT_ARM, 3 * sizeof(GLfloat));

	mIndexThighbone.create();
	mIndexThighbone.bind();
	mIndexThighbone.allocate(ARRAY_THIGHBONE, 3 * sizeof(GLfloat));

	mIndexLeftLeg.create();
	mIndexLeftLeg.bind();
	mIndexLeftLeg.allocate(ARRAY_LEFT_LEG, 3 * sizeof(GLfloat));

	mIndexRightLeg.create();
	mIndexRightLeg.bind();
	mIndexRightLeg.allocate(ARRAY_RIGHT_LEG, 3 * sizeof(GLfloat));
	// ×ó¼çºÍ×ó¿è¹Ç
	mIndexLeftS2T.create();
	mIndexLeftS2T.bind();
	mIndexLeftS2T.allocate(ARRAY_LEFT_S2T, 3 * sizeof(GLfloat));
	// ÓÒ¼çºÍÓÒ¿è¹Ç
	mIndexRightS2T.create();
	mIndexRightS2T.bind();
	mIndexRightS2T.allocate(ARRAY_RIGHT_S2T, 3 * sizeof(GLfloat));
	// ÓÒ¿è¹ÇºÍÓÒ¿è¹Ç
	mIndexT2T.create();
	mIndexT2T.bind();
	mIndexT2T.allocate(ARRAY_RIGHT_S2T, 3 * sizeof(GLfloat));
	// Íâ¿ò
	mIndexOutLine.create();
	mIndexOutLine.bind();
	mIndexOutLine.allocate(ARRAY_RIGHT_S2T, 3 * sizeof(GLfloat));
}

SkeletonIndex::~SkeletonIndex()
{
}
int SkeletonIndex::getLastPoint(int idx)
{
	// todo £º±éÀú¹Ç¼Ü¶¨Òå£¬Ñ°ÕÒÉÏÒ»¸öµã
	// ×óÍÈ
	if (13 == idx) { return 11; }
	else if (11 == idx) { return 9; }
	else if (9 == idx) { return 2; }
	// ÓÒÍÈ
	else if (14 == idx) { return 12; }
	else if (12 == idx) { return 10; }
	else if (10 == idx) { return 2; }
	// ×ó±Û
	else if (7 == idx) { return 5; }
	else if (5 == idx) { return 3; }
	else if (3 == idx) { return 1; }
	// ÓÒ±Û
	else if (8 == idx) { return 6; }
	else if (6 == idx) { return 4; }
	else if (4 == idx) { return 1; }

	// Çû¸É
	else if (0 == idx) { return 1; }
	else if (2 == idx) { return 1; }

	return 1;
}