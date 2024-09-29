#include "aibody.h"
/** \class AIBody
*
* AI数据信息类
*
*/
AIBody::AIBody()
{
}

AIBody::~AIBody()
{
	reset();
}
void AIBody::reset() {
}
uint32_t AIBody::getId() {
	return mId;
}
void AIBody::setId(uint32_t id) {
	mId = id;
}

uint32_t AIBody::getJointWidth() {
	return mJointWidth;
}
void AIBody::setJointWidth(uint32_t width) {
	mJointWidth = width;
}
uint32_t AIBody::getJointHeight() {
	return mJointHeight;
}
void AIBody::setJointHeight(uint32_t height) {
	mJointHeight = height;
}
int AIBody::getJointFormat() {
	return mJointFormat;
}
void AIBody::setJointFormat(int joint_format) {
	mJointFormat = joint_format;
}
int AIBody::getJointSize() {
	return mAiJoints.size();
}
uint64_t AIBody::getTimestamp() {
	return mTimeStamp;
}
void AIBody::setTimestamp(uint64_t time_stamp) {
	mTimeStamp = time_stamp;
}
