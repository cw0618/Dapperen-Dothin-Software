/** \class IPipeLine
*
* 平面图像显示类
*
*/
#include "renderPipeline.h"
#include <QDebug>
#include <iostream>
#include <fstream>
void IPipeLine::setShape(IShape *value)
{
	shape = value;
}

GLuint IPipeLine::esCreateProgram(const char* vertexShaderSrc, const char* fragmentShaderSrc) {
	GLuint vertexShader;
	GLuint fragmentShader;
	GLuint programId;
	GLint linked;

	// Load the vertex shader
	vertexShader = esLoadShader(GL_VERTEX_SHADER, vertexShaderSrc);
	if (esCheckError()) {
		qDebug() << "error vertexShader: " << vertexShader;
		return 0;
	}

	// Load the fragment shader
	fragmentShader = esLoadShader(GL_FRAGMENT_SHADER, fragmentShaderSrc);
	if (esCheckError()) {
		qDebug() << "error fragmentShader: " << fragmentShader;
		glDeleteShader(vertexShader);
		return 0;
	}


	// Create the program
	programId = glCreateProgram();
	if (esCheckError()) {
		qDebug() << "error programId: " << programId;

		glDeleteShader(vertexShader);
		glDeleteShader(fragmentShader);
		return 0;
	}

	glAttachShader(programId, vertexShader);
	glAttachShader(programId, fragmentShader);
	glLinkProgram(programId);


	// Check the link status
	glGetProgramiv(programId, GL_LINK_STATUS, &linked);
	if (!linked) {
		GLint infoLen = 0;
		glGetProgramiv(programId, GL_INFO_LOG_LENGTH, &infoLen);

		if (infoLen > 1) {
			char *infoLog = (char*)malloc(sizeof(char) * infoLen);
			glGetProgramInfoLog(programId, infoLen, NULL, infoLog);
			qDebug() << QString("linking program :%1").arg(infoLog);
			free(infoLog);
		}
		glDeleteProgram(programId);
		programId = 0;
	}

	// Free up no longer needed shader resources
	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);
	return programId;
}

GLuint IPipeLine::esLoadShader(GLenum type, const char *shaderSrc) {
	GLuint shader;
	GLint compiled;

	// Create the shader object
	shader = glCreateShader(type);
	// Load the shader source
	glShaderSource(shader, 1, &shaderSrc, 0);
	// Compile the shader
	glCompileShader(shader);
	// Check the compile status
	glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);

	if (!compiled) {
		GLint infoLen = 0;
		glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &infoLen);

		if (infoLen > 1) {
			char *infoLog = (char*)malloc(sizeof(char) * infoLen);
			glGetShaderInfoLog(shader, infoLen, NULL, infoLog);

			qDebug() << QString("compiling shader:%1").arg(infoLog);
			free(infoLog);
		}

		glDeleteShader(shader);
		return 0;
	}

	return shader;
}

int IPipeLine::esCheckError() {
	GLenum error;
	int ret = 0;
	while ((error = glGetError()) != GL_NO_ERROR) {
		ret++;
		qDebug() << "glError:" << error;
	}

	return ret;
}


void PlanePipeLine::createProgram() {
	IPipeLine::createProgram();
	if (mProgramId != 0) {
		textureSamplerHandler = glGetUniformLocation(mProgramId, "sTexture");
		depthSamplerHandler = glGetUniformLocation(mProgramId, "dTexture");
		planeSamplerHandler = glGetUniformLocation(mProgramId, "planeData");
		hChangeType = glGetUniformLocation(mProgramId, "vChangeType");
		hPlaneCenter = glGetUniformLocation(mProgramId, "vPlaneCenter");
		qDebug() << "PlanePipeLine createProgram: " << mProgramId;

		//create color texture
		glGenTextures(3, textureId);
		glBindTexture(GL_TEXTURE_2D, textureId[0]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glBindTexture(GL_TEXTURE_2D, textureId[1]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glBindTexture(GL_TEXTURE_2D, textureId[2]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		esCheckError();
	}
}

void PlanePipeLine::updateTextureBufferData(int8_t* pixel, int width, int height, int pointSize) {
	textureBuf.setData(pixel, width, height, pointSize);
}
void PlanePipeLine::updateDepthBufferData(int8_t* pixel, int width, int height, int pointSize) {
	textureBuf.setDepthData(pixel, width, height, pointSize);
}

void PlanePipeLine::bindTextureBuffer() {
	if (textureId[0] != 0) {

		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, textureId[0]);
		if (esCheckError()) {
			qDebug("bindTextBuffer error ");
		}
		glUniform1i(textureSamplerHandler, 0);

		if (textureBuf.isFrameUpdate()) {
			GLint format = GL_RGB;
			if (textureBuf.PointSize() == 4) {
				format = GL_RGBA;
			}

			glTexImage2D(GL_TEXTURE_2D, 0, format, textureBuf.Width(), textureBuf.Height(), 0, format,   
				GL_UNSIGNED_BYTE, textureBuf.Data());


			//深度纹理传送
			if (d2CStart && textureBuf.DepthData() != nullptr)
			{
				//D2C渲染
				glUniform1i(hChangeType, 1);
				glActiveTexture(GL_TEXTURE1);
				glBindTexture(GL_TEXTURE_2D, textureId[1]);
				if (esCheckError()) {
					qDebug("bindTextBuffer error ");
				}
				if (textureBuf.DepthData() != nullptr && textureBuf.isDepthUpdate())
				{
					//qDebug() << "D2C" << " w" << textureBuf.DWidth() << " h=" << textureBuf.DHeight() << "planeSamplerHandler str" << sizeof(textureBuf.DepthData());
					glTexImage2D(GL_TEXTURE_2D, 0, format, textureBuf.DWidth(), textureBuf.DHeight() - 120,
						0, format, GL_UNSIGNED_BYTE, textureBuf.DepthData());

				}
				if (esCheckError()) {
					qDebug("textureBuf error ");
				}
				glUniform1i(depthSamplerHandler, 1);
				textureBuf.setDepthUpdate(false);
			}
			else if (body_2d.mJointFormat == body_2d.kPixelFormatFloodInfo) {
				//平面检测
				textureBuf.mMutexAiData.lock();
				glUniform1i(hChangeType, 2);
				glActiveTexture(GL_TEXTURE2);
				glBindTexture(GL_TEXTURE_2D, textureId[2]);
				if (esCheckError()) {
					qDebug("bindTextBuffer error ");
				}
				glUniform1i(planeSamplerHandler, 2);
				if (textureBuf.AIData() != nullptr && body_2d.mJointHeight > 120 && planeSamplerHandler > 0)
				{
					glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, body_2d.mJointWidth, body_2d.mJointHeight - 120,
						0, GL_RED, GL_UNSIGNED_BYTE, textureBuf.AIData());
				}
				if (esCheckError()) {
					qDebug("textureBuf error ");
				}

				textureBuf.mMutexAiData.unlock();
			}
			else if (body_2d.mJointFormat == body_2d.kPixelFormatBodyMask)
			{
				//抠图
				textureBuf.mMutexAiData.lock();
				glUniform1i(hChangeType, 3);
				glActiveTexture(GL_TEXTURE2);
				glBindTexture(GL_TEXTURE_2D, textureId[2]);
				if (esCheckError()) {
					qDebug("bindTextBuffer error ");
				}
				glUniform1i(planeSamplerHandler, 2);
				if (textureBuf.AIData() != nullptr && planeSamplerHandler > 0)
				{
					glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, body_2d.mJointWidth, body_2d.mJointHeight,
						0, GL_RED, GL_UNSIGNED_BYTE, textureBuf.AIData());
				}
				if (esCheckError()) {
					qDebug("textureBuf error ");
				}
				textureBuf.mMutexAiData.unlock();
			}
			else {
				glUniform1i(hChangeType, 0);
			}
			if (esCheckError()) {
				qDebug("bindTextBuffer error ");
			}
			textureBuf.setFrameUpdate(false);

		}

	}
}

void PlanePipeLine::draw() {
	IPipeLine::useProgram();
	if (body_2d.mJointFormat == body_2d.kPixelFormatFloodInfo)
	{
		qDebug() << "howard PIXEL_FORMAT_FLOOR_INFO";
	}
	bindTextureBuffer();
	if (shape != NULL) {
		shape->renderShape();
	}
	glBindTexture(GL_TEXTURE_2D, 0);


}
void PlanePipeLine::updateAIData(AIBody& body) {

	if (body.mJointFormat != body.kPixelFormatNone)
	{
		if (body.mJointFormat == body.kPixelFormatFloodInfo) {

			body_2d.mJointWidth = body.mJointWidth;
			body_2d.mJointHeight = body.mJointHeight;
			body_2d.mJointFormat = body.kPixelFormatFloodInfo;
			body_2d.mDataSize = body.mDataSize;
			for (int i = 0; i < 3; i++) {
				body_2d.mPlaneCenter[i] = body.mPlaneCenter[i];
			}
			for (int i = 0; i < 3; i++) {
				body_2d.mPlaneCenter[i] = body.mPlaneCenter[i];
			}
			textureBuf.mMutexAiData.lock();
			textureBuf.setAIData(body.mDataFrame, body_2d.mDataSize);
			textureBuf.mMutexAiData.unlock();
		}
		else if (body.mJointFormat == body.kPixelFormatBodyMask)
		{
			body_2d.mJointWidth = body.mJointWidth;
			body_2d.mJointHeight = body.mJointHeight;
			body_2d.mJointFormat = body.kPixelFormatBodyMask;
			body_2d.mDataSize = body.mDataSize;
			textureBuf.mMutexAiData.lock();
			textureBuf.setAIData(body.mDataFrame, body_2d.mDataSize);
			textureBuf.mMutexAiData.unlock();
		}
	}
	else {
		body_2d.mJointFormat = body_2d.kPixelFormatNone;
	}
}
/*********************************PointCloudPipeLine******************************************/

void PointCloudPipeLine::createProgram() {
	IPipeLine::createProgram();
	if (mProgramId != 0) {
		mvpMatrixHandler = glGetUniformLocation(mProgramId, "uMVPMatrix");
		qDebug() << "PointCloudPipeLine createProgram: " << mProgramId;
	}
}

void PointCloudPipeLine::updateSurfaceSize(int width, int height)
{
	IPipeLine::updateSurfaceSize(width, height);
	aspect = (float)width / height;
	projection = glm::perspective(glm::radians(fov), aspect, 0.01f, 80000.0f);

	mvpMatrix = projection * view * model;
}


void PointCloudPipeLine::updateMvpMatrix() {
	glUniformMatrix4fv(mvpMatrixHandler, 1, GL_FALSE, &mvpMatrix[0][0]);
}

void PointCloudPipeLine::rotationModel(float x, float y)
{
	m_angleX += x;
	m_angleY += y;

	rotationM = glm::mat4(1.0f);
	rotationM = glm::rotate(rotationM, m_angleY, glm::vec3(1.0f, 0.0f, 0.0f));
	rotationM = glm::rotate(rotationM, m_angleX, glm::vec3(0.0f, 1.0f, 0.0f));

	model = rotationM * baseModel;
	mvpMatrix = projection * view * model;
}

void PointCloudPipeLine::translateViewer(float* vec3)
{

	view = glm::translate(view, glm::vec3(vec3[0], vec3[1], vec3[2]));
	mvpMatrix = projection * view * model;
}

void PointCloudPipeLine::draw()
{
	IPipeLine::useProgram();
	updateMvpMatrix();
	if (shape != NULL) {
		shape->renderShape();
	}
}
void PlanePipeLine::setD2CStart(bool state) {
	d2CStart = state;
	textureBuf.setD2CStart(state);
}
void SkeletonPipeLine::draw()
{
	IPipeLine::useProgram();
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textureId);
	if (esCheckError()) {
		qDebug("bindTextBuffer error ");
	}
	glUniformMatrix4fv(muMVPMatrixHandle, 1, false,
		matrix_mvp.data());

	if (body_2d.mJointFormat != body_2d.kPixelFormatNone)
	{
		if (body_2d.mJointFormat == body_2d.kPixelFormatJoint2D)
		{

			drawPartLine(0, 0, 1);
			drawPartLine(0, 1, 2);

			//shoulder
			drawPartLine(0, 3, 1);
			drawPartLine(0, 1, 4);

			//left arm
			drawPartLine(0, 3, 5);
			drawPartLine(0, 5, 7);
			drawPartLine(0, 3, 9);

			//right arm
			drawPartLine(0, 4, 6);
			drawPartLine(0, 6, 8);
			drawPartLine(0, 4, 10);

			//thighbone
			drawPartLine(0, 9, 2);
			drawPartLine(0, 2, 10);
			drawPartLine(0, 9, 10);

			//left leg

			drawPartLine(0, 9, 11);
			drawPartLine(0, 11, 13);

			//right leg
			drawPartLine(0, 10, 12);
			drawPartLine(0, 12, 14);

			for (int j = 0; j < mGroups.mSkeletons.size(); j++) {
				SKELETON_DATA_T mData = mGroups.mSkeletons.at(j);
				if (mData.score >= 0.1) {
					vertices_circle[0] = mData.x;
					vertices_circle[1] = mData.y;
					vertices_circle[2] = mData.z;
					glVertexAttribPointer(vertexSamplerHandler, 3, GL_FLOAT,
						false, 3 * sizeof(GLfloat), vertices_circle);

					glVertexAttribPointer(colorSamplerHandler, 4, GL_FLOAT, false,
						4 * sizeof(GLfloat), colors_circle);

					glEnableVertexAttribArray(vertexSamplerHandler);

					glEnableVertexAttribArray(colorSamplerHandler);
					glPointSize(10);
					glDrawArrays(GL_POINTS, 0, 1);
				}
			}
		}
		else if (body_2d.mJointFormat == body_2d.kPixelFormatBodyShape)
		{
			vertices_circle[0] = 0;
			vertices_circle[1] = 0;
			vertices_circle[2] = 0;
			glVertexAttribPointer(vertexSamplerHandler, 3, GL_FLOAT,
				false, 3 * sizeof(GLfloat), vertices_circle);

			glVertexAttribPointer(colorSamplerHandler, 4, GL_FLOAT, false,
				4 * sizeof(GLfloat), colors_circle_red);

			glEnableVertexAttribArray(vertexSamplerHandler);

			glEnableVertexAttribArray(colorSamplerHandler);
			glPointSize(8);
			glDrawArrays(GL_POINTS, 0, 1);
		}
	}
	//glBindTexture(GL_TEXTURE_2D, 0);
}
void SkeletonPipeLine::createProgram() {
	IPipeLine::createProgram();
	if (mProgramId != 0) {
		vertexSamplerHandler = glGetAttribLocation(mProgramId, "aPosition");
		colorSamplerHandler = glGetAttribLocation(mProgramId, "aColor");
		muMVPMatrixHandle = glGetUniformLocation(mProgramId, "uMVPMatrix");
		qDebug() << "PlanePipeLine createProgram: " << mProgramId;
		matrix_mvp.isIdentity();
		//create color texture
		glGenTextures(1, &textureId);
		glBindTexture(GL_TEXTURE_2D, textureId);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glBindTexture(GL_TEXTURE_2D, 0);
	}
}
void SkeletonPipeLine::updata2DBody(AIBody& body) {

	if (body.mJointFormat != body.kPixelFormatNone)
	{
		float width = body.getJointWidth();
		float height = body.getJointHeight();
		float displayInfo_x_start = 0;
		float displayInfo_y_start = 0;
		//circle_2d.user_id = body.getId();
		body_2d.setId(body.getId());
		body_2d.setJointFormat(body.getJointFormat());
		body_2d.setJointWidth(width);
		body_2d.setJointHeight(height);
		body_2d.setTimestamp(body.getTimestamp());
		//float frame_ratio = width / (float)height;
		//float wind_ratio = surfaceWidth / (float)surfaceHeight;
		frameWidth = width;
		frameHeight = height;

		mWidthRatio = (float)surfaceWidth / width;
		mHeightRatio = (float)surfaceHeight / height;
		if (mHeightRatio >= mWidthRatio) {
			ratioX = mWidthRatio;
			ratioY = mWidthRatio;
		}
		else {
			ratioX = mHeightRatio;
			ratioY = mHeightRatio;
		}
		mGroups.id = 0;
		if (body.mJointFormat == body.kPixelFormatJoint2D) {
			mGroups.mSkeletons.clear();
			if (rgb_width > 0 || rgb_height > 0)
			{
				for (int j = 0; j < body.getJointSize(); ++j)
				{
					SKELETON_DATA_T mData;
					float x, y, z = 0;
					//qDebug() << "j="<<j<<" x =" << (float)body.joints[j].position.x << " y=" << (float)body.joints[j].position.y;
					x = ((float)body.mAiJoints.at(j).positionX) * rgb_width / frameWidth;
					y = ((float)body.mAiJoints.at(j).positionY) * rgb_height / frameHeight;
					x = (x - rgb_width / 2) / (rgb_width / 2);
					y = -(y - rgb_height / 2) / (rgb_height / 2);
					mData.index = j;
					mData.x = x;
					mData.y = y;
					mData.z = 0;
					mData.score = body.mAiJoints.at(j).score;
					mGroups.mSkeletons.append(mData);
				}
			}
		}
		else if (body.mJointFormat == body.kPixelFormatBodyShape)
		{

		}
	}
	else
	{
		body_2d.mJointFormat = body_2d.kPixelFormatNone;
	}

}
void SkeletonPipeLine::drawPartLine(int index, int index0, int index1) {

	if (mGroups.mSkeletons.at(index0).score < 0.1 || mGroups.mSkeletons.at(index1).score < 0.1) {
		return;
	}
	vertices_line[0] = mGroups.mSkeletons.at(index0).x;
	vertices_line[1] = mGroups.mSkeletons.at(index0).y;
	vertices_line[2] = mGroups.mSkeletons.at(index0).z;
	vertices_line[3] = mGroups.mSkeletons.at(index1).x;
	vertices_line[4] = mGroups.mSkeletons.at(index1).y;
	vertices_line[5] = mGroups.mSkeletons.at(index1).z;
	glVertexAttribPointer(vertexSamplerHandler, 3, GL_FLOAT,
		false, 3 * sizeof(GLfloat), vertices_line);

	glVertexAttribPointer(colorSamplerHandler, 4, GL_FLOAT, false,
		4 * sizeof(GLfloat), colors_line);

	glEnableVertexAttribArray(vertexSamplerHandler);

	glEnableVertexAttribArray(colorSamplerHandler);
	glLineWidth(8);
	glDrawArrays(GL_LINES, 0, 2);
}
void SkeletonPipeLine::setRGBSize(int width, int height) {
	rgb_width = width;
	rgb_height = height;

}
