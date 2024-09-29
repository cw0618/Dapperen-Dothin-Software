/***************************************************************************/
/* */
/* Copyright (c) 2013-2021 Orbbec 3D Technology, Inc */
/* 奥比中光科技有限公司 版权所有 2013-2031 */
/* */
/* PROPRIETARY RIGHTS of Orbbec 3D Technology are involved in the */
/* subject matter of this material. All manufacturing, reproduction, use, */
/* and sales rights pertaining to this subject matter are governed by the */
/* license agreement. The recipient of this software implicitly accepts */
/* the terms of the license. */
/* */
/* 本软件文档资料是奥比中光科技有限公司的资产, 任何人士阅读和使用本资料必须获得 */
/* 相应的书面授权, 承担保密责任和接受相应的法律约束. */
/* */
/***************************************************************************/

#ifndef SHAPE_H_
#define SHAPE_H_

#include <QMutex>
#include <QOpenGLFunctions_3_3_Core>
class IShape : protected QOpenGLFunctions_3_3_Core{
protected:

	GLuint VAO,VBO;

public:
    IShape(){

		VAO = 0;
		VBO = 0;
    }

	virtual ~IShape(){
		if(VAO != 0) glDeleteVertexArrays(1, &VAO);
		if(VBO != 0) glDeleteBuffers(1, &VBO);
    }

	virtual void createVAO(){
        initializeOpenGLFunctions();
		glGenVertexArrays(1, &VAO);
		glGenBuffers(1, &VBO);
	}

	virtual void renderShape() = 0;
};


class PCloudShape : public IShape{
private:
    int pointCount;
    int pointSize;
    float *data;
    bool needBind;
    bool needLine;

public:
    PCloudShape():IShape(){
        pointCount = 0;
        pointSize = 6;
        data = NULL;
        needBind = false;
        needLine = false;
    }

    ~PCloudShape(){
        if(data != NULL){
            delete data;
            data = NULL;
        }
    }

	void updatePointCloudData(float *data, int pointCount);
    virtual void createVAO();
	virtual void renderShape();
    void setNeedDrawCoord(bool isNeed){needLine = isNeed;}

private:
    void bindPointCloudData();
};

class PlaneShape : public IShape{
private:
	GLuint EBO;
	int indicesCount;
public:
	PlaneShape(){
		EBO = 0;
		indicesCount = 0;
	}

	~PlaneShape(){
		if(EBO != 0) glDeleteBuffers(1, &EBO);
	}
	virtual void createVAO();
	virtual void renderShape();
};
#endif /* SHAPE_H_ */
