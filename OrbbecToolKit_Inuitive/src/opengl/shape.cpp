/*
 * Shape.cpp
 *
 *  Created on: Jun 9, 2018
 *      Author: yuxuewen
 */

#include "shape.h"
#include <QDebug>


void PCloudShape::createVAO(){

    float vertices[] = {
        0.0f,  0.0f, 0.0f,   1.0f, 0.0f, 0.0f,
         25000.0f,  25000.0f, 50000.0f,   1.0f, 0.0f, 0.0f,

        0.0f,  0.0f, 0.0f,   1.0f, 0.0f, 0.0f,
          -25000.0f,  25000.0f, 50000.0f,   1.0f, 0.0f, 0.0f,

         0.0f,  0.0f, 0.0f,   1.0f, 0.0f, 0.0f,
         25000.0f,  -25000.0f, 50000.0f,   1.0f, 0.0f, 0.0f,

        0.0f,  0.0f, 0.0f,   1.0f, 0.0f, 0.0f,
        -25000.0f,  -25000.0f, 50000.0f,   1.0f, 0.0f, 0.0f,

         25000.0f,  25000.0f, 50000.0f,   1.0f, 0.0f, 0.0f,
          -25000.0f,  25000.0f, 50000.0f,   1.0f, 0.0f, 0.0f,

        25000.0f,  25000.0f, 50000.0f,   1.0f, 0.0f, 0.0f,
         25000.0f,  -25000.0f, 50000.0f,   1.0f, 0.0f, 0.0f,

         -25000.0f,  -25000.0f, 50000.0f,   1.0f, 0.0f, 0.0f,
         -25000.0f,  25000.0f, 50000.0f,   1.0f, 0.0f, 0.0f,

        -25000.0f,  -25000.0f, 50000.0f,   1.0f, 0.0f, 0.0f,
         25000.0f,  -25000.0f, 50000.0f,   1.0f, 0.0f, 0.0f,

    };

    IShape::createVAO();

    glBindVertexArray(VAO);//鐢荤嚎
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, 16*sizeof(vertices), vertices, GL_DYNAMIC_DRAW);


    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // texture coord attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void PCloudShape::updatePointCloudData(float *new_data, int new_pointCount){

    if(new_data == NULL || new_pointCount == 0){
        return;
    }


    if(data == NULL || pointCount < new_pointCount){
        if(data != NULL){
            delete data;
        }
        data = new float[new_pointCount * pointSize * sizeof(float)];//闇€鍔犱笂sizeof(float) 鍚﹀垯鍙樉绀轰竴鍗?
    }

    memcpy(data, new_data, new_pointCount * pointSize * sizeof(float));
    needBind = true;

    pointCount = new_pointCount;
}

void PCloudShape::bindPointCloudData()
{
    if(!needBind) return;
    needBind = false;

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, pointCount * pointSize *(sizeof(data[0])), data, GL_DYNAMIC_DRAW);

    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, pointSize * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // color attribute
    glVertexAttribPointer(1, 3,  GL_FLOAT, GL_FALSE, pointSize * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);


    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void PCloudShape::renderShape(){
	if(VAO != 0){
        if(needLine){
            glBindVertexArray(VAO);
            glDrawArrays(GL_LINES, 0, 16);
            glBindVertexArray(0);
        }else{
            bindPointCloudData();
            glBindVertexArray(VAO);
            glDrawArrays(GL_POINTS, 0, pointCount);
            glBindVertexArray(0);
        }


	}
}


void PlaneShape::createVAO(){

    float vertices[] = {
        -1.0f,  1.0f, -1.0f,   0.0f, 0.0f,
         1.0f,  1.0f, -1.0f,   1.0f, 0.0f,
         1.0f, -1.0f, -1.0f,   1.0f, 1.0f,
        -1.0f, -1.0f, -1.0f,   0.0f, 1.0f,

    };

    unsigned int indices[] = {
        0,1,2,
        0,2,3,
    };

	indicesCount = 6;

	IShape::createVAO();
	glGenBuffers(1, &EBO);
	glBindVertexArray(VAO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
	// position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	// texture coord attribute
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 10 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(2);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

}

void PlaneShape::renderShape(){
	glBindVertexArray(VAO);
	glDrawElements(GL_TRIANGLES, indicesCount, GL_UNSIGNED_INT, 0);
//	glDrawElements(GL_LINES,     indicesCount, GL_UNSIGNED_INT, 0);
//	glDrawElements(GL_LINE_LOOP, indicesCount, GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);
}
