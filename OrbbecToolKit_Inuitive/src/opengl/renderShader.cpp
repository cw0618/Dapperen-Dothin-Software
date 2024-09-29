/*
* RenderShader.cpp
*
*  Created on: Jun 9, 2018
*      Author: yuxuewen
*/

#include "renderShader.h"


const char* SamplerVertexShader =
"#version 300 es                                             \n\
        layout(location = 0) in vec3 aPosition;                     \n\
layout(location = 1) in vec2 aTexCoord;                     \n\
out vec2 vTexCoord;                                         \n\
uniform mat4 uMVPMatrix;                                    \n\
void main(void) {                                           \n\
            gl_Position = uMVPMatrix * vec4(aPosition,1);           \n\
            vTexCoord = aTexCoord;                                  \n\
                }\0";



const char* SamplerFragmentShader =
"#version 300 es                                             \n\
        precision mediump float;                                    \n\
in vec2 vTexCoord;                                          \n\
out vec4 fragColor;                                         \n\
uniform sampler2D sTexture;                                 \n\
void main(void) {                                           \n\
            fragColor = texture(sTexture, vTexCoord);               \n\
                }\0";



const char* PCloudVertexShader =
"#version 300 es                                             \n\
        layout(location = 0) in vec3 aPosition;                     \n\
layout(location = 1) in vec3 aColor;                        \n\
out vec3 vColor;                                            \n\
uniform mat4 uMVPMatrix;                                    \n\
void main(void) {                                           \n\
            gl_Position = uMVPMatrix * vec4(aPosition,1);           \n\
            vColor = aColor;                                        \n\
                }\0";



const char* PCloudFragmentShader =
"#version 300 es                                             \n\
        precision mediump float;                                    \n\
in vec3 vColor;                                             \n\
out vec4 fragColor;                                         \n\
void main(void) {                                           \n\
            fragColor = vec4(vColor, 1);                            \n\
                }\0";


const char* PlaneVertexShader =
"#version 330 core                                           \n\
        layout(location = 0) in vec3 aPosition;                     \n\
layout(location = 1) in vec2 aTexCoord;                     \n\
out vec2 vTexCoord;                                         \n\
void main(void) {                                           \n\
            gl_Position = vec4(aPosition,1);                        \n\
            vTexCoord = aTexCoord;                                  \n\
                }\0";



const char* PlaneFragmentShader =
"#version 330 core                                           \n\
        precision mediump float;                                    \n\
in vec2 vTexCoord;                                          \n\
out vec4 fragColor;                                         \n\
uniform sampler2D sTexture;                                 \n\
void main(void) {                                           \n\
            fragColor = texture(sTexture, vTexCoord);               \n\
                }\0";

const char* D2CVertexShader =
"#version 330 core                                           \n\
        layout(location = 0) in vec3 aPosition;                     \n\
layout(location = 1) in vec2 aTexCoord;                     \n\
layout(location = 2) in vec2 dTexCoord;                     \n\
out vec2 vTexCoord;                                         \n\
out vec2 depthTexCoord;                                     \n\
void main(void) {                                           \n\
            gl_Position = vec4(aPosition,1);                        \n\
            vTexCoord = aTexCoord;                                  \n\
            depthTexCoord = dTexCoord;                               \n\
                }\0";



const char* D2CFragmentShader =
"#version 330 core                                             \n\
precision mediump float;                                       \n\
in vec2 vTexCoord;                                             \n\
in vec2 depthTexCoord;                                         \n\
out vec4 fragColor;                                            \n\
out vec4 aPosition;                                          \n\
uniform int vChangeType;                                       \n\
uniform vec3 vPlaneCenter;                                     \n\
uniform sampler2D sTexture;                                    \n\
uniform sampler2D  planeData;                                  \n\
uniform sampler2D dTexture;                                    \n\
        void main(void) {                                           \n\
		vec4 nColor = texture2D(sTexture, vTexCoord);                  \n\
        if(vChangeType==1){                                           \n\
	    vec4 v4_depth_src = texture2D(dTexture, vTexCoord);         \n\
	    if (v4_depth_src.r == 1.0 && v4_depth_src.g == 1.0 && v4_depth_src.b == 1.0) {                      \n\
           fragColor = vec4(nColor.rgb,1.0);                                        \n\
        }                                                               \n\
        else {                                                          \n\
	       fragColor = vec4(v4_depth_src.r*0.4 + nColor.r*0.6,v4_depth_src.g*0.4 + nColor.g*0.6,v4_depth_src.b*0.4 + nColor.b*0.6, 1.0);  \n\
              }                                                          \n\
        }else if(vChangeType==2){                                         \n\
	         vec4 v4_plane_src = texture2D(planeData, vTexCoord);          \n\
             vec4 plane_color = vec4(0.0,1.0,0.0,1.0);                      \n\
              if(v4_plane_src.r>0.0){                                       \n\
                  fragColor = vec4(plane_color.r*0.5 + nColor.r*0.5,plane_color.g*0.5 + nColor.g*0.5,plane_color.b*0.5 + nColor.b*0.5, 1.0);   \n\
                  }else{                                                          \n\
                      fragColor = nColor;                                   \n\
                       }                                                     \n\
             }else if(vChangeType==3){                                       \n\
	vec4 body_mask_src = texture2D(planeData, vTexCoord);                     \n\
	vec4 bg_color = vec4(0.0, 1.0, 0.0, 1.0);                      \n\
     if(body_mask_src.r>0.5){                                       \n\
	                          fragColor = nColor;                    \n\
                            }                                        \n\
                            else {                                    \n\
                                     fragColor = vec4(bg_color.rgb, 1.0);               \n\
                                  }                                     \n\
                          }                                              \n\
                          else{                                           \n\
			 fragColor = nColor;                                           \n\
             }                                                                \n\
        }\0";

const char* TextVertexShader =
"#version 330 core                                           \n\
        layout(location = 0) in vec4 vertex;                        \n\
uniform mat4 projection;                                    \n\
out vec2 vTexCoord;                                         \n\
void main(void) {                                           \n\
            aPosition = projection*vec4(vertex.xy, 0.0, 1.0);     \n\
            vTexCoord = vertex.zw;                                  \n\
                }\0";


const char* TextFragmentShader =
"#version 330 core                                           \n\
        precision mediump float;                                    \n\
in vec2 vTexCoord;                                          \n\
out vec4 fragColor;                                         \n\
\n\
uniform sampler2D text;                                     \n\
uniform vec3 textColor;                                     \n\
void main(void) {                                           \n\
            vec4 sampled = vec4(1.0, 1.0, 1.0, texture(text, vTexCoord).r);               \n\
            fragColor = vec4(textColor, 1.0) * sampled;             \n\
                }\0";
const char* SkeletonVertexShader =
"#version 330 core                                                \n\
        uniform mat4 uMVPMatrix;                  \n\
in vec3 aPosition;                        \n\
in vec4 aColor;                           \n\
out vec4 vColor;                           \n\
void main() {                             \n\
            gl_Position = uMVPMatrix * vec4(aPosition,1);      \n\
            vColor = aColor;                        \n\
            }\0";



const char* SkeletonFragmentShader =
"#version 330 core                                           \n\
        in vec4 vColor;                             \n\
out vec4 fragColor;                           \n\
void main() {                              \n\
            fragColor = vColor;                         \n\
            }\0";

