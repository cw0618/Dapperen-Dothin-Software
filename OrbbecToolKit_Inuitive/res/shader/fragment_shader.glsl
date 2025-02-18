#version 120

uniform float pointsCount;
uniform float colorAxisMode;
uniform vec3 pointsBoundMin;
uniform vec3 pointsBoundMax;

varying vec3 vert;
varying float pointIdx;
varying vec2 v_texPo;
varying vec3 ourColor;

void main() {
  float intensity = pointIdx/pointsCount;
  if (colorAxisMode == 1) {
    intensity = (vert.z + abs(pointsBoundMin.z))/(pointsBoundMax.z - pointsBoundMin.z);
  }
  //gl_FragColor = vec4(intensity, intensity, intensity, 0.);
  gl_FragColor = vec4(ourColor, 1.0f);
}
