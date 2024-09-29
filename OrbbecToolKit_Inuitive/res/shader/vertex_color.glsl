attribute vec4 aPosition;           //顶点位置
attribute vec2 aCoordinate;         //纹理位置
attribute vec2 a_rgb_src;    //彩色掩码纹理位置
attribute vec2 a_depth_src;  //深度掩码纹理位置
uniform mat4 uMatrix;

varying vec2 v_texPo;       // 纹理位置  与fragment_shader交互
varying vec2 v_rgb_src;       // 纹理位置  与fragment_shader交互
varying vec2 v_depth_src;       // 纹理位置  与fragment_shader交互

void main()
{
    gl_Position=uMatrix*aPosition;
    v_texPo = aCoordinate;
    v_rgb_src = a_rgb_src;
    v_depth_src = a_depth_src;
}
