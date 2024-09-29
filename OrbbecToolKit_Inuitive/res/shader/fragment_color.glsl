varying vec2 v_depth_src;
varying vec2 v_texPo;
//背景纹理位置  接收于vertex_shader_bg
uniform sampler2D  sampler_rgb_;            // rgb掩码，用于抠图。
uniform sampler2D  sampler_depth_;          // 地面掩码，用于显示地面。

uniform int vChangeType;    // 处理类型
uniform float vWindowRatio; // 窗口宽高比
uniform float vFrameRatio;  // 图像宽高比

varying vec4 gPosition;

vec3 Pallet(float z_val)
{
    float rf, gf, bf;
    if (z_val <= 29.0) {
        rf = (129.36 - z_val * 4.36);
        gf = 0.0;
        bf = 255.0;
    }
    else if (z_val <= 86.0) {
        rf = 0.0;
        gf = (-133.54 + z_val * 4.52);
        bf = 255.0;
    }
    else if (z_val <= 141.0) {
        rf = 0.0;
        gf = 255.0;
        bf = (665.83 - z_val * 4.72);
    }
    else if (z_val <= 199.0) {
        rf = (-635.26 + z_val * 4.47);
        gf = 255.0;
        bf = 0.0;
    }
    else {
        rf = 255.0;
        gf = (1166.81 - z_val * 4.57);
        bf = 0.0;
    }

    return  vec3(rf / 255.0, gf / 255.0, bf / 255.0);
}

void main() {
     vec4 nColor = texture2D(sampler_rgb_, v_texPo);
    if(1 == vChangeType)
    {
        // AR 交互，半透明显示深度，用于调试
        vec4 v4_depth_src = texture2D(sampler_depth_, v_depth_src);

        // 深度值和对应地面掩码
        float depth_val = (v4_depth_src.y * 256.0 + v4_depth_src.x) * 255.0;
        float mea_val = v4_depth_src.a;

        if(depth_val > 0.0 && mea_val > 0.0)
        {
            vec3 depth_rgb = Pallet((depth_val) * 255.0 / 7600.0); // 255.0 / max_depth_value
            gl_FragColor = vec4(depth_rgb.r*0.5+nColor.r*0.5,
                                depth_rgb.g*0.5+nColor.g*0.5,
                                depth_rgb.b*0.5+nColor.b*0.5, 1.0);
        }
        else
        {
            gl_FragColor = nColor;
        }
    }else{
        gl_FragColor = nColor;
    }
}
