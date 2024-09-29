#include "ObPng.h"
#include <memory.h>
#include <stdlib.h>
#include <QDebug>

#if 1
void print_obpng_info(PNG_FILE* png_file)
{

    int ret;
    OB_PNG_INFO obpng_info;
    ret = get_chunk_info(png_file, CHUNK_OBPNGINFO, &obpng_info, sizeof(obpng_info));
    if (ret != SUCCESS)
    {
        printf("obpng info null\n");
        return;
    }
    printf("=== obpng info ====\n");
    printf("obpng magic: %c%c%c%c%c\n", obpng_info.obpng_magic[0], obpng_info.obpng_magic[1], obpng_info.obpng_magic[2], obpng_info.obpng_magic[3], obpng_info.obpng_magic[4]);
    printf("tool name: %s\n", obpng_info.tool_name.name);
    uint8_t major = obpng_info.obver >> 24;
    uint8_t minor = obpng_info.obver >> 16 & 0xFF;
    uint8_t maintenance = obpng_info.obver >> 8 & 0xFF;
    uint8_t build = obpng_info.obver & 0xFF;
    printf("obpng ver: %d.%d.%d.%d\n", major, minor, maintenance, build);
}

void print_device_info(PNG_FILE* png_file)
{
    int ret;
    DEVICE_INFO dev_info;
    ret = get_chunk_info(png_file, CHUNK_DEVICE, &dev_info, sizeof(dev_info));
    if (ret != SUCCESS)
    {
        printf("proj device null\n");
        return;
    }
    printf("=== deviceinfo ====\n");
    printf("device: %s\n", dev_info.device.name);
    printf("ir_model: %s\n", dev_info.ir_model.name);
    printf("IR Sensor: %s\n", dev_info.ir_sensor.name);
    printf("ldmp model: %s\n", dev_info.ldmp_model.name);
    printf("dev serial: %s\n", dev_info.dev_serial.name);
    printf("ir serial: %s\n", dev_info.ir_serial.name);
    printf("ldmp serial: %s\n", dev_info.ldmp_serial.name);
    printf("engine: %s\n", dev_info.depth_engine.name);
    const char *model_type_str[] = {
        "MODEL_MONO_STRUCTURE_LIGHT",
        "MODEL_STEREO_ACTIVE,",
        "MODEL_STEREO_PASSIVE",
        "unknown"
    };
    const int model_type_str_len = sizeof(model_type_str) / sizeof(char*);
    int index = (dev_info.model_type >= model_type_str_len ?
        model_type_str_len - 1 :
        dev_info.model_type);
    printf("model_type: %s\n", model_type_str[index]);
    printf("\n");
}

void print_ir_info(PNG_FILE* png_file)
{
    int ret;
    IR_SENSOR_INFO ir_info;
    ret = get_chunk_info(png_file, CHUNK_IR_SENSOR, &ir_info, sizeof(ir_info));
    if (ret != SUCCESS)
    {
        printf("proj ir null\n");
        return;
    }
    printf("IR Sensor:\n");
    printf("width: %d\n", ir_info.width);
    printf("height: %d\n", ir_info.height);
    const char *rotation_str[] = {
        "rotation_zero",
        "rotation_clockwise_90",
        "rotation_180",
        "rotation_anticlock_90",
        ""
    };
    const int rotation_str_len = sizeof(rotation_str) / sizeof(char*);
    int index = (ir_info.rotation >= rotation_str_len ?
        rotation_str_len - 1 :
        ir_info.rotation);
    printf("rotation: %s\n", rotation_str[index]);
    const char *mirror_str[] = {
        "mirror_non",
        "mirror_vertical",
        "mirror_horizontal",
        "mirror_vert_horiz",
    };
    const int mirror_str_len = sizeof(mirror_str) / sizeof(char*);
    index = (ir_info.mirror >= mirror_str_len ?
        mirror_str_len - 1 :
        ir_info.mirror);
    printf("mirror: %s\n", mirror_str[index]);
    printf("pixelsize: %.2fmm\n", ir_info.pixel_size);
    qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "fx:" << ir_info.fx << "fy:" << ir_info.fy;
    qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "cx:" << ir_info.cx << "cy:" << ir_info.cy;
    printf("fx: %.2f pixel, fy: %.2f pixel\n", ir_info.fx, ir_info.fy);
    printf("cx: %.2f pixel, cy: %.2f pixel\n", ir_info.cx, ir_info.cy);
    printf("\n");
}

void print_proj_info(PNG_FILE* png_file)
{
    int ret;
    PROJECTOR_INFO proj_info;
    ret = get_chunk_info(png_file, CHUNK_PROJECTOR, &proj_info, sizeof(proj_info));
    if (ret != SUCCESS)
    {
        printf("proj is null\n");
        return;
    }
    printf("Projector:\n");
    printf("DOE_Model: %s\n", proj_info.doe_model.name);
    printf("lp_per_block: %d\n", proj_info.lp_per_block);
    printf("num_block: %d\n", proj_info.num_block);
    printf("wave_length: %d\n", proj_info.wave_length);
    printf("\n");
}

void print_ext_info(PNG_FILE* png_file)
{
    int ret;
    EXTERNAL_INFO ext_info;
    ret = get_chunk_info(png_file, CHUNK_EXTERNAL, &ext_info, sizeof(ext_info));
    if (ret != SUCCESS)
    {
        return;
    }
    printf("ext_info:\n");
    printf("baseline: %.2fmm\n", ext_info.baseline);
    printf("rx: %.2f, ry: %.2f, rz: %.2f\n", ext_info.rx, ext_info.ry, ext_info.rz);
    printf("tx: %.2f, ty: %.2f, tz: %.2f\n", ext_info.tx, ext_info.ty, ext_info.tz);
    printf("\n");
}

void print_drv_info(PNG_FILE* png_file)
{
    printf("driver info:\n");
    int ret;
    DRIVER_INFO drv_info;
    ret = get_chunk_info(png_file, CHUNK_DRIVER, &drv_info, sizeof(drv_info));
    if (ret != SUCCESS)
    {
        return;
    }
    printf("DriverInfo:\n");
    printf("firmware_ver: %s\n", drv_info.firmware_ver.name);
    printf("ir_gain     : %d\n", drv_info.ir_gain);
    printf("ir_exposure : %dus\n", drv_info.ir_exposure);
    printf("laser_curr  : %duA\n", drv_info.laser_curr);
    printf("laser_time  : %dus\n", drv_info.laser_time);
    printf("\n");
}

void print_env_info(PNG_FILE* png_file)
{
    int ret;
    ENV_INFO env_info;
    ret = get_chunk_info(png_file, CHUNK_ENV, &env_info, sizeof(env_info));
    if (ret != SUCCESS)
    {
        return;
    }
    printf("env_info:\n");
    printf("ir_temp      : %dx0.1K\n", env_info.ir_temp);
    printf("ldmp_temp    : %dx0.1K\n", env_info.ldmp_temp);
    printf("illuminance  : %dlux\n", env_info.illuminance);
    printf("\n");
}

void print_data_info(PNG_FILE* png_file)
{
    int ret;
    DATA_INFO data_info;
    ret = get_chunk_info(png_file, CHUNK_DATA, &data_info, sizeof(data_info));
    if (ret != SUCCESS)
    {
        return;
    }
    const char* dataname[] = { "DATA_GRAY_SCALE_BITMAP",
        "DATA_DISPARITY_MAP",
        "DATA_DEPTH_MAP" };

    const char* packfmtstr[] = {
        "PACK_FMT_OPENNI",
        "PACK_FMT_ORIGINAL"
    };
    printf("data info:\n");
    printf("data_type: %s\n", dataname[data_info.data_type]);
    printf("depth_unit: %.2fum\n", data_info.depth_uint);
    printf("disp_bits: %d bit\n", data_info.disp_bits);
    printf("disp_sub_bits: %d bit\n", data_info.disp_sub_bits);
    //printf("data_pack_fmt: %s\n", packfmtstr[data_info.pack_fmt]);
    printf("\n");
}

void print_dist_info(PNG_FILE* png_file)
{
    int ret;
    DISTORTION_INFO dist_info;
    ret = get_chunk_info(png_file, CHUNK_DISTORTION, &dist_info, sizeof(dist_info));
    if (ret != SUCCESS)
    {
        return;
    }
    printf("distort_info:\n");
    switch (dist_info.disto_model)
    {
    case DISTO_MODEL_BROWN_K2T2:
        printf("distort_mode_type: BROWN_K2T2\n");
        printf("k1: %.6f, k2: %.6f\n",
            dist_info.disto_data.k1,
            dist_info.disto_data.k2);
        printf("t1: %.6f, t2: %.6f\n",
            dist_info.disto_data.t1,
            dist_info.disto_data.t2);
        break;
    case DISTO_MODEL_BROWN_K3T2:
        printf("distort_mode_type: BROWN_K3T2\n");
        printf("k1: %.6f, k2: %.6f, k3: %.6f\n",
            dist_info.disto_data.k1,
            dist_info.disto_data.k2,
            dist_info.disto_data.k3);
        printf("t1: %.6f, t2: %.6f\n",
            dist_info.disto_data.t1,
            dist_info.disto_data.t2);
        break;
    case DISTO_MODEL_BROWN_K4T2:
        printf("distort_mode_type: BROWN_K4T2\n");
        break;
    case DISTO_MODEL_FISHEYE:
        printf("distort_mode_type: FISHEYE\n");
        break;
    case DISTO_MODEL_SPHERICAL:
        printf("distort_mode_type: SPHERICAL\n");
        break;
    default:
        printf("distort_mode_type: unknown\n");
        break;
    }
    printf("\n");
}
#endif

ObPng::ObPng()
{
    m_ir_fx = 0;
    m_ir_fy = 0;
    m_ir_cx = 0;
    m_ir_cy = 0;
    m_ir_data = nullptr;
    m_ir_width = 0;
    m_ir_height = 0;
    m_ir_size = 0;

    m_png_ptr = nullptr;
    m_info_ptr = nullptr;

    m_obpngVersion = obpng_version();

    printf("obpng version: %u.%u.%u(build: %u)\n", m_obpngVersion.major, m_obpngVersion.minor, m_obpngVersion.maintenance, m_obpngVersion.build);

    memset(&m_PngFile, 0, sizeof(m_PngFile));
}


ObPng::~ObPng()
{
}

// 鍔犺浇ob png鍥剧墖
bool ObPng::LoadObpng(const char* fileName)
{
    int iRet = -1;
    FILE* fIn = fopen(fileName, "rb");
    if (nullptr == fIn)
    {
        printf("open %s failed\n", fileName);
        return false;
    }

    bool bObPng = is_obpng(fileName);
    if (false == bObPng)
    {
        printf("%s is not ob png\n", fileName);
        return false;
    }

    iRet = load_png_file(fIn, &m_PngFile);
    if (iRet != SUCCESS)
    {
        printf("load png file failed\n");
        return false;
    }

    PNG_CHUNK* pchuck = m_PngFile.chunk_list;
    //print_obpng_info(&m_PngFile);
    //
    //print_device_info(&m_PngFile);
    //print_ir_info(&m_PngFile);
    //print_proj_info(&m_PngFile);
    //print_ext_info(&m_PngFile);
    //print_drv_info(&m_PngFile);
    //print_env_info(&m_PngFile);
    //print_data_info(&m_PngFile);
    //print_dist_info(&m_PngFile);

    get_ir_param(); // 璇诲彇ir鍐呭弬锛?

    fclose(fIn);

    return true;
}

void ObPng::get_ir_param()
{
    int ret;
    IR_SENSOR_INFO ir_info;
    ret = get_chunk_info(&m_PngFile, CHUNK_IR_SENSOR, &ir_info, sizeof(ir_info));
    if (ret != SUCCESS)
    {
        printf("proj ir null\n");
        return;
    }

    m_ir_fx = ir_info.fx;
    m_ir_fy = ir_info.fy;
    m_ir_cx = ir_info.cx;
    m_ir_cy = ir_info.cy;

    m_ir_width = ir_info.width;
    m_ir_height = ir_info.height;
    m_ir_size = ir_info.width * ir_info.height * sizeof(uint16_t);
    if (nullptr != m_ir_data)
    {
        delete m_ir_data;
        m_ir_data = nullptr;
    }

    m_ir_data = new char[m_ir_size * sizeof(char)]();
    memcpy(m_ir_data, m_PngFile.image.data, m_ir_size);
}




// 鍐欏叆ir png

void write_device_info(void* png_ptr) {

    DEVICE_INFO info;
    info.device.length = 8;
    sprintf((char*)info.device.name, "%s", "845oppo");

    info.ir_model.length = 8;
    sprintf((char*)info.ir_model.name, "%s", "MIR1801");

    info.ir_sensor.length = 7;
    sprintf((char*)info.ir_sensor.name, "%s", "OV9282");

    info.ldmp_model.length = 7;
    sprintf((char*)info.ldmp_model.name, "%s", "PJ1802");

    info.dev_serial.length = 7;
    sprintf((char*)info.dev_serial.name, "%s", "123456");

    info.ir_serial.length = 7;
    sprintf((char*)info.ir_serial.name, "%s", "654321");

    info.ldmp_serial.length = 7;
    sprintf((char*)info.ldmp_serial.name, "%s", "222222");

    info.depth_engine.length = 6;
    sprintf((char*)info.depth_engine.name, "%s", "depth");

    info.model_type = MODEL_STEREO_ACTIVE;

    add_chunk_info(png_ptr, CHUNK_DEVICE, &info, sizeof(info));
}



void write_projectorinfo(void* png_ptr) {

    PROJECTOR_INFO info;
    memset(info.doe_model.name, 0, sizeof(info.doe_model.name));
    memcpy(info.doe_model.name, "DOE", 4);
    info.doe_model.length = 4;
    info.lp_per_block = 181;
    info.num_block = 30;
    info.wave_length = 940;

    add_chunk_info(png_ptr, CHUNK_PROJECTOR, &info, sizeof(info));
}

void write_extr(void* png_ptr)
{
    EXTERNAL_INFO info;
    info.baseline = 25.00;
    info.rx = 0.00;
    info.ry = -0.00;
    info.rz = -0.01;

    info.tx = 0.00;
    info.ty = 40.60;
    info.tz = 0.00;

    add_chunk_info(png_ptr, CHUNK_EXTERNAL, &info, sizeof(info));
}


void write_drv_info(void* png_ptr)
{
    DRIVER_INFO info;
    memcpy(info.firmware_ver.name, "1.9.3", 5);
    info.firmware_ver.length = 5;
    info.ir_gain = 50;
    info.ir_exposure = 0.3;
    info.laser_curr = 100;
    info.laser_time = 30;

    add_chunk_info(png_ptr, CHUNK_DRIVER, &info, sizeof(info));
}

void write_env_info(void* png_ptr)
{
    ENV_INFO info;
    info.ir_temp = 30;
    info.ldmp_temp = 32;
    info.illuminance = 300;

    add_chunk_info(png_ptr, CHUNK_ENV, &info, sizeof(info));
}

void write_data_info(void* png_ptr)
{
    DATA_INFO  info;
    info.data_type = DATA_DEPTH_MAP;
    info.depth_uint = 100;
    info.disp_bits = 12;
    info.disp_sub_bits = 11;

    add_chunk_info(png_ptr, CHUNK_DATA, &info, sizeof(info));
}

void write_dist_info(void* png_ptr)
{
    DISTORTION_INFO info;
    info.disto_model = DISTO_MODEL_BROWN_K2T2;
    info.disto_data.k1 = 22.3;
    info.disto_data.k2 = 12.3;
    info.disto_data.t1 = 220.3;
    info.disto_data.t2 = 221.3;
    info.disto_data.k2 = 221.3;

    add_chunk_info(png_ptr, CHUNK_DISTORTION, &info, sizeof(info));
}


bool ObPng::WriteObpng(const char* fileName, uint16_t* data, writeInfo info)
{
    FILE* fOut = fopen(fileName, "wb");
    if (NULL == fOut)
    {
        printf("fopen %s failed\n", fileName);
        return -1;
    }

    m_png_ptr = create_writepng_struct();
    if (NULL == m_png_ptr) {
        printf("create png struct failed\n");
        return -1;
    }

    m_info_ptr = create_writepng_info(fOut, m_png_ptr, info.width, info.height, info.bit_depth, info.channels, info.compression_level);
    if (NULL == m_info_ptr)
    {
        printf("create_png_info failed \n");
        return -1;
    }

    write_png_image(m_png_ptr, m_info_ptr, info.width, info.height, (char*)data);   // 鍐欏叆鏁版嵁
    mark_tool_name(m_png_ptr, "writechunk", 10);

    write_device_info(m_png_ptr);
    
    //write_sensorinfo(m_png_ptr);
    write_ir_param(info);   // 鍐欏叆鍙傛暟 = write_sensorinfo()

    write_projectorinfo(m_png_ptr);
    write_extr(m_png_ptr);

    write_drv_info(m_png_ptr);
    write_env_info(m_png_ptr);
    write_data_info(m_png_ptr);
    write_dist_info(m_png_ptr);

    ENV_INFO env_info;
    env_info.ir_temp = 300;
    env_info.ldmp_temp = 112;
    env_info.illuminance = 113;
	TOF_DATA_INFO tof_data_info;
	tof_data_info.data_type = 1;
	tof_data_info.data_bits = 16;
	tof_data_info.width = info.width;
	tof_data_info.height = info.height;

	add_chunk_info(m_png_ptr, CHUNK_TOF_DATA, &tof_data_info, sizeof(tof_data_info));
    add_chunk_info(m_png_ptr, CHUNK_ENV, &env_info, sizeof(env_info));
    write_chunk_end(m_png_ptr, m_info_ptr);
    
    fclose(fOut);
    pngwrite_release(&m_png_ptr, &m_info_ptr);

    return true;
}


void ObPng::write_ir_param(writeInfo info)
{
    IR_SENSOR_INFO ir_info;
    ir_info.width = info.width;
    ir_info.height = info.height;
    ir_info.rotation = ROTATION_180;
    ir_info.mirror = MIRROR_VERTICAL;
    ir_info.pixel_size = 0.416511;

    ir_info.fx = info.fx;
    ir_info.fy = info.fy;
    ir_info.cx = info.cx;
    ir_info.cy = info.cy;

    add_chunk_info(m_png_ptr, CHUNK_IR_SENSOR, &ir_info, sizeof(ir_info));

}

//void write_sensorinfo(void* png_ptr)
//{
//    IR_SENSOR_INFO info;
//    info.width = 640;
//    info.height = 400;
//    info.rotation = ROTATION_180;
//    info.mirror = MIRROR_VERTICAL;
//    info.pixel_size = 0.416511;
//    info.fx = 325.021;
//    info.fy = 395.235;
//    info.cx = 325.021;
//    info.cy = 395.235;
//
//    add_chunk_info(png_ptr, CHUNK_IR_SENSOR, &info, sizeof(info));
//}