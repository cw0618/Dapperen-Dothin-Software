#include <orbbec_png_info.h>
#include <memory.h>
#include <string.h>


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
    const char* dataname [] = {"DATA_GRAY_SCALE_BITMAP",
                               "DATA_DISPARITY_MAP",
                               "DATA_DEPTH_MAP"};

    const char* packfmtstr[] = {
        "PACK_FMT_OPENNI",
        "PACK_FMT_ORIGINAL"
    };
    printf("data info:\n");
    printf("data_type: %s\n", dataname[data_info.data_type]);
    printf("depth_unit: %.2fum\n", data_info.depth_uint);
    printf("disp_bits: %d bit\n", data_info.disp_bits);
    printf("disp_sub_bits: %d bit\n", data_info.disp_sub_bits);
    printf("data_pack_fmt: %s\n", packfmtstr[data_info.pack_fmt]);
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

int main(int argc, char* argv[])
{
    if (argc == 1)
    {
        printf("argc < 2\n");
        return 0;
    }

    OBPNG_VERSION version = obpng_version();
    printf("obpng version: %u.%u.%u(build: %u)\n", version.major, version.minor, version.maintenance, version.build);


    PNG_FILE png_file;
    memset(&png_file, 0, sizeof(png_file));

    int ret;
    FILE* fin = fopen(argv[1], "rb");
    if (fin == NULL)
    {
        printf("open %s failed\n", argv[1]);
        return 0;
    }

    bool find = is_obpng(argv[1]);
    printf("is obpng %d\n", find);

    ret = load_png_file(fin, &png_file);

    if (ret != SUCCESS)
    {
        printf("load png file failed\n");
        return 0;
    }
    PNG_CHUNK* pchuck = png_file.chunk_list;

    print_obpng_info(&png_file);

    print_device_info(&png_file);
    print_ir_info(&png_file);
    print_proj_info(&png_file);
    print_ext_info(&png_file);
    print_drv_info(&png_file);
    print_env_info(&png_file);
    print_data_info(&png_file);
    print_dist_info(&png_file);

    FILE* fout = fopen(argv[2], "wb+");
    if(!fout){
        printf(" open out data file error\n");
        return 0;
    }
    PNG_IMAGE* pImage = &png_file.image;
    int size = pImage->rowbytes * pImage->height;
    printf("size: %d\n", size);
    fwrite(pImage->data, 1, size, fout);
    fclose(fout);


    free_png_file(&png_file);

    return 0;
}
