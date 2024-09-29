#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <orbbec_png_info.h>
#include <orbbec_chunk_type.h>
#include <orbbec_png_utils.h>
#include <errno.h>

FILE *g_png_file = NULL;

using namespace std;

void write_device_info(void* png_ptr){

    DEVICE_INFO info;
    info.device.length =  8;
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

void write_sensorinfo(void* png_ptr)
{
    IR_SENSOR_INFO info;
    info.width = 1280;
    info.height = 800;
    info.rotation = ROTATION_180;
    info.mirror = MIRROR_VERTICAL;
    info.pixel_size = 0.416511;
    info.fx = 625.021;
    info.fy = 395.235;
    info.cx = 625.021;
    info.cy = 395.235;

    add_chunk_info(png_ptr, CHUNK_IR_SENSOR, &info, sizeof(info));
}

void write_projectorinfo(void* png_ptr){

    PROJECTOR_INFO info;
    memset(info.doe_model.name, 0, sizeof(info.doe_model.name));
    memcpy(info.doe_model.name, "DOE", 4);
    info.doe_model.length = 4;
    info.lp_per_block     = 181;
    info.num_block        = 30;
    info.wave_length      = 940;

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

int main(int argc, char* argv[]){

    const char* argv1 = "./1280_800.raw";
    const char* argv2 = "./1280_800.png";
    if(argc > 2){
        //return 0;
        argv1 = argv[1];
        argv2 = argv[2];
    }else{
        cout << "please input file, and output png file"<< endl;
        cout << "etc.  ./write_chunk 1280_800.raw 1280_800.png"<< endl;
    }
    

    OBPNG_VERSION version = obpng_version();
    printf("obpng version: %u.%u.%u(build: %u)\n", version.major, version.minor, version.maintenance, version.build);

    png_struct_ptr png_ptr = NULL;
    png_info_ptr   info_ptr = NULL;

    ifstream ifs(argv1, std::ios::binary|ios::in);
    
    if(!ifs.is_open()){
        cout << "open "<< argv1 << " failed"<< endl;
        return 0;
    }

    ifs.seekg(0, ios::end);
    int length = ifs.tellg();
    ifs.seekg(ios::beg);
    
    char* data = new char[length];
    ifs.read(data, length);
    ifs.close();

    FILE* fb = fopen(argv2, "wb");
    if( NULL == fb ){
        printf("fopen %s failed, err: %s\n", argv2, strerror(errno));
        return -1;
    }
    png_ptr = create_writepng_struct();
    if(NULL == png_ptr){
        printf("create png struct failed\n");
        return -1;
    }
    info_ptr = create_writepng_info(fb, png_ptr, 1280, 800, 16, 1, 2);
    if(NULL == info_ptr){
        printf("create_png_info failed \n");
        return -1;
    }

    write_png_image(png_ptr, info_ptr, 1280, 800, data);
  
    mark_tool_name(png_ptr, "writechunk", 10);

    write_device_info(png_ptr);
    write_sensorinfo(png_ptr);
    write_projectorinfo(png_ptr);
    write_extr(png_ptr);

    write_drv_info(png_ptr);
    write_env_info(png_ptr);
    write_data_info(png_ptr);
    write_dist_info(png_ptr);

    ENV_INFO info;
    info.ir_temp = 300;
    info.ldmp_temp = 112;
    info.illuminance = 113;

    add_chunk_info(png_ptr, CHUNK_ENV, &info, sizeof(info));

    write_chunk_end(png_ptr, info_ptr);
    fclose(fb);
    pngwrite_release(&png_ptr, &info_ptr);

    return 0;
}
