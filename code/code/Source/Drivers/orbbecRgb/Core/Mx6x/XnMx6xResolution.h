#ifndef XN_MX6X_RESOLUTION
#define XN_MX6X_RESOLUTION



typedef enum ob_resolution_t {
    RESOLUTION_UNKNOWN = 0,
    RESOLUTION_1920_2880 = 1,//pleco
    RESOLUTION_960_1440 = 2,
    RESOLUTION_1920_1440 = 3,
    RESOLUTION_960_720 = 4,
    RESOLUTION_1280_3840 = 5,//s5k33d
    RESOLUTION_640_1920 = 6,
    RESOLUTION_320_960 = 7,
    RESOLUTION_2560_3840 = 8,
    RESOLUTION_2560_7680 = 9,
    RESOLUTION_640_480 = 200,//depth, amplitude, ir, etc, vga format
    RESOLUTION_320_240 = 201,
    RESOLUTION_160_120 = 202,
}ob_resolution_key;


typedef struct ob_videomode_t {
	XnInt width;
	XnInt height;
}ob_resolution_value;


typedef struct ob_resolution_keyvalue_t {
	ob_resolution_key key;
	ob_resolution_value  vmode;
}ob_resolution_map;


static ob_resolution_map ob_res_arr[] = {
    { RESOLUTION_1920_2880, { 1920, 2880 } },
    { RESOLUTION_960_1440, { 960, 1440 } },
    { RESOLUTION_1920_1440, { 1920, 1440 } },//2
    { RESOLUTION_960_720, { 960, 720 } },
    { RESOLUTION_1280_3840, { 1280, 3840 } }, //4
    { RESOLUTION_640_1920, { 640, 1920 } },
    { RESOLUTION_320_960, { 320, 960 } },//6
    { RESOLUTION_640_480, { 640, 480 } },
    { RESOLUTION_320_240, { 320, 240 } },//8
    { RESOLUTION_160_120, { 160, 120 } },
};


#endif //XN_MX6X_RESOLUTION