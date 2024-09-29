#ifndef _OBEXTENSION_H_
#define _OBEXTENSION_H_

#include <OpenNI.h> 

#define EXTENSION_INFO_GET    0x00000001
#define EXTENSION_INFO_SET    0x00000002
#define EXTENSION_END         0xFFFFFFFF      

#define FLASH_SERIALNUMBER           0x30000
#define FLASH_CAM_PARAMS             0x70000
#define FLASH_RGB_INIT_TABLE_ADDR    0x80000
#define FLASH_D2C_MATRIX_ADDR        0x90000
#define FLASH_REF_ADDR               0xA0000
#define FLASH_IR_GAIN                0x130000
#define FLASH_TEC_LDP_FLAG           0x1A0000

#define MX6000_I2C_IR_GAIN 0x0035    //0x3509
#define MX4000_I2C_IR_GAIN 0x0035
#define I2C_IR_EXP 0x0009

typedef  void * OB_USB_HANDLE;

typedef struct RotateMatrix{
	float r00; float r01; float r02;
	float r10; float r11; float r12;
	float r20; float r21; float r22;
}RotateMatrix;

typedef struct Intrinsic{
    float fx; float fy; float cx; float cy;
}Intrinsic;

typedef struct Translate{
    float t0; float t1; float t2;
}Translate;

typedef struct Distortion{
    float k0; float k1; float k2; float k3; float k4;
}Distortion;

/*typedef struct OBCameraParams
{
    float l_intr_p[4];//[fx,fy,cx,cy]
    float r_intr_p[4];//[fx,fy,cx,cy]
    float r2l_r[9];//[r00,r01,r02;r10,r11,r12;r20,r21,r22]
    float r2l_t[3];//[t1,t2,t3]
    float l_k[5];//[k1,k2,p1,p2,k3]
    float r_k[5];
    //int is_mirror;
}OBCameraParams;*/

//dual camera struct
struct ObIntrinsicRefinement
{
	uint8_t     refinement_[2016];
};

struct ObCameraIntrinsic
{
	uint32_t    vc_mode_;       /// AF摄像头的VC值
	uint32_t    img_height_;    /// 图像高
	uint32_t    img_width_;     /// 图像宽
	float       focal_x_;       /// x方向焦距
	float       focal_y_;       /// y方向焦距
	float       cx_;            /// 主点x坐标
	float       cy_;            /// 主点y坐标
	float       k1_;            /// 径向畸变系数k1-k4
	float       k2_;
	float       k3_;
	float       k4_;
	float       p1_;            /// 切向畸变系数p1-p2
	float       p2_;
	ObIntrinsicRefinement refine_;    // refinement参数
};

struct ObRelativePose
{
	float   rx_;    // x方向旋转分量
	float   ry_;    // y方向旋转分量
	float   rz_;    // z方向旋转分量
	float   tx_;    // x方向平移分量
	float   ty_;    // y方向平移分量
	float   tz_;    // z方向平移分量
};
//
typedef struct {
	struct {
		struct {
			uint32_t buf_pos : 12;		//0-11
			uint32_t totalBufRows : 12;	//12-23
			uint32_t rsv0 : 6;			//24-29
			uint32_t dep_sign : 1;		//30
			uint32_t shft_dep : 1;		//31

			uint32_t iZ0;
			uint32_t PBF;
			uint32_t W[12];
		}dep2color;
		struct {
			uint32_t k[9];
			uint32_t r[9];
			uint32_t fx;
			uint32_t fy;
			uint32_t cx;
			uint32_t cy;
		}mdlCamL;
		struct {
			uint32_t k[9];
			uint32_t r[9];
			uint32_t fx;
			uint32_t fy;
			uint32_t cx;
			uint32_t cy;
		}mdlCamR;
		struct {
			uint32_t dY;
			uint32_t dT;
			uint32_t iRow;
			uint32_t img_size;
		}mdlRef;
		struct {
			uint32_t cam : 16;
			uint32_t ref : 16;
		}startPixBuf;
	}DPU;
	struct {
		struct {
			uint32_t w, h;
			float fx, fy, cx, cy, bl;
			float rotL[3];
			float rotR[3];
		}virCam;
		struct {
			ObCameraIntrinsic	irL;
			ObCameraIntrinsic	irR;
			ObCameraIntrinsic	rgb;
			ObRelativePose		irL_pose;
			ObRelativePose		irR_pose;
			ObRelativePose		rgb_pose;

		}camera_params;
		struct {
			float d_intr_p[4];//[fx,fy,cx,cy]
			float c_intr_p[4];//[fx,fy,cx,cy]
			float d2c_r[9];//[r00,r01,r02;r10,r11,r12;r20,r21,r22]
			float d2c_t[3];//[t1,t2,t3]
			float d_k[5];//[k1,k2,k3,p1,p2]
			float c_k[5];
			//float pixelsize;
		}soft_d2c;
	}HOST;

}ObContent_t;

typedef struct _cam_hdr{
    uint8_t magic[2];
    uint16_t len;
    uint16_t cmd;
    uint16_t tag;
} cam_hdr;
#if 0
enum EPsProtocolOpCodes
{

    OPCODE_GET_VERSION = 0,
    OPCODE_KEEP_ALIVE = 1,
    OPCODE_GET_PARAM = 2,
    OPCODE_SET_PARAM = 3,
    OPCODE_GET_FIXED_PARAMS = 4,
    OPCODE_GET_MODE = 5,
    OPCODE_SET_MODE = 6,
    OPCODE_GET_LOG = 7,
    OPCODE_RESERVED_0 = 8,
    OPCODE_RESERVED_1 = 9,
    OPCODE_I2C_WRITE = 10,
    OPCODE_I2C_READ = 11,
    OPCODE_TAKE_SNAPSHOT = 12,
    OPCODE_INIT_FILE_UPLOAD = 13,
    OPCODE_WRITE_FILE_UPLOAD = 14,
    OPCODE_FINISH_FILE_UPLOAD = 15,
    OPCODE_DOWNLOAD_FILE = 16,
    OPCODE_DELETE_FILE = 17,
    OPCODE_GET_FLASH_MAP = 18,
    OPCODE_GET_FILE_LIST = 19,
    OPCODE_READ_AHB = 20,
    OPCODE_WRITE_AHB = 21,
    OPCODE_ALGORITM_PARAMS = 22,
    OPCODE_SET_FILE_ATTRIBUTES = 23,
    OPCODE_EXECUTE_FILE = 24,
    OPCODE_READ_FLASH = 25,
    OPCODE_SET_GMC_PARAMS = 26,
    OPCODE_GET_CPU_STATS = 27,
    OPCODE_BIST = 28,
    OPCODE_CALIBRATE_TEC = 29,
    OPCODE_GET_TEC_DATA = 30,
    OPCODE_CALIBRATE_EMITTER = 31,
    OPCODE_GET_EMITTER_DATA = 32,
    OPCODE_CALIBRATE_PROJECTOR_FAULT = 33,
    OPCODE_SET_CMOS_BLANKING = 34,
    OPCODE_GET_CMOS_BLANKING = 35,
    OPCODE_GET_CMOS_PRESETS = 36,
    OPCODE_GET_SERIAL_NUMBER = 37,
    OPCODE_GET_FAST_CONVERGENCE_TEC = 38,
    OPCODE_GET_PLATFORM_STRING = 39,
    OPCODE_GET_USB_CORE_TYPE = 40,
    OPCODE_SET_LED_STATE = 41,
    OPCODE_ENABLE_EMITTER = 42,
    /* ORBBEC OPCODE GROUP */
    CMD_GET_VERSION = 80,
    CMD_SET_TEC_LASER = 81,
    CMD_GET_TEC = 82,
    CMD_READ_AHB = 83,
    CMD_WRITE_AHB = 84,
    CMD_ENABLE_EMITTER = 85,
    CMD_RUN_BIST = 86,
    CMD_GAIN_SET = 87,
    CMD_TEC_ENABLE = 88,
    CMD_RGB_SET_AEMODE = 89,
    CMD_RGB_SET_AE_WEIGHT = 90,
    CMD_RGB_CHANGE_CONFIG = 91,
    OPCODE_KILL = 999,
};
#endif

#if 0
enum OBExternsionID
{
    OBEXTENSION_ID_IR_GAIN = 0,
    OBEXTENSION_ID_LDP_EN,
    OBEXTENSION_ID_CAM_PARAMS,
    OBEXTENSION_ID_LASER_EN,
    OBEXTENSION_ID_SERIALNUMBER,
    OBEXTENSION_ID_DEVICETYPE
};
#endif

struct OBExtension
{
   uint16_t cam_tag;
   int vid;
   int pid;
};

#endif /*_OBEXTENSION_H_*/
