#ifndef _MX6000_CFG_H__
#define _MX6000_CFG_H__

#include <stdint.h>

#define MX6000_CFG_ADDRESS 0x10000
#define MX6000_CFG_SIZE 0x10000
#define MX6000_CODE_ADDRESS 0x20000
#define MX6000_CODE_SIZE 0x20000
#define MX6000_REFERENCE_ADDRESS 0xA0000
#define MX6000_SW_ALIGN_ADDRESS 0x70000
#define MX6000_CFG_D2C_ADDRESS 0x1000	//hareware d2c
#define MX6000_CFG_REC_ADDRESS 0x2000   //hareware distortion
#define MX6000_DEPTH_PARAMS_ADDRESS 0x14004
#define DEVICE_CONFIG_ADDR        0x30010000

#define MX6000_Z0_OFFSET 0x4008;
#define MX6000_BASELINE_OFFSET 0x4004;

//update mx400 firmware
#define MX400_CODE_ADDRESS 0x10000
#define MX400_CODE_SIZE 0x10000

#ifdef _MSC_VER
#pragma pack(push,4)
#endif

#ifdef _MSC_VER
#  define PACKED_STRUCT(name) \
    struct name
#elif defined(__GNUC__)
#  define PACKED_STRUCT(name) struct __attribute__((packed)) name
#endif

PACKED_STRUCT(RegVal){
	uint32_t reg;
	uint32_t val;
} ;

PACKED_STRUCT(CfgItem){
	//
	uint32_t def;
	uint32_t max;
	uint32_t min;
	uint32_t res;
};


PACKED_STRUCT(DeviceInfo)
{
	uint32_t vid_pid;
	uint8_t sn[12];
	uint8_t pn[12];
	uint32_t device_string[12];
	uint32_t ldp_enabl_flag;
};

PACKED_STRUCT(SensorCfg)
{
	CfgItem gain;
	CfgItem exposure;
};


PACKED_STRUCT(StreamCfg)
{
    uint32_t type;

};



PACKED_STRUCT(LaserCfg)
{
	uint32_t type;
	CfgItem current;
	CfgItem time;

};


PACKED_STRUCT(DepthCfg)
{

	uint32_t postfilter;
	uint32_t rectify_regs_num;
	uint32_t d2c_regs_num;
	uint32_t prefilter_regs_num;

	RegVal *rectify_regs_ptr;//44
	RegVal *d2c_regs_ptr;//15
	RegVal *prefilter_regs;//15

};


PACKED_STRUCT( DeviceCfg)
{
	uint32_t version;
	DeviceInfo *device_info_ptr;
	SensorCfg *sensor_cfg_ptr[3];
	StreamCfg *stream_cfg_ptr[3];
	DepthCfg   *depth_cfg_ptr;
	LaserCfg  *laser_cfg_ptr;
};


PACKED_STRUCT(protocol_header){
	XnUInt16 magic;
	XnUInt16 size;
	XnUInt16 opcode;
	XnUInt16 id;
} ;


//
#ifdef _MSC_VER
#pragma pack(pop)
#endif

#define CMD_HEADER_MAGIC	(0x4d47)
#define CMD_HEADER_LEN		(0x08)

#endif
