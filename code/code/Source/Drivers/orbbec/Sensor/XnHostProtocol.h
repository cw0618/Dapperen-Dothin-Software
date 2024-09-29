/*****************************************************************************
*                                                                            *
*  OpenNI 2.x Alpha                                                          *
*  Copyright (C) 2012 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  Licensed under the Apache License, Version 2.0 (the "License");           *
*  you may not use this file except in compliance with the License.          *
*  You may obtain a copy of the License at                                   *
*                                                                            *
*      http://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
*  Unless required by applicable law or agreed to in writing, software       *
*  distributed under the License is distributed on an "AS IS" BASIS,         *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and       *
*  limitations under the License.                                            *
*                                                                            *
*****************************************************************************/
#ifndef XNHOSTPROTOCOL_H
#define XNHOSTPROTOCOL_H

#include <XnStreamParams.h>
#include "XnParams.h"
#include "XnDeviceSensor.h"
#include "XnFirmwareTypes.h"
#include "tofinfo.h"
#include "OBCProperties.h"

#define XN_HOST_MAGIC_25        0x5053 //PS
#define XN_FW_MAGIC_25          0x5350 //SP
#define XN_HOST_MAGIC_26        0x4d47 //MG
#define XN_FW_MAGIC_26          0x4252 //BR

#define XN_FPGA_VER_FPDB_26     0x21
#define XN_FPGA_VER_FPDB_25     0x0
#define XN_FPGA_VER_CDB         0x1
#define XN_FPGA_VER_RD3         0x2
#define XN_FPGA_VER_RD5         0x3
#define XN_FPGA_VER_RD1081      0x4
#define XN_FPGA_VER_RD1082      0x5
#define XN_FPGA_VER_RD109       0x6

#define XN_CHIP_VER_PS1000      0x00101010
#define XN_CHIP_VER_PS1080      0x00202020
#define XN_CHIP_VER_PS1080A6    0x00212020

#define XN_CHIP_VER_MX6000      0x2BC50601
#define XN_CHIP_VER_DUAL_MX6000 0x2BC50602

#define EATCH_PACKET_SIZE 32


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
    OPCODE_SET_SERIAL_NUMBER = 43,
    OPCODE_GET_SENSOR_ID = 44,         ///< Get sensor ID by stream type.
    OPCODE_SEND_COMMAND = 45,          ///< Send command line to device.
    OPCODE_QUERY_TIMESTAMP = 46,       ///< Query device timestamp.

    OB_OPCODE_GET_PARAM = 102, //read register ObParam	 (irgain irExp)
    OB_OPCODE_SET_PARAM = 103, //write register ObParam (irgain irExp)
    OPCODE_ENABLE_IRFLOOD = 104,
    OB_OPCODE_SET_PARAM_FLASH = 105, //write flash ObParam
    OB_OPCODE_GET_PARAM_FLASH = 106, //read flash ObParam
    OB_OPCODE_CHANGE_SENSOR = 107, //ado project change sensor

    OB_OPCODE_SET_PUBLIC_KEY = 110, //set public key and batch version (falsh)
    OB_OPCODE_GET_PUBLIC_KEY = 111, //get public key and batch version (falsh)
    OB_OPCODE_GET_RANDOM_STRING = 112, //get random string and batch version
    OB_OPCODE_SET_VERIFY_RS = 113, //send verfiy r,s key
    OB_OPCODE_GET_IS_SUPPORT_LASER_SECURE = 114,  //is support laser secure?
    OB_OPCODE_SET_ENABLE_LASER_SECURE = 115, //enable laser secure
    OB_OPCODE_GET_ENABLE_LASER_SECURE = 116, //get laser secure status

    OB_OPCODE_SOFT_RESET = 120, //soft reset
    CMD_RGB_GET_AEMODE = 121, //get rgb ae mode
    OB_OPCODE_READ_SUBCMD_PARAM = 122, //read  ObParam
    OB_OPCODE_WRITE_SUBCMD_PARAM = 123, //write ObParam

    OB_OPCODE_OPTIM_READ_SUBCMD = 126,
    OB_OPCODE_OPTIM_WRITE_SUBCMD = 127,

    //switch dual camera left and right ir
    OB_OPCODE_SET_SWITCH_IR = 130,
    OB_OPCODE_GET_DUAL_PARAM = 131, //get dual camera param

    OB_OPCODE_GET_LDP = 132, //get ldp register
    OB_OPCODE_SET_LDP = 133, //set ldp register
    OB_OPCODE_GET_EMITTER = 134, //get emitter status
    OB_OPCODE_WRITE_QN = 135, //write QN
    OB_OPCODE_READ_QN = 136, //read QN
    OB_OPCODE_VERIFY_QN = 137, //Verify QN
    OPCODE_GET_MX6300_VERSION = 138,
    OB_OPCODE_SET_SERIAL_NUMBER = 139,//set serial number (S1)
    OB_OPCODE_WRITE_PN = 140,//write PN
    OB_OPCODE_READ_PN = 141, //read PN
    OB_OPCODE_GET_SERIAL_NUMBER = 142,//Get serial number (S1)
    OB_OPCODE_SET_AE_ENABLE = 143,//AE enabling settings
    OB_OPCODE_GET_AE_ENABLE = 144,//AE Enabled Query
    OB_OPCODE_SET_MIPI_TEST_ENABLE = 145,//set u1 mipi test mode
    OB_OPCODE_GET_MIPI_TEST_ENABLE = 146,//get u1 mipi test mode
    OB_OPCODE_GET_IR_MODEl = 147, //Get ir sensor model
    OB_OPCODE_GET_RGB_MODEl = 148, //Get rgb sensor model
    OB_OPCODE_I2C_READ_FLASH_MIPI = 149, //i2c read flash for mipi

    OB_OPCODE_IR_FLOOD_OPTION = 150,
    OB_OPCODE_AE_OPTION = 151,//used to read/set AE options
    OPCODE_GET_PUBLIC_BOARD_VERSION = 152, // get public board version
    OPCODE_GET_CORE_BOARD_FLASH_ID = 153, //get mipi core board flash id

    OB_OPCODE_GET_PD = 154, // Get information about PD parameters and states
    OB_OPCODE_SET_PD = 155, // Setting PD parameters and states

    //Bootloader protection
    OB_OPCODE_BOOTLOADER_PTS_GET = 156, // Bootloader protection status get
    OB_OPCODE_BOOTLOADER_PTS_SET = 157, // Bootloader protection status settings
    OB_OPCODE_CUP_CERTIFY_GET = 158, //china union pay certification
    OB_OPCODE_SET_DEPTH_NIR_MODE = 159, //set depth and NIR mode
    OB_OPCODE_GET_DEPTH_NIR_MODE = 160, //get depth and NIR mode
    OB_OPCODE_SET_SUBTRACT_BG_MODE = 161, //set depth and NIR mode
    OB_OPCODE_GET_SUBTRACT_BG_MODE = 162, //get depth and NIR mode

    //pyramid public board reserve[optcode 161 to 163]
    //OB_OPCODE_GET_PD_VOLT = 161,
    //OB_OPCODE_READ_CAMERA_PARAM = 162,
    //OB_OPCODE_ENABLE_ADB_MODE = 163,

    OPCODE_GENERAL_FILE_TRANSFER_PREPARE = 164,   ///< Notify the device that host is ready to start transferring file.
    OPCODE_GENERAL_FILE_TRANSFERRING = 165,       ///< File transferring.
    OPCODE_GENERAL_FILE_TRANSFER_FINISH = 166,    ///< Notify the device that the file transfer is complete.
    OPCODE_GET_PLATFORM_VERSION = 167,            ///< Get the platform version.
    OPCODE_GET_PLATFORM_SDK_VERSION = 168,        ///< Get the platform SDK version.
    OPCODE_STREAM_SET_QUERY = 169,                ///< Query the stream set supported by the device.
    OPCODE_GET_TOF_FREQ_MODE = 170,               ///< Get the frequency mode of TOF sensor.
    OPCODE_SET_TOF_FREQ_MODE = 171,               ///< Set the frequency mode for TOF sensor.
    OPCODE_START_SERVICE = 172,                   ///< Start or stop service.

    OB_OPCODE_GET_MOTOR_PARAM = 239,    ///< Get motor param
    OB_OPCODE_SET_MOTOR_PARAM = 240,    ///< Set motor param

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

    CMD_EXP_SET = 96,
    OB_OPCODE_SET_D2C_RESOLUTION = 99,
    OB_OPCODE_GET_D2C_RESOLUTION = 100,
    OB_OPCODE_GET_CFG_PRODUCT_NUMBER = 101,
    OB_OPCODE_SUPPORT_SUB_CMD = 998,
    OPCODE_KILL = 999,

    //TOF
    OB_OPCODE_TOF_SENSOR_GET = 0x1001, // TOF sensor get
    OB_OPCODE_TOF_SENSOR_SET = 0x1002, // TOF sensor set
	OB_OPCODE_SET_HDRMODE_ENABLE = 305,//hdrmode enabling settings
	OB_OPCODE_GET_HDRMODE_ENABLE = 306,//hdrmode Enabled Query
	OB_OPCODE_SET_TOF_SENSOR_FILTER_LEVEL = 307,
	OB_OPCODE_GET_TOF_SENSOR_FILTER_LEVEL = 308,
	OB_OPCODE_SET_TOF_SENSOR_INTEGRATION_TIME = 309,
	OB_OPCODE_GET_TOF_SENSOR_INTEGRATION_TIME = 310,
	OB_OPCODE_SET_TOF_SENSOR_GAIN = 311,
	OB_OPCODE_GET_TOF_SENSOR_GAIN = 312,
	OB_OPCODE_SET_LASER_INTERFERENCE_ENABLE = 313,   
	OB_OPCODE_GET_LASER_INTERFERENCE_ENABLE = 314,
	OB_OPCODE_SET_WORKING_MODE = 315,   
	OB_OPCODE_GET_WORKING_MODE = 316,
	OB_OPCODE_SET_FREQUENCY=317,
	OB_OPCODE_GET_FREQUENCY=318,
	OB_OPCODE_SET_DUTY_CYCLE=319,
	OB_OPCODE_GET_DUTY_CYCLE=320,
	OB_OPCODE_SET_DRIVER_IC_REG=321,
	OB_OPCODE_GET_DRIVER_IC_REG=322,
	OB_OPCODE_SET_SENSOR_REG=323,
	OB_OPCODE_GET_SENSOR_REG=324
};

typedef enum {
    OB_PARAM_IR_GAIN = 0xF1000000,
    OB_PARAM_IR_EXPOSURE = 0xF1000004,
    OB_PARAM_DEPTH_POSTFILTER_THRESHOLD = 0xF3000010,
    OB_PARAM_LASER_CURRENT = 0xF4000000,
    OB_PARAM_LASER_TIME = 0xF4000004,
    OB_PARAM_LDP_ENABLE = 0xF4000008,
    OB_PARAM_LASER_STATUS = 0xF4000012,
}ObParam;

typedef enum {
    OB_DUAL_PARAM_DISPARTY_COFF = 0,
}ObDualParam;

typedef enum{
    TEMPERATURE_DEFL_ENABLE = 0,       //Temperature compensation switch
    TEMPERATURE_DEFL_TIR = 1,          //IR real-time temperature reading
    TEMPERATURE_DEFL_TLDMP = 2,        //LDMP real-time temperature reading
    TEMPERATURE_DEFL_CAL_TIR = 3,      //Calibrated IR temperature (read and write)
    TEMPERATURE_DEFL_CAL_TLDMP = 4,    //Calibrated laser module temperature (read and write)
    TEMPERATURE_DEFL_NCOST_IR = 5,     //IR compensation coefficient temperature (read and write)
    TEMPERATURE_DEFL_NCOST_LDMP = 6,   //LDMP compensation coefficient temperature (read and write)
}SupportSubCmd;

typedef enum{
    DEPTH_OPTIMIZATION_ENABLE = 0,          //Depth optimization switch
    DEPTH_OPTIMIZATION_PARAM = 1,           //Depth optimization parameters (read and write)
    MULTI_DISTANCE_CALIBRATION_PARAM = 2,   //Multi distance calibration parameters, transmit the first buffer through optcode.
    MULTI_DISTANCE_CALIBRATION_ENABLE = 3,  //Multi distance calibration enable.
}OptimizationSubCmd;

typedef enum{
    LDP_ENABLE = 0,      //ldp enable (read and write)
    LDP_SCALE = 1,       //ldp scale (read)
    LDP_STATUS = 2,      //ldp status
    LDP_THRES_UP = 3,    //ldp thres up
    LDP_THRES_LOW = 4,   //ldp thres low
    LDP_NOISE_VALUE = 5, //ldp noise value
}LdpSubCmd;

typedef enum{
    PD_ENABLE = 0,            //PD enable status
    PD_ALERT_STATUS = 1,      //PD alarm status
    PD_HTH_THRESHOLD = 2,     //PD upper limit threshold
    PD_LTH_THRESHOLD = 3,     //PD lower threshold
    PD_CURRENT_THRESHOLD = 4, //PD current threshold
}PDSubCmd;


typedef enum
{
    IR_READ_FLOOD_ENABLE = 0,
    IR_READ_FLOOD_LEVEL = 1,
    IR_READ_FLOOD_TIMES = 2, //reverse
    IR_WRITE_FLOOD_ENABLE = 3,
    IR_WRITE_FLOOD_LVEL = 4,
    IR_WRITE_FLOOD_TIMES = 5 //reverse

} IrFloodSubCmd;

typedef enum
{
    PROTOCOL_TOF_ENABLE = 0,                 //TOF Enabling Switch (Read-Write Register)
    PROTOCOL_TOF_RESULT = 1,                 //TOF measurement results acquisition (read register)
    PROTOCOL_TOF_APP_ID = 2,                 //TOF APP ID Acquisition (Read Register)
    PROTOCOL_TOF_CALIBRATION = 3,            //TOF Factory Calibration (Write Register)
    PROTOCOL_TOF_APP_START = 4,              //TOF APP Start/Stop (Write Register)
    PROTOCOL_TOF_CALIBRATION_DATA = 5,       //TOF calibration parameters (read/write), size 14 bytes
} TofSupportSubCmd;

typedef enum
{
    PROTOCOL_STEP_MOTOR_TEST = 0,       // Motor test(Read-Write Register), reference enum MotorTestResult
    PROTOCOL_STEP_MOTOR_POSITION = 1,   // Motor position(Read-Write Register) reference enum MotorPosition
    PROTOCOL_STEP_MOTOR_STATUS = 2,     // Motor status(Read Register), reference enum MotorStatus
    PROTOCOL_STEP_MOTOR_TEST_COUNT = 3, // Motor test count(Read Register), reference enum MotorStatus
    PROTOCOL_STEP_MOTOR_RUN_TIME = 4,   // Motor run time  unit:ms
    PROTOCOL_STEP_MOTOR_FEATURE = 5,    // Motor hardware feature, 1: one section, 2: two section, for F210120
    PROTOCOL_STEP_MOTOR_UPDOWN_STATE = 6,   // Motor position, 0: original position, 4: last position, for F210120
    PROTOCOL_STEP_MOTOR_UPDOWN_TIME = 7,    // Motor run time, unit:s, length: 1byte, for F210120
    PROTOCOL_STEP_MOTOR_SET_UPDOWN = 8,     // Motor control, 1: arrive first section, 2: arrive second section, for F210120
} MotorSupportSubCmd;

typedef enum
{
    MOTOR_NOT_DONE = 0,     // Motor not tested (Read Register)
    MOTOR_TESTING = 1,      // Motor testing (Read-Write Register)
    MOTOR_TEST_SUCCESS = 2, // Motor test succesed (Read Register)
    MOTOR_TEST_FAILED = 3,  // Motor test failed (Read Register)
} MotorTestResult;

typedef enum
{
    MOTOR_INITIAL_POSITION = 0, // Motor middle position
    MOTOR_MIDDLE_POSITION = 1,  // Motor initial position 
    MOTOR_TALLEST_POSITION = 2, // Motor tallest position
} MotorPosition;

typedef enum
{
    MOTOR_WORK = 0,         // Motor working
    MOTOR_SLEEP = 1,        // Motor sleep
    MOTOR_MALFUNCTION = 2,  // Motor malfunction 
} MotorStatus;

typedef enum
{
    AE_GET_FLOOD_OPTION = 0,
    AE_SET_FLOOD_OPTION,
    AE_GET_EMITTER_OPTION,
    AE_SET_EMITTER_OPTION,
} AEOption;

enum EPsProtocolOpCodes_V300
{
    OPCODE_V300_BIST = 26,
};

enum XnHostProtocolOpcodes_V110
{
    OPCODE_V110_GET_VERSION = 0,
    OPCODE_V110_KEEP_ALIVE = 1,
    OPCODE_V110_GET_PARAM = 2,
    OPCODE_V110_SET_PARAM = 3,
    OPCODE_V110_GET_FIXED_PARAMS = 4,
    OPCODE_V110_GET_MODE = 5,
    OPCODE_V110_SET_MODE = 6,
    OPCODE_V110_GET_LOG = 7,
    OPCODE_V110_GET_CMOS_REGISTER = 8,
    OPCODE_V110_SET_CMOS_REGISTER = 9,
    OPCODE_V110_GET_CODEC_REGISTER = 10,
    OPCODE_V110_SET_CODEC_REGISTER = 11,
    OPCODE_V110_TAKE_SNAPSHOT = 12,
    OPCODE_V110_INIT_FILE_UPLOAD = 13,
    OPCODE_V110_WRITE_FILE_UPLOAD = 14,
    OPCODE_V110_FINISH_FILE_UPLOAD = 15,
    OPCODE_V110_DOWNLOAD_FILE = 16,
    OPCODE_V110_DELETE_FILE = 17,
    OPCODE_V110_GET_FLASH_MAP = 18,
    OPCODE_V110_GET_FILE_LIST = 19,
    OPCODE_V110_READ_AHB = 20,
    OPCODE_V110_WRITE_AHB = 21,
    OPCODE_V110_ALGORITHM_PARAMS = 22,
    OPCODE_V110_SET_FILE_ATTRIBUTES = 23,
    OPCODE_V110_EXECUTE_FILE = 24,
};

enum EPsProtocolOpCodes_V017
{
    OPCODE_V017_GET_VERSION = 0,
    OPCODE_V017_KEEP_ALIVE = 1,
    OPCODE_V017_GET_PARAM = 2,
    OPCODE_V017_SET_PARAM = 3,
    OPCODE_V017_GET_FIXED_PARAMS = 4,
    OPCODE_V017_RESET = 5,
    OPCODE_V017_GET_LOG = 6,
    OPCODE_V017_GET_CMOS_REGISTER = 7,
    OPCODE_V017_SET_CMOS_REGISTER = 8,
    OPCODE_V017_GET_CODEC_REGISTER = 9,
    OPCODE_V017_SET_CODEC_REGISTER = 10,
    OPCODE_V017_TAKE_SNAPSHOT = 11,
    OPCODE_V017_INIT_FILE_UPLOAD = 12,
    OPCODE_V017_WRITE_FILE_UPLOAD = 13,
    OPCODE_V017_FINISH_FILE_UPLOAD = 14,
    OPCODE_V017_DOWNLOAD_FILE = 15,
    OPCODE_V017_DELETE_FILE = 16,
    OPCODE_V017_GET_FLASH_MAP = 17,
    OPCODE_V017_GET_FILE_LIST = 18,
    OPCODE_V017_READ_AHB = 19,
    OPCODE_V017_WRITE_AHB = 20,
    OPCODE_V017_ALGORITM_PARAMS = 21,
};

#define OPCODE_INVALID 0xffff

typedef enum
{
    XN_HOST_PROTOCOL_ALGORITHM_DEPTH_INFO = 0x00,
    XN_HOST_PROTOCOL_ALGORITHM_REGISTRATION = 0x02,
    XN_HOST_PROTOCOL_ALGORITHM_PADDING = 0x03,
    XN_HOST_PROTOCOL_ALGORITHM_BLANKING = 0x06,
    XN_HOST_PROTOCOL_ALGORITHM_DEVICE_INFO = 0x07,
    XN_HOST_PROTOCOL_ALGORITHM_FREQUENCY = 0x80
} XnHostProtocolAlgorithmType;

typedef enum
{
    XN_HOST_PROTOCOL_MODE_WEBCAM = 0,
    XN_HOST_PROTOCOL_MODE_PS,
    XN_HOST_PROTOCOL_MODE_MAINTENANCE,
    XN_HOST_PROTOCOL_MODE_SOFT_RESET,
    XN_HOST_PROTOCOL_MODE_REBOOT,
    XN_HOST_PROTOCOL_MODE_SUSPEND,
    XN_HOST_PROTOCOL_MODE_RESUME,
    XN_HOST_PROTOCOL_MODE_INIT,
    XN_HOST_PROTOCOL_MODE_SYSTEM_RESTORE,
    XN_HOST_PROTOCOL_MODE_WAIT_FOR_ENUM,
    XN_HOST_PROTOCOL_MODE_SAFE_MODE
} XnHostProtocolModeType;

enum XnHostProtocolNacks
{
    ACK = 0,
    NACK_UNKNOWN_ERROR = 1,
    NACK_INVALID_COMMAND = 2,
    NACK_BAD_PACKET_CRC = 3,
    NACK_BAD_PACKET_SIZE = 4,
    NACK_BAD_PARAMS = 5,
    NACK_I2C_TRANSACTION_FAILED = 6,
    NACK_FILE_NOT_FOUND = 7,
    NACK_FILE_CREATE_FAILURE = 8,
    NACK_FILE_WRITE_FAILURE = 9,
    NACK_FILE_DELETE_FAILURE = 10,
    NACK_FILE_READ_FAILURE = 11,
    NACK_BAD_COMMAND_SIZE = 12,
    NACK_NOT_READY = 13,
    NACK_OVERFLOW = 14,
    NACK_OVERLAY_NOT_LOADED = 15,
    NACK_FILE_SYSTEM_LOCKED = 16,
    NACK_NOT_WRITE_PUBLIC_KEY = 17,
    NACK_PUBLIC_KEY_MD5_VERIFY_FAILED = 18,
    NACK_NOT_WRITE_MD5 = 19
};

typedef enum
{
    A2D_SAMPLE_RATE_48KHZ,
    A2D_SAMPLE_RATE_44KHZ,
    A2D_SAMPLE_RATE_32KHZ,
    A2D_SAMPLE_RATE_24KHZ,
    A2D_SAMPLE_RATE_22KHZ,
    A2D_SAMPLE_RATE_16KHZ,
    A2D_SAMPLE_RATE_12KHZ,
    A2D_SAMPLE_RATE_11KHZ,
    A2D_SAMPLE_RATE_8KHZ,
    A2D_NUM_OF_SAMPLE_RATES
} EA2d_SampleRate;

typedef enum XnHostProtocolUsbCore
{
    XN_USB_CORE_JANGO = 0,
    XN_USB_CORE_GADGETFS = 1,
} XnHostProtocolUsbCore;

#pragma pack(push, 1)
typedef struct
{
    XnUInt16 nMagic;
    XnUInt16 nSize;
    XnUInt16 nOpcode;
    XnUInt16 nId;
    XnUInt16 nCRC16;
} XnHostProtocolHeaderV25;

typedef struct
{
    XnUInt16 nMagic;
    XnUInt16 nSize;
    XnUInt16 nOpcode;
    XnUInt16 nId;
} XnHostProtocolHeaderV26;

typedef struct
{
    XnUInt16 nMagic;
    XnUInt32 nSize;
    XnUInt16 nOpcode;
    XnUInt16 nId;
} XnHostProtocolBulkHeaderV26;

typedef struct
{
    XnUInt16 nErrorCode;
} XnHostProtocolReplyHeader;

typedef XnDeviceSensorGMCPoint XnHostProtocolGMCPoint_1080;

typedef struct XnHostProtocolGMCPoint_1000
{
    XnDeviceSensorGMCPoint m_GMCPoint;
    XnUInt16 m_Dummy;
} XnHostProtocolGMCPoint_1000;

typedef struct XnHostProtocolGMCLastConfData
{
    XnInt16 nLast;
    XnUInt16 nRICCLast;
    XnFloat fRICC_IIR;
} XnHostProtocolGMCLastConfData;

typedef enum XnHostProtocolGMCMode
{
    GMC_NORMAL_MODE = 0,
    GMC_SCAN_MODE
} XnHostProtocolGMCMode;

typedef struct XnHostProtocolGMCLastPacketData
{
    XnUInt16 m_GMCMode;
    XnUInt16 m_CoveragePass;
    XnHostProtocolGMCLastConfData m_LastConfData;
    XnFloat m_A;
    XnFloat m_B;
    XnFloat m_C;
    XnInt16 m_N;
    XnUInt16 m_RICC;
    XnUInt32 m_StartB;
    XnUInt32 m_DeltaB;
    XnInt16 m_FlashStoredRefOffset;
} XnHostProtocolGMCLastPacketData;

typedef struct XnBestTecConf
{
    XnUInt16 nBestHopsCount; // Lowest hops count among all unstable points
    XnUInt32 nBestSetPoint;  // The TEC set point that gave m_BestHopsCount
    XnInt32  nBestStep;	 // The TEC scan step that gave m_BestHopsCount
} XnBestTecConf;

typedef struct XnWavelengthCorrectionDebugPacket
{
    XnFloat fBLast;
    XnFloat fBCurrent;
    XnUInt16 nIsHop;
    XnUInt32 nCurrentSlidingWindow;
    XnUInt16 nCurrentHopsCount;
    XnUInt16 nIsTecCalibrated;
    XnUInt32 nWaitPeriod;
    XnUInt16 nIsWavelengthUnstable;
    XnBestTecConf BestConf;
    XnUInt16 nIsTotallyUnstable; //whole scan no stable point
    XnUInt32 nConfiguredTecSetPoint; // 0 if not configured
    XnInt32 nCurrentStep;
} XnWavelengthCorrectionDebugPacket;

typedef struct XnSupportSubCmdValue
{
    XnUInt8 nSupportRead;     //Whether to support reading; 1 to express support; 0: to express no support.
    XnUInt8 nSupportWrite;    //Whether to support writing; 1 to express support; 0: to express no support.
    XnUInt8 nSupportRange;    //Whether to support access scope; 1 support; 0: express no support.
    XnUInt8 nPlaceholder;
    XnInt32 nMinValue;        //The minimum range of range. NSupportRange=1 is effective.
    XnInt32 nMaxValue;        //The maximum range. NSupportRange=1 takes effect.
    XnInt32 nReservered1;     //4 bytes are reserved.
} XnSupportSubCmdValue;

#pragma pack(pop)



////////////////////////////////////// Exported h file should be only from here down
// Exported params

// All implemented protocol commands
// Init
XnStatus XnHostProtocolInitFWParams(XnDevicePrivateData* pDevicePrivateData, XnUInt8 nMajor, XnUInt8 nMinor, XnUInt16 nBuild, XnHostProtocolUsbCore usb, XnBool bGuessed);

XnStatus XnHostProtocolKeepAlive(XnDevicePrivateData* pDevicePrivateData);
XnStatus XnHostProtocolGetVersion(const XnDevicePrivateData* pDevicePrivateData, XnVersions& Version);
XnStatus XnHostProtocolAlgorithmParams(XnDevicePrivateData* pDevicePrivateData, XnHostProtocolAlgorithmType eAlgorithmType, void* pAlgorithmInformation, XnUInt16 nAlgInfoSize, XnResolutions nResolution, XnUInt16 nFPS);
XnStatus XnHostProtocolSetImageResolution(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nResolutionParamName, XnResolutions nRes);
XnStatus XnHostProtocolSetDepthResolution(XnDevicePrivateData* pDevicePrivateData, XnResolutions nRes);
XnStatus XnHostProtocolGetFixedParams(XnDevicePrivateData* pDevicePrivateData, XnFixedParams& FixedParams);

XnStatus XnHostProtocolSetAudioSampleRate(XnDevicePrivateData* pDevicePrivateData, XnSampleRate nSampleRate);
XnStatus XnHostProtocolGetAudioSampleRate(XnDevicePrivateData* pDevicePrivateData, XnSampleRate* pSampleRate);

XnStatus XnHostProtocolSetMode(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nMode);
XnStatus XnHostProtocolGetMode(XnDevicePrivateData* pDevicePrivateData, XnUInt16& nMode);

XnStatus XnHostProtocolSetParam(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nParam, XnUInt16 nValue);
XnStatus XnHostProtocolSetMultipleParams(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nNumOfParams, XnInnerParamData* anParams);
XnStatus XnHostProtocolReset(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nResetType);

XnStatus XnHostProtocolGetParam(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nParam, XnUInt16& nValue);

XnStatus XnHostProtocolSetDepthAGCBin(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nBin, XnUInt16 nMinShift, XnUInt16 nMaxShift);
XnStatus XnHostProtocolGetDepthAGCBin(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nBin, XnUInt16* pnMinShift, XnUInt16* pnMaxShift);

XnStatus XnHostProtocolSetCmosBlanking(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nLines, XnCMOSType nCMOSID, XnUInt16 nNumberOfFrames);
XnStatus XnHostProtocolGetCmosBlanking(XnDevicePrivateData* pDevicePrivateData, XnCMOSType nCMOSID, XnUInt16* pnLines);

XnStatus XnHostProtocolGetStreamSet(XnDevicePrivateData* pDevicePrivateData);
XnStatus XnHostProtocolGetCmosPresets(XnDevicePrivateData* pDevicePrivateData, XnCMOSType nCMOSID, XnCmosPreset* aPresets, XnUInt32& nCount);

XnStatus XnHostProtocolGetSerialNumber(XnDevicePrivateData* pDevicePrivateData, XnChar* cpSerialNumber);

XnStatus XnHostProtocolGetPlatformString(XnDevicePrivateData* pDevicePrivateData, XnChar* cpPlatformString);
XnStatus XnHostProtocolGetCfgProductNumber(XnDevicePrivateData* pDevicePrivateData, XnChar* cfgProductNumber);

XnStatus XnHostProtocalGetIRSensorModel(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &irModel);
XnStatus XnHostProtocalGetRgbSensorModel(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &rgbModel);

XnStatus XnHostProtocolGetCMOSRegister(XnDevicePrivateData* pDevicePrivateData, XnCMOSType nCMOS, XnUInt16 nAddress, XnUInt16& nValue);
XnStatus XnHostProtocolSetCMOSRegister(XnDevicePrivateData* pDevicePrivateData, XnCMOSType nCMOS, XnUInt16 nAddress, XnUInt16 nValue);
XnStatus XnHostProtocolGetCMOSRegisterI2C(XnDevicePrivateData* pDevicePrivateData, XnCMOSType nCMOS, XnUInt16 nAddress, XnUInt16& nValue);
XnStatus XnHostProtocolSetCMOSRegisterI2C(XnDevicePrivateData* pDevicePrivateData, XnCMOSType nCMOS, XnUInt16 nAddress, XnUInt16 nValue);
XnStatus XnHostProtocolReadAHB(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nAddress, XnUInt32 &nValue);
XnStatus XnHostProtocolWriteAHB(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nAddress, XnUInt32 nValue, XnUInt32 nMask);
XnStatus XnHostProtocolGetUsbCoreType(XnDevicePrivateData* pDevicePrivateData, XnHostProtocolUsbCore& nValue);
XnStatus XnHostProtocolSetLedState(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nLedId, XnUInt16 nState);
XnStatus XnHostProtocolSetEmitterState(XnDevicePrivateData* pDevicePrivateData, XnBool bActive);
XnStatus XnHostProtocolGetEmitterState(XnDevicePrivateData* pDevicePrivateData, XnUInt16* pState);
XnStatus XnHostProtocolSetIrfloodState(XnDevicePrivateData* pDevicePrivateData, XnBool bActive);

//set irgain
XnStatus XnHostProtocolSetIrGain(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nValue);
XnStatus XnHostProtocolGetIrGain(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue);

//ir exp
XnStatus XnHostProtocolSetIrExp(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nValue);
XnStatus XnHostProtocolGetIrExp(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue);

//ado change sensor
XnStatus XnHostProtocolSetChangeSensor(XnDevicePrivateData* pDevicePrivateData, XnBool bActive);

//set public key
XnStatus XnHostProtocolSetPublicKey(XnDevicePrivateData* pDevicePrivateData, const OBEccVerify* pPublicKey);

//get public key
XnStatus XnHostProtocolGetPublicKey(XnDevicePrivateData* pDevicePrivateData, OBEccVerify* pPublicKey);

//set RS key
XnStatus XnHostProtocolSetRSKey(XnDevicePrivateData* pDevicePrivateData, const OBEccRSKey* pRSKey);

//get random string
XnStatus XnHostProtocolGetRandomString(XnDevicePrivateData* pDevicePrivateData, OBEccInit* pInitString);

//laser secure
//nValue :1:support ,0:not support
XnStatus XnHostProtocolIsSupportLaserSecure(XnDevicePrivateData* pDevicePrivateData, XnBool &bIsSupport);

//set laser secure status :bStatus=1 (enable laser secure),bStatus=0 (disable laser secure)
XnStatus XnHostProtocolSetLaserSecureStatus(XnDevicePrivateData* pDevicePrivateData, XnBool bStatus);
XnStatus XnHostProtocolGetLaserSecureStatus(XnDevicePrivateData* pDevicePrivateData, XnBool &bStatus);

//set laser current
XnStatus XnHostProtocolSetLaserCurrent(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nValue);
XnStatus XnHostProtocolGetLaserCurrent(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue);

//soft reset
XnStatus XnHostProtocolSoftReset(XnDevicePrivateData* pDevicePrivateData);
//set switch dual left and right ir (0:left ir, 1:right ir)
XnStatus XnHostProtocolSetSwitchIr(XnDevicePrivateData* pDevicePrivateData, XnBool bActive);

//dual camera get DisparityCoeff params
XnStatus XnHostProtocolGetDisparityCoeff(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nParam, XnFloat& fDisparityCoeff);

//set rgb ae mode
XnStatus XnHostProtocolSetRgbAeMode(XnDevicePrivateData* pDevicePrivateData, const XnRgbAeMode* pRgbAeMode);

//get rgb ae mode
XnStatus XnHostProtocolGetRgbAeMode(XnDevicePrivateData* pDevicePrivateData, XnRgbAeMode* pRgbAeMode);

//get support sub cmd value
XnStatus XnHostProtocolGetSupportSubCmdValue(XnDevicePrivateData* pDevicePrivateData, XnUInt32 subcmd, XnDouble& nValue);
//set support sub cmd value
XnStatus XnHostProtocolSetSupportSubCmdValue(XnDevicePrivateData* pDevicePrivateData, XnUInt32 subcmd, XnDouble nValue);

//Temperature compensation switch setting (0 :disable,1:enable)
XnStatus XnHostProtocolTemperatureCompSwitch(XnDevicePrivateData* pDevicePrivateData, XnBool bActive);

//Temperature compensation status get(0 :disable,1:enable)
XnStatus XnHostProtocolGetTemperatureCompStatus(XnDevicePrivateData* pDevicePrivateData, XnBool &bActive);

//Depth optimization enable switch(0 :disable,1:enable)
XnStatus XnHostProtocolDepthOptimSwitch(XnDevicePrivateData* pDevicePrivateData, XnBool bActive);

//Depth optimization enable get(0 :disable,1:enable)
XnStatus XnHostProtocolGetDepthOptimStatus(XnDevicePrivateData* pDevicePrivateData, XnBool &bActive);

//get optimization param
XnStatus XnHostProtocolGetDepthOptimizationParam(XnDevicePrivateData* pDevicePrivateData, XnDepthOptimizationParam* depthOptimParam);
//Set optimization param
XnStatus XnHostProtocolSetDepthOptimizationParam(XnDevicePrivateData* pDevicePrivateData, const XnDepthOptimizationParam* depthOptimParam);

//Support  SubCmd
XnStatus XnHostProtocolSupportSubCmdMode(XnDevicePrivateData* pDevicePrivateData, XnUInt16 opcode, XnUInt32 subCmd, XnSupportSubCmdValue* supportValue);

//Ldp enable
XnStatus XnHostProtocolSetLdpEnable(XnDevicePrivateData* pDevicePrivateData, XnBool bActive);
XnStatus XnHostProtocolSetLdpEnableV1(XnDevicePrivateData* pDevicePrivateData, XnBool bActive);

XnStatus XnHostProtocolGetLdpEnable(XnDevicePrivateData* pDevicePrivateData, XnBool &bActive);
XnStatus XnHostProtocolGetLdpEnableV1(XnDevicePrivateData* pDevicePrivateData, XnBool &bActive);

XnStatus XnHostProtocolSetLdpScaleV1(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nScale);
XnStatus XnHostProtocolGetLdpScaleV1(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nScale);

XnStatus XnHostProtocolGetLdpStatusV1(XnDevicePrivateData* pDevicePrivateData, XnBool &bActive);
XnStatus XnHostProtocolGetLdpThresUpV1(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &value);
XnStatus XnHostProtocolSetLdpThresUpV1(XnDevicePrivateData* pDevicePrivateData, XnUInt32 value);
XnStatus XnHostProtocolGetLdpThresLowV1(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &value);
XnStatus XnHostProtocolSetLdpThresLowV1(XnDevicePrivateData* pDevicePrivateData, XnUInt32 value);
XnStatus XnHostProtocolGetLdpNoiseValueV1(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &value);

//get emitter status
XnStatus XnHostProtocolGetEmitterEnable(XnDevicePrivateData* pDevicePrivateData, XnBool &bActive);
XnStatus XnHostProtocolGetEmitterEnableV1(XnDevicePrivateData* pDevicePrivateData, XnBool &bActive);

//Set/Get auto ae
XnStatus XnHostProtocolSetAeEnable(XnDevicePrivateData* pDevicePrivateData, XnBool bActive);
XnStatus XnHostProtocolGetAeEnable(XnDevicePrivateData* pDevicePrivateData, XnBool &bActive);

XnStatus XnHostProtocolSetHdrModeEnable(XnDevicePrivateData* pDevicePrivateData, XnBool bActive);
XnStatus XnHostProtocolGetHdrModeEnable(XnDevicePrivateData* pDevicePrivateData, XnBool &bActive);
XnStatus XnHostProtocolSetMipiTestEnable(XnDevicePrivateData* pDevicePrivateData, XnBool bActive);
XnStatus XnHostProtocolGetMipiTestEnable(XnDevicePrivateData* pDevicePrivateData, XnBool &bActive);

XnStatus XnHostProtocolI2CReadFlash(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nOffset, XnUInt32 nSize, XnUChar* pBuffer);

XnStatus XnHostProtocolUpdateSupportedImageModes(XnDevicePrivateData* pDevicePrivateData);
XnStatus XnHostProtocolUpdateSupportedDepthModes(XnDevicePrivateData* pDevicePrivateData);
XnStatus XnHostProtocolUpdateSupportedIRModes(XnDevicePrivateData* pDevicePrivateData);
XnStatus XnHostProtocolUpdateSupportedPhaseModes(XnDevicePrivateData* pDevicePrivateData);
XnStatus XnHostProtocolUpdateSupportedAIModes(XnDevicePrivateData* pDevicePrivateData);

//Write/Read distortion param from flash
XnStatus XnHostProtocolWriteDistortionParam(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nSize, XnUChar* pBuffer);
XnStatus XnHostProtocolReadDistortionParam(XnDevicePrivateData* pDevicePrivateData, XnUInt32& nSize, XnUChar* pBuffer);

XnStatus XnHostProtocolDistortionStateSwitch(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nActive);
XnStatus XnHostProtocolGetDistortionState(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nActive);

//PD
XnStatus XnHostProtocolGetPdEnableStatus(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nActive);
XnStatus XnHostProtocolSetPdEnableStatus(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nActive);
XnStatus XnHostProtocolGetPdAlertStatus(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nActive);
XnStatus XnHostProtocolGetPdUpperTlv(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nActive);
XnStatus XnHostProtocolSetPdUpperTlv(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nActive);
XnStatus XnHostProtocolGetPdLowerTlv(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nActive);
XnStatus XnHostProtocolSetPdLowerTlv(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nActive);
XnStatus XnHostProtocolGetPdCurrentThreshold(XnDevicePrivateData* pDevicePrivateData, OBPdThreshold* pd);
XnStatus XnHostProtocolSetBootLoaderPtsStatus(XnDevicePrivateData* pDevicePrivateData, XnBool bActive);
XnStatus XnHostProtocolGetBootLoaderPtsStatus(XnDevicePrivateData* pDevicePrivateData, XnBool &nActive);

//
//set laser time
XnStatus XnHostProtocolSetLaserTime(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nValue);
XnStatus XnHostProtocolGetLaserTime(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue);

XnStatus XnHostProtocolSetPostFilterThreshold(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nValue);
XnStatus XnHostProtocolGetPostFilterThreshold(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue);

// Commands.txt
XnStatus XnHostProtocolGetLog(XnDevicePrivateData* pDevicePrivateData, XnChar* csBuffer, XnUInt32 nBufferSize);
XnStatus XnHostProtocolFileUpload(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nOffset, const XnChar* strFileName, XnUInt16 nAttributes);
XnStatus XnHostProtocolFileDownload(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nFileType, const XnChar* strFileName);
XnStatus XnHostProtocolReadFlash(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nOffset, XnUInt32 nSize, XnUChar* pBuffer);
XnStatus XnHostProtocolRunBIST(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nTestsMask, XnUInt32* pnFailures);
XnStatus XnHostProtocolGetCPUStats(XnDevicePrivateData* pDevicePrivateData, XnTaskCPUInfo* pTasks, XnUInt32 *pnTimesCount);
XnStatus XnHostProtocolCalibrateTec(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nSetPoint);
XnStatus XnHostProtocolGetTecData(XnDevicePrivateData* pDevicePrivateData, XnTecData* pTecData);
XnStatus XnHostProtocolGetTecFastConvergenceData(XnDevicePrivateData* pDevicePrivateData, XnTecFastConvergenceData* pTecData);
XnStatus XnHostProtocolCalibrateEmitter(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nSetPoint);
XnStatus XnHostProtocolGetEmitterData(XnDevicePrivateData* pDevicePrivateData, XnEmitterData* pEmitterData);
XnStatus XnHostProtocolCalibrateProjectorFault(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nMinThreshold, XnUInt16 nMaxThreshold, XnBool* pbProjectorFaultEvent);
XnStatus XnHostProtocolGetFileList(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nFirstFileId, XnFlashFile* pFileList, XnUInt16& nNumOfEntries);
XnStatus XnHostProtocolDeleteFile(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nFileId);
XnStatus XnHostProtocolWriteI2C(XnDevicePrivateData* pDevicePrivateData, const XnI2CWriteData* pI2CWriteData);
XnStatus XnHostProtocolReadI2C(XnDevicePrivateData* pDevicePrivateData, XnI2CReadData* pI2CReadData);
XnStatus XnHostProtocolSetFileAttributes(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nFileId, XnUInt16 nAttributes);

XnStatus XnHostProtocolSetFirmwareQN(XnDevicePrivateData* pDevicePrivateData, const OBFirmwareQN* qN);
XnStatus XnHostProtocolGetFirmwareQN(XnDevicePrivateData* pDevicePrivateData, OBFirmwareQN* qN);
XnStatus XnHostProtocolVerifyQN(XnDevicePrivateData* pDevicePrivateData, const OBFirmwareQN* qN);
XnStatus XnHostProtocolGetPublicBoardVersion(const XnDevicePrivateData* pDevicePrivateData, OBPublicBoardVersion* pVersion);
XnStatus XnHostProtocolMx6300FirmewarGetVersion(const XnDevicePrivateData* pDevicePrivateData, ObMX6300Version* pVersion);

XnStatus XnHostProtocolSetD2CResolution(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nResolution);
XnStatus XnHostProtocolGetD2CResolution(XnDevicePrivateData* pDevicePrivateData, XnUInt16 &nValue);

XnStatus XnHostProtocolGetUsbDeviceSpeed(XnDevicePrivateData* pDevicePrivateData, XnUInt16 &nValue);

XnStatus XnHostProtocolSetSerialNumber(XnDevicePrivateData* pDevicePrivateData, const OBSerialNumber* sN);
XnStatus XnHostProtocolGetSerialNumber(XnDevicePrivateData* pDevicePrivateData, OBSerialNumber* sN);


/*flood*/

//get
XnStatus XnHostProtocolGeminiGetIrFloodSwitchState(XnDevicePrivateData* pDevicePrivateData, XnUInt32* c_pValue);
XnStatus XnHostProtocolGeminiGetIrFloodLevelState(XnDevicePrivateData* pDevicePrivateData, XnUInt32* c_pValue);
XnStatus XnHostProtocolGeminiGetIrFloodState(XnDevicePrivateData* pDevicePrivateData, IrFloodSubCmd nSubType, XnUInt32* c_pValue);

//set
XnStatus XnHostProtocolGeminiSetIrFloodSwitchState(XnDevicePrivateData* pDevicePrivateData, const XnUInt32& c_nValue);
XnStatus XnHostProtocolGeminiSetIrFloodLevelState(XnDevicePrivateData* pDevicePrivateData, const XnUInt32& c_nValue);
XnStatus XnHostProtocolGeminiSetIrFloodState(XnDevicePrivateData* pDevicePrivateData, IrFloodSubCmd nSubType, const XnUInt32& c_nValue);

XnStatus XnHostProtocolSetKT_PN(XnDevicePrivateData* pDevicePrivateData, const OBKTProductNumber* pN);
XnStatus XnHostProtocolGetKT_PN(XnDevicePrivateData* pDevicePrivateData, OBKTProductNumber* pN);

XnStatus XnHostProtocolI2CReadFlashOnce(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nOffset, XnUInt16 nSize, XnUChar* pBuffer);
XnStatus XnHostProtocolI2CReadFlash(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nOffset, XnUInt32 nSize, XnUChar* pBuffer);


//AE
XnStatus XnHostProtocolGetAEOption(XnDevicePrivateData* pDevicePrivateData, AEOption nSubType, AeParamsStruct* pValue);
XnStatus XnHostProtocolSetAEOption(XnDevicePrivateData* pDevicePrivateData, AEOption nSubType, AeParamsStruct* pValue);

//get mipi project core broad status
XnStatus XnHostProtocolGetCoreBroadFlashId(const XnDevicePrivateData* pDevicePrivateData, XnUInt32 & nBroadId);

//Tof
XnStatus XnHostProtocolSetTofSensorEnable(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nValue);
XnStatus XnHostProtocolGetTofSensorEnable(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue);
XnStatus XnHostProtocolGetTofSensorMeasureResult(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue);
XnStatus XnHostProtocolGetTofSensorAppId(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue);
XnStatus XnHostProtocolSetTofSensorCalibrationValue(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nValue);
XnStatus XnHostProtocolSetTofSensorAppEnableState(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nValue);
XnStatus XnHostProtocolGetCupVerifyVersion(const XnDevicePrivateData* pDevicePrivateData, CupCertify* pCupCertify);
XnStatus XnHostProtocolSetTofSensorCalibrationParams(XnDevicePrivateData* pDevicePrivateData, const OBTofSensorCalParams* tofCalParams);
XnStatus XnHostProtocolGetTofSensorCalibrationParams(XnDevicePrivateData* pDevicePrivateData, OBTofSensorCalParams* tofCalParams);

//Motor
XnStatus XnHostProtocolSetMotorTest(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nValue);
XnStatus XnHostProtocolGetMotorTestResult(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue);
XnStatus XnHostProtocolSetMotorPosition(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nValue);
XnStatus XnHostProtocolGetMotorPosition(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue);
XnStatus XnHostProtocolGetMotorStatus(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue);
XnStatus XnHostProtocolGetMotorTestCount(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue);
XnStatus XnHostProtocolSetMotorRunTime(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nValue);
XnStatus XnHostProtocolGetMotorRunTime(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue);
XnStatus XnHostProtocolGetMotorFeature(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue);
XnStatus XnHostProtocolGetMotorUpdownState(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue);
XnStatus XnHostProtocolGetMotorUpdownTime(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue);
XnStatus XnHostProtocolSetMotorUpdown(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nValue);

//set depthir mode
XnStatus XnHostProtocolSetDepthIrMode(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nValue);
XnStatus XnHostProtocolGetDepthIrMode(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue);

XnStatus XnHostProtocolSetTecEnable(XnDevicePrivateData* pDevicePrivateData, XnBool bActive);

XnStatus XnHostProtocolSetSubtractBGMode(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nValue);
XnStatus XnHostProtocolGetSubtractBGMode(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue);

XnStatus XnHostProtocolSendUsbFile(XnDevicePrivateData* pDevicePrivateData, const XnUsbGeneralFile* pUsbFile);
XnStatus XnHostProtocolBulkDataSend(XnDevicePrivateData* pDevicePrivateData, const XnUChar* pData, const XnUInt32 nSize, const XnUInt16 opCode);
XnStatus XnHostProtocolBulkDataUploadState(XnDevicePrivateData* pDevicePrivateData, const XnUInt16 opCode, XnUInt32* pState);

XnStatus XnHostProtocolStartService(XnDevicePrivateData* pDevicePrivateData, const OniService* pService);

XnStatus XnHostProtocolGetTOFFreqMode(XnDevicePrivateData* pDevicePrivateData, XnUInt16* pMode);
XnStatus XnHostProtocolSetTOFFreqMode(XnDevicePrivateData* pDevicePrivateData, const XnUInt16 mode);

XnStatus XnHostProtocolGetTOFSensorFilterLevel(XnDevicePrivateData* pDevicePrivateData, XnUInt16* level);
XnStatus XnHostProtocolSetTOFSensorFilterLevel(XnDevicePrivateData* pDevicePrivateData, const XnUInt16 level);
XnStatus XnHostProtocolGetTOFSensorIntegrationTime(XnDevicePrivateData* pDevicePrivateData, XnUInt32* level);
XnStatus XnHostProtocolSetTOFSensorIntegrationTime(XnDevicePrivateData* pDevicePrivateData, const XnUInt32 level);
XnStatus XnHostProtocolGetTOFSensorGain(XnDevicePrivateData* pDevicePrivateData, XnUInt16* value);
XnStatus XnHostProtocolSetTOFSensorGain(XnDevicePrivateData* pDevicePrivateData, const XnUInt16 value);
XnStatus XnHostProtocolGetTOFSensorLaserInterference(XnDevicePrivateData* pDevicePrivateData, XnUInt16* value);
XnStatus XnHostProtocolSetTOFSensorLaserInterference(XnDevicePrivateData* pDevicePrivateData, const XnUInt16 value);
XnStatus XnHostProtocolGetTOFSensorWorkingMode(XnDevicePrivateData* pDevicePrivateData, XnUInt16* value);
XnStatus XnHostProtocolSetTOFSensorWorkingMode(XnDevicePrivateData* pDevicePrivateData, const XnUInt16 value);
XnStatus XnHostProtocolGetGeneralFrequency(XnDevicePrivateData* pDevicePrivateData,ORBTofFrequency* pData);
XnStatus XnHostProtocolSetGeneralFrequency(XnDevicePrivateData* pDevicePrivateData, const ORBTofFrequency* pData);
XnStatus XnHostProtocolGetGeneralDutyCycle(XnDevicePrivateData* pDevicePrivateData, ORBTofDuty* pData);
XnStatus XnHostProtocolSetGeneralDutyCycle(XnDevicePrivateData* pDevicePrivateData, const ORBTofDuty* pData);
XnStatus XnHostProtocolGetGeneralDriverICReg(XnDevicePrivateData* pDevicePrivateData, ObReg8Map* pData);
XnStatus XnHostProtocolSetGeneralDriverICReg(XnDevicePrivateData* pDevicePrivateData, const ObReg8Map* pData);
XnStatus XnHostProtocolGetGeneralSensorReg(XnDevicePrivateData* pDevicePrivateData, ObReg16Map* pData);
XnStatus XnHostProtocolSetGeneralSensorReg(XnDevicePrivateData* pDevicePrivateData, const ObReg16Map* pData);
XnStatus XnHostProtocolGetSensorID(XnDevicePrivateData* pDevicePrivateData, OniSensorIDMap* pSensorID);
XnStatus XnHostProtocolGetGeneralSerialNumber(XnDevicePrivateData* pDevicePrivateData, OniSerialNumberMap* pSerialMap);
XnStatus XnHostProtocolSetGeneralSerialNumber(XnDevicePrivateData* pDevicePrivateData, const OniSerialNumberMap* pSerialMap);
XnStatus XnHostProtocolGetPlatformVersion(XnDevicePrivateData* pDevicePrivateData, const XnUInt16 opCode, XnChar* pVersion);

XnStatus XnHostProtocolSendCommand(XnDevicePrivateData* pDevicePrivateData, OniSerialCmd* pCmd);
XnStatus XnHostProtocolQueryTimestamp(XnDevicePrivateData* pDevicePrivateData, XnUInt64* pTimestamp);

#endif // XNHOSTPROTOCOL_H
