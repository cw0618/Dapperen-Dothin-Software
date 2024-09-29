#ifndef _OBC_TEE_FUNCS_H_
#define _OBC_TEE_FUNCS_H_
#include <stdint.h>

#ifdef __cplusplus
extern "C"{
#endif


typedef void(*func_ptr_void_uint8_vr)(uint8_t lv, const char * fmt, ...);
typedef void * (*func_ptr_voidp_uint64)(uint64_t len);
typedef void(*func_ptr_void_voidp)(void * ptr);
typedef int(*func_ptr_int_uint8p_uint32)(uint8_t * buf, uint32_t size);
typedef void(*func_ptr_void_uint64)(uint32_t usec);

typedef int(*func_ptr_int32_void_int32_int32)(void * config,int32_t size,int32_t id);


typedef int(*func_ptr_int_int_int)(int enable,int id);
typedef int(*func_ptr_int_int_uint16_int)(int bOnOff,uint16_t uHundKhz, int id);
typedef int(*func_ptr_int_uint8_int_int)(uint8_t byPin, int enable,int id);

typedef int(*func_ptr_int_uintptr_intptr_int_int)(int Power[],int Voltage[],int iCount,int iDevID);

typedef int(*func_ptr_int_int_intptr_int)(int iPin, int *pLevel, int iDevID);
typedef int(*func_ptr_int_int_int_int)(int iPin, int bDir, int iDevID);

typedef int(*func_ptr_int_uint8p_uint16)(const uint8_t* pbuffer, const uint16_t size, const uint32_t dev_id);

typedef int (*func_SensorSpiRWEx)(int bStart, int bStop, int bMsb, unsigned char *TxData, unsigned char *RxData, unsigned int uTxLen, unsigned int uRxLen, int iDevID);
//int SensorConfig(SensorConfigParams *sensor_config_, int size, int id)

//DTCCM_API int _DTCALL_ SetGpioPinDir(int iPin, BOOL bDir, int iDevID = DEFAULT_DEV_ID);
//DTCCM_API int _DTCALL_ SetGpioPinLevel(int iPin, BOOL bLevel, int iDevID = DEFAULT_DEV_ID);

// �˴������ӿ� �ο�����API ����
/// @brief �������Խӿ�
///
/// @param PinConfig�����Խӿ����ö���
///
/// @retval DT_ERROR_OK�����Խӿ����óɹ�
/// @retval DT_ERROR_FAILED�����Խӿ�����ʧ��
/// @retval DT_ERROR_COMM_ERROR��ͨѶ����
//DTCCM_API int _DTCALL_ SetSoftPin(BYTE PinConfig[26], int iDevID = DEFAULT_DEV_ID);
typedef int(*func_SetSoftPin)(unsigned char[26],  int iDevID);

/* SPI���ýṹ�� */
typedef struct ObMasterSpiConfig
{
    double  fMhz;               ///< ����SPI��ʱ��
    unsigned char    byWordLen;          ///< Word length in bits. 0�� 8bit ��1��16bit����ʱ��Ч��������Ĭ��Ϊ0
    unsigned char    byCtrl;             ///< ֧�ֵ�λ�����룺MASTER_CTRL_DATA_SHIFT/MASTER_CTRL_CPOL/ MASTER_CTRL_CPHA/MASTER_CTRL_DELAY
    unsigned char    Rsv[64];            ///< ���� */
}ObMasterSpiConfig_t;


/// @brief uPort:SPI����������
/// 
/// @brief pSPIConfig:SPI���ýṹ�壬�μ�imagekit.h
/// 
/// @retval DT_ERROR_OK��SPI���óɹ�
/// @retval DT_ERROR_FAILD��SPI����ʧ��
/// @retval DT_ERROR_COMM_ERROR��ͨѶ����
//DTCCM_API int _DTCALL_ MasterSpiConfig(UCHAR uPort, MasterSpiConfig_t *pSPIConfig, int iDevID = DEFAULT_DEV_ID);
typedef int(*func_MasterSpiConfig)(unsigned char uPort, ObMasterSpiConfig_t *pSPIConfig, int iDevID);

/// @brief дSENSOR�Ĵ���,I2CͨѶģʽbyI2cMode������ֵ��I2CMODE����
/// 
/// @param uAddr����������ַ
/// @param uReg���Ĵ�����ַ
/// @param uValue��д��Ĵ�����ֵ
/// @param byMode��I2Cģʽ
///
/// @retval DT_ERROR_OK��дSENSOR�Ĵ��������ɹ�
/// @retval DT_ERROR_COMM_ERROR��ͨѶ����
/// @retval DT_ERROR_PARAMETER_INVALID��byMode������Ч
/// @retval DT_ERROR_TIME_OUT��ͨѶ��ʱ
/// @retval DT_ERROR_INTERNAL_ERROR���ڲ�����
///
/// @see I2CMODE
//DTCCM_API int _DTCALL_ WriteSensorReg(UCHAR uAddr, USHORT uReg, USHORT uValue, BYTE byMode, int iDevID = DEFAULT_DEV_ID);

typedef int(*func_WriteSensorReg)(unsigned char uAddr, unsigned short uReg, unsigned short uValue, unsigned char byMode, int iDevID);


// ��Ӧ������������ж����һ��
struct obc_ops_ap_device
{
	func_ptr_int_int_int  EnableSoftPin;
	func_ptr_int_int_int  EnableGpio;
	func_ptr_int_uintptr_intptr_int_int PmuSetVoltage;
	func_ptr_int_uintptr_intptr_int_int PmuSetOnOff;
	func_ptr_int_int_uint16_int  SetSensorClock;
	func_ptr_int_int_int SetSoftPinPullUp;
	func_ptr_int_int_int SetSensorI2cRate;
	func_ptr_int_uint8_int_int SensorEnable;
	int32_t device_id;
	func_ptr_int_int_int_int	SetGpioPinLevel;
	func_ptr_int_int_int_int	SetGpioPinDir;
    func_ptr_int_uint8p_uint16 SetMipiConfiguration;
    func_SensorSpiRWEx  SensorSpiRWEx;
    func_SetSoftPin     SetSoftPin;
    func_MasterSpiConfig MasterSpiConfig;
    func_WriteSensorReg     WriteSensorReg;
};

typedef struct obc_ops_ap_device obc_ops_ap_device_t;


struct obc_ops
{
	func_ptr_void_uint8_vr qsee_log;
	func_ptr_voidp_uint64 qsee_malloc;
	func_ptr_void_voidp qsee_free;
	func_ptr_void_uint64 tee_usleep;
	func_ptr_int_uint8p_uint32 ops_writeread;
	obc_ops_ap_device_t  ap_ops;
};

typedef struct obc_ops obc_ops_t;



//func_ptr_int32_void_int32_int32  device_config;

#ifdef __cplusplus
}
#endif

#endif // !_OBC_TEE_FUNCS_H_