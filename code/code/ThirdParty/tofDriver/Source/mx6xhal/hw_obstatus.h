#ifndef __HW_OBSTATIS__
#define __HW_OBSTATIS__

typedef enum HW_ERR_T {
    HW_NO_ERR = 0,  // ������������
    HW_ERR_NULL = 2000,  // ��ָ��
    HW_ERR_INVALID = 2001,  // �Ƿ�����
    HW_ERR_NO_SUPPORT = 2002,  // ��֧�ֵĲ���
    HW_ERR_RW = 2003,  // ���ݶ�д(һ����ָSPI)����
    HW_ERR_LIB = 2004,  // �⺯�����ó���
    HW_ERR_TIMEOUT = 2005,  // ��ʱ
    HW_ERR_TEE_OPS_NULL = 2006,  //�ղ���
    HW_ERR_NO_MEM = 2007,  // �ڴ治��
    HW_ERR_CAM_PARAMS_NULL = 2008,  // �ղ���
	HW_ERR_NO_INIT = 2009,
    HW_ERR_BAD_SENSOR_ID_VALUE = 2010,  // !<δ��ʼ��
    HW_ERR_BAD_VALUE = 2011,  // !<δ��ʼ��
	HW_ERR_NO_MATCH
} HW_ERR;




#endif