#ifndef XN_DOTHIN_CONFIG
#define XN_DOTHIN_CONFIG


typedef struct ntc_config{
	XnFloat Rfix;
	XnFloat Voltage;
}ntc_config_t;



typedef struct i2c_power{
	XnFloat avdd;
	XnFloat dovdd;
	XnFloat dvdd;
	XnFloat afvcc;
}i2c_power_t;


#endif