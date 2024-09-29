#ifndef XN_MX6X_MODULES
#define XN_MX6X_MODULES

#include "XnMx6xProperties.h"




typedef struct mx6x_module{

	XnInt(*init_ops)(void* ops);
	XnInt(*set_property)(hw_command_t command, command_data_t* data);
	XnInt(*get_property)(hw_command_t command, command_data_t* data);
	XnInt(*deinit)();

}mx6x_module_t;




#endif //XN_MX6X_MODULES