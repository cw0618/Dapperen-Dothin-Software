/********************************************************************************
 *
 ********************************************************************************/
#ifndef __VERSION_H__
#define __VERSION_H__

// macro defined in CMakeLists.txt
#define VERSION_EPOCH 3
#define VERSION_MAJOR 0
#define VERSION_MINOR 3

#define STR_EXP(__A)    #__A
#define STR(__A)        STR_EXP(__A)

#define STRW_EXP(__A)   L#__A
#define STRW(__A)       STRW_EXP(__A)

#define  TOF_CALIB_PARAMS_LOADER_VERSION	STR(VERSION_EPOCH) "." STR(VERSION_MAJOR) "." STR(VERSION_MINOR)

#endif //   __API_DEFINE_H__
