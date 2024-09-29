/*****************************************************************************
*  Orbbec ToFDepth 1.0
*  Copyright (C) 2019 by ORBBEC Technology., Inc.
*
*  This file is part of Orbbec ToFDepth.
*
*  This file belongs to ORBBEC Technology., Inc.
*  It is considered a trade secret, and is not to be divulged or used by
* parties who have NOT received written authorization from the owner.
*
*  Description:
*
****************************************************************************/
#ifndef __OB_TFODEPTH_VERSION_H__
#define __OB_TFODEPTH_VERSION_H__

// macro defined in CMakeLists.txt
#define TOFDEPTH_VERSION_EPOCH 3
#define TOFDEPTH_VERSION_MAJOR 1 
#define TOFDEPTH_VERSION_MINOR 0

#define TOFDEPTH_STR_EXP(__A)    #__A
#define TOFDEPTH_STR(__A)        TOFDEPTH_STR_EXP(__A)

#define TOFDEPTH_STRW_EXP(__A)   L#__A
#define TOFDEPTH_STRW(__A)       TOFDEPTH_STRW_EXP(__A)

#define TOFDEPTH_VERSION     TOFDEPTH_STR(TOFDEPTH_VERSION_EPOCH) "." TOFDEPTH_STR(TOFDEPTH_VERSION_MAJOR) "." TOFDEPTH_STR(TOFDEPTH_VERSION_MINOR)


#endif // __OB_TFODEPTH_VERSION_H__
