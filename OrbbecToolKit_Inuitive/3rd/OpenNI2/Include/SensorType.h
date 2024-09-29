//
// Copyright 2020 ORBBEC Technology., Inc.
//
// This file belongs to ORBBEC Technology., Inc.
// It is considered a trade secret, and is not to be divulged or used by
// parties who have NOT received written authorization from the owner.
// see https:
//

/***********************************************************************************************
***           C O N F I D E N T I A L  ---  O R B B E C   Technology                        ***
***********************************************************************************************
*
*  @file      :    SenorType.h
*  @brief     :    用于区分不用的sensor
*  @version   :    0.0.0.1
*  @date      :    2022.06.17                                                                 *
*  @update    :    2022.06.17                                                                 *
*                                                                                             *
*---------------------------------------------------------------------------------------------*
* Functions:  																				  *
* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
#ifndef _INCLUDE_SENSORTYPE_H_
#define _INCLUDE_SENSORTYPE_H_
#include <stdint.h>

#define PLECO_SENSOR 			0
#define MF_SENSOR 			    0
#define GAEA_SENSOR 			1
#if (PLECO_SENSOR == 1)

#elif (MF_SENSOR == 1)

#endif
#endif  // _INCLUDE_SENSORTYPE_H_
