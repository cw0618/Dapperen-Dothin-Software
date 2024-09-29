# ----------------------------------------------------------------------------
#  CMake file for libobdepth. See root CMakeLists.txt
#
# ----------------------------------------------------------------------------


project(${TOF_LIBRARY} C)


if(NOT WIN32)
  find_library(M_LIBRARY
    NAMES m
    PATHS /usr/lib /usr/local/lib
  )
  if(NOT M_LIBRARY)
    message(STATUS "math lib 'libm' not found; floating point support disabled")
  endif()
else()
  # not needed on windows
  set(M_LIBRARY "")
endif()


set (TOF_DRIVER_VERSION_MAJOR 0)
set (TOF_DRIVER_VERSION_MINOR 0)
set (TOF_DRIVER_VERSION_REVISION 7)
set (TOF_DRIVER_VERSION_BUILD 3)

ob_clear_vars(TOF_LIBRARY TOF_LIBRARIES TOF_INCLUDE_DIRS)
set(TOF_LIBRARY tof)
set(TOF_INCLUDE_DIRS "${${TOF_LIBRARY}_SOURCE_DIR}" "${${TOF_LIBRARY}_BINARY_DIR}")
set(TOF_LIBRARIES ${TOF_LIBRARY})

set (include_path 
	${ROOT_SOURCE_DIR}/include/
	"mlx75027/" 
	"mlx75027/.."
	"s5k33dxx/s5k33d_setting/"  
	"s5k33dxx/" 	
	"s5k33dxx/.." 
	"stmpe801/" 
	"stmpe801/.." 
    "imx516/"
	"imx456/"
	"delay_board/"
	"delay_board/temperature_table/"
	"imx518/"
	"pleco/"
	"rk1608/"
	"rk1608/spi/inc/"
	"rk1608/spi/src/"
	"imx627/"
	"imx316/"
	${MX6XHW_INCLUDE}
	)
include_directories(${include_path})

configure_file (
  "${PROJECT_SOURCE_DIR}/tof_sensors_version.h.in"
  "${PROJECT_BINARY_DIR}/tof_sensors_version.h"
  )


set(source
	tof_module.cpp
	tof_sensors.c
	tof_board.c
	mlx75027/mlx75027.c
    mlx75027/mlx75027_max96717_max96716a.c 		
    s5k33dxx/s5k33dxx.c	
	stmpe801/stmpe801.c	
    imx516/imx516.c
	imx456/imx456.c
	delay_board/delay_board.c
	imx518/imx518.c
	pleco/pleco.c
	rk1608/rk1608.c
	rk1608/spi/src/msg-interface.c
	rk1608/spi/src/isp-fw.c
	rk1608/spi/src/msg-queue.c
	rk1608/spi/src/spi2apb.c
	rk1608/rk1608_pleco.c
	rk1608/rk1608_s5k33dxx.c
	imx627/imx627.c
	imx316/imx316.c
    )


file(GLOB_RECURSE CURRENT_HEADERS  
 ${include_path}/*.h
  ${ROOT_SOURCE_DIR}/include/tofinfo.h
 tof_sensors.h
 tof_board.h
 project_config.h
 mlx75027/*.h 
 s5k33dxx/s5k33d_setting/*.h 
 s5k33dxx/*.h 
 stmpe801/*.h 
 imx516/*.h 
 imx456/*.h 
 delay_board/*.h
 imx518/*.h
 pleco/*.h
 rk1608/*.h
 rk1608/spi/inc/*.h
 )
source_group("Include" FILES ${CURRENT_HEADERS}) 




#################################################
##
#################################################
add_definitions(-D USE_WHICH_CONVERTER=1)

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

add_library(${TOF_LIBRARY} SHARED ${source} ${CURRENT_HEADERS} duxin/which_converter.h)

target_link_libraries(${TOF_LIBRARY}
    #-lm
    )

#################################################
##
#################################################

#project(${CX3_TOF_LIBRARY} C)
#set(CX3_TOF_LIBRARY cx3-tof)
#add_definitions(-D USE_WHICH_CONVERTER=2)
#include_directories("cx3")

#get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
#foreach(dir ${dirs})
#  message(STATUS "dir='${dir}'")
#endforeach()

#add_library(${CX3_TOF_LIBRARY} SHARED ${source}  ${CURRENT_HEADERS} cx3/which_converter.h)

#target_link_libraries(${CX3_TOF_LIBRARY}
#    #-lm
#   )


install(TARGETS  ${TOF_LIBRARY} RUNTIME DESTINATION libs)


