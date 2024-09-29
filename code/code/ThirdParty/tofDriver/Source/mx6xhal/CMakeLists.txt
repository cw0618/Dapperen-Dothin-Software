# ----------------------------------------------------------------------------
#  CMake file for libobobhw. See root CMakeLists.txt
#
# ----------------------------------------------------------------------------


project(${OBHW_LIBRARY} C)



set (OBHW_VERSION_MAJOR 0)
set (OBHW_VERSION_MINOR 0)
set (OBHW_VERSION_REVISION 1)
set (OBHW_VERSION_BUILD 0)

configure_file (
  "${PROJECT_SOURCE_DIR}/obhw_version.h.in"
  "${PROJECT_BINARY_DIR}/obhw_version.h"
  )


set(source
    hw_modules.c
    )

add_library(${OBHW_LIBRARY} SHARED ${source})

target_link_libraries(${OBHW_LIBRARY}
    -lm
    )

if(UNIX)
  if(CMAKE_COMPILER_IS_GNUCXX OR __ICC)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
  endif()
endif()

#file(COPY  "${OBHW_PUBLIC_HEADERS}/orbbec_png_info.h" DESTINATION  "${LIBRARY_OUTPUT_PATH}/include"  )
#file(COPY  "${OBHW_PUBLIC_HEADERS}/orbbec_chunk_type.h" DESTINATION  "${LIBRARY_OUTPUT_PATH}/include"  )
#file(COPY  "${OBHW_PUBLIC_HEADERS}/orbbec_png_utils.h" DESTINATION  "${LIBRARY_OUTPUT_PATH}/include"  )

