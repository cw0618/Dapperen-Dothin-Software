# ----------------------------------------------------------------------------
#  CMake file for libini. See root CMakeLists.txt
#
# ----------------------------------------------------------------------------

project(${LIBINI_LIBRARY})

ob_include_directories("${CMAKE_CURRENT_SOURCE_DIR}" ${LIBINI_INCLUDE_DIRS})

file(GLOB lib_srcs *.cpp)
file(GLOB lib_hdrs *.h)


# ----------------------------------------------------------------------------------
#         Define the library target:
# ----------------------------------------------------------------------------------

if(MSVC)
  add_definitions(-D_CRT_SECURE_NO_DEPRECATE)
endif(MSVC)

add_library(${LIBINI_LIBRARY} SHARED ${lib_srcs} ${lib_hdrs})

if(UNIX)
  if(CMAKE_COMPILER_IS_GNUCXX OR __ICC)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
  endif()
endif()

