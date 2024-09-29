
project(read_chunk)

ob_include_directories("${LIBRARY_OUTPUT_PATH}/include" ${PNG_INCLUDE_DIR})


set(SRC_FILE read_chunk.c)

add_executable(read_chunk ${SRC_FILE})

target_link_libraries(read_chunk  ${OBPNG_LIBRARY})


