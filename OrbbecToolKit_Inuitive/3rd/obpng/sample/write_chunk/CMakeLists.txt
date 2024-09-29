
project(write_chunk)

ob_include_directories("${LIBRARY_OUTPUT_PATH}/include" ${PNG_INCLUDE_DIR})


set(SRC_FILE write_chunk.cpp)

add_executable(write_chunk ${SRC_FILE})

target_link_libraries(write_chunk  ${OBPNG_LIBRARY})

file(COPY  "1280_800.raw" DESTINATION  "${LIBRARY_OUTPUT_PATH}/"  )


