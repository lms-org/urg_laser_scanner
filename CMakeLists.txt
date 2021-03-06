if(USE_CONAN)
cmake_minimum_required(VERSION 2.8.12)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()
endif(USE_CONAN)

set ( SOURCES 
        "src/laser_scanner.cpp"
        "src/interface.cpp"
)

set (HEADERS 
        "include/laser_scanner.h"
)

#set compiler flags
if((${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang") OR (${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU"))
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -Wreturn-type -Wpedantic ")
endif()

include_directories("include")
add_library (laser_scanner SHARED ${SOURCES} ${HEADERS})
if(USE_CONAN)
target_link_libraries(laser_scanner PRIVATE ${CONAN_LIBS})
else()
target_link_libraries(laser_scanner PRIVATE lmscore lms_math urg_cpp lms_sensor_utils)
endif(USE_CONAN)
