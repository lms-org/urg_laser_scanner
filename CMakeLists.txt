cmake_minimum_required(VERSION 2.8.12)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

set ( SOURCES 
        "src/laser_scanner.cpp"
	"src/interface.cpp"
        "src/Connection_information.cpp"
)

set (HEADERS 
        "include/laser_scanner.h"
        "include/Connection_information.h"
)

#set compiler flags
if((${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang") OR (${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU"))
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -Wreturn-type -Wpedantic ")
endif()

include_directories("include")
add_library (laser_scanner SHARED ${SOURCES} ${HEADERS})
target_link_libraries(laser_scanner PRIVATE ${CONAN_LIBS})
