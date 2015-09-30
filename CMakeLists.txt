set ( SOURCES 
	"src/laser_scanner.cpp"
	"src/Connection_information.cpp"
	"src/interface.cpp"
)

set (HEADERS 
	"include/laser_scanner.h"
	"include/Connection_information.h"
)

include_directories("include")
add_library (laser_scanner SHARED ${SOURCES} ${HEADERS})
target_link_libraries(laser_scanner PRIVATE lmscore urg_cpp imaging opencv_binding)
