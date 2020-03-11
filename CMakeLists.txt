cmake_minimum_required(VERSION 2.8.3)
project( icebus_host )

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/devel/lib)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -g -fext-numeric-literals")

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

#==============================================================================
# Dependencies
#==============================================================================
find_package(catkin REQUIRED COMPONENTS
		roscpp
		rospy
		rosbag
		std_msgs
    geometry_msgs
		message_generation
		common_utilities
		roboy_middleware_msgs
)

catkin_package( CATKIN_DEPENDS roboy_middleware_msgs common_utilities
        INCLUDE_DIRS include)

include_directories(
		include/
    ${catkin_INCLUDE_DIRS}
)
#==============================================================================
# Application
#==============================================================================
# icebus library
add_library(IcebusHost SHARED
		src/IcebusHost.cpp
	)
add_dependencies(IcebusHost roboy_middleware_msgs_msgs_generate_messages_cpp)
target_link_libraries(IcebusHost ${catkin_LIBRARIES} )

# icebus_demo
add_executable( icebus_host src/icebus_demo_host.cpp)
target_link_libraries( icebus_host IcebusHost)

add_executable( icebus_receiver src/icebus_demo_receiver.cpp)
target_link_libraries( icebus_receiver IcebusHost)

#==============================================================================
# Installation
#==============================================================================
install(TARGETS
		IcebusHost
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
    package.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
		include/icebus/
		DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
		FILES_MATCHING PATTERN "*.hpp"
		)
