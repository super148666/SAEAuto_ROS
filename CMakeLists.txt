cmake_minimum_required(VERSION 2.8)
project(rossae)

set(CMAKE_CXX_STANDARD 11)

# catkin
find_package(catkin REQUIRED COMPONENTS
        message_generation
        roscpp
        nav_msgs
        geometry_msgs
        sensor_msgs
        tf
        dynamic_reconfigure)
        
# message
add_message_files(
        FILES)

# service
add_service_files(
		FILES)

set(Boost_LIBRARY_DIR "/usr/lib/aarch64-linux-gnu")
set(BOOST_INCLUDEDIR "/usr/include/boost")

find_package(Boost REQUIRED COMPONENTS
        system
        thread)

find_package(Curses REQUIRED)


include_directories(
        include/
        src/
        ${CURSES_INCLUDE_DIRS}
        ${BOOST_INCLUDE_DIRS})

set(SOURCE_FILES
        src/AsyncSerial.cpp
        src/CarNetwork.cpp
        src/Control.cpp
        src/Controller.cpp
        src/HeartBeatSerialIn.cpp
        src/IPC.cpp
        src/Logger.cpp
        src/LowLevelSerialOut.cpp
        src/SafetySerialOut.cpp
        src/Timer.cc
        src/TripStateMonitor.cpp
        src/PID.cpp)

add_executable(RosSAE ${SOURCE_FILES})

target_link_libraries(RosSAE boost_system boost_thread ${CURSES_LIBRARIES} gflags glog pthread)
#set_target_properties(RosSAE PROPERTIES COMPILE_FLAGS "-fPIC")
#set_target_properties(RosSAE PROPERTIES COMPILE_FLAGS "-Wl")


#generate_messages(
#		DEPENDENCIES)

catkin_package(
		DEPENDS
		roscpp)

install(
		TARGETS
		RosSAE
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
