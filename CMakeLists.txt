cmake_minimum_required(VERSION 3.8)
project(control)

set(CMAKE_CXX_STANDARD 11)

find_package(Boost 1.58 REQUIRED COMPONENTS
        system
        thread
        )

find_package( Curses REQUIRED )


include_directories(
        include/
        src/
        ${CURSES_INCLUDE_DIRS}
        ${BOOST_INCLUDE_DIRS}
)

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

add_executable(control ${SOURCE_FILES})

target_link_libraries(control Boost::system Boost::thread ${CURSES_LIBRARIES} gflags glog)
