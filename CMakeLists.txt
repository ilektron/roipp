cmake_minimum_required(VERSION 3.10)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# set the project name
project(roipp)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(Boost 1.71 REQUIRED COMPONENTS
             log)
include_directories(${Boost_INCLUDE_DIRS})

# create our library
add_library(roipp SHARED src/roipp.cpp include/roipp.hpp)
target_link_libraries(roipp PUBLIC Boost::log)

# Set up compilation for our example programs
# add the executable
add_executable(song_example src/roipp_example_song.cpp)
add_executable(drive_example src/roipp_example_driving.cpp)
add_executable(sensors_example src/roipp_example_sensors.cpp)
add_executable(debug src/roipp_debug.cpp)

target_link_libraries(song_example PRIVATE Threads::Threads roipp)
target_link_libraries(drive_example PRIVATE Threads::Threads roipp)
target_link_libraries(sensors_example PRIVATE Threads::Threads roipp)
target_link_libraries(debug PRIVATE Threads::Threads ncurses roipp)
