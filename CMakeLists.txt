cmake_minimum_required(VERSION 3.10)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# set the project name
project(roipp)

# create our library
add_library(roipp SHARED src/roipp.cpp include/roipp.hpp)

# add the executable
add_executable(song_example src/roipp_example_song.cpp)
add_executable(drive_example src/roipp_example_driving.cpp)
add_executable(sensors_example src/roipp_example_sensors.cpp)
add_executable(debug src/roipp_debug.cpp)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(song_example PRIVATE Threads::Threads roipp)
target_link_libraries(drive_example PRIVATE Threads::Threads roipp)
target_link_libraries(sensors_example PRIVATE Threads::Threads roipp)
target_link_libraries(debug PRIVATE Threads::Threads ncurses roipp)

