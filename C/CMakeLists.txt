cmake_minimum_required (VERSION 2.8)
project(razorAHRS)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_C_FLAGS "-O2 -g")

#Source folders
add_subdirectory(src)

add_executable(main_razorAHRS src/main_razorAHRS.c src/razorAHRS.c)
target_link_libraries(main_razorAHRS LINK_PUBLIC pthread)

add_executable(main_razorAHRSvirtual src/main_virtualTracker.c src/virtualTracker.c)
