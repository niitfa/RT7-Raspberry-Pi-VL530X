cmake_minimum_required(VERSION 3.9)
project(send)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(source_dir ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(header_ext .h)
set(source_ext .c)

# Targets
set(exec_target run)
set(off_target off)

# ! В БУДУЩЕМ УКАЗАТЬ КОНКРЕТНЫЕ СПИСКИ ! 
file(GLOB_RECURSE sources ${source_dir}/*${source_ext})
file(GLOB_RECURSE headers ${source_dir}/*${header_ext})

# Create executable
add_executable(${exec_target} main${source_ext} ${sources})
add_executable(${off_target} off${source_ext} ${sources})
add_library(DistSensor SHARED ${sources})


target_link_libraries(${exec_target} m wiringPi)
target_link_libraries(${off_target} m wiringPi)

# Set headers
foreach(_header ${headers})
    get_filename_component(_include_dir ${_header} DIRECTORY)
    target_include_directories(${exec_target} PRIVATE ${_include_dir})
    target_include_directories(${off_target} PRIVATE ${_include_dir})
endforeach()
