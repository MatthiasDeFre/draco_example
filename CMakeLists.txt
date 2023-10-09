cmake_minimum_required(VERSION 3.27)
set(CMAKE_CXX_STANDARD 17) 
SET(CMAKE_BUILD_TYPE Release)
project (draco_example)

find_package(PCL CONFIG REQUIRED)
find_package(draco CONFIG REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (
    ${PROJECT_NAME} main.cpp encoder_draco.cpp decoder_draco.cpp
)

# whole-archive stuff is only required to enable draco based io functions to read .ply files
# If you don't need to read .ply with draco you can remove the -Wl, --whole-archive
if (MSVC)
    SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES LINK_FLAGS "/WHOLEARCHIVE:draco")
    target_link_libraries(${PROJECT_NAME} draco)
else()
    target_link_libraries(${PROJECT_NAME} -Wl,--whole-archive draco -Wl,--no-whole-archive)
endif()

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} pcl_io_ply)

