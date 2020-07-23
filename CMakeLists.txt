cmake_minimum_required(VERSION 3.5)
project(galaxy_camera)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_generation
    image_transport
    camera_info_manager
    )

catkin_package(
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    include/
    ./
)

add_executable(galaxy_camera
    "src/galaxy_camera.cpp"
    "src/galaxy_camera_node.cpp")
target_link_libraries(galaxy_camera
    gxiapi ${catkin_LIBRARIES})
