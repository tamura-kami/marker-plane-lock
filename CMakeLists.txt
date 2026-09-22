cmake_minimum_required(VERSION 3.16)

project(obs_fixed_camera_stabilizer VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED COMPONENTS core imgproc video videoio calib3d aruco)

add_library(stabilizer
    src/stabilizer/stabilizer.cpp
)

target_include_directories(stabilizer
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(stabilizer
    PUBLIC
        opencv_core
        opencv_imgproc
        opencv_video
        opencv_calib3d
)

add_executable(video_test
    tools/video_test.cpp
)

target_link_libraries(video_test
    PRIVATE
        stabilizer
        opencv_videoio
)

add_executable(video_offline
    tools/video_offline.cpp
)

add_executable(video_marker_offline
    tools/video_marker_offline.cpp
)

target_link_libraries(video_marker_offline
    PRIVATE
        opencv_core
        opencv_imgproc
        opencv_videoio
        opencv_calib3d
        opencv_aruco
)

target_link_libraries(video_offline
    PRIVATE
        stabilizer
        opencv_videoio
)

include(CTest)

if(BUILD_TESTING)
    add_executable(stabilizer_test tests/stabilizer_test.cpp)
    target_link_libraries(stabilizer_test PRIVATE stabilizer)
    add_test(NAME stabilizer_test COMMAND stabilizer_test)
endif()
