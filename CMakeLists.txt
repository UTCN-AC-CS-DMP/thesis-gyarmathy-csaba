cmake_minimum_required(VERSION 3.24.0 FATAL_ERROR)
project(gyarmathy-csaba-thesis
        VERSION 0.1.0
        LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

find_package(OpenCV REQUIRED)

add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/3rdparty/nativefiledialog-extended")

add_executable(${PROJECT_NAME}
        src/util/file-util.cpp
        src/util/image-util.cpp
        src/g-code.cpp
        src/edge-detection.cpp
        src/main.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include ${CMAKE_CURRENT_LIST_DIR}/include/util)
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} nfd)
