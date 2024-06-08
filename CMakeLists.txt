cmake_minimum_required(VERSION 3.24.0 FATAL_ERROR)
project(gyarmathi-csaba-thesis
        VERSION 0.1.0
        LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

set(BOOST_ROOT "C:/boost_1_85_0")
set(BOOST_LIBRARYDIR "C:/boost_1_85_0/stage/lib")

set(Boost_NO_SYSTEM_PATHS ON)
find_package(Boost 1.85.0 REQUIRED)

if(NOT Boost_FOUND)
    message(FATAL_ERROR "Boost not found!")
endif()

find_package(OpenCV REQUIRED)

add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/3rdparty/nativefiledialog-extended")

add_executable(${PROJECT_NAME}
        src/util/file-util.cpp
        src/util/image-util.cpp
        src/util/serial-util.cpp
        src/g-code.cpp
        src/cnc-controller.cpp
        src/edge-detection.cpp
        src/main.cpp "src/util/serial-util.cpp")

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include ${CMAKE_CURRENT_LIST_DIR}/include/util ${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} nfd ${Boost_LIBRARIES})
