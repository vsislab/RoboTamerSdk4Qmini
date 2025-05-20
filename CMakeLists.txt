cmake_minimum_required(VERSION 3.5)
project(q1_sdk VERSION 1.0)

## Set compiler to use c++ 17 features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


set(default_build_type "Debug")
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
            STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
            "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif ()

## Use GNUInstallDirs to install libraries into correct locations on all platforms.
include(GNUInstallDirs)

## Put all binary files into /bin and libraries into /lib
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})
## Check system architecture
message(STATUS "Current system architecture: ${CMAKE_SYSTEM_PROCESSOR}")
## Import thirdparty libraries
add_subdirectory(thirdparty)

## Import Unitree SDK2 library
set(UNITREE_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR})
find_library(UNITREE_SDK_LIB unitree_sdk2 PATHS ${UNITREE_SDK_PATH} NO_DEFAULT_PATH)

if (NOT UNITREE_SDK_LIB)
    message(FATAL_ERROR "Unitree SDK library for the architecture is not found")
else ()
    message(STATUS "Unitree SDK library found at: ${UNITREE_SDK_LIB}")
endif ()

message(STATUS "Importing: ${UNITREE_SDK_LIB}")

find_package(Threads REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(unitree_sdk2 STATIC IMPORTED GLOBAL)
set_target_properties(unitree_sdk2 PROPERTIES
        IMPORTED_LOCATION ${UNITREE_SDK_LIB})
target_link_libraries(unitree_sdk2 INTERFACE ddsc ddscxx Threads::Threads)
target_include_directories(unitree_sdk2 INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
        unitree_sdk2ConfigVersion.cmake
        VERSION "${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}.${${PROJECT_NAME}_VERSION_PATCH}"
        COMPATIBILITY ExactVersion)

# set(BUILD_TYPE_RELEASE TRUE)
set(CMAKE_CXX_FLAGS "-O3 -no-pie -ggdb -Wall \
-Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
-Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
-Wshadow -Wsign-promo")

set(CMAKE_C_FLAGS "-O3  -ggdb -std=gnu99 -I.")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O3 -fPIC")

include_directories("include")
include_directories("include/onnx")
include_directories("include/user")
include_directories("include/utils")
include_directories("include/unitree")

include_directories("/usr/local/include/eigen3")
include_directories("/usr/local/include/yaml-cpp")

include_directories(${EIGENS_INCLUDE_DIR})

include_directories(/usr/include/jsoncpp)
include_directories(/usr/local/include/ddscxx /usr/local/include/iceoryx/v2.0.2)
#include_directories(/usr/local/include)
include_directories(/usr/include/python3.12/)


FIND_PACKAGE(yaml-cpp REQUIRED)

aux_source_directory(source/user SrcFiles_user)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


add_executable(run_interface source/run_interface.cpp ${SrcFiles_user})
#add_executable(test_interface source/test_interface.cpp ${SrcFiles_user})

#option(PLATFORM "arm64 or x86_64" arm64)
#if (${PLATFORM} STREQUAL "arm64")
target_link_libraries(run_interface ddsc unitree_sdk2 ddscxx rt pthread libonnxruntime.so yaml-cpp Python3::Python JsonCpp::JsonCpp libUnitreeMotorSDK_Arm64.so)
#    target_link_libraries(test_interface unitree_sdk2 ddsc ddscxx rt pthread libonnxruntime.so yaml-cpp Python3::Python JsonCpp::JsonCpp libUnitreeMotorSDK_Arm64.so)
#    target_link_libraries(imu_reader unitree_sdk2 ddsc ddscxx rt pthread libonnxruntime.so yaml-cpp Python3::Python JsonCpp::JsonCpp libUnitreeMotorSDK_Arm64.so)
#elseif (${PLATFORM} STREQUAL "x86_64")
#    target_link_libraries(run_interface unitree_sdk2 ddsc ddscxx rt pthread libonnxruntime.so yaml-cpp Python3::Python libjsoncpp.a libUnitreeMotorSDK_Linux64.so )
#    target_link_libraries(test_interface unitree_sdk2 ddsc ddscxx rt pthread libonnxruntime.so yaml-cpp Python3::Python libjsoncpp.a libUnitreeMotorSDK_Linux64.so)
#
#endif ()


