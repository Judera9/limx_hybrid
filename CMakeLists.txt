cmake_minimum_required(VERSION 3.10)
project(limx_hybrid)

set(CMAKE_CXX_STANDARD 11)
set(BUILD_TYPE_RELEASE TRUE)

option(EXAMPLE "Build examples ON/OFF" OFF) # build limx examples

# TODO: what does these mean
set(CMAKE_CXX_FLAGS "-O3 -no-pie -ggdb -march=native -mavx -mfma -ffast-math")
set(CMAKE_C_FLAGS "-O3  -ggdb  -march=native -std=gnu99 -I.")
list(APPEND CMAKE_PREFIX_PATH "/opt/openrobots/")

find_package(pinocchio REQUIRED)
if (pinocchio_FOUND)
    message("pinocchio found")
    include_directories(SYSTEM ${pinocchio_INCLUDE_DIRS})
    message("pinocchio include dir at " ${pinocchio_INCLUDE_DIRS})
else ()
    message(FATAL_ERROR "Cannot find pinocchio")
endif ()

# generate Configuration.h file
SET(THIS_COM ${CMAKE_CURRENT_SOURCE_DIR})
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
        ${CMAKE_BINARY_DIR}/Configuration.h)

include_directories(./utility/limxsdk)
include_directories(./data)
include_directories(./controller)
include_directories(./estimator)
include_directories(./gait)
include_directories(./planner)
include_directories(./user)
include_directories(./utility)
include_directories(./utility/limxsdk)
# include_directories(./lcm)
include_directories(SYSTEM /usr/include/eigen3)

# TODO: which way could work?
include_directories(${CMAKE_BINARY_DIR})
include_directories(./third_party/PoplarLib/include)
include_directories(./third_party/PoplarLib/third_party/qpOASES/include)
include_directories(./third_party/PoplarLib/third_party/eiquadprog/include)
include_directories(SYSTEM ${THIS_COM}/PoplarLib/include)
include_directories(SYSTEM ${THIS_COM}/PoplarLib/third_party/qpOASES/include)
include_directories(SYSTEM ${THIS_COM}/PoplarLib/third_party/eiquadprog/include)
message("test: " ${THIS_COM}/PoplarLib/third_party/eiquadprog/include)

include_directories(./third_party/ParamHandler)
include_directories(./third_party/ParamHandler/src)
include_directories(./third_party/ParanHandler/dynacore_yaml-cpp)
include_directories(./build)

# recursively add all source code
file(GLOB_RECURSE lib_source "./controller/*.cpp"
  "./estimator/*.cpp"
  "./gait/*.cpp"
  "./planner/*.cpp" 
  "./user/*.cpp" 
  "./utility/*.cpp"
  "./third_party/*.cpp"
  "./third_party/*.hpp"
  "./third_party/*.h")

file(GLOB_RECURSE children RELATIVE ./third_party/PoplarLib/third_party ./third_party/PoplarLib/third_party*)
foreach(child ${children})
    if(IS_DIRECTORY ./third_party/PoplarLib/third_party/${child})
        include_directories(./third_party/PoplarLib/third_party/${child})
    endif()
endforeach()

# add_subdirectory(sim) # TODO: open sim (no CMakeLists.txt?)
# add_subdirectory(test)
add_subdirectory(./third_party/PoplarLib)
# add_subdirectory(./third_party/ParamHandler)

#### limx
if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
  set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/aarch64)
elseif (${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
  set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/arm32)
elseif (${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
  set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/amd64)
elseif (WIN32)
  set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/win)
  install(DIRECTORY ${LIB_PATH}/. DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
else()
  message(FATAL_ERROR "Unsupported architecture!")
endif()

link_directories(${LIB_PATH})

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include DESTINATION ${CMAKE_INSTALL_PREFIX})

# Install library files to build output
install(DIRECTORY ${LIB_PATH}/. DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

if(EXAMPLE)
  add_subdirectory(examples)
endif()

add_library(XiaotianHybridLib SHARED ${lib_source})
target_link_libraries(XiaotianHybridLib qpOASES pinocchio::pinocchio rt pthread matio PoplarLib)
add_executable(xiaotian_ctrl main.cpp)
target_link_libraries(xiaotian_ctrl XiaotianHybridLib)


