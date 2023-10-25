cmake_minimum_required (VERSION 3.8)

if(WIN32)
    set(CMAKE_GENERATOR_TOOLSET "Intel(R) oneAPI DPC++ Compiler 2023")

else()
    set(CMAKE_CXX_COMPILER "icpx")
endif()

set(CMAKE_CXX_FLAGS " -fsycl  ")

if(WIN32)
string(APPEND CMAKE_CXX_FLAGS " /EHsc") 
endif()


project ("perf_octree" LANGUAGES CXX)

if(LINUX)
    find_package(PCL 1.12 REQUIRED)
    find_package(PCL-ONEAPI 1.12 REQUIRED)
    include_directories(${PCL_INCLUDE_DIRS} ${PCL-ONEAPI_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS} ${PCL-ONEAPI_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS} ${PCL-ONEAPI_DEFINITIONS})
    add_executable( hello_sycl src/hello_sycl.cpp)
    add_executable( perf_octree src/perf_octree.cpp)
    target_link_libraries (perf_octree pcl_oneapi_containers pcl_oneapi_octree pcl_octree)
endif()


if(WIN32)
# 将 cmake/Modules 目录放到 PATH 下面，主要是使用 FindEigen.cmake 寻找 cmake
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})

set(SYCL_PATH "C:/Program Files (x86)/Intel/oneAPI/compiler/latest/windows/lib/ocloc;C:/Program Files (x86)/Intel/oneAPI/compiler/latest/windows/bin;C:/Program Files (x86)/Intel/oneAPI/compiler/latest/windows/lib;C:/Program Files (x86)/Intel/oneAPI/compiler/latest/windows/bin/intel64;C:/Program Files (x86)/Intel/oneAPI/compiler/latest/windows/redist/intel64_win/compiler;C:/Program Files (x86)/Intel/oneAPI/compiler/latest/windows/lib/oclfpga/host/windows64/bin;C:/Program Files (x86)/Intel/oneAPI/compiler/latest/windows/lib/oclfpga/llvm/aocl-bin;C:/Program Files (x86)/Intel/oneAPI/compiler/latest/windows/lib/oclfpga/windows64/bin;C:/Program Files (x86)/Intel/oneAPI/compiler/latest/windows/lib/oclfpga/bin;C:/Program Files (x86)/Intel/oneAPI/compiler/latest/windows/../../../tbb/latest/redist/intel64/vc_mt;")
set(PCL_ROOT "C:/Program Files/PCL 1.12.1")
set(EIGEN_ROOT "${PCL_ROOT}/3rdParty/Eigen")
set(BOOST_ROOT "C:/Users/RS/Downloads/boost_1_81_0")
set(BOOST_LIBRARYDIR "C:/Users/RS/Downloads/boost_1_81_0/stage/lib")


set(PCL_ONEAPI_ROOT "C:/Program Files (x86)/PCL-ONEAPI")
set(PCL_ONEAPI_LIB_DIR "${PCL_ONEAPI_ROOT}/lib")

set(PCL_NATIVE_ROOT "C:/Program Files (x86)/PCL")
set(PCL_NATIVE_LIB_DIR "${PCL_NATIVE_ROOT}/lib")




set(PCL_SRC_DIR "/home/intel/suhao/pcl_latest/pcl")


set(PCL_INCLUDE "C:/Program Files (x86)/PCL/include/pcl-1.12")

set(PCL_ONEAPI_INCLUDE "C:/Program Files (x86)/PCL-ONEAPI/include/pcl-oneapi-1.12")

find_package(Eigen 3.1)
find_package(Boost REQUIRED)

message(STATUS "eigen include path is ${EIGEN_INCLUDE_DIRS}")

include_directories(
    ${PCL_INCLUDE}
    ${PCL_ONEAPI_INCLUDE}
    ${EIGEN_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    $ENV{ONEAPI_ROOT}/dpcpp-ct/latest/include
)

find_library(PCL_COMMON             pcl_common            PATHS ${PCL_NATIVE_LIB_DIR})
find_library(PCL_OCTREE             pcl_octree            PATHS ${PCL_NATIVE_LIB_DIR})
find_library(PCL_ONEAPI_CONTAINERS  pcl_oneapi_containers PATHS ${PCL_ONEAPI_LIB_DIR})
find_library(PCL_ONEAPI_OCTREE      pcl_oneapi_octree     PATHS ${PCL_ONEAPI_LIB_DIR})






add_executable( hello_sycl src/hello_sycl.cpp)
add_executable( perf_octree src/perf_octree.cpp )
add_executable( oneapi_octree src/oneapi_octree.cpp)



target_link_libraries(oneapi_octree  ${PCL_ONEAPI_CONTAINERS} ${PCL_ONEAPI_OCTREE})
target_link_libraries(perf_octree    ${PCL_COMMON} ${PCL_OCTREE}  ${PCL_ONEAPI_CONTAINERS} ${PCL_ONEAPI_OCTREE})



if(WIN32)
set_target_properties(  oneapi_octree
                        perf_octree
                        PROPERTIES VS_DEBUGGER_ENVIRONMENT  "SYCL_DEVICE_FILTER=ext_oneapi_level_zero:gpu
PATH=%PATH%;${SYCL_PATH};${PCL_ONEAPI_ROOT}/bin;${PCL_NATIVE_ROOT}/bin;")

endif()

endif()
