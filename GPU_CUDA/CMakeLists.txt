cmake_minimum_required (VERSION 3.8)



project ("gpu_octree" LANGUAGES CXX)


set(PCL_SRC_DIR "/home/intel/suhao/pcl_latest/pcl")

set(PCL_LIB_DIR "/home/intel/suhao/pcl_latest/pcl/build/lib")

include_directories(
    "${PCL_SRC_DIR}/octree/include"
    "${PCL_SRC_DIR}/test/include"
    "${PCL_SRC_DIR}/common/include"
    "${PCL_SRC_DIR}/build/include"
    "${PCL_SRC_DIR}/gpu/octree/include"
    "${PCL_SRC_DIR}/gpu/containers/include"
    "/usr/include/eigen3"
)



# pcl_octree pcl_gpu_octree


find_library(PCL_GTEST          pcl_gtest           PATHS ${PCL_LIB_DIR})
find_library(PCL_COMMON         pcl_common          PATHS ${PCL_LIB_DIR})
find_library(PCL_OCTREE         pcl_octree          PATHS ${PCL_LIB_DIR})
find_library(PCL_GPU_OCTREE     pcl_gpu_octree      PATHS ${PCL_LIB_DIR})
find_library(PCL_GPU_CONTAINERS pcl_gpu_containers  PATHS ${PCL_LIB_DIR})
find_library(PCL_GPU_UTILS      pcl_gpu_utils       PATHS ${PCL_LIB_DIR})





add_executable (test_gpu_octree_performance    src/performance.cpp)
add_executable( test_approx_nearest            src/test_approx_nearest.cpp)
add_executable( test_bfrs_gpu                  src/test_bfrs_gpu.cpp )
add_executable( test_gpu_host_radius_search    src/test_host_radius_search.cpp )
add_executable( test_gpu_knn_search            src/test_knn_search.cpp )
add_executable( test_gpu_radius_search         src/test_radius_search.cpp )


add_executable( perf_approx         src/perf_approx.cpp )





target_link_libraries(test_gpu_octree_performance   ${PCL_GTEST}  ${PCL_COMMON} ${PCL_OCTREE}  ${PCL_GPU_OCTREE} ${PCL_GPU_CONTAINERS} )
target_link_libraries(test_approx_nearest           ${PCL_GTEST}  ${PCL_COMMON} ${PCL_OCTREE}  ${PCL_GPU_OCTREE} ${PCL_GPU_CONTAINERS} ${PCL_GPU_UTILS} )
target_link_libraries(test_bfrs_gpu                 ${PCL_GTEST}  ${PCL_COMMON} ${PCL_OCTREE}  ${PCL_GPU_OCTREE} ${PCL_GPU_CONTAINERS} )
target_link_libraries(test_gpu_host_radius_search   ${PCL_GTEST}  ${PCL_COMMON} ${PCL_OCTREE}  ${PCL_GPU_OCTREE} ${PCL_GPU_CONTAINERS} )
target_link_libraries(test_gpu_knn_search           ${PCL_GTEST}  ${PCL_COMMON} ${PCL_OCTREE}  ${PCL_GPU_OCTREE} ${PCL_GPU_CONTAINERS} )
target_link_libraries(test_gpu_radius_search        ${PCL_GTEST}  ${PCL_COMMON} ${PCL_OCTREE}  ${PCL_GPU_OCTREE} ${PCL_GPU_CONTAINERS} )


target_link_libraries(perf_approx                   ${PCL_GTEST}  ${PCL_COMMON} ${PCL_OCTREE}  ${PCL_GPU_OCTREE} ${PCL_GPU_CONTAINERS} )

# PCL_ADD_TEST(gpu_octree_host_radius test_gpu_host_radius_search FILES test_host_radius_search.cpp LINK_WITH pcl_gtest pcl_common pcl_octree pcl_gpu_octree)


# PCL_ADD_TEST(gpu_octree_knn_search test_gpu_knn_search FILES test_knn_search.cpp          LINK_WITH pcl_gtest pcl_common pcl_octree pcl_gpu_octree)
# PCL_ADD_TEST(gpu_octree_radius_search test_gpu_radius_search FILES test_radius_search.cpp LINK_WITH pcl_gtest pcl_common pcl_octree pcl_gpu_octree)














