# CMake Version
cmake_minimum_required(VERSION 3.18.0)

set(ENABLE_CUDA TRUE CACHE BOOL "enable cuda or not")
message(STATUS "ENABLE_CUDA: ${ENABLE_CUDA}")

# Project Name
if(WIN32 AND ${ENABLE_CUDA})
  project(KIRI LANGUAGES C CXX CUDA)
else()
  project(KIRI LANGUAGES C CXX)

  if(APPLE)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS ON)
  endif()
endif()

# Get CUDA INFO
if(NOT APPLE AND ${ENABLE_CUDA})
  find_package(CUDA)

  if(CUDA_FOUND)
    set(CUDA_ARCH_DYNAMIC "TRUE")
    set(CUDA_FIXED_ARCH "75")

    # Get CUDA compute capability
    set(OUTPUTFILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/checkCUDA/compile/cuda_script)
    set(CUDAFILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/checkCUDA/check_cuda.cu)
    execute_process(COMMAND nvcc -lcuda ${CUDAFILE} -o ${OUTPUTFILE} -Xcompiler "/wd4819")
    execute_process(COMMAND ${OUTPUTFILE}
      RESULT_VARIABLE CUDA_RETURN_CODE
      OUTPUT_VARIABLE ARCH)

    if(${CUDA_RETURN_CODE} EQUAL 0)
      set(CUDA_SUCCESS "TRUE")
    else()
      set(CUDA_SUCCESS "FALSE")
    endif()

    if(${CUDA_SUCCESS})
      if(${CUDA_ARCH_DYNAMIC})
        message(STATUS "CUDA Architecture: ${ARCH}")
        set(CUDA_NVCC_FLAGS "{$CUDA_NVCC_FLAGS};--generate-code arch=compute_${ARCH},code=\"compute_${ARCH},sm_${ARCH}\"")
      else()
        message(STATUS "User Set CUDA Architecture: ${CUDA_FIXED_ARCH}")
        set(CUDA_NVCC_FLAGS "{$CUDA_NVCC_FLAGS};--generate-code arch=compute_${CUDA_FIXED_ARCH},code=\"compute_${CUDA_FIXED_ARCH},sm_${CUDA_FIXED_ARCH}\"")
      endif()

      message(STATUS "CUDA Version: ${CUDA_VERSION_STRING}")
      message(STATUS "CUDA Path: ${CUDA_TOOLKIT_ROOT_DIR}")
      message(STATUS "CUDA Libararies: ${CUDA_LIBRARIES}")

    else()
      message(WARNING ${ARCH})
    endif()
  endif()
endif()

# For Visual Studio
if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  # Optimization options for C/C++
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++17 /wd4819 /wd4068 /wd26812 /wd26495 /O2 /Ob2 /Ot /Oi /Gy /MP")
endif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")

# Compile Type
set(CMAKE_CONFIGURATION_TYPES "Debug;Release")

# Kiri2d
add_subdirectory(core)

# Examples
if(NOT ${ENABLE_CUDA})
  add_subdirectory(demos/voronoi2d_scene1)
  add_subdirectory(demos/voronoi2d_scene2)
  add_subdirectory(demos/lloyd2d_scene1)
  add_subdirectory(demos/lloyd2d_scene2)
  add_subdirectory(demos/delaunay2d_scene1)
  add_subdirectory(demos/convexhull2d_scene1)
  add_subdirectory(demos/convexhull2d_scene2)
  add_subdirectory(demos/poisson_disk_sampler2d)
  add_subdirectory(demos/poisson_disk_multi_sampler2d)
  add_subdirectory(demos/straight_skeleton2d)
  add_subdirectory(demos/voronoi_treemap_nocaj_scene1)
  add_subdirectory(demos/voronoi_treemap_nocaj_scene2)
  add_subdirectory(demos/ms_sampler2d)

  add_subdirectory(demos/proto_sphere_scene1)
  add_subdirectory(demos/proto_sphere_scene2)
  add_subdirectory(demos/proto_sphere_sdf_parallel)
  add_subdirectory(demos/proto_sphere_bon2020_scene1)
  add_subdirectory(demos/proto_sphere_test)
  add_subdirectory(demos/xproto_sphere_parallel_comp_strategy)

  add_subdirectory(demos/sdf2d_scene1)
  add_subdirectory(demos/rigidbody2d_scene)
  add_subdirectory(demos/dynamic_fracture_scene1)
  add_subdirectory(demos/bdem_example)
  
  # add_subdirectory(demos/test_bayesian)
  # add_subdirectory(demos/interior_point_method)
  # add_subdirectory(demos/primal_dual_ipm)
  # add_subdirectory(demos/pdipm_sampler)
  # add_subdirectory(demos/pdipm_sampler2)
  # add_subdirectory(demos/pdipm_sampler3)

  # add_subdirectory(demos/eigen_mkl_tester)
  add_subdirectory(Kiri2dExamples)

  
elseif(NOT APPLE AND ${ENABLE_CUDA})
  add_subdirectory(kiri2dPBSCuda)
  add_subdirectory(demos/dem2d_scene1)
endif()
