# Project Name
set(PROJECT_NAME DEM2dScene1)

# Enable CUDA
set(ENABLE_CUDA TRUE CACHE BOOL "enable cuda or not")

# External Library Package
set(EXTLIBS_INCLUDE ${CMAKE_SOURCE_DIR}/external/KiriExtlib/include)
set(EXTLIBS_LIB ${CMAKE_SOURCE_DIR}/external/KiriExtlib/lib)
link_directories(${EXTLIBS_LIB})

# Configuration File
configure_file(${CMAKE_SOURCE_DIR}/configuration/root_directory.h.in  ${CMAKE_BINARY_DIR}/configuration/root_directory.h)
set(CONFIGURATION_INCLUDE ${CMAKE_BINARY_DIR}/configuration)

# Kiri2d Core Library
set(KIRI2D_LIB_INLCUDE ${CMAKE_SOURCE_DIR}/core/include)

# OpenMP
if(WIN32)
    find_package(OpenMP REQUIRED)
    if(OPENMP_FOUND)
        message(STATUS "OpenMP_C_Found   : ${OpenMP_C_FOUND}")
        message(STATUS "OpenMP_CXX_Found : ${OpenMP_CXX_FOUND}")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    endif()
endif()

if(APPLE)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS ON)
    set(OpenCV_DIR "/Users/raymondmg/Project/opencv/build")
        find_package(OpenCV REQUIRED)
endif()

# Project Config
file(GLOB_RECURSE SOURCES  "src/*.cpp" "src/*.c")
file(GLOB_RECURSE HEADERS  "include/*.hpp" "include/*.h")
set(PROJECT_FILES ${SOURCES} ${HEADERS})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ../bin/Release)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ../bin/Debug)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ../lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ../lib/Release)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ../lib/Debug)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ../lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ../lib/Release)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ../lib/Debug)

add_executable(${PROJECT_NAME} ${PROJECT_FILES})

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${PROJECT_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC
    include
    ${CONFIGURATION_INCLUDE}
    ${KIRI2D_LIB_INLCUDE}
)

if(WIN32 AND ${ENABLE_CUDA})
    target_link_libraries(${PROJECT_NAME} debug opencv_world451d optimized opencv_world451 debug partiod optimized partio kiri2d kiri2d_pbs_cuda)
endif()

# WorkSpace
set(WD_DEBUG_FILE_PATH ${CMAKE_BINARY_DIR}/demos/bin/Debug/)
set(WD_RELEASE_FILE_PATH ${CMAKE_BINARY_DIR}/demos/bin/Release/)

set_target_properties(
    ${PROJECT_NAME} PROPERTIES
    OUTPUT_NAME_DEBUG ${PROJECT_NAME}d
    OUTPUT_NAME_RELEASE ${PROJECT_NAME}
    VS_DEBUGGER_WORKING_DIRECTORY "$<$<CONFIG:debug>:${WD_DEBUG_FILE_PATH}>$<$<CONFIG:release>:${WD_RELEASE_FILE_PATH}>"
)

#Copy DLLs
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_SOURCE_DIR}/external/KiriExtlib/dlls" $<TARGET_FILE_DIR:${PROJECT_NAME}>)
