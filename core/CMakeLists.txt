# Project Name
set(PROJECT_NAME KIRI2D)

if(WIN32)
    find_package(OpenMP REQUIRED)
    if(OPENMP_FOUND)
        message(STATUS "OpenMP_C_Found   : ${OpenMP_C_FOUND}")
        message(STATUS "OpenMP_CXX_Found : ${OpenMP_CXX_FOUND}")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    endif()
elseif(APPLE)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
endif()
# KIRI Math Library
set(KIRI_MATH_LIB_INLCUDE ${CMAKE_SOURCE_DIR}/external/KiriMathMini/include)

# Extra Library
set(EXTLIBS_INCLUDE ${CMAKE_SOURCE_DIR}/external/KiriExtlib/include)
set(EXTLIBS_LIB ${CMAKE_SOURCE_DIR}/external/KiriExtlib/lib)
link_directories(${EXTLIBS_LIB})

# Configuration
set(CONFIGURATION_INCLUDE ${CMAKE_BINARY_DIR}/configuration)

# KIRI 2d Library
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

add_library(${PROJECT_NAME} ${PROJECT_FILES})
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${PROJECT_FILES})

set_target_properties(
    ${PROJECT_NAME} PROPERTIES
    OUTPUT_NAME_DEBUG ${PROJECT_NAME}d
    OUTPUT_NAME_RELEASE ${PROJECT_NAME}
)
target_include_directories(${PROJECT_NAME} PUBLIC
    include
    ${EXTLIBS_INCLUDE}
    ${KIRI_MATH_LIB_INLCUDE}
    ${CONFIGURATION_INCLUDE}
)