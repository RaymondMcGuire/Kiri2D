# CMake Version
cmake_minimum_required(VERSION 3.18.0)

# Project Name
project(kiri2d LANGUAGES C CXX)

# For Visual Studio
if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
	# Optimization options for C/C++
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++17 /wd4819 /wd26812 /wd26495 /O2 /Ob2 /Ot /Oi /Gy /MP")
endif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")


# Compile Type
set(CMAKE_CONFIGURATION_TYPES "Debug;Release")

# Sub Projects
add_subdirectory(Kiri2d)
add_subdirectory(Kiri2dExamples)