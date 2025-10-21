# cmake_minimum_required(VERSION 3.16)

set(SOPHUS_VERSION 0.0)
# project(Sophus VERSION ${SOPHUS_VERSION})
project(Sophus)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

# Determine if sophus is built as a subproject (using add_subdirectory) or if it
# is the master project.
if(NOT DEFINED SOPHUS_MASTER_PROJECT)
  set(SOPHUS_MASTER_PROJECT OFF)
  if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    set(SOPHUS_MASTER_PROJECT ON)
    message(STATUS "CMake version: ${CMAKE_VERSION}")
  endif()
endif()

option(SOPHUS_INSTALL "Generate the install target." ${SOPHUS_MASTER_PROJECT})

if(SOPHUS_MASTER_PROJECT)
  # Release by default Turn on Debug with "-DCMAKE_BUILD_TYPE=Debug"
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
  endif()

  set(CMAKE_CXX_STANDARD 14)

  set(CMAKE_COMPILE_WARNING_AS_ERROR Off)

  # Set compiler specific settings (FixMe: Should not cmake do this for us
  # automatically?)
  if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    add_compile_options(-Wall -Wextra -Wno-deprecated-register
                        -Qunused-arguments -fcolor-diagnostics)
  elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    add_compile_options(-Wall -Wextra -Wno-deprecated-declarations
                        -ftemplate-backtrace-limit=0 -Wno-array-bounds)
  elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    add_compile_options(/bigobj /wd4305 /wd4244 /MP)
    add_compile_definitions(_USE_MATH_DEFINES)
  endif()

  # Add local path for finding packages, set the local version first
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules")
endif()

# Find public dependencies if targets are not yet defined. (Targets might be for
# example defined by a parent project including Sophus via `add_subdirectory`.)
# if(NOT TARGET Eigen3::Eigen)
#   find_package(Eigen3 3.3.9 REQUIRED)
# endif()

# Define interface library target
set(SOPHUS_HEADER_FILES
    sophus/scso3.hpp
    sophus/se2.hpp
    sophus/se3.hpp
    sophus/sim3.hpp
    # sophus/so2.hpp
    sophus/so3.hpp)

set(LIBNAME Sophus)

include_directories(${SOPHUS_HEADER_FILES}
                    ${EIGEN3_INCLUDE_DIR})
                   
add_library(Sophus STATIC
  sophus/scso3.cpp
  sophus/se2.cpp
  sophus/se3.cpp
  sophus/sim3.cpp
#   sophus/so2.cpp
  sophus/so3.cpp
)
