cmake_minimum_required(VERSION 3.10)

project(sf)

set(LIB_PREFIX sf_)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")

option(BUILD_SHARED_LIBS "Build Shared Libraries" ON)
set(SF_LIB_TYPE STATIC)
if(BUILD_SHARED_LIBS)
  set(SF_LIB_TYPE SHARED)
endif(BUILD_SHARED_LIBS)

set(CMAKE_CXX_FLAGS "-g -Wall")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
#if (TARGET Eigen3::Eigen)
#  set(SF_EIGEN3_EIGEN_TARGET Eigen3::Eigen)
#else()
#  include_directories(${EIGEN3_INCLUDE_DIR})
#endif ()

# building our tests
option(BUILD_TESTS "build our tests" ON)
if(BUILD_TESTS)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  add_subdirectory(3rdparty/googletest)

  include_directories(${gtest_SOURCE_DIR}/include 
                    ${gtest_SOURCE_DIR})
  enable_testing()
  add_subdirectory(tests)
endif(BUILD_TESTS)

add_subdirectory(slamfactory)
