cmake_minimum_required (VERSION 3.21.3)
project (Transformation)

find_package(Eigen3 REQUIRED)

set( EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )
if ( NOT EIGEN3_INCLUDE_DIR )
    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
ENDIF()
INCLUDE_DIRECTORIES ("${EIGEN3_INCLUDE_DIR}")

add_executable (Transformation main.cpp)
