# Created by the script cgal_create_CMakeLists
# This is the CMake script for compiling a set of CGAL applications.

cmake_minimum_required(VERSION 3.1...3.15)

project( Geometry )


# CGAL and its components
find_package( CGAL QUIET COMPONENTS Core )

if ( NOT CGAL_FOUND )

  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()  

endif()


# Boost and its components
find_package( Boost REQUIRED )

if ( NOT Boost_FOUND )

  message(STATUS "This project requires the Boost library, and will not be compiled.")

  return()  

endif()


# OpenCV
find_package( OpenCV REQUIRED )
if ( NOT OpenCV_FOUND )
  message(STATUS "This project requires the opencv library, and will not be compiled.")

  return()  

endif()


# include for local directory
add_library( Geometry geometry.cpp)

include_directories( ${OpenCV_INCLUDE_DIRS} )

# include for local package

target_link_libraries(Geometry CGAL::CGAL)

# Creating entries for all C++ files with "main" routine
target_link_libraries( Geometry ${OpenCV_LIBS} )
# ##########################################################