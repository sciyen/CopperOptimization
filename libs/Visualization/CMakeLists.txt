cmake_minimum_required(VERSION 2.8)

# OpenCV
find_package( OpenCV REQUIRED )
if ( NOT OpenCV_FOUND )
  message(STATUS "This project requires the opencv library, and will not be compiled.")

  return()  

endif()

add_library( Visualization visualization.cpp)

include_directories(../Geometry)
include_directories(../FileProcessing)
include_directories( ${OpenCV_INCLUDE_DIRS} )

target_link_libraries( Visualization PUBLIC Geometry)
target_link_libraries( Visualization PUBLIC FileProcessing)
target_link_libraries( Visualization PUBLIC ${OpenCV_LIBS} )
