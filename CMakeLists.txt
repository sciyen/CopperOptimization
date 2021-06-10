# Created by the script cgal_create_CMakeLists
# This is the CMake script for compiling a set of CGAL applications.

cmake_minimum_required(VERSION 3.1...3.15)

project( CopperOptimization VERSION 0.0)


# CGAL and its components
find_package( CGAL QUIET COMPONENTS Cores )

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
add_subdirectory(libs/Geometry)
list(APPEND EXTRA_LIBS Geometry)
list(APPEND EXTRA_INCLUDES "${PROJECT_SOURCE_DIR}/libs/Geometry")

add_subdirectory(libs/FileProcessing)
list(APPEND EXTRA_LIBS FileProcessing)
list(APPEND EXTRA_INCLUDES "${PROJECT_SOURCE_DIR}/libs/FileProcessing")

add_subdirectory(libs/Visualization)
list(APPEND EXTRA_LIBS Visualization)
list(APPEND EXTRA_INCLUDES "${PROJECT_SOURCE_DIR}/libs/Visualization")

# include for local package

# Boost
#set(Boost_USE_STATIC_LIBS OFF) 
#set(Boost_USE_MULTITHREADED ON)  
#set(Boost_USE_STATIC_RUNTIME OFF) 
#find_package(Boost REQUIRED COMPONENTS) 
#include_directories(${Boost_INCLUDE_DIRS})


# OpenCV
include_directories( ${OpenCV_INCLUDE_DIRS} )


# Creating entries for target: CopperOptimization
# ############################

add_executable( CopperOptimization ./src/main.cpp)

#add_to_cached_list( CGAL_EXECUTABLE_TARGETS CopperOptimization )

# Link the executable to CGAL and third-party libraries
target_link_libraries(CopperOptimization PRIVATE CGAL::CGAL )
target_link_libraries(CopperOptimization PUBLIC ${EXTRA_LIBS})

#target_link_libraries(CopperOptimization ${Boost_LIBRARIES})
target_link_libraries( CopperOptimization PUBLIC ${OpenCV_LIBS} )

target_include_directories(CopperOptimization PUBLIC
                           "${PROJECT_BINARY_DIR}"
                           ${EXTRA_INCLUDES}
                           )

