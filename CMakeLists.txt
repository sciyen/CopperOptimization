# Created by the script cgal_create_CMakeLists
# This is the CMake script for compiling a set of CGAL applications.

cmake_minimum_required(VERSION 3.1...3.15)

project( CopperOptimization VERSION 0.0)


# CGAL and its components
find_package( CGAL QUIET COMPONENTS  )

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

# include for local directory
add_subdirectory(libs/Geometry)
list(APPEND EXTRA_LIBS Geometry)
list(APPEND EXTRA_INCLUDES "${PROJECT_SOURCE_DIR}/libs/Geometry")

add_subdirectory(libs/FileProcessing)
list(APPEND EXTRA_LIBS FileProcessing)
list(APPEND EXTRA_INCLUDES "${PROJECT_SOURCE_DIR}/libs/FileProcessing")
# include for local package


# Creating entries for target: CopperOptimization
# ############################

add_executable( CopperOptimization ./src/main.cpp)

#add_to_cached_list( CGAL_EXECUTABLE_TARGETS CopperOptimization )

# Link the executable to CGAL and third-party libraries
target_link_libraries(CopperOptimization PRIVATE CGAL::CGAL )
target_link_libraries(CopperOptimization PUBLIC ${EXTRA_LIBS})

target_include_directories(CopperOptimization PUBLIC
                           "${PROJECT_BINARY_DIR}"
                           ${EXTRA_INCLUDES}
                           )

