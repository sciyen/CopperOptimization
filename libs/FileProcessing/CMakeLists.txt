add_library(FileProcessing load.cpp)

include_directories(../Geometry)

target_link_libraries(FileProcessing PUBLIC Geometry)
