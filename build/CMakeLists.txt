cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(demo)

##set(PCL_DIR /usr/share/pcl-1.7)

##find_package(PCL 1.7 REQUIRED)

include_directories(/usr/local/include /usr/local/include/opencv) ##${PCL_INCLUDE_DIRS})

link_directories(/usr/local/lib)  #${OPENCV_LIBS} ${OPENCV_LIB})
##add_definitions(${PCL_DEFINITIONS})

add_executable (demo ../demo.cpp ../surf.cpp ../utils.cpp ../integral.cpp ../fasthessian.cpp ../ipoint.cpp ../capture.hpp)
target_link_libraries (demo opencv_contrib opencv_core opencv_highgui opencv_imgproc opencv_calib3d opencv_features2d)
