##########################################################################
#### This is a CMakeLists.txt file for building project
####  using OpenCV library and raspicam library
#### Please modify indicated area to make it work on your system
#### Erase [Your File Name] part, and put your file name instead
####
####                       (Example)
#### add_executable ([Your File Name] [Your File Name].cpp)  
#### target_link_libraries ([Your File Name] ${raspicam_CV_LIBS})
####                           To
#### add_executable (simpletest_raspicam_cv simpletest_raspicam_cv.cpp)  
#### target_link_libraries (simpletest_raspicam_cv ${raspicam_CV_LIBS})
##########################################################################


######################################################
cmake_minimum_required (VERSION 2.8) 
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/local/lib/cmake/)
project (raspicam_test)
find_package(raspicam REQUIRED)
find_package(OpenCV)
IF  ( OpenCV_FOUND AND raspicam_CV_FOUND)
MESSAGE(STATUS "COMPILING OPENCV TESTS")
#### add_executable ([Your File Name] [Your File Name].cpp)  
#### target_link_libraries ([Your File Name] ${raspicam_CV_LIBS})
ELSE()
MESSAGE(FATAL_ERROR "OPENCV NOT FOUND IN YOUR SYSTEM")
ENDIF()
#######################################################

