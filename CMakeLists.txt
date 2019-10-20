cmake_minimum_required(VERSION 2.8.3)
project(3dDefectDetection_online_ros)
message(STATUS "Executig CMakeLists of project  ${PROJECT_NAME}.")


#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)



# OpenCV
set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# GoSDK and KAPI
# Set GO_SDK header files (That is different names for Sdk5.x )
SET(GO_SDK ~/GO_SDK)
FIND_PATH(
    GOCATOR_INCLUDES
    NAMES GoSdk/GoSdk.h
    PATHS ${GO_SDK}/Gocator/GoSdk)
FIND_PATH(
    KAPI_INCLUDES
    NAMES kApi/kApi.h
    PATHS ${GO_SDK}/Platform/kApi)
INCLUDE_DIRECTORIES(${GOCATOR_INCLUDES} ${KAPI_INCLUDES} ./include)

# Set GO_SDK libs (That is the same name for both GoSDK4.x and GoSDK5.x )

# Release Version
if(${CMAKE_BUILD_TYPE} STREQUAL "Release")
message("CMAKE_BUILD_TYPE is Release")
FIND_LIBRARY(
    GOCATOR_LIBRARIES
    NAMES GoSdk
    PATHS ${GO_SDK}/lib/linux_x64/) 
FIND_LIBRARY(
    KAPI_LIBRARIES
    NAMES kApi
    PATHS ${GO_SDK}/lib/linux_x64/)
message(STATUS "Gocator Headers   found at ${GOCATOR_INCLUDES}.")
message(STATUS "Gocator Libraries found at ${GOCATOR_LIBRARIES}.")
endif()

# Debug Version
if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
FIND_LIBRARY(
    GOCATOR_LIBRARIES
    NAMES GoSdk
    PATHS ${GO_SDK}/lib/linux_x64d/) 
FIND_LIBRARY(
    KAPI_LIBRARIES
    NAMES kApi
    PATHS ${GO_SDK}/lib/linux_x64d/)
message(STATUS "Gocator Headers   found at ${GOCATOR_INCLUDES}.")
message(STATUS "Gocator Libraries found at ${GOCATOR_LIBRARIES}.")
endif()

include_directories(include)

#needs to be put after find_package()? yes. they depend on PCL OpenCV
#add_library() can be linked between sub_dirs? seems yes. to be confirmed.
add_subdirectory(src/tools)
add_subdirectory(src/libs)
add_subdirectory(src/demo)



# Build library
# Set source files
SET(SRCS src/gocator3100.cpp src/gocator3200.cpp)
# Set header files 
SET(HDRS include/gocator3100.h include/gocator3200.h)
# Add library
ADD_LIBRARY(${PROJECT_NAME}_LIB SHARED ${SRCS})
TARGET_LINK_LIBRARIES(${PROJECT_NAME}_LIB ${GOCATOR_LIBRARIES} ${KAPI_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

#Build examples 
ADD_SUBDIRECTORY(src/examples)


# Build demo_3200
ADD_EXECUTABLE(demo_3200 src/demo_3200.cpp src/dbscan.cpp src/DefectDetect.cpp)
TARGET_LINK_LIBRARIES(demo_3200 ${PROJECT_NAME}_LIB config pointcloud_helper)


##################################### build #####################################################
add_executable(normalEstimate_demo src/normalEstimate_demo.cpp src/dbscan.cpp)
target_link_libraries (normalEstimate_demo ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} config pointcloud_helper)

add_executable(defectdetect_demo src/DefectDetect_demo.cpp src/dbscan.cpp src/DefectDetect.cpp)
target_link_libraries (defectdetect_demo ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} config pointcloud_helper)

add_executable(normalEstimate_pcl_website src/normalEstimate_pcl_website.cpp )
target_link_libraries (normalEstimate_pcl_website ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(fit_plane_from_file src/fit_plane_from_file.cpp )
target_link_libraries (fit_plane_from_file ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} config pointcloud_helper)


