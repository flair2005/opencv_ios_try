CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(lsd_chamo)

# Check C++11 or C++0x support
INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
IF(COMPILER_SUPPORTS_CXX11)
   SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   ADD_DEFINITIONS(-DCOMPILEDWITHC11)
   MESSAGE(STATUS "Using flag -std=c++11.")
ELSEIF(COMPILER_SUPPORTS_CXX0X)
   SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   ADD_DEFINITIONS(-DCOMPILEDWITHC0X)
   MESSAGE(STATUS "Using flag -std=c++0x.")
ELSE()
   MESSAGE(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
ENDIF()

LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/opencv_ios_try/FindPackage)
FIND_PACKAGE(Eigen3 3.1.0 REQUIRED)
FIND_PACKAGE(OpenCV 3.0.0 REQUIRED core highgui imgproc video features2d calib3d imgcodecs xfeatures2d)
FIND_PACKAGE(G2O REQUIRED)

SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})


INCLUDE_DIRECTORIES(
  ${G2O_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/opencv_ios_try
  ${CMAKE_CURRENT_LIST_DIR}/opencv_ios_try/sophus
)

set(LIBS
  ${G2O_CORE_LIBRARY}
  ${CHOLMOD_LIBRARIES}
  ${OpenCV_LIBS}
)

message(header: ${G2O_INCLUDE_DIR})
message(lib: ${G2O_TYPES_SLAM3D_LIBRARY})


OPTION(USE_SGD_SWITCH "Option description" ON)
IF(USE_SGD_SWITCH MATCHES ON)
  ADD_DEFINITIONS(-DUSE_SGD)
ENDIF()

ADD_LIBRARY(${PROJECT_NAME} SHARED
  ${CMAKE_CURRENT_LIST_DIR}/opencv_ios_try/TrackingReference.cpp
  ${CMAKE_CURRENT_LIST_DIR}/opencv_ios_try/TrackingReference.hpp
  ${CMAKE_CURRENT_LIST_DIR}/opencv_ios_try/SlamSystem.cpp
  ${CMAKE_CURRENT_LIST_DIR}/opencv_ios_try/SlamSystem.hpp
  ${CMAKE_CURRENT_LIST_DIR}/opencv_ios_try/setting.cpp
  ${CMAKE_CURRENT_LIST_DIR}/opencv_ios_try/setting.hpp
  ${CMAKE_CURRENT_LIST_DIR}/opencv_ios_try/SE3Tracker.cpp
  ${CMAKE_CURRENT_LIST_DIR}/opencv_ios_try/SE3Tracker.hpp
  ${CMAKE_CURRENT_LIST_DIR}/opencv_ios_try/KeyFrameGraph.cpp
  ${CMAKE_CURRENT_LIST_DIR}/opencv_ios_try/KeyFrameGraph.hpp
  ${CMAKE_CURRENT_LIST_DIR}/opencv_ios_try/FramePoseStruct.cpp
  ${CMAKE_CURRENT_LIST_DIR}/opencv_ios_try/FramePoseStruct.hpp
  ${CMAKE_CURRENT_LIST_DIR}/opencv_ios_try/FrameMemory.cpp
  ${CMAKE_CURRENT_LIST_DIR}/opencv_ios_try/FrameMemory.hpp
  ${CMAKE_CURRENT_LIST_DIR}/opencv_ios_try/Frame.cpp
  ${CMAKE_CURRENT_LIST_DIR}/opencv_ios_try/Frame.hpp
  ${CMAKE_CURRENT_LIST_DIR}/opencv_ios_try/DepthMapPixelHypothesis.cpp
  ${CMAKE_CURRENT_LIST_DIR}/opencv_ios_try/DepthMapPixelHypothesis.hpp
  ${CMAKE_CURRENT_LIST_DIR}/opencv_ios_try/DepthMap.cpp
  ${CMAKE_CURRENT_LIST_DIR}/opencv_ios_try/DepthMap.hpp

  ${CMAKE_CURRENT_LIST_DIR}/opencv_ios_try/SophusUtil.h
  ${CMAKE_CURRENT_LIST_DIR}/opencv_ios_try/LGSX.h
  ${CMAKE_CURRENT_LIST_DIR}/opencv_ios_try/globalFuncs.h
)
target_link_libraries(${PROJECT_NAME} ${LIBS})
set_target_properties( ${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_LIST_DIR} )
set_target_properties( ${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_LIST_DIR} )

#######################

ADD_EXECUTABLE( ${PROJECT_NAME}_main main.cpp )

IF(APPLE)
target_link_libraries( ${PROJECT_NAME}_main
  ${OpenCV_LIBS}
  ${PROJECT_SOURCE_DIR}/liblsd_chamo.dylib
)
ELSE()
target_link_libraries( ${PROJECT_NAME}_main
  ${OpenCV_LIBS}
  ${PROJECT_SOURCE_DIR}/liblsd_chamo.so
)
ENDIF()

set_target_properties( ${PROJECT_NAME}_main PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR} )
set_target_properties( ${PROJECT_NAME}_main PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR} )
set_target_properties( ${PROJECT_NAME}_main PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_SOURCE_DIR} )
