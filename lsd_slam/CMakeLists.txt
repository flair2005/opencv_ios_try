CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(lsd_slam)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../opencv_ios_try/FindPackage)
FIND_PACKAGE(Eigen3 3.1.0 REQUIRED)
FIND_PACKAGE(OpenCV 3.0.0 REQUIRED core highgui imgproc video features2d calib3d imgcodecs xfeatures2d)
FIND_PACKAGE(G2O REQUIRED)
find_package(SuiteParse REQUIRED)
FIND_PACKAGE(Boost REQUIRED system serialization filesystem thread)

SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})


INCLUDE_DIRECTORIES(
  ${G2O_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/thirdparty/Sophist
  ${CSPARSE_INCLUDE_DIR} #Has been set by SuiteParse
  ${CHOLMOD_INCLUDE_DIR} #Has been set by SuiteParse
  ${Boost_INCLUDE_DIRS}
)
message(${CHOLMOD_LIBRARIES})
message(${G2O_CORE_LIBRARY})
message(${G2O_SOLVER_CHOLMOD})
set(LIBS
  ${G2O_CORE_LIBRARY}
  ${CHOLMOD_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${G2O_SOLVER_CHOLMOD}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_SOLVER_DENSE}
  ${G2O_SOLVER_PCG}
  ${G2O_SOLVER_SLAM2D_LINEAR}
  ${G2O_SOLVER_STRUCTURE_ONLY}
  ${G2O_SOLVER_EIGEN}

  ${G2O_TYPES_DATA}
  ${G2O_TYPES_ICP}
  ${G2O_TYPES_SBA}
  ${G2O_TYPES_SCLAM2D}
  ${G2O_TYPES_SIM3}

  ${G2O_STUFF_LIBRARY}
  ${G2O_CLI_LIBRARY}
  ${G2O_TYPES_SLAM3D}
)

message(header: ${G2O_INCLUDE_DIR})
message(lib: ${G2O_TYPES_SLAM3D_LIBRARY})

set(lsd_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/DataStructures/Frame.cpp
  ${PROJECT_SOURCE_DIR}/src/DataStructures/FramePoseStruct.cpp
  ${PROJECT_SOURCE_DIR}/src/DataStructures/FrameMemory.cpp
  ${PROJECT_SOURCE_DIR}/src/SlamSystem.cpp
  ${PROJECT_SOURCE_DIR}/src/LiveSLAMWrapper.cpp
  ${PROJECT_SOURCE_DIR}/src/DepthEstimation/DepthMap.cpp
  ${PROJECT_SOURCE_DIR}/src/DepthEstimation/DepthMapPixelHypothesis.cpp
  ${PROJECT_SOURCE_DIR}/src/util/globalFuncs.cpp
  ${PROJECT_SOURCE_DIR}/src/util/SophusUtil.cpp
  ${PROJECT_SOURCE_DIR}/src/util/settings.cpp
  ${PROJECT_SOURCE_DIR}/src/util/Undistorter.cpp
  ${PROJECT_SOURCE_DIR}/src/Tracking/Sim3Tracker.cpp
  ${PROJECT_SOURCE_DIR}/src/Tracking/Relocalizer.cpp
  ${PROJECT_SOURCE_DIR}/src/Tracking/SE3Tracker.cpp
  ${PROJECT_SOURCE_DIR}/src/Tracking/TrackingReference.cpp
  ${PROJECT_SOURCE_DIR}/src/IOWrapper/Timestamp.cpp
  ${PROJECT_SOURCE_DIR}/src/GlobalMapping/FabMap.cpp
  ${PROJECT_SOURCE_DIR}/src/GlobalMapping/KeyFrameGraph.cpp
  ${PROJECT_SOURCE_DIR}/src/GlobalMapping/g2oTypeSim3Sophus.cpp
  ${PROJECT_SOURCE_DIR}/src/GlobalMapping/TrackableKeyFrameSearch.cpp

${PROJECT_SOURCE_DIR}/src/DataStructures/Frame.h
${PROJECT_SOURCE_DIR}/src/DataStructures/FramePoseStruct.h
${PROJECT_SOURCE_DIR}/src/DataStructures/FrameMemory.h
${PROJECT_SOURCE_DIR}/src/SlamSystem.h
${PROJECT_SOURCE_DIR}/src/LiveSLAMWrapper.h
${PROJECT_SOURCE_DIR}/src/DepthEstimation/DepthMap.h
${PROJECT_SOURCE_DIR}/src/DepthEstimation/DepthMapPixelHypothesis.h
${PROJECT_SOURCE_DIR}/src/util/globalFuncs.h
${PROJECT_SOURCE_DIR}/src/util/SophusUtil.h
${PROJECT_SOURCE_DIR}/src/util/settings.h
${PROJECT_SOURCE_DIR}/src/util/Undistorter.h
${PROJECT_SOURCE_DIR}/src/Tracking/Sim3Tracker.h
${PROJECT_SOURCE_DIR}/src/Tracking/Relocalizer.h
${PROJECT_SOURCE_DIR}/src/Tracking/SE3Tracker.h
${PROJECT_SOURCE_DIR}/src/Tracking/TrackingReference.h
${PROJECT_SOURCE_DIR}/src/IOWrapper/Timestamp.h
${PROJECT_SOURCE_DIR}/src/GlobalMapping/FabMap.h
${PROJECT_SOURCE_DIR}/src/GlobalMapping/KeyFrameGraph.h
${PROJECT_SOURCE_DIR}/src/GlobalMapping/g2oTypeSim3Sophus.h
${PROJECT_SOURCE_DIR}/src/GlobalMapping/TrackableKeyFrameSearch.h
)
set(SOURCE_FILES
  ${lsd_SOURCE_FILES}
  ${PROJECT_SOURCE_DIR}/src/IOWrapper/ROS/ROSImageStreamThread.cpp
  ${PROJECT_SOURCE_DIR}/src/IOWrapper/ROS/ROSOutput3DWrapper.cpp
  ${PROJECT_SOURCE_DIR}/src/IOWrapper/OpenCV/ImageDisplay_OpenCV.cpp
)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/thirdparty/Sophus
  ${CSPARSE_INCLUDE_DIR} #Has been set by SuiteParse
  ${CHOLMOD_INCLUDE_DIR} #Has been set by SuiteParse
)


ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} ${LIBS})
set_target_properties( ${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_LIST_DIR} )
set_target_properties( ${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_LIST_DIR} )

#######################

ADD_EXECUTABLE( ${PROJECT_NAME}_main main.cpp )

IF(APPLE)
target_link_libraries( ${PROJECT_NAME}_main
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/liblsd_slam.dylib
)
ELSE()
target_link_libraries( ${PROJECT_NAME}_main
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/liblsd_slam.so
)
ENDIF()

set_target_properties( ${PROJECT_NAME}_main PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR} )
set_target_properties( ${PROJECT_NAME}_main PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR} )
set_target_properties( ${PROJECT_NAME}_main PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_SOURCE_DIR} )
