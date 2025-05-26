cmake_minimum_required(VERSION 3.10)
project(ORB_SLAM3_ROS2)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
IF(NOT COMPILER_SUPPORTS_CXX14)
  MESSAGE(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
ENDIF()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4.4)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.4 not found.")
   endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(Pangolin REQUIRED)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_sources(${PROJECT_NAME}
PRIVATE
   src/System.cc
   src/Tracking.cc
   src/LocalMapping.cc
   src/LoopClosing.cc
   src/ORBextractor.cc
   src/ORBmatcher.cc
   src/FrameDrawer.cc
   src/Converter.cc
   src/MapPoint.cc
   src/KeyFrame.cc
   src/Atlas.cc
   src/Map.cc
   src/MapDrawer.cc
   src/Optimizer.cc
   src/Frame.cc
   src/KeyFrameDatabase.cc
   src/Sim3Solver.cc
   src/Viewer.cc
   src/ImuTypes.cc
   src/G2oTypes.cc
   src/CameraModels/Pinhole.cpp
   src/CameraModels/KannalaBrandt8.cpp
   src/OptimizableTypes.cpp
   src/MLPnPsolver.cpp
   src/GeometricTools.cc
   src/TwoViewReconstruction.cc
   src/Config.cc
   src/Settings.cc
)

# Useless for now, but in future could be used for IDE tooling.
set(PUBLIC_HEADERS
   include/System.h
   include/Tracking.h
   include/LocalMapping.h
   include/LoopClosing.h
   include/ORBextractor.h
   include/ORBmatcher.h
   include/FrameDrawer.h
   include/Converter.h
   include/MapPoint.h
   include/KeyFrame.h
   include/Atlas.h
   include/Map.h
   include/MapDrawer.h
   include/Optimizer.h
   include/Frame.h
   include/KeyFrameDatabase.h
   include/Sim3Solver.h
   include/Viewer.h
   include/ImuTypes.h
   include/G2oTypes.h
   include/CameraModels/GeometricCamera.h
   include/CameraModels/Pinhole.h
   include/CameraModels/KannalaBrandt8.h
   include/OptimizableTypes.h
   include/MLPnPsolver.h
   include/GeometricTools.h
   include/TwoViewReconstruction.h
   include/SerializationUtils.h
   include/Config.h
   include/Settings.h
)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)

target_include_directories(${PROJECT_NAME} 
   PRIVATE
   ${PROJECT_SOURCE_DIR}
   ${PROJECT_SOURCE_DIR}/include
   ${PROJECT_SOURCE_DIR}/include/CameraModels
   ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
   ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2
   ${Pangolin_INCLUDE_DIRS}
)

# target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -pedantic)
# Disable all warnings
target_compile_options(${PROJECT_NAME} PRIVATE -w)

add_subdirectory(Thirdparty/g2o)
add_subdirectory(Thirdparty/DBoW2)

target_link_libraries(${PROJECT_NAME}
   ${OpenCV_LIBS}
   ${Pangolin_LIBRARIES}
   Eigen3::Eigen
   DBoW2::DBoW2
   g2o::g2o
   -lboost_serialization
   -lcrypto
)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# 1. Install targets
install(TARGETS
   ${PROJECT_NAME} DBoW2 g2o
   EXPORT ${PROJECT_NAME}Targets
   LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
   ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
   RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
   INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

# 2. Install main headers
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
   DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# Install third-party headers
install(
  DIRECTORY ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/Thirdparty
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
install(
  DIRECTORY ${PROJECT_SOURCE_DIR}/Thirdparty/g2o
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/Thirdparty
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
install(
  DIRECTORY ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/Thirdparty
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# 3. Export targets
install(EXPORT ${PROJECT_NAME}Targets
   FILE ${PROJECT_NAME}Targets.cmake
   NAMESPACE ${PROJECT_NAME}::
   DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# 4. Config file for find package
configure_package_config_file(
   cmake_modules/Config.cmake.in
   "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
   INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# 5. 
install(FILES
   "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
   DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# 6. Version File
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION 1.0.0
  COMPATIBILITY SameMajorVersion
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
