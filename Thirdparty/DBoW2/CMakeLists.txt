cmake_minimum_required(VERSION 3.10)
project(DBoW2)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(OpenCV 4 REQUIRED)

set(HDRS_DBOW2
  DBoW2/BowVector.h
  DBoW2/FORB.h 
  DBoW2/FClass.h       
  DBoW2/FeatureVector.h
  DBoW2/ScoringObject.h   
  DBoW2/TemplatedVocabulary.h)
set(SRCS_DBOW2
  DBoW2/BowVector.cpp
  DBoW2/FORB.cpp      
  DBoW2/FeatureVector.cpp
  DBoW2/ScoringObject.cpp)

set(HDRS_DUTILS
  DUtils/Random.h
  DUtils/Timestamp.h)
set(SRCS_DUTILS
  DUtils/Random.cpp
  DUtils/Timestamp.cpp)

add_library(DBoW2 STATIC ${SRCS_DBOW2} ${SRCS_DUTILS})
add_library(DBoW2::DBoW2 ALIAS DBoW2)
# Needed to avoid -fPIC errors when linking against OpenCV
set_target_properties(DBoW2 PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(DBoW2 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(DBoW2 PUBLIC ${OpenCV_LIBS})
