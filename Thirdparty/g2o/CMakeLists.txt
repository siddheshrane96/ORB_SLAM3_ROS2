CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
PROJECT(g2o)

if(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen3 3.4 REQUIRED NO_MODULE)

set(G2O_SOURCES
  #types
  g2o/types/types_sba.h
  g2o/types/types_six_dof_expmap.h
  g2o/types/types_sba.cpp
  g2o/types/types_six_dof_expmap.cpp
  g2o/types/types_seven_dof_expmap.cpp
  g2o/types/types_seven_dof_expmap.h
  g2o/types/se3quat.h
  g2o/types/se3_ops.h
  g2o/types/se3_ops.hpp
  #core
  g2o/core/base_edge.h
  g2o/core/base_binary_edge.h
  g2o/core/hyper_graph_action.cpp
  g2o/core/base_binary_edge.hpp
  g2o/core/hyper_graph_action.h
  g2o/core/base_multi_edge.h           
  g2o/core/hyper_graph.cpp
  g2o/core/base_multi_edge.hpp         
  g2o/core/hyper_graph.h
  g2o/core/base_unary_edge.h          
  g2o/core/linear_solver.h
  g2o/core/base_unary_edge.hpp         
  g2o/core/marginal_covariance_cholesky.cpp
  g2o/core/base_vertex.h               
  g2o/core/marginal_covariance_cholesky.h
  g2o/core/base_vertex.hpp             
  g2o/core/matrix_structure.cpp
  g2o/core/batch_stats.cpp             
  g2o/core/matrix_structure.h
  g2o/core/batch_stats.h               
  g2o/core/openmp_mutex.h
  g2o/core/block_solver.h              
  g2o/core/block_solver.hpp            
  g2o/core/parameter.cpp               
  g2o/core/parameter.h                 
  g2o/core/cache.cpp                   
  g2o/core/cache.h
  g2o/core/optimizable_graph.cpp       
  g2o/core/optimizable_graph.h         
  g2o/core/solver.cpp                  
  g2o/core/solver.h
  g2o/core/creators.h                 
  g2o/core/optimization_algorithm_factory.cpp
  g2o/core/estimate_propagator.cpp     
  g2o/core/optimization_algorithm_factory.h
  g2o/core/estimate_propagator.h       
  g2o/core/factory.cpp                 
  g2o/core/optimization_algorithm_property.h
  g2o/core/factory.h                   
  g2o/core/sparse_block_matrix.h
  g2o/core/sparse_optimizer.cpp  
  g2o/core/sparse_block_matrix.hpp
  g2o/core/sparse_optimizer.h
  g2o/core/hyper_dijkstra.cpp 
  g2o/core/hyper_dijkstra.h
  g2o/core/parameter_container.cpp     
  g2o/core/parameter_container.h
  g2o/core/optimization_algorithm.cpp 
  g2o/core/optimization_algorithm.h
  g2o/core/optimization_algorithm_with_hessian.cpp 
  g2o/core/optimization_algorithm_with_hessian.h
  g2o/core/optimization_algorithm_levenberg.cpp 
  g2o/core/optimization_algorithm_levenberg.h
  g2o/core/optimization_algorithm_gauss_newton.cpp 
  g2o/core/optimization_algorithm_gauss_newton.h
  g2o/core/jacobian_workspace.cpp 
  g2o/core/jacobian_workspace.h
  g2o/core/robust_kernel.cpp 
  g2o/core/robust_kernel.h
  g2o/core/robust_kernel_factory.cpp
  g2o/core/robust_kernel_factory.h
  g2o/core/robust_kernel_impl.cpp 
  g2o/core/robust_kernel_impl.h
  #stuff
  g2o/stuff/string_tools.h
  g2o/stuff/color_macros.h 
  g2o/stuff/macros.h
  g2o/stuff/timeutil.cpp
  g2o/stuff/misc.h
  g2o/stuff/timeutil.h
  g2o/stuff/os_specific.c    
  g2o/stuff/os_specific.h
  g2o/stuff/string_tools.cpp
  g2o/stuff/property.cpp       
  g2o/stuff/property.h
)

set(G2O_CXX_COMPILER "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER}")
# this writes config.h alongside config.h.in in the source
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/config.h.in
  ${CMAKE_CURRENT_SOURCE_DIR}/config.h
)

# Include the subdirectories
add_library(g2o STATIC ${G2O_SOURCES})
add_library(g2o::g2o ALIAS g2o)

set_target_properties(g2o PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(g2o
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/g2o/core
    ${CMAKE_CURRENT_SOURCE_DIR}/g2o/types
    ${CMAKE_CURRENT_SOURCE_DIR}/g2o/stuff
    ${CMAKE_CURRENT_SOURCE_DIR}/g2o/solvers
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(g2o
  PUBLIC
    Eigen3::Eigen
)

# target_compile_options(g2o PRIVATE -Wall -Wextra -pedantic)