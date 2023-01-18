cmake_minimum_required(VERSION 3.16)
project(Periodic_MAPP)

add_compile_options(-g -O3 -std=c++17)
find_package(yaml-cpp)
find_package( Eigen3 3.3 REQUIRED )
find_package(g2o REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/src")
include_directories(${g2o_SOURCE_DIR} ${PROJECT_BINARY_DIR})

add_executable(optimize_periodic_plan src/optimize_periodic_plan.cpp)
target_link_libraries(optimize_periodic_plan yaml-cpp)
add_executable(optimize_periodic_plan_g2o src/optimize_periodic_plan_g2o.cpp)
target_link_libraries(optimize_periodic_plan_g2o yaml-cpp g2o_stuff g2o_core glog)
add_executable(generate_initial_periodic_plan src/generate_initial_periodic_plan.cpp)
add_executable(generate_general_initial_periodic_plan src/generate_general_initial_periodic_plan.cpp)
add_executable(generate_path_and_intersection src/generate_path_and_intersection.cpp)
