# Periodic Multi-Agent Path Planning

Study of multi-agent path planning under the setting in which agents appear periodically.

## Getting Started

This project uses C++17 standard library. Make sure your compiler support it.
Some tools, such as visualizer, uses Python3 with numpy and matplotlib.

### Prerequisities
- [CMake](https://cmake.org) &mdash; an open-source, cross-platform family of tools designed to build, test and package software.
- [yaml-cpp](https://github.com/jbeder/yaml-cpp) &mdash; a YAML parser and emitter in C++ matching the YAML 1.2 spec.
- [Eigen](https://eigen.tuxfamily.org) &mdash; a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.
- [g2o](https://github.com/RainerKuemmerle/g2o) &mdash; an open-source C++ framework for optimizing graph-based nonlinear error functions.
### Installing

Download this repository:
```
git clone https://github.com/omron-sinicx/PeriodicMAPP.git
```
At the downloaded repository, build by make command:
```
cmake -H. -Bbuild && make -j -C build
```

## Usage

### Initial Plan Generation

### Optimization

## Config Parameters

## File Formats
### Environment
> *number_of_obstacles* *number_of_agents* \
*description_of_polygon_for_outline*\
*description_of_polygon_for_obstacle_1*\
*description_of_polygon_for_obstacle_2*\
&#xFE19;\
*radius_of_agents*\
*agent_1_start_x* *agent_1_start_y* *agent_1_goal_x* *agent_1_goal_y*\
*agent_2_start_x* *agent_2_start_y* *agent_2_goal_x* *agent_2_goal_y*\
&#xFE19;

Polygons are described as follows:
> *number_of_vertices*\
 *vertex_1_x* *vertex_1_y*\
 *vertex_2_x* *vertex_2_y*\
&#xFE19;

### Plan
> *number_of_agents* *radius_of_agents*\
*description_of_path_for_agent_1*\
*description_of_path_for_agent_2*\
&#xFE19;

Paths are described as follows:
> *number_of_points*\
*time_1*\
 *x_1* *y_1*\
*time_2*\
 *x_2* *y_2*\
&#xFE19;

## License
This software is released under the MIT License, see [LICENSE](LICENSE).
