# Periodic Multi-Agent Path Planning

This is a repository for the following paper:
- Kazumi Kasaura, Ryo Yonetani, and Mai Nishimura. 2023. “Periodic Multi-Agent Path Planning.” In <i>AAAI Conference on Artificial
Intelligence</i>.

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

`test/scripts/optimize.sh` is a script used in experiments, which is an example of usage.

### Initial Plan Generation
`generate_path_and_intersection` inputs a cycle $M$ as an argument and a problem instance from standard input.
It outputs a condition for initial plan.

`generate_general_initial_periodic_plan` inputs a parameterm, which represents a temporal room at intersections and set to $1.0$ in our experiments, as an argument
and a condition for initial plan from standart input. It outputs an initial plan.
<pre><code>./generate_path_and_intersection <i>M</i> < problem_instance.txt > condition.txt
./generate_general_initial_periodic_plan <i>temporal_room</i> < condition.txt > initial_plan.txt
</code></pre>

### Optimization
`optimize_periodic_plan_g2o` optimizes initial plan. Its input and output files are specified by config file.
If there is no argument, it read config from "../config/optimize_periodic_plan_g2o.yaml".
<pre><code>./optimize_periodic_plan_g2o</code></pre>
You can also use your configuration file as follows:
<pre><code>./optimize_periodic_plan_g2o your_config.yaml</code></pre>
## Config Parameters

These parameters can be set by config file for optimization.

- `input`: The file path to input the initial plan.
- `output`: The file path to output the optimized plan.
- `Environment`: The file path to input the environment.
- `number_of_way_points`: The number of way points, which is $K+1$ in our paper.
- `target_agent_radius`: The target value $r$ of agent radius.
- `max_velocity`: The upperbound $v_{\mathrm{max}}$ of velocity.
- `target_period`: The target value of period, which is $\frac{2r}{v_{\mathrm{max}}}$ in our paper.
- `path_cost_method`: If it is `default`, the term of squared sum of velocities are added to objective function. If it is `nothing`, this term is not added.
- `path_cost`: The value of path cost coefficient $\sigma_{\mathrm{t}}$.
- `agent_radius_cost`: The value of radius penalty coefficient $\sigma_{\mathrm{r}}$.
- `max_velocity_cost`: The value of max valocity penalty coefficient $\sigma_{\mathrm{v}}$.
- `collision_cost`: The value of obstacle penalty coefficient $\sigma_{\mathrm{o}}$ and collision penalty coefficient $\sigma_{\mathrm{c}}$.
- `variable_delta`: If it is `true`, time durations $\Delta t$ are considered as variables and modified when optimization. Otherwise, they are fixed.
- `number_of_iterations`: A sequence of numbers of iterations
- `graph_update_intervals`: A sequence of periods of graph updations. The length of this sequences must be the same as that of `number_of_iterations`. First, optimizer runs with the first value of `number_of_iterations` as the number of iterations and the first value of `graph_update_intervals` as period of graph updations. Second, optimizer runs with the second values. And so on.
- `until_converge`: If this parameter is specified, optimizer runs until the difference of objective values becomes less than its value.
- `increase_coefficients`: If it is `true`, the penalty coefficients are multiplied when the last optimization steps corresponding the last value of `number_of_iterations`.
- `coefficient_growth_rate`: The value multiplied to the penalty coefficients.
- `uninitialized_lambda`: If it is `true`, the lambda value for Levenberg-Marquardt Algorithm is not reset when the graph is updated.

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

### Problem Instance
> *description_of_environment* \
*number_of_pairs* *radius_of_agents* \
*start_1_x start_1_y goal_1_x goal_1_y* \
*start_2_x start_2_y goal_2_x goal_2_y* \
&#xFE19;

### Plan
> *cycle* *radius_of_agents* *period* *number_of_pairs* \
*description_of_trajectory_for_pair_1_period_0*\
*description_of_trajectory_for_pair_1_period_1*\
&#xFE19; \
*description_of_trajectory_for_pair_2_period_0*\
*description_of_trajectory_for_pair_2_period_1*\
&#xFE19; \
&#xFE19;

Trajectories are described as follows:
> *number_of_way_points*\
 *x_1* *y_1* *time_1*\
 *x_2* *y_2* *time_2*\
&#xFE19;

## License
This software is released under the MIT License, see [LICENSE](LICENSE).
