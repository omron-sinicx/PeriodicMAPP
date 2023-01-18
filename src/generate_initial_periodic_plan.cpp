#include "initial_periodic_planner.hpp"

int main() {
  int cycle, number_of_groups;
  scanf("%d%d", &cycle, &number_of_groups);

  SimpleInitialPeriodicPlanner planner(cycle, number_of_groups);

  planner.groups.resize(number_of_groups);
  for (int group = 0; group < number_of_groups; group++) {
    double x_0, y_0, x_1, y_1, length;
    scanf("%lf%lf%lf%lf%lf", &x_0, &y_0, &x_1, &y_1, &length);
    auto &group_data = planner.groups[group];
    group_data.start = Geometry2D::Point(x_0, y_0);
    group_data.goal = Geometry2D::Point(x_1, y_1);
    group_data.length = length;
    group_data.start_strage.resize(cycle);
    group_data.start_strage[cycle - 1] = std::make_pair(group_data.start, 0.);
    for (int order = 0; order < cycle - 1; order++) {
      double x_2, y_2, time;
      scanf("%lf%lf%lf", &x_2, &y_2, &time);
      group_data.start_strage[order] =
          std::make_pair(Geometry2D::Point(x_2, y_2), time);
    }
    group_data.goal_strage.resize(cycle);
    group_data.goal_strage[cycle - 1] = std::make_pair(group_data.goal, 0.);
    for (int order = 0; order < cycle - 1; order++) {
      double x_2, y_2, time;
      scanf("%lf%lf%lf", &x_2, &y_2, &time);
      group_data.goal_strage[order] =
          std::make_pair(Geometry2D::Point(x_2, y_2), time);
    }
  }

  int number_of_constraints;
  scanf("%d", &number_of_constraints);
  planner.constraints.resize(number_of_constraints);
  for (int i = 0; i < number_of_constraints; i++) {
    auto &constraint = planner.constraints[i];
    scanf("%d%d%d%d%lf", &constraint.group_0, &constraint.order_0,
          &constraint.group_1, &constraint.order_1, &constraint.time_diff);
  }
  int number_of_constraints_2;
  scanf("%d", &number_of_constraints_2);
  planner.constraints_2.resize(number_of_constraints_2);
  for (int i = 0; i < number_of_constraints_2; i++) {
    auto &constraint = planner.constraints_2[i];
    scanf("%d%d%d%d%lf", &constraint.group_0, &constraint.order_0,
          &constraint.group_1, &constraint.order_1, &constraint.time_diff);
  }
  if (!planner.calculate_plan()) {
    fprintf(stderr, "failed\n");
    return 0;
  }

  printf("%d %d 0.5 %lf %d\n", number_of_groups * cycle, cycle, planner.period,
         number_of_groups);
  for (int group = 1; group < number_of_groups; group++) {
    printf("0.0 ");
  }
  putchar('\n');
  for (int agent = 0; agent < number_of_groups * cycle; agent++) {
    auto &path = planner.paths[agent];
    printf("%lu %d %d\n", path.size(), agent % cycle, agent / cycle);
    for (auto &pair : path) {
      printf("%lf %lf %lf\n", pair.first.x, pair.first.y, pair.second);
    }
  }

  return 0;
}
