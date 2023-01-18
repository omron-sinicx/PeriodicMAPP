#include "initial_periodic_planner.hpp"

int main(int argc, char **argv) {
  int cycle, number_of_groups;
  scanf("%d%d", &cycle, &number_of_groups);

  InitialPeriodicPlanner planner(cycle, number_of_groups);

  planner.groups.resize(number_of_groups);
  for (int group = 0; group < number_of_groups; group++) {
    int number_of_waypoints;
    scanf("%d", &number_of_waypoints);
    auto &way_points = planner.groups[group].way_points;
    way_points.resize(number_of_waypoints);
    for (int vertex = 0; vertex < number_of_waypoints; vertex++) {
      double x, y;
      scanf("%lf%lf", &x, &y);
      way_points[vertex].first = Geometry2D::Point(x, y);
      if (vertex > 0) {
        way_points[vertex].second =
            (way_points[vertex].first - way_points[vertex - 1].first).length();
      }
    }
  }
  if (argc > 1) {
    planner.redundancy = std::stof(argv[1]);
  }
  if (argc > 2) {
    long long all_pattern = std::stoll(argv[2]);
    int number_of_intersections;
    scanf("%d", &number_of_intersections);
    const int n_patterns = 6;
    const char patterns[6][5] = {"0011", "0101", "0110",
                                 "1001", "1010", "1100"};
    for (int t = 0; t < number_of_intersections; t++) {
      int group_0, vertex_0, group_1, vertex_1;
      scanf("%d%d%d%d", &group_0, &vertex_0, &group_1, &vertex_1);
      auto pattern = patterns[all_pattern % n_patterns];
      all_pattern /= n_patterns;
      int order_0 = 0, order_1 = 0;
      for (int t = 0; t < 2 * cycle - 1; t++) {
        if (pattern[t] == '0' && pattern[t + 1] == '1') {
          InitialPeriodicPlanner::Constraints constraint(
              group_0, order_0, vertex_0, group_1, order_1, vertex_1);
          planner.constraints.push_back(constraint);
        } else if (pattern[t] == '1' && pattern[t + 1] == '0') {
          InitialPeriodicPlanner::Constraints constraint(
              group_1, order_1, vertex_1, group_0, order_0, vertex_0);
          planner.constraints.push_back(constraint);
        }
        if (pattern[t] == '0') {
          order_0++;
        } else {
          order_1++;
        }
      }
      if (pattern[2 * cycle - 1] == '0' && pattern[0] == '1') {
        InitialPeriodicPlanner::Constraints constraint(group_0, vertex_0,
                                                       group_1, vertex_1);
        planner.constraints_2.push_back(constraint);
      } else if (pattern[2 * cycle - 1] == '1' && pattern[0] == '0') {
        InitialPeriodicPlanner::Constraints constraint(group_1, vertex_1,
                                                       group_0, vertex_0);
        planner.constraints_2.push_back(constraint);
      }
    }
  } else {
    int number_of_constraints;
    scanf("%d", &number_of_constraints);
    planner.constraints.resize(number_of_constraints);
    for (int i = 0; i < number_of_constraints; i++) {
      auto &constraint = planner.constraints[i];
      scanf("%d%d%d%d%d%d", &constraint.group_0, &constraint.vertex_0,
            &constraint.order_0, &constraint.group_1, &constraint.vertex_1,
            &constraint.order_1);
    }
    int number_of_constraints_2;
    scanf("%d", &number_of_constraints_2);
    planner.constraints_2.resize(number_of_constraints_2);
    for (int i = 0; i < number_of_constraints_2; i++) {
      auto &constraint = planner.constraints_2[i];
      scanf("%d%d%d%d", &constraint.group_0, &constraint.vertex_0,
            &constraint.group_1, &constraint.vertex_1);
    }
  }
  if (!planner.calculate_plan()) {
    fprintf(stderr, "failed\n");
    return 0;
  }

  printf("%d %lf %lf %d\n", cycle, 0.0, planner.period, number_of_groups);
  /*for (int group = 1; group < number_of_groups; group++) {
    printf("0.0 ");
    }
    putchar('\n');*/
  for (int agent = 0; agent < number_of_groups * cycle; agent++) {
    auto &path = planner.paths[agent];
    printf("%lu\n", path.size());
    for (auto &pair : path) {
      printf("%lf %lf %lf\n", pair.first.x, pair.first.y, pair.second);
    }
  }

  return 0;
}
