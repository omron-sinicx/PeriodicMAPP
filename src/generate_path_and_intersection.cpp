#include "compute_path_and_intersection.hpp"

const double EPS = 1e-8, pi = 3.14159265359;

int main(int argc, char **argv) {
  int number_of_holes;
  scanf("%d", &number_of_holes);
  for (int ring = 0; ring < number_of_holes + 1; ring++) {
    int number_of_vertices;
    scanf("%d", &number_of_vertices);
    for (int i = 0; i < number_of_vertices; i++) {
      double x, y;
      scanf("%lf%lf", &x, &y);
    }
  }
  int number_of_agents;
  double agent_radius;
  scanf("%d%lf", &number_of_agents, &agent_radius);
  std::vector<std::vector<Point>> paths(number_of_agents),
      refined_paths(number_of_agents);
  for (int i = 0; i < number_of_agents; i++) {
    int number_of_vertices = -1;
    // scanf("%d", &number_of_vertices);
    if (number_of_vertices == -1) {
      double x_0, y_0, x_1, y_1;
      scanf("%lf%lf%lf%lf", &x_0, &y_0, &x_1, &y_1);
      // paths[i] =
      //    generate_perturbated_path(Point(x_0, y_0), Point(x_1, y_1), 0.01);
      // if(abs(x_0-x_1)<EPS || abs(y_0-y_1)<EPS){
      paths[i].resize(2);
      paths[i][0] = Point(x_0, y_0);
      paths[i][1] = Point(x_1, y_1);
      /*}
    else{
      int N = 100;
      paths[i].resize(N+1);
      if(y_0<-abs(x_0)){
        if(x_1>0){
          double r = x_1-x_0;
          for(int k=0;k<=N;k++){
            double th = pi/2.*(double)k/N;
            paths[i][k] = Point(x_1-r*cos(th), y_0+r*sin(th));
          }
        }
        else{
          double r = x_0-x_1;
          for(int k=0;k<=N;k++){
            double th = pi/2.*(double)k/N;
            paths[i][k] = Point(x_1+r*cos(th), y_0+r*sin(th));
          }
        }
      }
      else if(x_0>abs(y_0)){
        double r = x_0-x_1;
        for(int k=0;k<=N;k++){
          double th = pi/2.*(double)k/N;
          paths[i][k] = Point(x_0-r*sin(th), y_1+r*cos(th));
        }
      }
      else{
        double r = x_1-x_0;
        for(int k=0;k<=N;k++){
          double th = pi/2.*(double)k/N;
          paths[i][k] = Point(x_0+r*sin(th), y_1+r*cos(th));
        }
      }
      }*/
    } else {
      paths[i].resize(number_of_vertices);
      for (int j = 0; j < number_of_vertices; j++) {
        double x, y;
        scanf("%lf%lf", &x, &y);
        paths[i][j] = Point(x, y);
      }
    }
  }
  int cycle = std::stoi(argv[1]);
  std::vector<std::tuple<int, int, int, int>> intersections;
  calculate_intersection(paths, 0, refined_paths, intersections);
  printf("%d %d\n", cycle, number_of_agents);
  for (int i = 0; i < number_of_agents; i++) {
    printf("%lu\n", refined_paths[i].size());
    for (int x = 0; x < refined_paths[i].size(); x++) {
      Geometry2D::printPoint(stdout, refined_paths[i][x]);
    }
  }
  for (auto &it : intersections) {
    int a0 = std::get<0>(it), t0 = std::get<1>(it), a1 = std::get<2>(it),
        t1 = std::get<3>(it);
    Point vec0 = refined_paths[a0][t0] - refined_paths[a0][0];
    Point vec1 = refined_paths[a1][t1] - refined_paths[a1][0];
    if (vec0.length() > vec1.length() + EPS ||
        abs(vec0.length() - vec1.length()) < EPS &&
            Geometry2D::outer(vec0, vec1) > 0) {
      std::swap(std::get<0>(it), std::get<2>(it));
      std::swap(std::get<1>(it), std::get<3>(it));
    }
  }
  printf("%lu\n", intersections.size());
  for (auto &it : intersections) {
    int a0 = std::get<0>(it), t0 = std::get<1>(it), a1 = std::get<2>(it),
        t1 = std::get<3>(it);
    printf("%d %d %d %d %d 0\n", a0, t0, cycle - 1, a1, t1);
  }
  printf("%lu\n", intersections.size());
  for (auto &it : intersections) {
    int a0 = std::get<0>(it), t0 = std::get<1>(it), a1 = std::get<2>(it),
        t1 = std::get<3>(it);
    printf("%d %d %d %d\n", a1, t1, a0, t0);
  }
  return 0;
}
