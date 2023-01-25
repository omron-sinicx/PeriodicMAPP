/*
Copyright (c) 2023 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#include "geometry_2D.hpp"
#include <cassert>
#include <map>
#include <vector>

using Point = Geometry2D::Point;

std::vector<Point> generate_perturbated_path(const Point &start,
                                             const Point &goal,
                                             const double d) {
  Point middle = 1. / 2. * (start + goal);

  static std::random_device rd;
  static std::default_random_engine eng(rd());
  std::uniform_real_distribution<double> distr(-d, d);
  middle += Point(distr(eng), distr(eng));
  std::vector<Point> path(3);
  path[0] = start, path[1] = middle, path[2] = goal;
  return path;
}

void calculate_intersection(
    const std::vector<std::vector<Point>> &paths, const int cycle,
    std::vector<std::vector<Point>> &refined_paths,
    std::vector<std::tuple<int, int, int, int>> &intersections) {
  refined_paths.resize(paths.size());
  std::map<std::pair<int, int>, int> waypoint_ids;
  for (int agent_0 = 0; agent_0 < paths.size(); agent_0++) {
    auto &path_0 = paths[agent_0];
    auto &refined_path = refined_paths[agent_0];
    refined_path.clear();
    refined_path.push_back(path_0[0]);
    for (int x_0 = 0; x_0 < path_0.size() - 1; x_0++) {
      auto point_00 = path_0[x_0], point_01 = path_0[x_0 + 1];
      std::vector<std::pair<double, int>> times;
      for (int agent_1 = 0; agent_1 < paths.size(); agent_1++) {
        if (agent_0 == agent_1)
          continue;
        auto &path_1 = paths[agent_1];
        for (int x_1 = 0; x_1 < path_1.size() - 1; x_1++) {
          auto point_10 = path_1[x_1], point_11 = path_1[x_1 + 1];
          if (Geometry2D::segment_intersect(point_00, point_01, point_10,
                                            point_11)) {
            double t = Geometry2D::ccw(point_10, point_11, point_00) /
                       (Geometry2D::ccw(point_10, point_11, point_00) -
                        Geometry2D::ccw(point_10, point_11, point_01));
            times.push_back(std::make_pair(t, agent_1));
          }
        }
      }
      std::sort(times.begin(), times.end());
      for (int i = 0; i < times.size(); i++) {
        double t_0 = i == 0 ? 0. : times[i - 1].first, t_1 = times[i].first;
        Point dir = point_01 - point_00;
        for (int c = 0; c <= cycle; c++) {
          refined_path.push_back(point_00 +
                                 ((cycle - c) * t_0 + (c + 1) * t_1) /
                                     (cycle + 1) * dir);
        }
        assert(waypoint_ids.find(std::make_pair(agent_0, times[i].second)) ==
               waypoint_ids.end());
        waypoint_ids[std::make_pair(agent_0, times[i].second)] =
            refined_path.size() - 1;
      }
      refined_path.push_back(point_01);
    }
  }
  for (auto &wp : waypoint_ids) {
    if (wp.first.first < wp.first.second) {
      intersections.push_back(std::make_tuple(
          wp.first.first, wp.second, wp.first.second,
          waypoint_ids[std::make_pair(wp.first.second, wp.first.first)]));
    }
  }
}
