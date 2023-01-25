/*
Copyright (c) 2023 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

template <class Geometry> class GeometricPath {
public:
  using Point = typename Geometry::Point;
  std::vector<Point> way_points, moments, moments_2;
  double delta, delta_moment = 0., delta_moment_2 = 0., delay;
  int group_id;

  GeometricPath(){};
  GeometricPath(const std::vector<Point> &way_points) {
    this->way_points = way_points;
  }

  void getPoint(const double &time, Point &p, Point &dpdt, int &i,
                double &alpha) {
    i = std::ceil((time - delay) / delta);
    if (i <= 0) {
      p = way_points[0];
      dpdt = Point();
      alpha = 1.0;
    } else if (i < way_points.size()) {
      dpdt = (way_points[i] - way_points[i - 1]) / delta;
      alpha = (time - delay - (i - 1) * delta) / delta;
      p = (1.0 - alpha) * way_points[i - 1] + alpha * way_points[i];
    } else {
      p = way_points[way_points.size() - 1];
      dpdt = Point();
      alpha = 0.0;
    }
  }
};

template <class Geometry> class GeometricPaths {
public:
  using Point = typename Geometry::Point;
  std::vector<GeometricPath<Geometry>> paths;
  std::vector<int> delay;
  int cycle;
  double period;
  int number_of_groups;
  double time_margin = 0.;
  std::vector<double> start_times, start_times_memonts, start_times_memonts_2;
  std::vector<std::vector<bool>> collision_allowed;
  double agent_radius, agent_radius_moment = 0., agent_radius_moment_2 = 0.;

  // Hyperparameter
  double learning_rate, learning_epsiron = 1e-2, adam_beta_1 = 0.9,
                        adam_beta_2 = 0.999;
  bool optimize_start_time;

  double period_moment = 0., period_moment_2 = 0.;
  int count = 0;

  FILE *log = NULL;
  int log_epoch = 1;

  double EPS = 1e-6;

private:
  void update_ADAM(double &w, double &m, double &v, const double d) {
    m = adam_beta_1 * m + (1 - adam_beta_1) * d;
    v = adam_beta_2 * v + (1 - adam_beta_2) * d * d;
    double corr_1 = 1. / (1. - std::pow(adam_beta_1, count));
    double corr_2 = 1. / (1. - std::pow(adam_beta_2, count));
    w -= learning_rate / (std::sqrt(corr_2 * v) + learning_epsiron) * m;
  }

public:
  // Const Parameter
  double period_cost;
  std::function<std::tuple<double, double, double>(const double, const double)>
      distance_cost_func;
  std::function<double(const std::vector<Point> &, const double,
                       std::vector<Point> &, double &)>
      path_cost_func;
  // std::function<double(const double)> path_goal_force, path_goal_cost_func;
  std::function<std::tuple<double, Point, double>(const double, const Point)>
      field_potential;
  bool variale_agent_radius = false, variable_delta = false;
  std::function<std::pair<double, double>(const double)> agent_radius_cost_func;

  // Store best plan
  double best_cost = std::numeric_limits<double>::infinity(), best_period;
  std::vector<double> best_start_times;
  std::vector<std::vector<Point>> best_way_points;

  double update() {
    double time_cost = period_cost * period, path_cost = 0.,
           collision_cost = 0., obstacle_cost = 0.;
    std::vector<std::vector<Point>> gradients(paths.size());
    for (int path_id = 0; path_id < paths.size(); path_id++) {
      gradients[path_id].resize(paths[path_id].way_points.size());
    }
    std::vector<double> start_gradients(number_of_groups, 0.0),
        delta_gradients(paths.size(), 0.);
    double period_gradient = 0., radius_grad = 0.;
    for (int path_id = 0; path_id < paths.size(); path_id++) {
      auto &path = paths[path_id];
      auto &grad = gradients[path_id];
      path_cost += path_cost_func(path.way_points, path.delta, grad,
                                  delta_gradients[path_id]);
      auto &goal = *path.way_points.rbegin();
      for (int i = 1; i < path.way_points.size() - 1; i++) {
        auto &point = path.way_points[i];
        double time = path.delay + i * path.delta;
        Point dpdt = Point();
        double length = (path.way_points[i] - path.way_points[i - 1]).length();
        if (length > 0) {
          dpdt += (path.way_points[i] - path.way_points[i - 1]) / length / 2.;
        }
        double length_2 =
            (path.way_points[i] - path.way_points[i + 1]).length();
        if (length_2 > 0) {
          dpdt += (path.way_points[i + 1] - path.way_points[i]) / length_2 / 2.;
        }
        // double goal_dist = (goal - path.way_points[i]).length();
        // path_cost += path_goal_cost_func(goal_dist);
        // grad[i] +=
        //    path_goal_force(goal_dist) * (path.way_points[i] - goal).unit();

        auto obstacle_cost_and_dif =
            field_potential(agent_radius, path.way_points[i]);
        obstacle_cost += std::get<0>(obstacle_cost_and_dif);
        grad[i] += std::get<1>(obstacle_cost_and_dif);
        radius_grad += std::get<2>(obstacle_cost_and_dif);

        for (int path_id_0 = 0; path_id_0 < paths.size(); path_id_0++) {
          auto &path_0 = paths[path_id_0];
          int l = path_0.way_points.size();
          // path_0.time_stamps[l-1] + order * period * cycle >= time
          // path_0.time_stamps[0] + order * period * cycle <= time
          int lower_order =
              std::ceil((time - path_0.delay - (l - 1) * path_0.delta) /
                        (cycle * period));
          int upper_order =
              std::floor((time - path_0.delay) / (cycle * period));
          for (int order = lower_order; order <= upper_order; order++) {
            if (path_id == path_id_0 && order == 0)
              continue;
            if (order == 0 && collision_allowed[path_id][path_id_0])
              continue;
            double time_0 = time - order * cycle * period;
            Point point_0, dpdt_0;
            int i_0;
            double alpha;
            path_0.getPoint(time_0, point_0, dpdt_0, i_0, alpha);
            if ((point_0 - point).length() == 0)
              continue;
            double dist = (point - point_0).length();
            Point direction = (point - point_0) / dist;
            dist -= time_margin * std::abs(Geometry::inner(direction, dpdt)) +
                    time_margin * std::abs(Geometry::inner(direction, dpdt_0));
            if (dist <= 0.) {
              continue;
            }
            auto collision_tpl = distance_cost_func(agent_radius, dist);
            collision_cost += std::get<0>(collision_tpl);
            double F = std::get<1>(collision_tpl);
            radius_grad += std::get<2>(collision_tpl);
            grad[i] += F * direction;
            delta_gradients[path_id] +=
                -1. * F * i * Geometry::inner(direction, dpdt_0);
            if (i_0 - 1 > 0 && i_0 - 1 < path_0.way_points.size() - 1) {
              gradients[path_id_0][i_0 - 1] += -(1.0 - alpha) * F * direction;
            }
            if (i_0 > 0 && i_0 < path_0.way_points.size() - 1) {
              gradients[path_id_0][i_0] += -alpha * F * direction;
            }
            delta_gradients[path_id_0] +=
                F * time_0 / path_0.delta * Geometry::inner(direction, dpdt_0);
            period_gradient +=
                F * Geometry::inner(direction,
                                    delay[path_id] * dpdt -
                                        (delay[path_id] - delay[path_id_0] -
                                         order * cycle) *
                                            dpdt_0);
            if (path.group_id > 0) {
              start_gradients[path.group_id] -=
                  F * Geometry::inner(direction, dpdt);
            }
            if (path_0.group_id > 0) {
              start_gradients[path_0.group_id] -=
                  F * Geometry::inner(direction, -1. * dpdt_0);
            }
          }
        }
      }
    }
    count++;
    update_ADAM(period, period_moment, period_moment_2,
                period_cost + period_gradient);
    double radius_cost = 0.;
    if (variale_agent_radius) {
      auto radius_cost_tpl = agent_radius_cost_func(agent_radius);
      update_ADAM(agent_radius, agent_radius_moment, agent_radius_moment_2,
                  radius_grad + radius_cost_tpl.second);
      radius_cost = radius_cost_tpl.first;
    }
    if (optimize_start_time) {
      for (int group = 1; group < number_of_groups; group++) {
        update_ADAM(start_times[group], start_times_memonts[group],
                    start_times_memonts_2[group], start_gradients[group]);
      }
    }
    for (int path_id = 0; path_id < paths.size(); path_id++) {
      auto &path = paths[path_id];
      if (variable_delta) {
        update_ADAM(path.delta, path.delta_moment, path.delta_moment_2,
                    delta_gradients[path_id]);
      }
      for (int i = 1; i < path.way_points.size() - 1; i++) {
        update_ADAM(path.way_points[i].x, path.moments[i].x,
                    path.moments_2[i].x, gradients[path_id][i].x);
        update_ADAM(path.way_points[i].y, path.moments[i].y,
                    path.moments_2[i].y, gradients[path_id][i].y);
      }
      path.delay = start_times[path.group_id] + delay[path_id] * period;
    }
    if (log != NULL && count % log_epoch == 0) {
      fprintf(log, "%lf\n", period);
      for (int group = 1; group < number_of_groups; group++) {
        fprintf(log, "%lf ", start_times[group]);
      }
      fputc('\n', log);
      for (int agent = 0; agent < paths.size(); agent++) {
        auto &path = paths[agent];
        fprintf(log, "%lu %d %d\n", path.way_points.size(), delay[agent],
                path.group_id);
        for (int i = 0; i < path.way_points.size(); i++) {
          fprintf(log, "%lf %lf %lf\n", path.way_points[i].x,
                  path.way_points[i].y, path.delay + i * path.delta);
        }
      }
      fprintf(log, "%lf %lf %lf %lf\n", time_cost, path_cost, collision_cost,
              radius_cost);
    }
    double cost = time_cost + path_cost + collision_cost + radius_cost;
    if (cost < best_cost) {
      best_period = period;
      best_start_times = start_times;
      best_way_points.resize(paths.size());
      for (int path_id = 0; path_id < paths.size(); path_id++) {
        best_way_points[path_id] = paths[path_id].way_points;
      }
      best_cost = cost;
    }
    return cost;
  }
};
