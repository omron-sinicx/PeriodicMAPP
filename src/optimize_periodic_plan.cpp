#include <yaml-cpp/yaml.h>

#include <cassert>

#include "geometry_2D_for_generator.hpp"
#include "path_optimizer.hpp"

int main(int argc, char **argv) {
  using Point = Geometry2D::Point;
  GeometricPaths<Geometry2D> plan;
  YAML::Node config = YAML::LoadFile(
      argc < 2 ? "../config/optimize_periodic_plan.yaml" : argv[1]);
  FILE *in = fopen(config["input"].as<std::string>().c_str(), "r");
  assert(in != NULL);
  FILE *out = fopen(config["output"].as<std::string>().c_str(), "w");
  assert(out != NULL);
  int number_of_agents, cycle, number_of_groups;
  double agent_radius, period;
  fscanf(in, "%d%lf%lf%d", &cycle, &agent_radius, &period, &number_of_groups);
  number_of_agents = cycle * number_of_groups;
  plan.paths.resize(number_of_agents);
  plan.delay.resize(number_of_agents);
  plan.cycle = cycle;
  plan.period = period;
  plan.number_of_groups = number_of_groups;
  plan.start_times.resize(number_of_groups);
  plan.start_times_memonts.resize(number_of_groups);
  plan.start_times_memonts_2.resize(number_of_groups);
  plan.start_times[0] = 0.;
  plan.optimize_start_time = config["optimize_start_time"].as<bool>();
  plan.agent_radius = agent_radius;
  /*for (int group = 1; group < number_of_groups; group++) {
    fscanf(in, "%lf", &plan.start_times[group]);
    }*/
  if (config["variale_agent_radius"] &&
      config["variale_agent_radius"].as<bool>()) {
    plan.variale_agent_radius = true;
    double agent_radius_cost_eta = config["agent_radius_cost_eta"].as<double>();
    double objective_agent_radius =
        config["objective_agent_radius"].as<double>();
    plan.agent_radius_cost_func =
        [agent_radius_cost_eta, objective_agent_radius](
            const double radius) -> std::pair<double, double> {
      return radius >= objective_agent_radius
                 ? std::make_pair(0., 0.)
                 : std::make_pair(
                       agent_radius_cost_eta / 2. *
                           std::pow(radius - objective_agent_radius, 2),
                       agent_radius_cost_eta *
                           (radius - objective_agent_radius));
    };
  }
  plan.variable_delta = config["variable_delta"].as<bool>();
  if (config["time_margin"]) {
    plan.time_margin = config["time_margin"].as<double>();
  }
  if (config["log"]) {
    plan.log = fopen(config["log"].as<std::string>().c_str(), "w");
    fprintf(plan.log, "%d %d %lf %d\n", number_of_agents, cycle,
            plan.agent_radius, number_of_groups);
    if (config["log_epoch"]) {
      plan.log_epoch = config["log_epoch"].as<int>();
    }
  }
  plan.period_cost = config["period_cost"].as<double>();

  double eta = config["eta"].as<double>();
  // double delta = config["delta"].as<double>();

  plan.distance_cost_func =
      [eta](const double agent_radius,
            const double distance) -> std::tuple<double, double, double> {
    return distance <= 2 * agent_radius
               ? std::make_tuple(
                     1. / 2. * eta *
                         std::pow(1. / distance - 1 / (2 * agent_radius), 2),
                     -eta * (1. / distance - 1 / (2 * agent_radius)) /
                         std::pow(distance, 2),
                     1. / 2. * eta * (1. / distance - 1 / (2 * agent_radius)) /
                         std::pow(agent_radius, 2))
               : std::make_tuple(0., 0., 0.);
  };
  double span_cost_const = config["span_cost_const"].as<double>();
  double path_cost = config["path_cost"].as<double>();
  double max_velocity = config["max_velocity"].as<double>();
  double max_velocity_cost = config["max_velocity_cost"].as<double>();
  plan.path_cost_func =
      [span_cost_const, path_cost, max_velocity, max_velocity_cost](
          const std::vector<Point> &way_points, const double delta,
          std::vector<Point> &grad, double &delta_grad) -> double {
    double score = 0.;
    int N = way_points.size() - 1;
    for (int i = 0; i < way_points.size() - 1; i++) {
      Point edge = way_points[i + 1] - way_points[i];
      double length = edge.length();
      score += path_cost * 1. / 2. * std::pow(length / delta, 2) / N;
      if (i > 0) {
        grad[i] += -path_cost * edge / std::pow(delta, 2) / N;
      }
      if (i + 1 < way_points.size() - 1) {
        grad[i + 1] += path_cost * edge / std::pow(delta, 2) / N;
      }
      delta_grad += -path_cost * std::pow(length / delta, 2) / delta / N;
      if (length / delta > max_velocity) {
        score += max_velocity_cost * 1. / 2. *
                 std::pow(length / delta - max_velocity, 2) / N;
        grad[i] += -max_velocity_cost * (length / delta - max_velocity) /
                   delta * (edge / length) / N;
        grad[i] += max_velocity_cost * (length / delta - max_velocity) / delta *
                   (edge / length) / N;
        delta_grad += -max_velocity_cost * (length / delta - max_velocity) *
                      length / std::pow(delta, 2) / N;
      }
    }
    delta_grad += span_cost_const / delta;
    return score + span_cost_const * std::log(N * delta);
  };
  /*plan.path_goal_cost_func = [agent_radius](const double distance) {
    double d = distance / agent_radius;
    // return d >= 1.0
    //? 1.0
    //: std::pow(d, 2) * (3-2*d);
    return 0.0;
  };
  plan.path_goal_force = [agent_radius](const double distance) {
    double d = distance / agent_radius;
    // return d >= 1.0
    //? 0.
    //: 6 * d * (1-d);
    return 0.0;
    };*/

  if (config["Environment"]) {
    FILE *env_file =
        fopen(config["Environment"].as<std::string>().c_str(), "r");
    assert(env_file != NULL);
    Space2D::Space space;
    Space2D::input_space(space, env_file);
    plan.field_potential =
        [space, eta](const double agent_radius,
                     const Point point) -> std::tuple<double, Point, double> {
      double dist = std::numeric_limits<double>::infinity();
      Point point_0;
      boost::geometry::for_each_point(
          space, [&dist, &point_0, &point](const auto &vertex) {
            double d = Geometry2D::distance(point, vertex);
            if (d < dist) {
              dist = d;
              point_0 = vertex;
            }
          });
      boost::geometry::for_each_segment(
          space, [&dist, &point_0, &point](const auto &edge) {
            Point e0 = edge.first, e1 = edge.second, u = (e1 - e0).unit();
            double ip = Geometry2D::inner(u, point - e0);
            if (ip <= 0 || 1 <= ip)
              return;
            Point foot = e0 + ip * u;
            double d = Geometry2D::distance(point, foot);
            if (d < dist) {
              dist = d;
              point_0 = foot;
            }
          });
      if (dist == 0.) {
        return std::make_tuple(0., Point(), 0.);
      }
      double cost = 0., radius_grad = 0.;
      Point force;
      if (dist <= agent_radius) {
        cost = 1. / 2. * eta * std::pow(1. / dist - 1 / agent_radius, 2);
        force = -eta * (1. / dist - 1 / agent_radius) / std::pow(dist, 3) *
                (point - point_0);
        radius_grad =
            -eta * (1. / dist - 1 / agent_radius) / std::pow(agent_radius, 2);
      }
      return std::make_tuple(cost, force, radius_grad);
    };
  } else {
    plan.field_potential =
        [](const double agent_radius,
           const Point point) -> std::tuple<double, Point, double> {
      return std::make_tuple(0., Point(), 0.);
    };
  }
  int number_of_way_points = config["number_of_way_points"].as<int>();

  double time_span;
  for (int agent = 0; agent < number_of_agents; agent++) {
    int number_of_points, delay = agent % cycle, group_id = agent / cycle;
    fscanf(in, "%d", &number_of_points);
    auto &path = plan.paths[agent];
    path.way_points.clear();
    std::vector<Point> points;
    std::vector<double> times;
    for (int i = 0; i < number_of_points; i++) {
      double x, y, t;
      fscanf(in, "%lf%lf%lf", &x, &y, &t);
      Point point(x, y);
      points.push_back(point);
      times.push_back(t);
    }
    time_span = (times[number_of_points - 1] - times[0]);
    path.delta = time_span / (number_of_way_points - 1);
    points.push_back(points[points.size() - 1]);
    times.push_back(std::numeric_limits<double>::infinity());
    path.way_points.resize(number_of_way_points);
    for (int i = 0, j = 0; i < number_of_points; i++) {
      while (j < number_of_way_points &&
             times[0] + path.delta * j < times[i + 1]) {
        path.way_points[j] =
            points[i] + (times[0] + path.delta * j - times[i]) /
                            (times[i + 1] - times[i]) *
                            (points[i + 1] - points[i]);
        j++;
      }
    }
    path.moments.resize(number_of_way_points);
    path.moments_2.resize(number_of_way_points);
    path.group_id = group_id;
    path.delay = plan.start_times[path.group_id] + delay * period;
    // path.delay /= time_span;
    path.delta /= time_span;
    plan.delay[agent] = delay;
  }
  plan.collision_allowed.resize(number_of_agents);
  for (int agent = 0; agent < number_of_agents; agent++) {
    plan.collision_allowed[agent].resize(number_of_agents, false);
  }
  if (config["collision_allowed"] && config["collision_allowed"].as<bool>()) {
    int n;
    fscanf(in, "%d", &n);
    for (int i = 0; i < n; i++) {
      int p0, p1;
      fscanf(in, "%d%d", &p0, &p1);
      plan.collision_allowed[p0][p1] = plan.collision_allowed[p1][p0] = true;
    }
  }
  fclose(in);
  /*plan.period /= time_span;
  for(int group_id = 1; group_id < number_of_groups; group_id++){
    plan.start_times[group_id] /= time_span;
    }*/
  plan.learning_rate = config["learning_rate"].as<double>();
  plan.learning_epsiron = config["learning_epsiron"].as<double>();
  plan.adam_beta_1 = config["adam_beta_1"].as<double>();
  plan.adam_beta_2 = config["adam_beta_2"].as<double>();
  int ite = config["iteration"].as<int>();
  for (int count = 0; count < ite; count++) {
    double cost = plan.update();
    printf("%lf %d\n", cost, plan.count);
  }
  fprintf(out, "%d %lf %lf %d\n", cycle, plan.agent_radius, plan.best_period,
          plan.number_of_groups);
  /*for (int group = 1; group < number_of_groups; group++) {
    fprintf(out, "%lf ", plan.best_start_times[group]);
    }
  fputc('\n', out);*/
  for (int agent = 0; agent < number_of_agents; agent++) {
    auto &path = plan.paths[agent];
    auto &way_points = plan.best_way_points[agent];
    fprintf(out, "%lu\n", way_points.size());
    double delay = plan.best_start_times[path.group_id] +
                   plan.delay[agent] * plan.best_period;
    for (int i = 0; i < way_points.size(); i++) {
      fprintf(out, "%lf %lf %lf\n", way_points[i].x, way_points[i].y,
              delay + i * path.delta);
    }
  }
  fprintf(out, "%lf\n", plan.best_cost);
  fclose(out);
  if (config["log"]) {
    fclose(plan.log);
  }
  return 0;
}
