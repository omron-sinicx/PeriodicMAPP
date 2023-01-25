/*
Copyright (c) 2023 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#include "path_optimizer_g2o.hpp"

int main(int argc, char **argv) {
  YAML::Node config = YAML::LoadFile(
      argc < 2 ? "../config/optimize_periodic_plan_g2o.yaml" : argv[1]);
  std::vector<int> number_of_iterations =
      config["number_of_iterations"].as<std::vector<int>>();
  std::vector<int> graph_update_interval =
      config["graph_update_interval"].as<std::vector<int>>();
  assert(number_of_iterations.size() == graph_update_interval.size());
  FILE *in = fopen(config["input"].as<std::string>().c_str(), "r");
  assert(in != NULL);
  FILE *out = fopen(config["output"].as<std::string>().c_str(), "w");
  assert(out != NULL);
  int cycle, number_of_groups;
  double agent_radius, period;
  fscanf(in, "%d%lf%lf%d", &cycle, &agent_radius, &period, &number_of_groups);
  auto plan = new PeriodicPlanOptimizer(config, cycle, number_of_groups, period,
                                        agent_radius);
  int number_of_agents = plan->number_of_groups * plan->cycle;
  for (int group = 0; group < plan->number_of_groups; group++) {
    for (int order = 0; order < plan->cycle; order++) {
      int agent = group * plan->cycle + order;
      int number_of_points;
      fscanf(in, "%d", &number_of_points);
      std::vector<Eigen::Vector2d> points;
      std::vector<double> times;
      for (int i = 0; i < number_of_points; i++) {
        double x, y, t;
        fscanf(in, "%lf%lf%lf", &x, &y, &t);
        Eigen::Vector2d point(x, y);
        points.push_back(point);
        times.push_back(t);
      }
      double time_span = (times[number_of_points - 1] - times[0]);
      double delta = time_span / (plan->number_of_way_points - 1);
      plan->deltas[agent]->setEstimate(delta);
      points.push_back(points[points.size() - 1]);
      times.push_back(std::numeric_limits<double>::infinity());
      for (int i = 0, j = 0; i < number_of_points; i++) {
        while (j < plan->number_of_way_points &&
               times[0] + delta * j < times[i + 1]) {
          plan->paths[agent][j]->setEstimate(points[i] +
                                             (times[0] + delta * j - times[i]) /
                                                 (times[i + 1] - times[i]) *
                                                 (points[i + 1] - points[i]));
          j++;
        }
      }
    }
  }
  fclose(in);
  bool increase_coefficients = config["increase_coefficients"]
                                   ? config["increase_coefficients"].as<bool>()
                                   : false;
  double coefficient_growth_rate;
  int sum_of_iteration = 0;
  if (increase_coefficients) {
    coefficient_growth_rate = config["coefficient_growth_rate"].as<double>();
  }
  for (int phase = 0; phase < number_of_iterations.size(); phase++) {
    for (int t = 0;
         t < number_of_iterations[phase] / graph_update_interval[phase]; t++) {
      if (phase == number_of_iterations.size() - 1 && increase_coefficients) {
        plan->growCoefficients(coefficient_growth_rate);
      }
      plan->update(graph_update_interval[phase]);
      sum_of_iteration += graph_update_interval[phase];
    }
  }
  if (config["until_converge"]) {
    double dif = config["until_converge"].as<double>();
    std::vector<double> chi2_log;
    while (true) {
      plan->update(1);
      sum_of_iteration++;
      double current_chi2 = plan->optimizer.activeChi2();
      if (chi2_log.size() >= 100 &&
          chi2_log[chi2_log.size() - 100] - current_chi2 < dif)
        break;
      chi2_log.push_back(current_chi2);
    }
  }
  fprintf(out, "%d %lf %lf %d\n", plan->cycle, plan->agent_radius->estimate(),
          plan->period->estimate(), plan->number_of_groups);
  for (int agent = 0; agent < number_of_agents; agent++) {
    double delta = plan->deltas[agent]->estimate();
    fprintf(out, "%d\n", plan->number_of_way_points);
    for (int i = 0; i < plan->number_of_way_points; i++) {
      Eigen::Vector2d point = plan->paths[agent][i]->estimate();
      fprintf(out, "%lf %lf %lf\n", point.x(), point.y(), i * delta);
    }
  }
  fprintf(out, "%lf\n", plan->optimizer.activeChi2());
  fclose(out);
  fprintf(stderr, "iteration: %d\n", sum_of_iteration);
  return 0;
}
