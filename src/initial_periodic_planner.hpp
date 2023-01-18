#include <queue>
#include <vector>

#include "geometry_2D.hpp"

class SimpleInitialPeriodicPlanner {
public:
  int cycle;
  int number_of_groups;

  struct Constraints {
    int group_0, order_0, group_1, order_1;
    double time_diff;
  };

  std::vector<Constraints> constraints, constraints_2;

  struct Group {
    Geometry2D::Point start, goal;
    double length;
    std::vector<std::pair<Geometry2D::Point, double>> start_strage, goal_strage;
  };

  std::vector<Group> groups;

  std::vector<std::pair<int, double>> start_time_exp;

  SimpleInitialPeriodicPlanner(const int cycle, const int number_of_groups) {
    this->cycle = cycle;
    this->number_of_groups = number_of_groups;
  }

  double period;
  std::vector<double> start_times;

  std::vector<std::vector<std::pair<Geometry2D::Point, double>>> paths;

  bool calculate_plan() {
    int number_of_agents = number_of_groups * cycle;
    std::vector<std::vector<std::pair<int, double>>> edges(number_of_agents);
    std::vector<int> degree(number_of_agents);
    for (auto &constraint : constraints) {
      int agent_0 = constraint.group_0 * cycle + constraint.order_0;
      int agent_1 = constraint.group_1 * cycle + constraint.order_1;
      edges[agent_0].push_back(std::make_pair(agent_1, constraint.time_diff));
      degree[agent_1]++;
    }
    for (int group = 0; group < number_of_groups; group++) {
      for (int order = 0; order < cycle - 1; order++) {
        int agent_0 = group * cycle + order;
        int agent_1 = agent_0 + 1;
        edges[agent_0].push_back(std::make_pair(agent_1, 1.));
        degree[agent_1]++;
      }
    }

    start_time_exp.resize(number_of_agents);

    std::queue<int> Q;
    for (int agent = 0; agent < number_of_agents; agent++) {
      start_time_exp[agent] = std::make_pair(agent % cycle, 0.);
      if (degree[agent] == 0) {
        Q.push(agent);
      }
    }
    int count = 0;
    while (!Q.empty()) {
      int agent_0 = Q.front();
      Q.pop();
      for (auto &edge : edges[agent_0]) {
        int agent_1 = edge.first;
        start_time_exp[agent_1] = std::max(
            start_time_exp[agent_1],
            std::make_pair(start_time_exp[agent_0].first,
                           start_time_exp[agent_0].second + edge.second));
        degree[agent_1]--;
        if (degree[agent_1] == 0) {
          Q.push(agent_1);
        }
      }
      count++;
    }
    if (count < number_of_agents)
      return false;
    period = 0.;
    for (int agent_0 = 0; agent_0 < number_of_agents; agent_0++) {
      for (auto &edge : edges[agent_0]) {
        int agent_1 = edge.first;
        if (start_time_exp[agent_0].first < start_time_exp[agent_1].first) {
          period =
              std::max(period, (start_time_exp[agent_0].second -
                                start_time_exp[agent_1].second + edge.second) /
                                   (start_time_exp[agent_1].first -
                                    start_time_exp[agent_0].first));
        }
      }
    }
    for (int group = 0; group < number_of_groups; group++) {
      period = std::max(period,
                        start_time_exp[group * cycle + cycle - 1].second + 1.);
    }
    start_times.resize(number_of_agents);
    std::pair<int, double> makespan_exp(0, 0.);
    for (int agent = 0; agent < number_of_agents; agent++) {
      auto length = std::make_pair(start_time_exp[agent].first - agent % cycle,
                                   start_time_exp[agent].second +
                                       groups[agent / cycle].length);
      if (makespan_exp.first < length.first) {
        period = std::max(period, (makespan_exp.second - length.second) /
                                      (length.first - makespan_exp.first));
      }
      makespan_exp = std::max(makespan_exp, length);
    }
    for (int group = 0; group < number_of_groups; group++) {
      auto &group_data = groups[group];
      period = std::max(period, (makespan_exp.second - group_data.length +
                                 group_data.goal_strage[0].second) /
                                    (cycle - makespan_exp.first));
    }
    for (auto &constraint : constraints_2) {
      int agent_0 = constraint.group_0 * cycle + constraint.order_0;
      int agent_1 = constraint.group_1 * cycle + constraint.order_1;
      period = std::max(period, (start_time_exp[agent_0].second -
                                 start_time_exp[agent_1].second +
                                 constraint.time_diff) /
                                    (start_time_exp[agent_1].first + cycle -
                                     start_time_exp[agent_0].first));
    }
    double makespan = makespan_exp.first * period + makespan_exp.second;
    for (int agent = 0; agent < number_of_agents; agent++) {
      start_times[agent] =
          start_time_exp[agent].first * period + start_time_exp[agent].second;
    }

    paths.resize(number_of_agents);
    for (int group = 0; group < number_of_groups; group++) {
      for (int order = 0; order < cycle; order++) {
        int agent = group * cycle + order;
        auto &path = paths[agent];
        auto group_data = groups[group];
        path.push_back(std::make_pair(group_data.start, order * period));
        path.push_back(std::make_pair(
            group_data.start_strage[order].first,
            order * period + group_data.start_strage[order].second));
        path.push_back(std::make_pair(
            group_data.start_strage[order].first,
            start_times[agent] + group_data.start_strage[order].second));
        /*path.push_back(std::make_pair(group_data.goal_strage[0].first,
                                      start_times[agent] + group_data.length -
                                          group_data.goal_strage[0].second));
        int k = 1;
        while (k < cycle) {
          double next_start_time =
              (order + k < cycle
                   ? start_times[agent + k]
                   : start_times[agent + k - cycle] + cycle * period) +
              group_data.length - group_data.goal_strage[0].second - 1.;
          if (next_start_time >= makespan + order * period -
                                     group_data.goal_strage[k - 1].second) {
            break;
          }
          path.push_back(std::make_pair(group_data.goal_strage[k - 1].first,
                                        next_start_time));
          path.push_back(std::make_pair(
              group_data.goal_strage[k].first,
              next_start_time + (group_data.goal_strage[k - 1].second -
                                 group_data.goal_strage[k].second)));
          k++;
        }
        path.push_back(std::make_pair(
            group_data.goal_strage[k - 1].first,
            makespan + order * period - group_data.goal_strage[k - 1].second));
        path.push_back(
        std::make_pair(group_data.goal, ));*/
        path.push_back(std::make_pair(group_data.goal,
                                      start_times[agent] + group_data.length));
      }
    }
    return true;
  }
};

class InitialPeriodicPlanner {
public:
  int cycle;
  int number_of_groups;
  double redundancy = 0.5;

  struct Constraints {
    int group_0, order_0, vertex_0, group_1, order_1, vertex_1;
    Constraints() {}
    Constraints(const int group_0, const int order_0, const int vertex_0,
                const int group_1, const int order_1, const int vertex_1) {
      this->group_0 = group_0;
      this->order_0 = order_0;
      this->vertex_0 = vertex_0;
      this->group_1 = group_1;
      this->order_1 = order_1;
      this->vertex_1 = vertex_1;
    }
    Constraints(const int group_0, const int vertex_0, const int group_1,
                const int vertex_1) {
      this->group_0 = group_0;
      this->vertex_0 = vertex_0;
      this->group_1 = group_1;
      this->vertex_1 = vertex_1;
    }
  };

  std::vector<Constraints> constraints, constraints_2;

  struct Group {
    std::vector<std::pair<Geometry2D::Point, double>> way_points;
  };

  std::vector<Group> groups;

  std::vector<std::pair<int, double>> arrival_time_exp;

  InitialPeriodicPlanner(const int cycle, const int number_of_groups) {
    this->cycle = cycle;
    this->number_of_groups = number_of_groups;
  }

  double period;
  std::vector<double> arrival_times;

  std::vector<std::vector<std::pair<Geometry2D::Point, double>>> paths;

  bool calculate_plan() {
    int number_of_agents = number_of_groups * cycle;

    std::vector<int> vertex_ids(number_of_groups);
    int number_of_vertices = 0;

    for (int group = 0; group < number_of_groups; group++) {
      vertex_ids[group] = number_of_vertices;
      number_of_vertices += groups[group].way_points.size();
    }

    int number_of_events = number_of_vertices * cycle;
    std::vector<std::vector<std::pair<int, double>>> edges(number_of_events);
    std::vector<int> degree(number_of_events);
    for (auto &constraint : constraints) {
      int event_0 =
          (vertex_ids[constraint.group_0] + constraint.vertex_0) * cycle +
          constraint.order_0;
      int event_1 =
          (vertex_ids[constraint.group_1] + constraint.vertex_1) * cycle +
          constraint.order_1;
      edges[event_0].push_back(std::make_pair(event_1, redundancy));
      degree[event_1]++;
    }
    for (int group = 0; group < number_of_groups; group++) {
      for (int vertex = 0; vertex < groups[group].way_points.size(); vertex++) {
        for (int order = 0; order < cycle - 1; order++) {
          int event_0 = (vertex_ids[group] + vertex) * cycle + order;
          int event_1 = event_0 + 1;
          edges[event_0].push_back(std::make_pair(event_1, redundancy));
          degree[event_1]++;
        }
      }
      for (int vertex = 0; vertex < groups[group].way_points.size() - 1;
           vertex++) {
        for (int order = 0; order < cycle; order++) {
          int event_0 = (vertex_ids[group] + vertex) * cycle + order;
          int event_1 = event_0 + cycle;
          edges[event_0].push_back(std::make_pair(
              event_1, groups[group].way_points[vertex + 1].second));
          degree[event_1]++;
        }
      }
    }

    arrival_time_exp.resize(number_of_events);

    std::queue<int> Q;
    for (int event = 0; event < number_of_events; event++) {
      arrival_time_exp[event] = std::make_pair(event % cycle, 0.);
      if (degree[event] == 0) {
        Q.push(event);
      }
    }
    int count = 0;
    while (!Q.empty()) {
      int event_0 = Q.front();
      Q.pop();
      for (auto &edge : edges[event_0]) {
        int event_1 = edge.first;
        arrival_time_exp[event_1] = std::max(
            arrival_time_exp[event_1],
            std::make_pair(arrival_time_exp[event_0].first,
                           arrival_time_exp[event_0].second + edge.second));
        degree[event_1]--;
        if (degree[event_1] == 0) {
          Q.push(event_1);
        }
      }
      count++;
    }
    if (count < number_of_events)
      return false;
    for (int group = 0; group < number_of_groups; group++) {
      for (int order = 0; order < cycle; order++) {
        int event = vertex_ids[group] * cycle + order;
        if (arrival_time_exp[event].first != order ||
            arrival_time_exp[event].second != 0) {
          return false;
        }
      }
    }
    period = 0.;
    for (int event_0 = 0; event_0 < number_of_events; event_0++) {
      for (auto &edge : edges[event_0]) {
        int event_1 = edge.first;
        if (arrival_time_exp[event_0].first < arrival_time_exp[event_1].first) {
          period = std::max(period,
                            (arrival_time_exp[event_0].second -
                             arrival_time_exp[event_1].second + edge.second) /
                                (arrival_time_exp[event_1].first -
                                 arrival_time_exp[event_0].first));
        }
      }
    }
    for (int vertex = 0; vertex < number_of_vertices; vertex++) {
      int event_0 = vertex * cycle, event_1 = event_0 + cycle - 1;
      period = std::max(period, (arrival_time_exp[event_1].second + redundancy -
                                 arrival_time_exp[event_0].second) /
                                    (arrival_time_exp[event_0].first + cycle -
                                     arrival_time_exp[event_1].first));
    }
    for (auto &constraint : constraints_2) {
      int event_0 =
          (vertex_ids[constraint.group_0] + constraint.vertex_0) * cycle +
          cycle - 1;
      int event_1 =
          (vertex_ids[constraint.group_1] + constraint.vertex_1) * cycle;
      period =
          std::max(period, (arrival_time_exp[event_0].second -
                            arrival_time_exp[event_1].second + redundancy) /
                               (arrival_time_exp[event_1].first + cycle -
                                arrival_time_exp[event_0].first));
    }
    arrival_times.resize(number_of_events);
    for (int event = 0; event < number_of_events; event++) {
      arrival_times[event] = arrival_time_exp[event].first * period +
                             arrival_time_exp[event].second;
    }

    paths.resize(number_of_groups * cycle);
    for (int group = 0; group < number_of_groups; group++) {
      for (int order = 0; order < cycle; order++) {
        int agent = group * cycle + order;
        auto &path = paths[agent];
        auto group_data = groups[group];
        path.push_back(
            std::make_pair(group_data.way_points[0].first,
                           arrival_times[(vertex_ids[group]) * cycle + order]));
        for (int vertex = 1; vertex < group_data.way_points.size(); vertex++) {
          /*path.push_back(std::make_pair(
              group_data.way_points[vertex - 1].first,
              arrival_times[(vertex_ids[group] + vertex) * cycle + order] -
              group_data.way_points[vertex].second));*/
          path.push_back(std::make_pair(
              group_data.way_points[vertex].first,
              arrival_times[(vertex_ids[group] + vertex) * cycle + order]));
        }
      }
    }
    return true;
  }
};
