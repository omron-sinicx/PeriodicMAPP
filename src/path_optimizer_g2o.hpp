/*
Copyright (c) 2023 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <yaml-cpp/yaml.h>

const double EPS = 1e-9;

class WayPointVertex : public g2o::BaseVertex<2, Eigen::Vector2d> {
public:
  void setToOriginImpl() { _estimate.setZero(); }
  void oplusImpl(const double *update) {
    _estimate[0] += update[0];
    _estimate[1] += update[1];
  }
  virtual bool read(std::istream &is) { return true; }
  virtual bool write(std::ostream &os) const { return true; }
};

class ParameterVertex : public g2o::BaseVertex<1, double> {
public:
  ParameterVertex() {}
  void setToOriginImpl() { _estimate = 0.; }
  void oplusImpl(const double *update) { _estimate += *update; }
  virtual bool read(std::istream &is) { return true; }
  virtual bool write(std::ostream &os) const { return true; }
};

class CollisionConstraint : public g2o::BaseMultiEdge<1, double> {
public:
  // constant parameters
  const int order_diff, i, j;
  CollisionConstraint(WayPointVertex *point_0, WayPointVertex *point_a,
                      WayPointVertex *point_b, ParameterVertex *period,
                      ParameterVertex *agent_radius, ParameterVertex *delta_0,
                      ParameterVertex *delta_1, const int _order_diff,
                      const int _i, const int _j, const double coefficient)
      : g2o::BaseMultiEdge<1, double>(), order_diff(_order_diff), i(_i), j(_j) {
    resize(7);
    _vertices[0] = point_0;
    _vertices[1] = period;
    _vertices[2] = agent_radius;
    _vertices[3] = point_a;
    _vertices[4] = point_b;
    _vertices[5] = delta_0;
    _vertices[6] = delta_1;
    _information[0] = coefficient;
  }
  void computeError() {
    Eigen::Vector2d point_0 =
        static_cast<const WayPointVertex *>(_vertices[0])->estimate();
    double period =
        static_cast<const ParameterVertex *>(_vertices[1])->estimate();
    double agent_radius =
        static_cast<const ParameterVertex *>(_vertices[2])->estimate();
    Eigen::Vector2d point_a =
        static_cast<const WayPointVertex *>(_vertices[3])->estimate();
    Eigen::Vector2d point_b =
        static_cast<const WayPointVertex *>(_vertices[4])->estimate();
    double delta_0 =
        static_cast<const ParameterVertex *>(_vertices[5])->estimate();
    double delta_1 =
        static_cast<const ParameterVertex *>(_vertices[6])->estimate();
    double alpha = (order_diff * period + i * delta_0) / delta_1 - j;
    // assert(-1. <= alpha && alpha < 2.);
    Eigen::Vector2d point_1 = (1. - alpha) * point_a + alpha * point_b;
    double distance = (point_1 - point_0).norm();
    if (distance >= 2 * agent_radius) {
      _error[0] = 0.;
    } else {
      _error[0] = 1. / (distance + EPS) - 1. / (2 * agent_radius);
    }
    assert(std::isfinite(_error[0]));
  }
  void linearizeOplus() override {
    Eigen::Vector2d point_0 =
        static_cast<const WayPointVertex *>(_vertices[0])->estimate();
    double period =
        static_cast<const ParameterVertex *>(_vertices[1])->estimate();
    double agent_radius =
        static_cast<const ParameterVertex *>(_vertices[2])->estimate();
    Eigen::Vector2d point_a =
        static_cast<const WayPointVertex *>(_vertices[3])->estimate();
    Eigen::Vector2d point_b =
        static_cast<const WayPointVertex *>(_vertices[4])->estimate();
    double delta_0 =
        static_cast<const ParameterVertex *>(_vertices[5])->estimate();
    double delta_1 =
        static_cast<const ParameterVertex *>(_vertices[6])->estimate();
    double alpha = (order_diff * period + i * delta_0) / delta_1 - j;
    // assert(-1. <= alpha && alpha < 2.);
    Eigen::Vector2d point_1 = (1. - alpha) * point_a + alpha * point_b;
    double distance = (point_1 - point_0).norm();
    if (distance >= 2 * agent_radius) {
      _jacobianOplus[0] = Eigen::MatrixXd::Zero(1, 2);
      _jacobianOplus[1] = Eigen::MatrixXd::Zero(1, 1);
      _jacobianOplus[2] = Eigen::MatrixXd::Zero(1, 1);
      _jacobianOplus[3] = Eigen::MatrixXd::Zero(1, 2);
      _jacobianOplus[4] = Eigen::MatrixXd::Zero(1, 2);
      _jacobianOplus[5] = Eigen::MatrixXd::Zero(1, 1);
      _jacobianOplus[6] = Eigen::MatrixXd::Zero(1, 1);
    } else {
      double dedd = -std::pow(distance + EPS, -2);
      Eigen::Vector2d direction = (point_1 - point_0) / (distance + EPS);
      double deda = direction.dot(point_b - point_a) * dedd;
      _jacobianOplus[0] = (-dedd * direction).transpose();
      _jacobianOplus[1](0, 0) = order_diff / delta_1 * deda;
      _jacobianOplus[2](0, 0) = 1. / 2. * std::pow(agent_radius, -2);
      _jacobianOplus[3] = ((1. - alpha) * dedd * direction).transpose();
      _jacobianOplus[4] = (alpha * dedd * direction).transpose();
      _jacobianOplus[5](0, 0) = i / delta_1 * deda;
      _jacobianOplus[6](0, 0) =
          -(order_diff * period + i * delta_0) * std::pow(delta_1, -2) * deda;
    }
  }
  virtual bool read(std::istream &is) { return true; }
  virtual bool write(std::ostream &os) const { return true; }
};

class CollisionConstraint2 : public g2o::BaseMultiEdge<1, double> {
public:
  // constant parameters
  const int order_diff, point_diff;
  CollisionConstraint2(WayPointVertex *point_0, WayPointVertex *point_a,
                       WayPointVertex *point_b, ParameterVertex *period,
                       ParameterVertex *agent_radius, ParameterVertex *delta,
                       const int _order_diff, const int _point_diff,
                       const double coefficient)
      : g2o::BaseMultiEdge<1, double>(), order_diff(_order_diff),
        point_diff(_point_diff) {
    resize(6);
    _vertices[0] = point_0;
    _vertices[1] = period;
    _vertices[2] = agent_radius;
    _vertices[3] = point_a;
    _vertices[4] = point_b;
    _vertices[5] = delta;
    _information[0] = coefficient;
  }
  void computeError() {
    Eigen::Vector2d point_0 =
        static_cast<const WayPointVertex *>(_vertices[0])->estimate();
    double period =
        static_cast<const ParameterVertex *>(_vertices[1])->estimate();
    double agent_radius =
        static_cast<const ParameterVertex *>(_vertices[2])->estimate();
    Eigen::Vector2d point_a =
        static_cast<const WayPointVertex *>(_vertices[3])->estimate();
    Eigen::Vector2d point_b =
        static_cast<const WayPointVertex *>(_vertices[4])->estimate();
    double delta =
        static_cast<const ParameterVertex *>(_vertices[5])->estimate();
    double alpha = order_diff * period / delta + point_diff;
    // assert(-1. <= alpha && alpha < 2.);
    Eigen::Vector2d point_1 = (1. - alpha) * point_a + alpha * point_b;
    double distance = (point_1 - point_0).norm();
    if (distance >= 2 * agent_radius) {
      _error[0] = 0.;
    } else {
      _error[0] = 1. / (distance + EPS) - 1. / (2 * agent_radius);
    }
  }
  void linearizeOplus() override {
    Eigen::Vector2d point_0 =
        static_cast<const WayPointVertex *>(_vertices[0])->estimate();
    double period =
        static_cast<const ParameterVertex *>(_vertices[1])->estimate();
    double agent_radius =
        static_cast<const ParameterVertex *>(_vertices[2])->estimate();
    Eigen::Vector2d point_a =
        static_cast<const WayPointVertex *>(_vertices[3])->estimate();
    Eigen::Vector2d point_b =
        static_cast<const WayPointVertex *>(_vertices[4])->estimate();
    double delta =
        static_cast<const ParameterVertex *>(_vertices[5])->estimate();
    double alpha = order_diff * period / delta + point_diff;
    Eigen::Vector2d point_1 = (1. - alpha) * point_a + alpha * point_b;
    double distance = (point_1 - point_0).norm();
    if (distance >= 2 * agent_radius) {
      _jacobianOplus[0] = Eigen::MatrixXd::Zero(1, 2);
      _jacobianOplus[1] = Eigen::MatrixXd::Zero(1, 1);
      _jacobianOplus[2] = Eigen::MatrixXd::Zero(1, 1);
      _jacobianOplus[3] = Eigen::MatrixXd::Zero(1, 2);
      _jacobianOplus[4] = Eigen::MatrixXd::Zero(1, 2);
      _jacobianOplus[5] = Eigen::MatrixXd::Zero(1, 1);
    } else {
      double dedd = -std::pow(distance + EPS, -2);
      Eigen::Vector2d direction = (point_1 - point_0) / (distance + EPS);
      double deda = direction.dot(point_b - point_a) * dedd;
      _jacobianOplus[0] = (-dedd * direction).transpose();
      _jacobianOplus[1](0, 0) = order_diff / delta * deda;
      _jacobianOplus[2](0, 0) = 1. / 2. * std::pow(agent_radius, -2);
      _jacobianOplus[3] = ((1. - alpha) * dedd * direction).transpose();
      _jacobianOplus[4] = (alpha * dedd * direction).transpose();
      _jacobianOplus[5](0, 0) =
          -order_diff * period * std::pow(delta, -2) * deda;
    }
  }
  virtual bool read(std::istream &is) { return true; }
  virtual bool write(std::ostream &os) const { return true; }
};

class PathCostConstraint : public g2o::BaseMultiEdge<2, Eigen::Vector2d> {
public:
  PathCostConstraint(WayPointVertex *point_0, WayPointVertex *point_1,
                     ParameterVertex *delta, const double coefficient)
      : g2o::BaseMultiEdge<2, Eigen::Vector2d>() {
    resize(3);
    _vertices[0] = point_0;
    _vertices[1] = point_1;
    _vertices[2] = delta;
    _information = coefficient * Eigen::Matrix2d::Identity();
  }
  void computeError() {
    Eigen::Vector2d point_0 =
        static_cast<const WayPointVertex *>(_vertices[0])->estimate();
    Eigen::Vector2d point_1 =
        static_cast<const WayPointVertex *>(_vertices[1])->estimate();
    double delta =
        static_cast<const ParameterVertex *>(_vertices[2])->estimate();
    _error = (point_1 - point_0) / delta;
  }
  void linearizeOplus() override {
    Eigen::Vector2d point_0 =
        static_cast<const WayPointVertex *>(_vertices[0])->estimate();
    Eigen::Vector2d point_1 =
        static_cast<const WayPointVertex *>(_vertices[1])->estimate();
    double delta =
        static_cast<const ParameterVertex *>(_vertices[2])->estimate();
    _jacobianOplus[0] = -1. / delta * Eigen::Matrix2d::Identity();
    _jacobianOplus[1] = 1. / delta * Eigen::Matrix2d::Identity();
    _jacobianOplus[2] = -std::pow(delta, -2) * (point_1 - point_0);
  }
  virtual bool read(std::istream &is) { return true; }
  virtual bool write(std::ostream &os) const { return true; }
};

class PathCostConstraintMaxVelocity : public g2o::BaseMultiEdge<1, double> {
public:
  PathCostConstraintMaxVelocity(WayPointVertex *point_0,
                                WayPointVertex *point_1, ParameterVertex *delta,
                                const double max_velocity,
                                const double coefficient)
      : g2o::BaseMultiEdge<1, double>() {
    resize(3);
    _vertices[0] = point_0;
    _vertices[1] = point_1;
    _vertices[2] = delta;
    _measurement = max_velocity;
    _information[0] = coefficient;
  }
  void computeError() {
    Eigen::Vector2d point_0 =
        static_cast<const WayPointVertex *>(_vertices[0])->estimate();
    Eigen::Vector2d point_1 =
        static_cast<const WayPointVertex *>(_vertices[1])->estimate();
    double delta =
        static_cast<const ParameterVertex *>(_vertices[2])->estimate();
    _error[0] = std::max(0., (point_1 - point_0).norm() / delta - _measurement);
  }
  void linearizeOplus() override {
    Eigen::Vector2d point_0 =
        static_cast<const WayPointVertex *>(_vertices[0])->estimate();
    Eigen::Vector2d point_1 =
        static_cast<const WayPointVertex *>(_vertices[1])->estimate();
    double delta_ =
        static_cast<const ParameterVertex *>(_vertices[2])->estimate();
    double distance = (point_1 - point_0).norm();
    if (distance / delta_ < _measurement) {
      _jacobianOplus[0] = Eigen::MatrixXd::Zero(1, 2);
      _jacobianOplus[1] = Eigen::MatrixXd::Zero(1, 2);
      _jacobianOplus[2] = Eigen::MatrixXd::Zero(1, 1);
    } else {
      Eigen::Vector2d direction = (point_1 - point_0) / (distance + EPS);
      // double r = distance/delta - _measurement;
      _jacobianOplus[0] = (-direction / delta_).transpose();
      _jacobianOplus[1] = (direction / delta_).transpose();
      _jacobianOplus[2](0, 0) = -std::pow(delta_, -2) * distance;
    }
  }
  virtual bool read(std::istream &is) { return true; }
  virtual bool write(std::ostream &os) const { return true; }
};

class PathCostConstraintAccelaration
    : public g2o::BaseMultiEdge<2, Eigen::Vector2d> {
public:
  PathCostConstraintAccelaration(WayPointVertex *point_0,
                                 WayPointVertex *point_1,
                                 WayPointVertex *point_2,
                                 ParameterVertex *delta,
                                 const double coefficient)
      : g2o::BaseMultiEdge<2, Eigen::Vector2d>() {
    resize(4);
    _vertices[0] = point_0;
    _vertices[1] = point_1;
    _vertices[2] = point_2;
    _vertices[3] = delta;
    _information = coefficient * Eigen::Matrix2d::Identity();
  }
  void computeError() {
    Eigen::Vector2d point_0 =
        static_cast<const WayPointVertex *>(_vertices[0])->estimate();
    Eigen::Vector2d point_1 =
        static_cast<const WayPointVertex *>(_vertices[1])->estimate();
    Eigen::Vector2d point_2 =
        static_cast<const WayPointVertex *>(_vertices[2])->estimate();
    double delta =
        static_cast<const ParameterVertex *>(_vertices[3])->estimate();
    _error = (point_0 + point_2 - 2. * point_1) / std::pow(delta, 2);
  }
  void linearizeOplus() override {
    Eigen::Vector2d point_0 =
        static_cast<const WayPointVertex *>(_vertices[0])->estimate();
    Eigen::Vector2d point_1 =
        static_cast<const WayPointVertex *>(_vertices[1])->estimate();
    Eigen::Vector2d point_2 =
        static_cast<const WayPointVertex *>(_vertices[2])->estimate();
    double delta =
        static_cast<const ParameterVertex *>(_vertices[3])->estimate();
    _jacobianOplus[0] = std::pow(delta, -2) * Eigen::Matrix2d::Identity();
    _jacobianOplus[1] = -2. * std::pow(delta, -2) * Eigen::Matrix2d::Identity();
    _jacobianOplus[2] = std::pow(delta, -2) * Eigen::Matrix2d::Identity();
    _jacobianOplus[3] =
        -2. * std::pow(delta, -3) * (point_0 + point_2 - 2. * point_1);
  }
  virtual bool read(std::istream &is) { return true; }
  virtual bool write(std::ostream &os) const { return true; }
};

class ParameterTargetConstraint
    : public g2o::BaseUnaryEdge<1, double, ParameterVertex> {
public:
  ParameterTargetConstraint(ParameterVertex *parameter, const double target,
                            const double coefficient)
      : g2o::BaseUnaryEdge<1, double, ParameterVertex>() {
    _vertices[0] = parameter;
    _measurement = target;
    _information[0] = coefficient;
  }
  void computeError() {
    _error[0] = static_cast<const ParameterVertex *>(_vertices[0])->estimate() -
                _measurement;
  }
  void linearizeOplus() override { std::get<0>(_jacobianOplus)(0, 0) = 1.; }
  virtual bool read(std::istream &is) { return true; }
  virtual bool write(std::ostream &os) const { return true; }
};

class WayPointConstraint
    : public g2o::BaseBinaryEdge<1, double, WayPointVertex, ParameterVertex> {
public:
  std::shared_ptr<std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>>
      segments;
  WayPointConstraint(
      WayPointVertex *point, ParameterVertex *agent_radius,
      const std::shared_ptr<
          std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>>
          segments,
      const double coefficient)
      : g2o::BaseBinaryEdge<1, double, WayPointVertex, ParameterVertex>() {
    _vertices[0] = point;
    _vertices[1] = agent_radius;
    this->segments = segments;
    _information[0] = coefficient;
  }
  void computeError() {
    Eigen::Vector2d point =
        static_cast<const WayPointVertex *>(_vertices[0])->estimate();
    double agent_radius =
        static_cast<const ParameterVertex *>(_vertices[1])->estimate();
    double distance = 2. * agent_radius;
    for (int i = 0; i < segments->size(); i++) {
      Eigen::Vector2d point_a = (*segments)[i].first,
                      point_b = (*segments)[i].second;
      distance = std::min(distance, (point - point_a).norm());
      distance = std::min(distance, (point - point_b).norm());
      double length = (point_b - point_a).norm(),
             ip = (point_b - point_a).dot(point - point_a) / length;
      if (0 < ip && ip < length) {
        Eigen::Vector2d foot = point_a + (point_b - point_a) / length * ip;
        distance = std::min(distance, (point - foot).norm());
      }
    }
    _error[0] = distance < agent_radius
                    ? 1. / (distance + EPS) - 1. / agent_radius
                    : 0.;
  }
  void linearizeOplus() override {
    Eigen::Vector2d point =
        static_cast<const WayPointVertex *>(_vertices[0])->estimate();
    double agent_radius =
        static_cast<const ParameterVertex *>(_vertices[1])->estimate();
    double distance = 2. * agent_radius;
    Eigen::Vector2d nearest;
    for (int i = 0; i < segments->size(); i++) {
      Eigen::Vector2d point_a = (*segments)[i].first,
                      point_b = (*segments)[i].second;
      if (distance > (point - point_a).norm()) {
        distance = (point - point_a).norm();
        nearest = point_a;
      }
      if (distance > (point - point_b).norm()) {
        distance = (point - point_b).norm();
        nearest = point_b;
      }
      double length = (point_b - point_a).norm(),
             ip = (point_b - point_a).dot(point - point_a) / length;
      if (0 < ip && ip < length) {
        Eigen::Vector2d foot = point_a + (point_b - point_a) / length * ip;
        if (distance > (point - foot).norm()) {
          distance = (point - foot).norm();
          nearest = foot;
        }
      }
    }
    if (distance < agent_radius) {
      double dedd = -std::pow(distance + EPS, -2);
      std::get<0>(_jacobianOplus) =
          (dedd * (point - nearest) / (distance + EPS)).transpose();
      std::get<1>(_jacobianOplus)(0, 0) = std::pow(agent_radius, -2);
    } else {
      std::get<0>(_jacobianOplus) = Eigen::MatrixXd::Zero(1, 2);
      std::get<1>(_jacobianOplus) = Eigen::MatrixXd::Zero(1, 1);
    }
  }
  virtual bool read(std::istream &is) { return true; }
  virtual bool write(std::ostream &os) const { return true; }
};

class PeriodicPlanOptimizer {
public:
  // Constants
  const int cycle, number_of_groups, number_of_way_points;

  // Variables and constraints
  std::vector<std::vector<WayPointVertex *>> paths;
  ParameterVertex *period;
  ParameterVertex *agent_radius;
  std::vector<ParameterVertex *> deltas;
  ParameterTargetConstraint *period_constraint, *agent_radius_constraint;
  std::vector<PathCostConstraint *> path_constraints;
  std::vector<PathCostConstraintMaxVelocity *> max_velocity_constraints;
  std::map<std::tuple<int, int, int, int, int>, CollisionConstraint *>
      collision_constraints;
  std::map<std::tuple<int, int, int, int>, CollisionConstraint2 *>
      collision_constraints_2;

  // Cost Parameters
  double period_cost_const, collision_cost_const, path_cost_const,
      agent_radius_cost_const;

  // Target Values
  double target_period, target_agent_radius;

  // int max_order_diff;

  int number_of_agents;

  g2o::SparseOptimizer optimizer;

  g2o::HyperGraph::EdgeSet default_set;

  std::shared_ptr<std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>>
      environment;

  g2o::OptimizationAlgorithmLevenberg *solver;

  double uninitialize_lambda = false;

  PeriodicPlanOptimizer(const YAML::Node &config, const int _cycle,
                        const int _number_of_groups,
                        const double initial_period,
                        const double initial_agent_size)
      : cycle(_cycle), number_of_groups(_number_of_groups),
        number_of_way_points(config["number_of_way_points"].as<int>()) {
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>> ABlockSolver;
    typedef g2o::LinearSolverEigen<ABlockSolver::PoseMatrixType> ALinearSolver;
    auto linearSolver = g2o::make_unique<ALinearSolver>();
    linearSolver->setBlockOrdering(false);
    solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<ABlockSolver>(std::move(linearSolver)));
    if (config["initial_lambda"]) {
      solver->setUserLambdaInit(config["initial_lambda"].as<double>());
    }
    if (config["uninitialize_lambda"]) {
      uninitialize_lambda = config["uninitialize_lambda"].as<bool>();
    }
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    int vertex_id = 0;
    period = new ParameterVertex;
    period->setId(vertex_id++);
    period->setEstimate(initial_period);
    optimizer.addVertex(period);
    if (config["target_period"]) {
      target_period = config["target_period"].as<double>();
      period_cost_const = config["period_cost"].as<double>();
      period_constraint = new ParameterTargetConstraint(period, target_period,
                                                        period_cost_const);
      optimizer.addEdge(period_constraint);
      default_set.insert(period_constraint);
    } else {
      period->setFixed(true);
    }
    agent_radius = new ParameterVertex;
    agent_radius->setId(vertex_id++);
    agent_radius->setEstimate(initial_agent_size);
    optimizer.addVertex(agent_radius);
    if (config["target_agent_radius"]) {
      target_agent_radius = config["target_agent_radius"].as<double>();
      agent_radius_cost_const = config["agent_radius_cost"].as<double>();
      agent_radius_constraint = new ParameterTargetConstraint(
          agent_radius, target_agent_radius, agent_radius_cost_const);
      optimizer.addEdge(agent_radius_constraint);
      default_set.insert(agent_radius_constraint);
    } else {
      agent_radius->setFixed(true);
    }
    path_cost_const = config["path_cost"].as<double>();
    number_of_agents = number_of_groups * cycle;
    paths.resize(number_of_agents);
    deltas.resize(number_of_agents);
    std::string path_cost_method = config["path_cost_method"].as<std::string>();
    double max_velocity = -1.0;
    if (config["max_velocity"]) {
      max_velocity = config["max_velocity"].as<double>();
    }
    for (int agent = 0; agent < number_of_agents; agent++) {
      deltas[agent] = new ParameterVertex;
      if (!config["variable_delta"].as<bool>()) {
        deltas[agent]->setFixed(true);
      }
      auto &path = paths[agent];
      path.resize(number_of_way_points);
      for (int i = 0; i < number_of_way_points; i++) {
        path[i] = new WayPointVertex;
        path[i]->setId(vertex_id++);
        optimizer.addVertex(path[i]);
      }
      path[0]->setFixed(true);
      path[number_of_way_points - 1]->setFixed(true);
      if (max_velocity > 0.) {
        for (int i = 0; i + 1 < number_of_way_points; i++) {
          auto path_cost_constraint = new PathCostConstraintMaxVelocity(
              path[i], path[i + 1], deltas[agent], max_velocity,
              config["max_velocity_cost"].as<double>() /
                  (number_of_way_points - 1));
          optimizer.addEdge(path_cost_constraint);
          default_set.insert(path_cost_constraint);
          max_velocity_constraints.push_back(path_cost_constraint);
        }
      }
      if (path_cost_method == "default") {
        for (int i = 0; i + 1 < number_of_way_points; i++) {
          auto path_cost_constraint = new PathCostConstraint(
              path[i], path[i + 1], deltas[agent],
              path_cost_const / (number_of_way_points - 1));
          optimizer.addEdge(path_cost_constraint);
          default_set.insert(path_cost_constraint);
          path_constraints.push_back(path_cost_constraint);
        }
      } else if (path_cost_method == "accelaration") {
        path[1]->setFixed(true);
        path[number_of_way_points - 2]->setFixed(true);
        for (int i = 1; i + 1 < number_of_way_points; i++) {
          auto path_cost_constraint = new PathCostConstraintAccelaration(
              path[i - 1], path[i], path[i + 1], deltas[agent],
              path_cost_const / (number_of_way_points - 1));
          optimizer.addEdge(path_cost_constraint);
          default_set.insert(path_cost_constraint);
        }
      } else if (path_cost_method != "nothing") {
        throw std::runtime_error("unknown path cost method: " +
                                 path_cost_method);
      }
    }
    collision_cost_const = config["collision_cost"].as<double>();
    if (config["Environment"]) {
      FILE *env = fopen(config["Environment"].as<std::string>().c_str(), "r");
      assert(env != NULL);
      environment.reset(
          new std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>);
      int number_of_holes;
      fscanf(env, "%d", &number_of_holes);
      for (int i = 0; i < number_of_holes + 1; i++) {
        int number_of_vertices;
        fscanf(env, "%d", &number_of_vertices);
        std::vector<Eigen::Vector2d> vertices(number_of_vertices);
        for (int j = 0; j < number_of_vertices; j++) {
          double x, y;
          fscanf(env, "%lf%lf", &x, &y);
          vertices[j] = Eigen::Vector2d(x, y);
        }
        for (int j = 0; j < number_of_vertices; j++) {
          int k = (j + 1) % number_of_vertices;
          environment->push_back(std::make_pair(vertices[j], vertices[k]));
        }
      }
      fclose(env);
      for (int agent = 0; agent < number_of_agents; agent++) {
        auto &path = paths[agent];
        for (int i = 0; i < number_of_way_points; i++) {
          auto environment_constraint = new WayPointConstraint(
              path[i], agent_radius, environment, collision_cost_const);
          optimizer.addEdge(environment_constraint);
          default_set.insert(environment_constraint);
        }
      }
    }
    /*double period_min = period->fixed()? period->estimate(): target_period;
    max_order_diff = std::floor(1./period_min);
    for(int order_diff =
    -max_order_diff;order_diff<=max_order_diff;order_diff++){ for(int
    agent_0=0;agent_0<number_of_agents;agent_0++){ int group_0 = agent_0 /
    cycle, order_0 = agent_0 % cycle; for(int
    group_1=0;group_1<number_of_groups;group_1++){ if(order_diff == 0 && group_0
    == group_1)continue; int agent_1 = group_1 * cycle +
    ((order_0+order_diff)%cycle+cycle)%cycle; for(int
    i=0;i<number_of_way_points;i++){ for(int j=0;j<number_of_way_points-1;j++){
              if(order_diff == 0 && i!=j)continue;
              if(order_diff > 0 && order_diff * period_min / time_delta + i >
    j)continue; if(order_diff < 0 && order_diff * period_min / time_delta + i +
    1 <= j)continue; auto constraint_tuple = std::make_tuple(order_diff,
    agent_0, group_1, i, j); double time_diff = (i-j) * time_delta;
              collision_constraints[constraint_tuple] = new
    CollisionConstraint(paths[agent_0][i], paths[agent_1][j],
    paths[agent_1][j+1], period, agent_radius, order_diff, time_diff,
    time_delta, collision_cost_const);
              assert(optimizer.addEdge(collision_constraints[constraint_tuple]));
            }
          }
        }
      }
      }*/
  }

  void update(const int iteration) {
    /*if(period->estimate()<target_period){
      period->setEstimate(target_period);
      }*/
    auto active_edges = default_set;
    double current_period = period->estimate();
    for (int agent_0 = 0; agent_0 < number_of_agents; agent_0++) {
      int group_0 = agent_0 / cycle, order_0 = agent_0 % cycle;
      double delta_0 = deltas[agent_0]->estimate();
      for (int i = 0; i < number_of_way_points; i++) {
        for (int agent_1 = 0; agent_1 < number_of_agents; agent_1++) {
          int group_1 = agent_1 / cycle, order_1 = agent_1 % cycle;
          double delta_1 = deltas[agent_1]->estimate();
          double order_diff_0 = order_0 - order_1;
          double time_diff = order_diff_0 * current_period + i * delta_0;
          for (int cycle_diff =
                   std::ceil(-time_diff / (cycle * current_period));
               cycle_diff <
               (-time_diff + (number_of_way_points - 1) * delta_1) /
                   (cycle * current_period);
               cycle_diff++) {
            if (agent_0 == agent_1 && cycle_diff == 0)
              continue;
            double time_0 = time_diff + cycle_diff * cycle * current_period;
            int j = std::max(
                0, std::min((int)(time_0 / delta_1), number_of_way_points - 2));
            if (agent_0 != agent_1) {
              auto constraint_tuple =
                  std::make_tuple(cycle_diff, agent_0, agent_1, i, j);
              if (collision_constraints.find(constraint_tuple) ==
                  collision_constraints.end()) {
                collision_constraints[constraint_tuple] =
                    new CollisionConstraint(
                        paths[agent_0][i], paths[agent_1][j],
                        paths[agent_1][j + 1], period, agent_radius,
                        deltas[agent_0], deltas[agent_1],
                        cycle_diff * cycle + order_diff_0, i, j,
                        collision_cost_const / (number_of_way_points - 1));
                assert(
                    optimizer.addEdge(collision_constraints[constraint_tuple]));
              } else {
                Eigen::Matrix<double, 1, 1, 0, 1, 1> information(
                    collision_cost_const);
                collision_constraints[constraint_tuple]->setInformation(
                    information);
              }
              active_edges.insert(collision_constraints[constraint_tuple]);
            } else {
              if (i == j || i == j + 1) {
                continue;
              }
              auto constraint_tuple =
                  std::make_tuple(cycle_diff, agent_0, i, j);
              if (collision_constraints_2.find(constraint_tuple) ==
                  collision_constraints_2.end()) {
                collision_constraints_2[constraint_tuple] =
                    new CollisionConstraint2(
                        paths[agent_0][i], paths[agent_1][j],
                        paths[agent_1][j + 1], period, agent_radius,
                        deltas[agent_0], cycle_diff * cycle + order_diff_0,
                        i - j,
                        collision_cost_const / (number_of_way_points - 1));
                assert(optimizer.addEdge(
                    collision_constraints_2[constraint_tuple]));
              } else {
                Eigen::Matrix<double, 1, 1, 0, 1, 1> information(
                    collision_cost_const);
                collision_constraints_2[constraint_tuple]->setInformation(
                    information);
              }
              active_edges.insert(collision_constraints_2[constraint_tuple]);
            }
          }
        }
      }
    }
    optimizer.initializeOptimization(active_edges);
    if (uninitialize_lambda) {
      solver->setUserLambdaInit(solver->currentLambda());
    }
    assert(optimizer.optimize(iteration) > 0);
    assert(optimizer.activeChi2() < 1e9);
  }
  void growCoefficients(const double rate) {
    collision_cost_const *= rate;
    if (agent_radius_constraint != NULL) {
      agent_radius_constraint->setInformation(
          rate * agent_radius_constraint->information());
    }
    for (auto &constraint : max_velocity_constraints) {
      constraint->setInformation(rate * constraint->information());
    }
    for (auto &constraint : path_constraints) {
      constraint->setInformation(constraint->information() / rate);
    }
  }
};
