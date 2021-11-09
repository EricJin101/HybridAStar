#pragma once

namespace eric {
namespace hybrid_a_star {

class Cost{
private:
  PathPoint current_;
  PathPoint father_;
  double g_;  // real cost
  double h_;  // expected cost
  double f_;  // final cost (f = g + h)
  std::vector<PathPoint> cost_path_;

public:
  void set_current(PathPoint& _t) {
    current_.set_utm_x(_t.utm_x());
    current_.set_utm_y(_t.utm_y());
    current_.set_utm_theta(_t.utm_theta());
  }
  void set_father(PathPoint& _t) {
    father_.set_utm_x(_t.utm_x());
    father_.set_utm_y(_t.utm_y());
    father_.set_utm_theta(_t.utm_theta());
  }
  PathPoint current() {return current_;}
  PathPoint father() {return father_;}

  void set_g(double t) {g_ = t;}
  void set_h(double t) {h_ = t;}
  void set_f(double t) {f_ = t;}

  double g() {return g_;}
  double h() {return h_;}
  double f() {return f_;}

  void set_cost_path(std::vector<PathPoint>& _t) {
    for (PathPoint _p : _t){
      cost_path_.emplace_back(_p);
    }
  }
  std::vector<PathPoint> cost_path() { return cost_path_;}
};

class Node{
public:
  double x_index;
  double y_index;
  double yaw_index;
  double trailer_index;  // trailer angle
  bool direction;
  std::vector<PathPoint> path;  // path from parent -> current
  // utm_x position [m], utm_y position [m], utm_theta angle [rad], trailer_angle [rad]
  std::vector<bool> directions;  // directions of each points forward: true, backward:false
  double steer;  // steer input
  double cost;  // cost
  double parent_index;  // parent index
};

class NodeList{
public:
  double index = 0;
  Node node;
};

class Config{
public:
  double min_x;
  double min_y;
  double min_yaw;
  double min_trailer_yaw;
  double max_x;
  double max_y;
  double max_yaw;
  double max_trailer_yaw;
  double x_w;
  double y_w;
  double yaw_w;
  double yaw_trailer_w;
  double xy_resolution;
  double yaw_resolution;
};

class AStarNodeCost{
public:
  double node_x;
  double node_y;
  double cost;
  double parent_index;
};

class AStarNodeList{
public:
  double index = 0;
  AStarNodeCost node;
};

class AStarOBMap{
public:
  double x;
  double y;
  bool occupation;
};

class AStarMotion{
public:
  double x;
  double y;
  double dis;
};

class ReedsSheppPathsCost{
public:
  std::vector<port::Trajectory> rs_path_;
  std::vector<PathPoint> rs_path_pts_;
  double cost_;
};


}  // namespace hybrid_a_star
}  // eric

