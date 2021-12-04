#include <GL/glew.h>
#include <GL/gl.h>
#include <stdint.h>

#include "src/world.h"
#include "src/ui/simple_window.h"
#include "src/ui/utigl/ogre_window.h"
#include "src/ui/ogre_win.h"
#include "src/agent/agent.h"
#include "src/agent/simple_agent.h"
#include "src/agent/search.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

// This an implementation of an agent that allows for high-level decision making.
// E.g,. are we moving towards a specific power-up, or attacking opponent?
//
// The basic state consists of:
//  position, health, armor
//  distance of self to power-up.
//  power-up respawn delay
//  distance of opponent to power-up
//  distance to objects in each possible move direction.
//  distance to plan-endpoint
//  last action.
//
// Actions are:
//  Move in one of 4 directions
//  Plan to move to a power-up.
//
// This implementation is meant to be called from python. The caller should
// set up the desired action before world->Step is called.
class HighLevelAgent : public PlanFollowingAgent {
public:
  enum Direction {
     FORWARD = 0,
     LEFT = 1,
     BACK = 2,
     RIGHT = 3
  };

  HighLevelAgent():
    PlanFollowingAgent(nacb::Vec3d(), nacb::Quaternion(), {Weapon()}) {}

  std::vector<double> Observe(const World* world) {
    const ObservableState state = world->GetObservableStateForAgent(this);

    std::vector<nacb::Vec3d> other_points;
    for (const auto& other_agents : state.visible_agents) {
      other_points.push_back(other_agents.pos);
    }

    nacb::Vec3d other_agent_pos = last_opponent_pos_;
    if (!other_points.empty()) {
      other_agent_pos = other_points.front();
    }

    // Basic state: position, health, armor
    std::vector<double> features;
    features.push_back(pos2d()[0] / 20.0);
    features.push_back(pos2d()[1] / 20.0);
    features.push_back(health() / max_health() * 2 - 1);
    features.push_back(armor() / max_armor() * 2 - 1);

    // Distance to power-ups (and opponent distance to power ups)
    for (const auto& pup: state.power_ups) {
      std::deque<nacb::Vec3d> points;
      const double pup_distance = Search(state.access_map, other_points, pos(), pup->pos(), &points);
      features.push_back(NormalizeDistance(pup_distance));
      features.push_back(pup->time_until_respawn() / pup->respawn_delay());
      features.push_back(NormalizeDistance(Search(state.access_map, {}, other_agent_pos, pup->pos(), &points)));
    }

    // Distance to opponent (if known)
    std::deque<nacb::Vec3d> points;
    features.push_back(NormalizeDistance(Search(state.access_map, {}, pos(), other_agent_pos, &points)));
    features.push_back(state.visible_agents.size());

    // Sense distance in each possible move direction.
    std::vector<const Agent*> agents = {}; // Don't care about proximity to agents.
    for (int i = 0; i < 4; ++i) {
      nacb::Vec3d dir = GetDirection(i);
      double t = 0;
      Agent* agent = nullptr;
      if (LineHitsAnything(pos(), pos() + dir, world->geoms(), agents, &t, &agent) && t >= 0 && t <= 1) {
        features.push_back(t);
      } else {
        features.push_back(1);
      }
    }

    // Distance to plan
    if (plan_) {
      features.push_back(NormalizeDistance((plan_->waypoints_.back() - pos()).len()));
    } else {
      features.push_back(-1);
    }
    // Last action
    features.push_back(last_action_);

    last_opponent_pos_ = other_agent_pos;
    num_power_ups_ = state.power_ups.size();
    return features;
  }

  void Reset() override {
    last_opponent_pos_ = nacb::Vec3d(0, 0, 0);
    current_action_ = last_action_ = 0;
    Agent::Reset();
    prev_ = pos();
  }

  nacb::Vec3d GetDirection(int dir) const {
    const nacb::Vec3d forward = quat().rotate(nacb::Vec3d(0, 0, 1));
    switch (dir) {
    case FORWARD:
      return forward;
    case LEFT:
      return nacb::Quaternion::rod(nacb::Vec3d(0, M_PI / 2.0, 0)).rotate(forward);
    case RIGHT:
      return nacb::Quaternion::rod(nacb::Vec3d(0, -M_PI / 2.0, 0)).rotate(forward);
    case BACK:
      return forward * -1;
    }
    assert(false);
    return nacb::Vec3d();
  }

  void SetAction(int action_index) {
    current_action_ = action_index;
  }

  void GetActions(const ObservableState& state,
                  battle::Actions* actions) {
    prev_ = pos();
    std::vector<nacb::Vec3d> other_points;
    for (const auto& other_agents : state.visible_agents) {
      other_points.push_back(other_agents.pos);
    }
    // 0-3: Forward, left, right, backward
    if (current_action_ < 4) {
      if (!plan_ || plan_->AlmostFinished(pos()) || last_action_ != current_action_) {
        nacb::Vec3d new_pos = pos() + GetDirection(current_action_);
        nacb::Vec2<int> pos_i = state.access_map->WorldToImage(pos());
        nacb::Vec2<int> new_pos_i = state.access_map->WorldToImage(new_pos);
        const int search_range = (int)((pos_i - new_pos_i).len() + 1);
        nacb::Vec2<int> best_pos_i = new_pos_i;
        double best_dist = 1e10;
        for (int x = -search_range; x <= search_range; ++x) {
          for (int z = -search_range; z <= search_range; ++z) {
            nacb::Vec2d cand_i = new_pos_i + nacb::Vec2<int>(x, z);
            if (cand_i.x < 0 || cand_i.y < 0 ||
                cand_i.x >= state.access_map->map().w ||
                cand_i.y >= state.access_map->map().h) continue;
            if (!state.access_map->map()(cand_i.x, cand_i.y)) continue;
            const double dist = (cand_i - new_pos_i).len();
            if (dist < best_dist) {
              best_dist = dist;
              best_pos_i = cand_i;
            }
          }
        }
        // TODO: detect if we are stuck.
        new_pos = state.access_map->ImageToWorld(best_pos_i);
        const bool attack = current_action_ == FORWARD && !other_points.empty();
        // When attacking, move on path towards agent.
        if (attack) {
          new_pos = other_points.front();
        }
        std::deque<nacb::Vec3d> points;
        Search(state.access_map, other_points, pos(), new_pos, &points);

        // When attacking, don't plan too far ahead.
        if (attack) {
          for (int i = 0; i < (int)points.size(); ++i) {
            if ((points[i] - pos()).len() > 2.0) {
              LOG(INFO) << "Dropping points after " << i
                        << " (size = " << points.size() << ")";
              points.erase(points.begin() + i, points.end());
              break;
            }
          }
        }
        plan_.reset(new Plan(points));
      }
    } else {
      const int pup_index = current_action_ - 4;
      assert(pup_index < state.power_ups_size());
      if (!plan_ || last_action_ != current_action_) {
        const auto& pup = state.power_ups[pup_index];
        std::deque<nacb::Vec3d> points;
        Search(state.access_map, other_points, pos(), pup->pos(), &points);
        plan_.reset(new Plan(points));
      }
    }
    last_action_ = current_action_;

    return PlanFollowingAgent::GetActions(state, actions);
  }    

  double NormalizeDistance(double d) const {
    // TODO: should be based on world bounds.
    return d / 20.0 - 1.0;
  }

  std::vector<double> pos2d() const {
    std::vector<double> p(2);
    p[0] = pos().x;
    p[1] = pos().z;
    return p;
  }

  double MovedDistance() const { return (pos() - prev_).len(); }
  int last_action() const { return last_action_; }

private:
  nacb::Vec3d prev_;
  nacb::Vec3d last_opponent_pos_;
  int num_power_ups_;
  int current_action_ = 0;
  int last_action_ = 0;
};

class DiscreteAgent : public Agent {
public:

  DiscreteAgent():
    Agent(nacb::Vec3d(), nacb::Quaternion(),{Weapon()}) {}

  void ClearActions() {
    pending_action_.Clear();
  }

  void GetActions(const ObservableState& observable_state,
                  battle::Actions* actions) {
    *actions = pending_action_;
  }

  void MoveDirection(const std::vector<double>& d) {
    nacb::Vec3d dir(d[0], 0, d[1]);
    auto* target = pending_action_.mutable_move()->mutable_target();
    prev_ = pos();
    target->set_x(pos().x + dir.x);
    target->set_y(pos().y + dir.y);
    target->set_z(pos().z + dir.z);
  }

  double MovedDistance() const {
    return (pos() - prev_).len();
  }

  bool NearPowerup(const World* world) const {
    for (const auto& pup : world->power_ups()) {
      if ((pos() - pup->pos()).len() < 1)
        return true;
    }
    return false;
  }

  double PowerupDist(const World* world) const {
    double min_dist = 1e10;
    for (const auto& pup : world->power_ups()) {
      double dist = (pos() - pup->pos()).len();
      min_dist = std::min(min_dist, dist);
    }
    return exp(-min_dist * min_dist / (20 * 20));
  }

  std::vector<double> pos2d() const {
    std::vector<double> p(2);
    p[0] = pos().x;
    p[1] = pos().z;
    return p;
  }

public:
  nacb::Vec3d prev_;
  battle::Actions pending_action_;
};

template <class A>
void ReplaceAgent(World* world, int index, A* agent) {
  std::cerr << "Replacing agent" << std::endl;
  world->ReplaceAgent(index, agent);
  std::cerr << "...done replacing agent" << std::endl;
}

std::vector<double> PowerUpTimes(World* world) {
  std::vector<double> res;
  for (const auto& pup : world->power_ups()) {
    res.push_back(pup->time_until_respawn());
  }
  return res;
}


std::vector<uint8_t> GetAccessMap(World* world, int index, DiscreteAgent* agent) {
  const auto& access_map = world->access_map();
  std::vector<uint8_t> im(access_map.map().w * access_map.map().h + 2);
  for (int i = 0; i < (int)im.size() - 2; ++i) {
    im[i] = access_map.map().data[i];
  }
  for (const auto& p: world->power_ups()) {
    nacb::Vec2<int> i = access_map.WorldToImage(p->pos());
    if (p->IsActive()) {
      for (int yi = -1; yi <= 1; ++yi) {
        for (int xi = -1; xi <= 1; ++xi) {
          im[(yi + i.y) * access_map.map().w + (i.x + xi)] = 32;
        }
      }
    }
  }

  nacb::Vec2<int> i = access_map.WorldToImage(agent->pos());
  im[i.y * access_map.map().w + i.x] = 192;
  im[(int)im.size() - 2] = (uint8_t)access_map.map().w;
  im[(int)im.size() - 1] = (uint8_t)access_map.map().h;
  return im;
}


PYBIND11_MODULE(battle_ai, m) {
  m.def("replace_agent", &ReplaceAgent<DiscreteAgent>);
  m.def("replace_agent_simple", &ReplaceAgent<SimpleAgent>);
  m.def("replace_agent", &ReplaceAgent<HighLevelAgent>);
  m.def("get_map", &GetAccessMap);
  m.def("powerup_times", &PowerUpTimes);

  py::class_<nacb::Image8>(m, "Image8")
    .def(py::init<int, int, int>())
    .def("width", &Image8::getWidth)
    .def("height", &Image8::getHeight)
    .def("get", &Image8::get)
    .def("set", [](Image8& self, int x, int y, int c, int v) {
       self(x, y, c) = v;
    })
    .def("resize", &Image8::resize)
    .def("save", &Image8::save);
    
  py::class_<AccessibilityMap>(m, "AccessibilityMap")
    .def("world_to_image", &AccessibilityMap::WorldToImage)
    .def("image", &AccessibilityMap::map);
      
  py::class_<World>(m, "World")
    .def(py::init<>())
    .def("load_from_file", &World::LoadFromFile)
    .def("replace_agent", &World::ReplaceAgent)
    .def("build_accessibility_map", &World::BuildAccessibilityMap)
    .def("reset_agent", &World::ResetAgent)
    .def("step", &World::Step)
    .def("reset", &World::Reset);
  
  py::class_<SimpleWindow>(m, "SimpleWindow")
    .def(py::init<>())
    .def("draw_scene", &SimpleWindow::drawScene)
    .def("set_world", &SimpleWindow::SetWorld)
    .def("refresh", &SimpleWindow::refresh)
    .def("is_closed", &SimpleWindow::IsClosed)
    .def("set_info_string", [](const SimpleWindow& win, const std::string){}) // no-op
    .def("process_events", &SimpleWindow::ProcessEvents);

  py::class_<OgreWin>(m, "OgreWin")
    .def(py::init<std::string>())
    .def("draw_scene", &OgreWin::DrawScene)
    .def("set_world", &OgreWin::SetWorld)
    .def("set_info_string", &OgreWin::SetInfoString)
    .def("is_closed", &OgreWin::IsClosed)
    .def("refresh", &OgreWin::refresh)
    .def("process_events", &OgreWin::ProcessEvents);
  
  py::class_<SimpleAgent>(m, "SimpleAgent")
    .def(py::init<>())
    .def("health", &SimpleAgent::health)
    .def("armor", &SimpleAgent::armor)
    .def("reset", &SimpleAgent::Reset);
  
  py::class_<DiscreteAgent>(m, "DiscreteAgent")
    .def(py::init<>())
    .def("clear_actions", &DiscreteAgent::ClearActions)
    .def("move_direction", &DiscreteAgent::MoveDirection)
    .def("moved_distance", &DiscreteAgent::MovedDistance)
    .def("near_powerup", &DiscreteAgent::NearPowerup)
    .def("powerup_dist", &DiscreteAgent::PowerupDist)
    .def("do_damage", &DiscreteAgent::DoDamage)
    .def("health", &DiscreteAgent::health)
    .def("armor", &DiscreteAgent::armor)
    .def("pos2d", &DiscreteAgent::pos2d);

  py::class_<HighLevelAgent>(m, "HighLevelAgent")
    .def(py::init<>())
    .def("set_action", &HighLevelAgent::SetAction)
    .def("moved_distance", &HighLevelAgent::MovedDistance)
    .def("observe", &HighLevelAgent::Observe)
    .def("do_damage", &HighLevelAgent::DoDamage)
    .def("health", &HighLevelAgent::health)
    .def("last_action", &HighLevelAgent::last_action)
    .def("armor", &HighLevelAgent::armor)
    .def("reset", &HighLevelAgent::Reset)
    .def("pos2d", &HighLevelAgent::pos2d);
}
