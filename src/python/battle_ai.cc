#include <GL/gl.h>
#include <GL/glew.h>
#include <stdint.h>

#include "src/agent/high_level_agent.h"
#include "src/ui/ogre_win.h"
#include "src/ui/simple_window.h"
#include "src/ui/utigl/ogre_window.h"
#include "src/world.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

class DiscreteAgent : public Agent {
public:
  DiscreteAgent() : Agent(nacb::Vec3d(), nacb::Quaternion(), {Weapon()}) {}

  void ClearActions() { pending_action_.Clear(); }

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

  double MovedDistance() const { return (pos() - prev_).len(); }

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

template <class A> void ReplaceAgent(World* world, int index, A* agent) {
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

std::vector<uint8_t> GetAccessMap(World* world, int index,
                                  DiscreteAgent* agent) {
  const auto& access_map = world->access_map();
  std::vector<uint8_t> im(access_map.map().w * access_map.map().h + 2);
  for (int i = 0; i < (int)im.size() - 2; ++i) {
    im[i] = access_map.map().data[i];
  }
  for (const auto& p : world->power_ups()) {
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
      .def("set",
           [](Image8& self, int x, int y, int c, int v) { self(x, y, c) = v; })
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
      .def("set_info_string",
           [](const SimpleWindow& win, const std::string) {}) // no-op
      .def("process_events", &SimpleWindow::ProcessEvents);

  py::class_<OgreWin>(m, "OgreWin")
      .def(py::init<std::string>())
      .def("draw_scene", &OgreWin::DrawScene)
      .def("set_world", &OgreWin::SetWorld)
      .def("set_info_string", &OgreWin::SetInfoString)
      .def("is_closed", &OgreWin::IsClosed)
      .def("refresh", &OgreWin::refresh)
      .def("process_events", &OgreWin::ProcessEvents);

  // Agents.
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
