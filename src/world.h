#ifndef WORLD_H
#define WORLD_H 1

#include <vector>
#include <memory>
#include <functional>
#include <glog/logging.h>

#include <nimage/image.h>

#include "axis_aligned_box.h"
#include "agent.h"
#include "simple_agent.h"
#include "power_up.h"
#include "src/proto/level.pb.h"
#include "observable_state.h"

inline nacb::Vec3d CreateVec3d(const state::Vec3& v) {
  return nacb::Vec3d(v.x(), v.y(), v.z());
}

inline bool IsValidPoint(const nacb::Vec3d& x) {
  return (2 * x.x == int(2 * x.x)) &&
    (x.y == int(x.y)) &&
    (2 * x.z == int(2 * x.z));
}


class World {
 public:
      
  World() {
    Reset();
  }

  void Reset() {
    world_time_ = 0;
    geoms_.clear();
    agents_.clear();
    power_ups_.clear();
    bounds_.min = nacb::Vec3d(1e10, 1e10, 1e10);
    bounds_.max = nacb::Vec3d(-1e10, -1e10, -1e10);
  }

  bool Step(const double dt);
  bool IsGameOver() const;
  bool LineHits(const nacb::Vec3d& p1, const nacb::Vec3d& p2, double *t, Agent** agent) const;

  bool LoadFromProto(const state::Level& level,
                     std::function<Agent*(int i, const nacb::Vec3d& pos,
                                          const nacb::Quaternion& quat,
                                          std::vector<Weapon> weapons)> create_agent = nullptr) {
    Reset();

    int box_index = 0;
    for (const auto& lbox : level.boxes()) {
      geoms_.push_back(
                       std::unique_ptr<AxisAlignedBox>(
                                                   new AxisAlignedBox(
                                                                      CreateVec3d(lbox.pos()),
                                                                      CreateVec3d(lbox.size()))));
      const auto& geom = *geoms_.back();
      const auto& n = geom.min();
      const auto& x = geom.max();
      if (!IsValidPoint(n) || !IsValidPoint(x) || n.y != 0) {
        LOG(ERROR) << "Invalid bounding box point:" << n << " to " << x
                   << " on " << lbox.DebugString() << " at index " << box_index;
        return false;
      }
      bounds_.min = bounds_.min.min(n);
      bounds_.max = bounds_.max.max(x);
      box_index++;
    }
    for (const auto& pup : level.power_ups()) {
      power_ups_.push_back(
                           std::unique_ptr<PowerUp>(
                                                    new PowerUp(CreateVec3d(pup.pos()),
                                                                pup.type(),
                                                                pup.amount())));
    }
    int agent_index = 0;
    for (const auto& spawn_points : level.spawn_points()) {
      nacb::Quaternion q(nacb::Vec3d(0, 0, 0), 1);
      std::vector<Weapon> weapons;
      weapons.push_back(Weapon());
      nacb::Vec3d pos = CreateVec3d(spawn_points.pos());
      pos.y = 0.5;
      if (create_agent) {
        agents_.push_back(std::unique_ptr<Agent>(create_agent(agent_index, pos, q, weapons)));
      } else {
        agents_.push_back(std::unique_ptr<Agent>(new SimpleAgent(pos,
                                                                 q,
                                                                 weapons)));
      }
      agent_index++;
    }
    access_map_.reset(new AccessibilityMap(bounds_, geoms_, 3));
    return true;
  }


  const GeometryVector& geoms() { return geoms_; }
  const std::vector<std::unique_ptr<Agent> >& agents() const { return agents_; }
  const std::vector<std::unique_ptr<PowerUp> >& power_ups() const { return power_ups_; }
  const ProjectileVector& projectiles() const { return projectiles_; }
  
 protected:
  World(const World&) = delete;
  void operator=(const World&) = delete;
  
  double world_time_ = 0;
  GeometryVector geoms_;
  std::vector<std::unique_ptr<Agent> > agents_;
  std::vector<std::unique_ptr<PowerUp> > power_ups_;
  ProjectileVector projectiles_;
  AxisAlignedBounds bounds_;
  std::unique_ptr<AccessibilityMap> access_map_;
};

#endif  //  WORLD_H
