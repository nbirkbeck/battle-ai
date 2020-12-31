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
    FullReset();
  }

  void FullReset() {
    world_time_ = 0;
    geoms_.clear();
    agents_.clear();
    power_ups_.clear();
    bounds_.min = nacb::Vec3d(1e10, 1e10, 1e10);
    bounds_.max = nacb::Vec3d(-1e10, -1e10, -1e10);
  }
  void Reset();
  void ResetAgent(int i);

  bool Step(const double dt);
  bool IsGameOver() const;
  bool LineHits(const nacb::Vec3d& p1, const nacb::Vec3d& p2, double *t, Agent** agent) const;

  bool LoadFromFile(const std::string& filename);
  bool LoadFromProto(const state::Level& level,
                     std::function<Agent*(int i, const nacb::Vec3d& pos,
                                          const nacb::Quaternion& quat,
                                          std::vector<Weapon> weapons)> create_agent = nullptr);

  const GeometryVector& geoms() const { return geoms_; }
  const std::vector<std::unique_ptr<Agent> >& agents() const { return agents_; }
  const std::vector<std::unique_ptr<PowerUp> >& power_ups() const { return power_ups_; }
  const ProjectileVector& projectiles() const { return projectiles_; }

  void ReplaceAgent(int i, Agent* agent) {
    const nacb::Vec3d p = agents_[i]->pos();
    agent->set_pos(p);
    agents_[i].reset(agent);
  }

  const AccessibilityMap& access_map() const {
    return *access_map_.get();
  }
  const double world_time() const { return world_time_; }
  
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
  state::Level level_proto_;
};

#endif  //  WORLD_H
