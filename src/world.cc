#include "world.h"

#include <algorithm>

#include "plane.h"
#include "proto.h"
#include "observable_state.h"
#include "src/proto/actions.pb.h"

extern std::deque<nacb::Vec3d> g_def;

template <class AgentType>
const Agent* get_ptr(AgentType& agent) {
  return agent;
}

template <>
const Agent* get_ptr(const std::unique_ptr<Agent>& agent) {
  return agent.get();
}

template <class AgentPointer>
bool LineHitsAnything(const nacb::Vec3d& p1, const nacb::Vec3d& p2,
                      const GeometryVector& geoms,
                      const std::vector<AgentPointer>& agents,
                      double* t,
                      Agent** agent) {
  double min_t = 1e10;
  for (const auto& g : geoms) {
    double t = 1e10;
    if (g->IntersectRay(p1, (p2 - p1), &t) && t < min_t && t > 0) {
      min_t = t;
    }
  }
  *agent = 0;
  for (const auto& a : agents) {
    double t = 1e10;
    nacb::Vec3d d = p2 - p1;
    if (a->IntersectRay(p1, d, &t)) {
      if (t < min_t && t > 0) {
        *agent = const_cast<Agent*>(get_ptr(a));
        min_t = t;
      }
    }
  }
  *t = min_t;
  return min_t <= 1;
}

bool IntersectRayAndGeoms(const GeometryVector& geoms,
                          const nacb::Vec3d& p,
                          const nacb::Vec3d& d,
                          const double radius,
                          nacb::Vec3d* isect_pos,
                          nacb::Vec3d* deflected_pos) {
  double min_t = 1;
  for (const auto& g: geoms) {
    double t = 1e10;
    nacb::Vec3d deflected_temp;
    if (g->IntersectRay(p, d, &t, radius, &deflected_temp) && t < min_t) {
      *deflected_pos = deflected_temp;
      min_t = t;
    }
  }
  if (min_t < 1) {
    *isect_pos = p + min_t * d;
    //g_def.push_back(deflected);
    //if (g_def.size() > 1000)
    //  g_def.pop_front();
    return true;
  }
  return false;
}

void HandleAgentMove(Agent* a,
                     const battle::Actions::Move& move,
                     const GeometryVector& geoms,
                     double dt,
                     battle::ActionResponse::Result* results) {
  nacb::Vec3d new_pos = CreateVec3d(move.target());
  nacb::Vec3d dir = new_pos - a->pos();
  const double l = dir.len();
  const double kMaxVelocity = 4; // TODO: fix this
  const double kMaxMove = kMaxVelocity * dt;
  if (l > 0) {
    new_pos = a->pos() + dir * std::min(1.0, kMaxMove / l);
    const double radius = a->GetCylinderRadius();
    nacb::Vec3d first_isect_pos, first_deflected_pos;

    if (IntersectRayAndGeoms(geoms, a->pos(), new_pos - a->pos(), radius,
                             &first_isect_pos, &first_deflected_pos)) {
      nacb::Vec3d second_isect_pos, second_deflected_pos;
      if (IntersectRayAndGeoms(geoms, first_isect_pos, first_deflected_pos - first_isect_pos,
                               radius, &second_isect_pos, &second_deflected_pos)) {
        new_pos = second_isect_pos;
      } else {
        new_pos = first_deflected_pos;
      }
    }
    results->set_code(battle::STATUS_OK);
    auto* m = results->mutable_move();
    auto* p = m->mutable_pos();
    p->set_x(new_pos.x);
    p->set_y(new_pos.y);
    p->set_z(new_pos.z);
  } else {
    results->set_code(battle::STATUS_NO_OP);
    auto* m = results->mutable_move();
    auto* p = m->mutable_pos();
    p->set_x(new_pos.x);
    p->set_y(new_pos.y);
    p->set_z(new_pos.z);
    
  }
}

void HandleAgentRotate(Agent* a,
                       const battle::Actions::Rotate& rotate,
                       const GeometryVector& geoms,
                       double dt,
                       battle::ActionResponse::Result* results) {

  auto target_quat = nacb::Quaternion(nacb::Vec3d(rotate.v().x(),
                                                  rotate.v().y(),
                                                  rotate.v().z()),
                                      rotate.a());
  //auto q = nacb::Quaternion(nacb::Vec3d(0, 1, 0), rotate.angle()) * a->quat();
  auto* r = results->mutable_rotate();
  r->mutable_v()->set_x(target_quat.v.x);
  r->mutable_v()->set_y(target_quat.v.y);
  r->mutable_v()->set_z(target_quat.v.z);
  r->set_a(target_quat.a);
}

void HandleAgentShoot(Agent* a,
                      const battle::Actions::Shoot& shoot,
                      ProjectileVector* projectiles,
                      double world_time,
                      battle::ActionResponse::Result* results) {
  if (!shoot.shoot()) return;
  if (a->current_weapon().CanFire(world_time)) {
    nacb::Vec3d p, d;
    a->GetWeaponRay(&p, &d);
    LOG(INFO) << p << " " << d;
    projectiles->push_back(a->current_weapon().Fire(world_time, p, d));

    results->set_code(battle::STATUS_OK);
    results->mutable_shoot();
  } else {
    results->set_code(battle::STATUS_INVALID_ACTION);
    results->mutable_shoot();
  }
}

bool World::Step(const double dt) {
  std::vector<battle::Actions> agent_actions;

  for (auto& a: agents_) {
    // TODO: give agent a chance to observe the world.
    battle::Actions actions;
    std::vector<ObservableState::Agent> visible_agents;

    for (auto& a2: agents_) {
      if (a2.get() == a.get()) continue;
      
      nacb::Vec3d p1 = a->pos();
      nacb::Vec3d p2 = a2->pos();
      nacb::Vec3d d = p2 - p1;
      nacb::Vec3d x = nacb::Vec3d(0, 1, 0).cross(d * (1.0/d.len()));

      std::vector<const Agent*> agents;
      agents.push_back(a2.get());
      Agent* hit_agent = nullptr;
      double t = 0;
      double confidence = 0;
      for (int n = -3; n <= 3; ++n) {
        if (LineHitsAnything(p1, p2 + x * (n / 3.0), geoms_, agents, &t, &hit_agent) &&
            t > 0 && hit_agent == a2.get()) {
          confidence += 1;
        }
      }
      confidence /= 7.0;
      if (confidence > 0) {
        visible_agents.push_back({confidence, a2->pos()});
      }
    }
    const ObservableState state = {power_ups_, access_map_.get(), visible_agents};
    a->GetActions(state, &actions);
    agent_actions.push_back(actions);     
  }

  // Validate and apply any actions
  {
    int i = 0;
    for (auto& a: agents_) {
      battle::ActionResponse response;
      if (agent_actions[i].has_shoot()) {
        HandleAgentShoot(a.get(),
                         agent_actions[i].shoot(),
                         &projectiles_, world_time_,
                         response.add_results());
      }
      if (agent_actions[i].has_move()) {
        HandleAgentMove(a.get(), agent_actions[i].move(), geoms_, dt,
                        response.add_results());
      }
      if (agent_actions[i].has_rotate()) {
        HandleAgentRotate(a.get(), agent_actions[i].rotate(), geoms_, dt,
                          response.add_results());
      }
      a->Update(response);
      ++i;
    }
  }
    
  // If any of the agents overlap with a power up, then give the
  // agent the power-up.
  for (auto& p: power_ups_) {
    p->Step(dt);
  }
  for (auto& a: agents_) {
    const auto agent_bounding_geom = a->GetBoundingGeometry();
    for (auto& p: power_ups_) {
      const auto power_up_bounding_geom = p->GetBoundingGeometry();
      if (!p->IsActive()) continue;
      if (agent_bounding_geom.Intersects(power_up_bounding_geom)) {
        p->Give(a.get());
      }
    }
  }

  // TODO: delete inactive power ups?

  // Move any active projectiles.
  for (auto& p : projectiles_) {
    if (!p.IsActive()) continue;
    const nacb::Vec3d& pos_now = p.p();
    const nacb::Vec3d pos_future = pos_now + p.v() * dt;
    Agent* agent = 0;
    double t = 0;
    if (LineHits(pos_now, pos_future, &t, &agent)) {
      
      if (agent) {
        agent->DoDamage(p.damage());
        LOG(INFO) << "Hit agent:" << agent->health();

        p.Kill(world_time_);
        p.MoveTo(pos_now + (pos_future - pos_now) * t);
      } else {
        p.MoveTo(pos_now + (pos_future - pos_now) * t);
        p.Kill(world_time_);
      }

    } else {
      p.MoveTo(pos_future);
    }
  }
  auto new_end = std::remove_if(projectiles_.begin(), projectiles_.end(), [=](const Projectile& p) {
                                                                            return p.CanRemove(world_time_);
                                                                          });
  projectiles_.erase(new_end, projectiles_.end());
  world_time_ += dt;
  return IsGameOver();
}

bool World::IsGameOver() const {
  for (const auto & a: agents_) {
    if (a->health() <= 0) return true;
  }
  return false;
}

bool World::LineHits(const nacb::Vec3d& p1, const nacb::Vec3d& p2, double* t,
                     Agent** agent) const {
  return LineHitsAnything(p1, p2, geoms_, agents_, t, agent);
}

bool World::LoadFromFile(const std::string& filename) {
  state::Level level;
  if (!Proto::ReadProto(filename, &level)) {
    LOG(INFO) << "Error reading level from:" << filename;
    return false;
  }
  return LoadFromProto(level);
}

bool World::LoadFromProto(const state::Level& level,
                          std::function<Agent*(int i, const nacb::Vec3d& pos,
                                               const nacb::Quaternion& quat,
                                               std::vector<Weapon> weapons)> create_agent) {
  FullReset();
  level_proto_ = level;
  
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
    pos.y = 1;
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

void World::Reset() {
  int agent_index = 0;
  // TODO: reset other attributes
  for (const auto& spawn_points : level_proto_.spawn_points()) {
    nacb::Vec3d pos = CreateVec3d(spawn_points.pos());
    agents_[agent_index]->set_pos(pos);
    agents_[agent_index]->Reset();
  }
  for (auto& pup : power_ups_) {
    pup->Reset();
  }
  projectiles_.clear();
  agent_index++;
}

void World::ResetAgent(int agent_index) {
  // TODO: reset other attributes
  const nacb::Vec3d pos = CreateVec3d(level_proto_.spawn_points(agent_index).pos());
  agents_[agent_index]->set_pos(pos);
  agents_[agent_index]->Reset();
}

AccessibilityMap::AccessibilityMap(const AxisAlignedBounds& bounds,
                                   const GeometryVector& geoms,
                                   int resolution) : bounds_(bounds), resolution_(resolution) {
  const nacb::Vec3d size = bounds_.max - bounds_.min;
  nacb::Image8 image(int(size.x * resolution + 0.5), int(size.z * resolution + 0.5), 1);
  image = 255;

  for (int zi = 0; zi < image.h; ++zi) {
    for (int xi = 0; xi < image.w; ++xi) {
      const double x = (double(xi + 0.5) / image.w) * size.x + bounds_.min.x;
      const double y = 1.0;
      const double z = (double(zi + 0.5) / image.h) * size.z + bounds_.min.z;
      nacb::Vec3d p(x, y, z);
      for (const auto& g : geoms) {
        if (g->IntersectSphere(p, 0.5)) {
          image(xi, zi, 0) = 0;
          break;
        }
      }
    }
  }
  /*
  for (const auto& pup : power_ups_) {
    nacb::Vec3d p = pup->pos() - bounds_.min;
    p.x *= (image.w / size.x);
    p.z *= (image.h / size.z);
    for (int zi = -resolution / 2; zi <= resolution / 2; ++zi) {
      for (int xi = -resolution / 2; xi <= resolution / 2; ++xi) {
        image((int)p.x + xi, (int)p.z + zi, 0) = 0;
        image((int)p.x + xi, (int)p.z + zi, 1) = 0;
        image((int)p.x + xi, (int)p.z + zi, 2) = 1.0;
      }
    }
  }

  for (const auto& agent : agents_) {
    nacb::Vec3d p = agent->pos() - bounds_.min;
    p.x *= (image.w / size.x);
    p.z *= (image.h / size.z);
    for (int zi = -resolution / 2; zi <= resolution / 2; ++zi) {
      for (int xi = -resolution / 2; xi <= resolution / 2; ++xi) {
        image((int)p.x + xi, (int)p.z + zi, 0) = 0;
        image((int)p.x + xi, (int)p.z + zi, 1) = 1.0;
        image((int)p.x + xi, (int)p.z + zi, 2) = 0.0;
      }
    }
    }*/
  map_ = image;
  map_.save("/tmp/map.png");
  image.boxFilter(1).boxFilter(1).boxFilter(1).save("/tmp/c.png");
  (map_ > 0).edt_8sed(0).save("/tmp/edt.png");
}
