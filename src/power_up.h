#ifndef _POWER_UP_H_
#define _POWER_UP_H_ 1

#include <memory>
#include <nmath/vec3.h>

#include "src/geometry/bounding_cylinder.h"
#include "src/proto/level.pb.h"

class Agent;

class PowerUp {
 public:
  PowerUp(const nacb::Vec3d pos,
          const state::PowerUp::Type& type,
          double amount, double respawn_delay = 30) :
    pos_(pos), type_(type), amount_(amount),
    respawn_delay_(respawn_delay), time_until_respawn_(0)  {}

  void Give(Agent* agent);

  void Reset() {
    time_until_respawn_ = 0;
  }

  void Step(double dt) {
    time_until_respawn_ = std::max(0.0, time_until_respawn_ - dt);
  }

  bool IsActive() const {
    return time_until_respawn_ <= 0.0;
  }

  const BoundingCylinder GetBoundingGeometry() const {
    const double kDefaultRadius = 0.5;
    const double kDefaultHeight = 1.0;
    return BoundingCylinder(pos_, kDefaultRadius, kDefaultHeight);
  }

  const nacb::Vec3d& pos() const {
    return pos_;
  }

  double time_until_respawn() const {
    return time_until_respawn_;
  }

  state::PowerUp::Type type() const {
    return type_;
  }

  double amount() const {
    return amount_;
  }

  double respawn_delay() const {
    return respawn_delay_;
  }

 protected:
  nacb::Vec3d pos_;
  state::PowerUp::Type type_;
  double amount_;
  double respawn_delay_;
  double time_until_respawn_;
};


#endif  // _POWER_UP_H_
