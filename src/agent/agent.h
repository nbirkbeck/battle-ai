#ifndef _AGENT_H_
#define _AGENT_H_

#include <algorithm>
#include <deque>
#include <memory>
#include <vector>

#include <nmath/matrix.h>
#include <nmath/quaternion.h>
#include <nmath/vec3.h>

#include "src/geometry/axis_aligned_box.h"
#include "src/geometry/bounding_cylinder.h"
#include "src/proto.h"
#include "src/proto/actions.pb.h"
#include "src/weapon.h"

class ObservableState;

extern std::deque<nacb::Vec3d> g_def;

class Agent {
public:
  static constexpr double kDefaultArmor = 0;
  static constexpr double kDefaultHealth = 100;
  static constexpr double kDefaultMaxHealth = 200;
  static constexpr double kDefaultMaxArmor = 200;

  // static constexpr nacb::Vec3d kDefaultSize = nacb::Vec3d(1, 2, 1);

  Agent(const nacb::Vec3d& pos, const nacb::Quaternion& quat,
        const std::vector<Weapon> weapons, double health = kDefaultHealth,
        double armor = kDefaultArmor)
      : pos_(pos), quat_(quat), size_(1, 2, 1), weapons_(std::move(weapons)),
        health_(health), armor_(armor), active_weapon_(0),
        max_armor_(kDefaultMaxArmor), max_health_(kDefaultMaxHealth) {
    assert(active_weapon_ >= 0);
    assert(active_weapon_ < (int)weapons_.size());
  }

  virtual ~Agent() {}

  virtual void Reset() {
    health_ = kDefaultHealth;
    armor_ = kDefaultArmor;
  }
  virtual void GetActions(const ObservableState& observable_state,
                          battle::Actions* actions) = 0;

  // Updates the client state as a result of the actions it has taken.
  void Update(const battle::ActionResponse& response) {
    for (const auto& result : response.results()) {
      if (result.has_move()) {
        last_pos_ = pos_;
        pos_ = CreateVec3d(result.move().pos());
      }
      if (result.has_rotate()) {
        quat_.v = CreateVec3d(result.rotate().v());
        quat_.a = result.rotate().a();
      }
    }
  }

  bool IsActive() const { return health() >= 0; }

  void AddArmor(double value) { armor_ = std::min(max_armor_, armor_ + value); }

  void AddHealth(double value) {
    health_ = std::min(max_health_, health_ + value);
  }

  void DoDamage(double d) {
    const double absorbed_by_armor = std::min(armor_, d / 2);
    d -= absorbed_by_armor;
    armor_ -= absorbed_by_armor;
    health_ -= d;
  }

  Weapon& current_weapon() {
    assert(active_weapon_ >= 0);
    assert(active_weapon_ < (int)weapons_.size());
    return weapons_[active_weapon_];
  }

  void GetWeaponRay(nacb::Vec3d* pos, nacb::Vec3d* dir) const {
    *pos = pos_;
    pos->y += size_.y / 4.0;
    *dir = quat_.rotate(nacb::Vec3d(0, 0, 1));
    *pos = *pos + (*dir) * GetCylinderRadius();
  }

  // Returns true if the ray intersects the agent.
  bool IntersectRay(const nacb::Vec3d& p, const nacb::Vec3d& d,
                    double* t) const {
    const nacb::Vec3d up = quat_.rotate(nacb::Vec3d(0, 1, 0));
    // const nacb::Vec3d cross = up.cross(d);
    //(pos_ + up * t1) - (p + d * t) = 0;
    nacb::Matrix A(3, 2);
    A(0, 0) = up.x;
    A(1, 0) = up.y;
    A(2, 0) = up.z;
    A(0, 1) = -d.x;
    A(1, 1) = -d.y;
    A(2, 1) = -d.z;
    nacb::Matrix b(3, 1);
    b[0] = (p.x - pos_.x);
    b[1] = (p.y - pos_.y);
    b[2] = (p.z - pos_.z);
    nacb::Matrix x = nacb::Matrix::linLeastSq(A, b);
    if (x[0] < -size_.y / 2 || x[0] > size_.y / 2)
      return false;
    nacb::Vec3d x0 = pos_ + up * x[0];
    nacb::Vec3d x1 = p + d * x[1];
    const double r = (x1 - x0).len();
    if (r > GetCylinderRadius())
      return false;
    *t = x[1]; // NOTE: This is only approximate!!!!
    return true;
  }

  const BoundingCylinder GetBoundingGeometry() const {
    return BoundingCylinder(pos(), GetCylinderRadius(), size_.y);
  }

  const double GetCylinderRadius() const {
    assert(size_.x == size_.z);
    return size_.x / 2;
  }

  void set_pos(const nacb::Vec3d& p) {
    last_pos_ = p;
    pos_ = p;
  }

  const nacb::Vec3d& pos() const { return pos_; }
  const nacb::Vec3d& last_pos() const { return last_pos_; }

  const nacb::Quaternion& quat() const { return quat_; }
  const nacb::Vec3d& size() const { return size_; }

  const double health() const { return health_; }
  const double armor() const { return armor_; }

  const double max_health() const { return max_health_; }
  const double max_armor() const { return max_armor_; }

protected:
  Agent(const Agent&) = delete;
  void operator=(const Agent&) = delete;

  nacb::Vec3d pos_;
  nacb::Vec3d last_pos_;
  nacb::Quaternion quat_;
  nacb::Vec3d size_;

  std::vector<Weapon> weapons_;
  double height_;
  double health_;
  double armor_;
  int active_weapon_;
  double max_armor_;
  double max_health_;
};

#endif // _AGENT_H_
