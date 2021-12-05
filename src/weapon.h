#ifndef WEAPON_H
#define WEAPON_H 1

#include <cassert>

class Projectile {
public:
  Projectile(const nacb::Vec3d& p, const nacb::Vec3d& v, double mass,
             double damage)
      : p_(p), v_(v), mass_(mass), damage_(damage), is_active_(true) {}

  bool CanRemove(double t) const {
    return !IsActive() && (t - deactivated_at_) > 1;
  }
  void Kill(double t) {
    is_active_ = false;
    deactivated_at_ = t;
  }

  bool IsActive() const { return is_active_; }

  void MoveTo(const nacb::Vec3d& p) { p_ = p; }
  double damage() const { return damage_; }

  const nacb::Vec3d& p() const { return p_; }
  const nacb::Vec3d& v() const { return v_; }

public:
  nacb::Vec3d p_;
  nacb::Vec3d v_;
  double mass_;
  double damage_;
  bool is_active_;
  double deactivated_at_;
};

class Weapon {
public:
  Weapon(double damage = 5, double firing_rate = 5, double speed = 200,
         double ammo = 100)
      : damage_(damage), firing_rate_(firing_rate), speed_(speed),
        projectile_mass_(0), ammo_(ammo), last_fired_(-100) {}

  Weapon(const Weapon&) = default;
  // const Weapon& operator=(const Weapon&) = default;

  bool CanFire(double t) const {
    const double dt = t - last_fired_;
    return (ammo_ > 0) && (dt > 1.0 / firing_rate_);
  }

  Projectile Fire(double t, const nacb::Vec3d& pos, const nacb::Vec3d& dir) {
    assert(CanFire(t));
    last_fired_ = t;
    const nacb::Vec3d v = dir * speed_;
    return Projectile(pos, v, projectile_mass_, damage_);
  }

  const double ammo() const { return ammo_; }

private:
  const double damage_;
  const double firing_rate_;
  const double speed_;
  const double projectile_mass_;
  double ammo_;
  double last_fired_;
};

#endif // WEAPON_H
