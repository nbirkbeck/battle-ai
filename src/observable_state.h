#ifndef _OBSERVABLE_STATE_H_
#define _OBSERVABLE_STATE_H_

#include <nimage/image.h>

#include "power_up.h"
#include "axis_aligned_box.h"
#include "weapon.h"

typedef std::vector<std::unique_ptr<AxisAlignedBox> > GeometryVector;
typedef std::vector<Projectile> ProjectileVector;

class AccessibilityMap {
public:
  AccessibilityMap(const AxisAlignedBounds& bounds,
                   const GeometryVector& geoms,
                   int resolution);
  nacb::Vec2<int> WorldToImage(const nacb::Vec3d& pos) const {
    const nacb::Vec3d size = bounds_.max - bounds_.min;
    return nacb::Vec2<int>(
                           (int)(map_.w * (pos.x - bounds_.min.x) / size.x),
                           (int)(map_.h * (pos.z - bounds_.min.z) / size.z));
  }
  nacb::Vec3d ImageToWorld(const nacb::Vec2<int>& p) const {
    const nacb::Vec3d size = bounds_.max - bounds_.min;
    // TODO:: fix y
    return nacb::Vec3d(size.x * (p.x + 0.5) / map_.w, 0.5,
                       size.z * (p.y + 0.5) / map_.h) + bounds_.min;
  }
  const nacb::Image8& map() const {
    return map_;
  }
  AxisAlignedBounds bounds_;
  nacb::Image8 map_;
  int resolution_;
};

struct ObservableState {
  const std::vector<std::unique_ptr<PowerUp> >& power_ups;
  const AccessibilityMap* access_map;
};

#endif
