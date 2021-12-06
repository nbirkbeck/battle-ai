#ifndef _ACCESSIBILITY_MAP_H_
#define _ACCESSIBILITY_MAP_H_

#include <memory>
#include <nimage/image.h>

#include "src/geometry/axis_aligned_box.h"
#include <nmath/vec2.h>
#include <nmath/vec3.h>

typedef std::vector<std::unique_ptr<AxisAlignedBox>> GeometryVector;

class AccessibilityMap {
public:
  AccessibilityMap(const AxisAlignedBounds& bounds, const GeometryVector& geoms,
                   int resolution);
  nacb::Vec2<int> WorldToImage(const nacb::Vec3d& pos) const {
    const nacb::Vec3d size = bounds_.max - bounds_.min;
    return nacb::Vec2<int>((int)(map_.w * (pos.x - bounds_.min.x) / size.x),
                           (int)(map_.h * (pos.z - bounds_.min.z) / size.z));
  }
  nacb::Vec3d ImageToWorld(const nacb::Vec2<int>& p) const {
    const nacb::Vec3d size = bounds_.max - bounds_.min;
    // TODO:: fix y
    return nacb::Vec3d(size.x * (p.x + 0.5) / map_.w, 0.5,
                       size.z * (p.y + 0.5) / map_.h) +
           bounds_.min;
  }
  const nacb::Image8& map() const { return map_; }
  AxisAlignedBounds bounds_;
  nacb::Image8 map_;
  int resolution_;
};

#endif
