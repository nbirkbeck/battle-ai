#ifndef _AXIS_ALIGNED_BOX_H_
#define _AXIS_ALIGNED_BOX_H_ 1

#include "plane.h"
#include <glog/logging.h>
#include <nmath/vec3.h>

class AxisAlignedBox {
 public:
  AxisAlignedBox(const nacb::Vec3d& p,
                 const nacb::Vec3d& s): pos(p), size(s) {}

  bool IntersectSphere(const nacb::Vec3d& p, double r) const {
    return
      (p.x >= pos.x - size.x / 2 - r) &&
      (p.y >= pos.y - size.y / 2 - r) &&
      (p.z >= pos.z - size.z / 2 - r) &&
      (p.x <= pos.x + size.x / 2 + r) &&
      (p.y <= pos.y + size.y / 2 + r) &&
      (p.z <= pos.z + size.z / 2 + r);
      
  }

  bool IntersectRay(const nacb::Vec3d& p, const nacb::Vec3d& d,
                    double* t, double r = 0,
                    nacb::Vec3d* deflected = 0) const {
    Plane planes[4] = {
                       {nacb::Vec3d(1, 0, 0), -(pos.x + size.x / 2 + r)},
                       {nacb::Vec3d(-1, 0, 0), (pos.x - size.x / 2 - r)},
                       {nacb::Vec3d(0, 0, 1), -(pos.z + size.z / 2 + r)},
                       {nacb::Vec3d(0, 0, -1), (pos.z - size.z / 2 - r)},
    };

    *t = 1e10;
    for (int plane_index = 0; plane_index < 4; ++plane_index) {
      const Plane& plane = planes[plane_index];
      double t1 = 1e10;
      if (d.dot(plane.n) >= 0) continue;
      if (plane.IntersectRay(p, d, &t1) && t1 >= -r/4 && t1 < *t) {
        const nacb::Vec3d x = p + d * t1;
        // TODO: y?
        const int check = plane_index <= 1 ? 2 : 0;
        if (x.data[check] >= pos.data[check] - size.data[check] / 2 - r &&
            x.data[check] <= pos.data[check] + size.data[check] / 2 + r) {
          nacb::Vec3d x_full = x + d;
          if (deflected) {
            nacb::Vec3d d2 = x_full - x;
            double rem = d2.dot(plane.n);
            d2 -= rem * plane.n;
            *deflected = x + d2;
          }
          *t = t1 - 1e-6; // Was 1e-3
        }
      }
    }
    return *t >= -r/4 && *t < 1e10;
  }
  nacb::Vec3d min() const {
    return pos - size * 0.5;
  }
  nacb::Vec3d max() const {
    return pos + size * 0.5;
  }
  nacb::Vec3d pos;
  nacb::Vec3d size;
};

struct AxisAlignedBounds {
  nacb::Vec3d min;
  nacb::Vec3d max;
};


#endif  // _AXIS_ALIGNED_BOX_H_
