#ifndef _PLANE_H_
#define _PLANE_H_ 1

#include <nmath/vec3.h>

struct Plane {
  const nacb::Vec3d n;
  double o;

  bool IntersectRay(const nacb::Vec3d& p, const nacb::Vec3d& d,
                    double* t) const {
    const double d_dot_n = d.dot(n);
    if (fabs(d_dot_n) < 1e-10 * d.len()) {
      return false;
    }
    // (p + t * d) * n = -o
    *t = (-o - p.dot(n)) / d_dot_n;
    return true;
  }
};

#endif // _PLANE_H_
