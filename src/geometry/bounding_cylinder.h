#ifndef BOUNDING_CYLINDER_H
#define BOUNDING_CYLINDER_H 1

#include <nmath/vec3.h>

class BoundingCylinder {
public:
  BoundingCylinder(const nacb::Vec3d& pos, double r, const double h)
      : pos_(pos), r_(r), h_(h) {}
  bool Intersects(const BoundingCylinder& other) const {
    const auto diff = nacb::Vec3d(pos_.x, 0, pos_.z) -
                      nacb::Vec3d(other.pos_.x, 0, other.pos_.z);
    const double l = diff.len();
    double dh = pos_.y - other.pos_.y;
    return l < (r_ + other.r_) && (sqrt(dh * dh) < h_ + other.h_);
  }

  nacb::Vec3d min() const { return pos_ - nacb::Vec3d(r_, h_ / 2, r_); }
  nacb::Vec3d max() const { return pos_ + nacb::Vec3d(r_, h_ / 2, r_); }

protected:
  const nacb::Vec3d pos_;
  const double r_;
  const double h_;
};

#endif
