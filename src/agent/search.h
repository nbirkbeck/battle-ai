#ifndef _SEARCH_H_
#define _SEARCH_H_

#include "src/observable_state.h"
#include <deque>
#include <nmath/vec3.h>
#include <vector>

double Search(const AccessibilityMap* access_map,
              const std::vector<nacb::Vec3d>& other_points,
              const nacb::Vec3d& start, const nacb::Vec3d& target,
              std::deque<nacb::Vec3d>* points);

#endif // _SEARCH_H_
