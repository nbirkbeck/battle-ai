#ifndef _OBSERVABLE_STATE_H_
#define _OBSERVABLE_STATE_H_

#include <nimage/image.h>

#include "src/accessibility_map.h"
#include "src/geometry/axis_aligned_box.h"
#include "src/power_up.h"
#include "src/weapon.h"

typedef std::vector<Projectile> ProjectileVector;

struct ObservableState {
  struct Agent {
    double confidence;
    nacb::Vec3d pos;
    std::vector<std::pair<nacb::Vec3d, nacb::Vec3d>> lines;
  };
  const std::vector<std::unique_ptr<PowerUp>>& power_ups;
  const AccessibilityMap* access_map;
  std::vector<Agent> visible_agents;
};

#endif
