#ifndef HIGH_LEVEL_AGENT
#define HIGH_LEVEL_AGENT

#include <vector>
#include <nmath/vec3.h>
#include "src/world.h"
#include "src/agent/agent.h"
#include "src/agent/simple_agent.h"

// This an implementation of an agent that allows for high-level decision making.
// E.g,. are we moving towards a specific power-up, or attacking opponent?
//
// The basic state consists of:
//  position, health, armor
//  distance of self to power-up.
//  power-up respawn delay
//  distance of opponent to power-up
//  distance to objects in each possible move direction.
//  distance to plan-endpoint
//  last action.
//
// Actions are:
//  Move in one of 4 directions
//  Plan to move to a power-up.
//
// This implementation is meant to be called from python. The caller should
// set up the desired action before world->Step is called.
class HighLevelAgent : public PlanFollowingAgent {
public:
  enum Direction {
     FORWARD = 0,
     LEFT = 1,
     BACK = 2,
     RIGHT = 3
  };

  HighLevelAgent():
    PlanFollowingAgent(nacb::Vec3d(), nacb::Quaternion(), {Weapon()}) {}

  std::vector<double> Observe(const World* world);

  void Reset() override;

  nacb::Vec3d GetDirection(int dir) const;

  void GetActions(const ObservableState& state,
                  battle::Actions* actions);

  double NormalizeDistance(double d) const {
    // TODO: should be based on world bounds.
    return d / 20.0 - 1.0;
  }

  // Helper function for python bindindgs
  std::vector<double> pos2d() const {
    std::vector<double> p(2);
    p[0] = pos().x;
    p[1] = pos().z;
    return p;
  }

  void SetAction(int action_index) {
    current_action_ = action_index;
  }

  double MovedDistance() const { return (pos() - prev_).len(); }
  int last_action() const { return last_action_; }

private:
  nacb::Vec3d prev_;
  nacb::Vec3d last_opponent_pos_;
  int num_power_ups_;
  int current_action_ = 0;
  int last_action_ = 0;
};

#endif
