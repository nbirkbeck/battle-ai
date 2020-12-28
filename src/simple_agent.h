#ifndef _SIMPLE_AGENT_H_
#define _SIMPLE_AGENT_H_ 1

#include <deque>
#include "agent.h"
#include "observable_state.h"

class Plan {
 public:

  const nacb::Vec3d GetFirstWaypoint() {
    return waypoints_.front();
  }

  bool HasWaypoints() const {
    return waypoints_.size();
  }
  std::deque<nacb::Vec3d> waypoints_;
};

class SimpleAgent : public Agent {
 public:
  SimpleAgent(const nacb::Vec3d& pos,
              const nacb::Quaternion& quat,
              const std::vector<Weapon> weapons) :
    Agent(pos, quat, std::move(weapons)) {}

  void GetActions(const ObservableState& state,
                  battle::Actions* actions) override;
 protected:
  std::unique_ptr<Plan> plan_;
};

#endif  // _SIMPLE_AGENT_H_
