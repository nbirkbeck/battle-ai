#ifndef _SIMPLE_AGENT_H_
#define _SIMPLE_AGENT_H_ 1

#include "src/agent/agent.h"
#include "src/observable_state.h"
#include "src/proto/simple_agent.pb.h"
#include <deque>

class Plan {
public:
  Plan(const std::deque<nacb::Vec3d>& points = {}) : waypoints_(points) {}

  bool IsPathObstructedByPoints(const std::vector<nacb::Vec3d>& other_points,
                                double limit) const;

  bool AlmostFinished(const nacb::Vec3d& pos) const {
    return waypoints_.size() == 1 && (waypoints_.back() - pos).len() < 0.1;
  }

  const nacb::Vec3d GetFirstWaypoint() { return waypoints_.front(); }
  bool HasWaypoints() const { return waypoints_.size(); }
  std::deque<nacb::Vec3d> waypoints_;
};

class PlanFollowingAgent : public Agent {
public:
  PlanFollowingAgent(const nacb::Vec3d& pos, const nacb::Quaternion& quat,
                     const std::vector<Weapon> weapons)
      : Agent(pos, quat, std::move(weapons)) {}
  virtual ~PlanFollowingAgent() {}

  virtual void GetActions(const ObservableState& state,
                          battle::Actions* actions) override;

protected:
  std::unique_ptr<Plan> plan_;
  double shoot_confidence_ = 0;
  double shoot_rate_ = 0.5;
  double accuracy_ = 0.5;
};

class SimpleAgent : public PlanFollowingAgent {
public:
  SimpleAgent(const nacb::Vec3d& pos = {}, const nacb::Quaternion& quat = {},
              const std::vector<Weapon> weapons = {Weapon()})
      : PlanFollowingAgent(pos, quat, std::move(weapons)) {}

  void GetActions(const ObservableState& state,
                  battle::Actions* actions) override;
  void SetParams(const battle::SimpleAgentParams& params) {
    params_ = params;
    PlanFollowingAgent::shoot_confidence_ = params.shoot_confidence();
    PlanFollowingAgent::shoot_rate_ = params.shoot_rate();
    PlanFollowingAgent::accuracy_ = params.accuracy();
  }

protected:
  battle::SimpleAgentParams params_;
};

#endif // _SIMPLE_AGENT_H_
