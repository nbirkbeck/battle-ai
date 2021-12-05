#include "src/agent/high_level_agent.h"

#include "src/agent/search.h"

std::vector<double> HighLevelAgent::Observe(const World* world) {
  const ObservableState state = world->GetObservableStateForAgent(this);

  std::vector<nacb::Vec3d> other_points;
  for (const auto& other_agents : state.visible_agents) {
    other_points.push_back(other_agents.pos);
  }

  nacb::Vec3d other_agent_pos = last_opponent_pos_;
  if (!other_points.empty()) {
    other_agent_pos = other_points.front();
  }

  // Basic state: position, health, armor
  std::vector<double> features;
  features.push_back(pos2d()[0] / 20.0);
  features.push_back(pos2d()[1] / 20.0);
  features.push_back(health() / max_health() * 2 - 1);
  features.push_back(armor() / max_armor() * 2 - 1);

  // Distance to power-ups (and opponent distance to power ups)
  for (const auto& pup: state.power_ups) {
    std::deque<nacb::Vec3d> points;
    const double pup_distance = Search(state.access_map, other_points, pos(), pup->pos(), &points);
    features.push_back(NormalizeDistance(pup_distance));
    features.push_back(pup->time_until_respawn() / pup->respawn_delay());
    features.push_back(NormalizeDistance(Search(state.access_map, {}, other_agent_pos, pup->pos(), &points)));
  }

  // Distance to opponent (if known)
  std::deque<nacb::Vec3d> points;
  features.push_back(NormalizeDistance(Search(state.access_map, {}, pos(), other_agent_pos, &points)));
  features.push_back(state.visible_agents.size());

  // Sense distance in each possible move direction.
  std::vector<const Agent*> agents = {}; // Don't care about proximity to agents.
  for (int i = 0; i < 4; ++i) {
    nacb::Vec3d dir = GetDirection(i);
    double t = 0;
    Agent* agent = nullptr;
    if (LineHitsAnything(pos(), pos() + dir, world->geoms(), agents, &t, &agent) && t >= 0 && t <= 1) {
      features.push_back(t);
    } else {
      features.push_back(1);
    }
  }

  // Distance to plan
  if (plan_) {
    features.push_back(NormalizeDistance((plan_->waypoints_.back() - pos()).len()));
  } else {
    features.push_back(-1);
  }
  // Last action
  features.push_back(last_action_);

  last_opponent_pos_ = other_agent_pos;
  num_power_ups_ = state.power_ups.size();
  return features;
}

void HighLevelAgent::Reset() {
  last_opponent_pos_ = nacb::Vec3d(0, 0, 0);
  current_action_ = last_action_ = 0;
  Agent::Reset();
  prev_ = pos();
}

nacb::Vec3d HighLevelAgent::GetDirection(int dir) const {
  const nacb::Vec3d forward = quat().rotate(nacb::Vec3d(0, 0, 1));
  switch (dir) {
  case FORWARD:
    return forward;
  case LEFT:
    return nacb::Quaternion::rod(nacb::Vec3d(0, M_PI / 2.0, 0)).rotate(forward);
  case RIGHT:
    return nacb::Quaternion::rod(nacb::Vec3d(0, -M_PI / 2.0, 0)).rotate(forward);
  case BACK:
    return forward * -1;
  }
  assert(false);
  return nacb::Vec3d();
}

void HighLevelAgent::GetActions(const ObservableState& state,
                                battle::Actions* actions) {
  prev_ = pos();
  std::vector<nacb::Vec3d> other_points;
  for (const auto& other_agents : state.visible_agents) {
    other_points.push_back(other_agents.pos);
  }
  // 0-3: Forward, left, right, backward
  if (current_action_ < 4) {
    if (!plan_ || plan_->AlmostFinished(pos()) || last_action_ != current_action_) {
      nacb::Vec3d new_pos = pos() + GetDirection(current_action_);
      nacb::Vec2<int> pos_i = state.access_map->WorldToImage(pos());
      nacb::Vec2<int> new_pos_i = state.access_map->WorldToImage(new_pos);
      const int search_range = (int)((pos_i - new_pos_i).len() + 1);
      nacb::Vec2<int> best_pos_i = new_pos_i;
      double best_dist = 1e10;
      for (int x = -search_range; x <= search_range; ++x) {
        for (int z = -search_range; z <= search_range; ++z) {
          nacb::Vec2d cand_i = new_pos_i + nacb::Vec2<int>(x, z);
          if (cand_i.x < 0 || cand_i.y < 0 ||
              cand_i.x >= state.access_map->map().w ||
              cand_i.y >= state.access_map->map().h) continue;
          if (!state.access_map->map()(cand_i.x, cand_i.y)) continue;
          const double dist = (cand_i - new_pos_i).len();
          if (dist < best_dist) {
            best_dist = dist;
            best_pos_i = cand_i;
          }
        }
      }
      // TODO: detect if we are stuck.
      new_pos = state.access_map->ImageToWorld(best_pos_i);
      const bool attack = current_action_ == FORWARD && !other_points.empty();
      // When attacking, move on path towards agent.
      if (attack) {
        new_pos = other_points.front();
      }
      std::deque<nacb::Vec3d> points;
      Search(state.access_map, other_points, pos(), new_pos, &points);

      // When attacking, don't plan too far ahead.
      if (attack) {
        for (int i = 0; i < (int)points.size(); ++i) {
          if ((points[i] - pos()).len() > 2.0) {
            LOG(INFO) << "Dropping points after " << i
                      << " (size = " << points.size() << ")";
            points.erase(points.begin() + i, points.end());
            break;
          }
        }
      }
      plan_.reset(new Plan(points));
    }
  } else {
    const int pup_index = current_action_ - 4;
    assert(pup_index < state.power_ups_size());
    if (!plan_ || last_action_ != current_action_) {
      const auto& pup = state.power_ups[pup_index];
      std::deque<nacb::Vec3d> points;
      Search(state.access_map, other_points, pos(), pup->pos(), &points);
      plan_.reset(new Plan(points));
    }
  }
  last_action_ = current_action_;

  PlanFollowingAgent::GetActions(state, actions);
}    
