#include "simple_agent.h"
#include <nmath/quaternion.h>

#include "src/agent/math.h"
#include "src/agent/search.h"

std::deque<nacb::Vec3d> SmoothPoints(std::deque<nacb::Vec3d> points) {
  std::deque<nacb::Vec3d> output_points;
  if (!points.size()) return points;
  for (int iter = 0; iter < 2; iter++) {
    for (int i = 0; i < points.size(); ++i) {
      output_points.push_back(points[i]);
      if (i + 1 < points.size()) {
        output_points.push_back(points[i] * 0.5 + points[i + 1] * 0.5);
      }
    }
    points = output_points;
  }
  for (int iter = 0; iter < 2; ++iter) {
    points = output_points;
    for (int i = 1; i < (int)output_points.size() - 1; ++i) {
      output_points[i] = (points[i - 1] + 2 * points[i] + points[i + 1]) * (1.0 / 4.0);
    }
  }
  return output_points;
}

bool Plan::IsPathObstructedByPoints(const std::vector<nacb::Vec3d>& other_points,
                                    double limit) const {
  if (waypoints_.empty()) return false;
  nacb::Vec3d last_waypoint = waypoints_.front();
  double len = 0;
  for (const auto& waypoint : waypoints_) {
    len += (waypoint - last_waypoint).len();
    if (len > limit) break;
    for (const auto& point : other_points) {
      if ((waypoint - point).len() < 0.5) {
        return true;
      }
    }
    last_waypoint = waypoint;
  }
  return false;
}

void PlanFollowingAgent::GetActions(const ObservableState& state,
                                    battle::Actions* actions) {
  if (!plan_) return;

  if (plan_->HasWaypoints()) {
    nacb::Vec3d waypoint = plan_->GetFirstWaypoint();
    if ((waypoint - pos()).len() < 0.1) {
      plan_->waypoints_.pop_front();
      if (plan_->HasWaypoints()) {
        waypoint = plan_->GetFirstWaypoint();
      }
    }
    auto* move = actions->mutable_move();
    SetVec3(move->mutable_target(), waypoint);

    nacb::Vec3d dir = waypoint - pos();
    const double angle = atan2(dir.x, dir.z);
    auto* rot = actions->mutable_rotate();
    const nacb::Quaternion q = nacb::Quaternion::rod(nacb::Vec3d(0, angle, 0));
    SetVec3(rot->mutable_v(), q.v);
    rot->set_a(q.a);
  } else {
    plan_.reset(nullptr);
  }

  if (state.visible_agents.size() > 0) {
    for (const auto& agent : state.visible_agents) {
      // Only shoot if another agent is visible.
      if (agent.confidence < shoot_confidence_) continue;
      const nacb::Vec3d dir = agent.pos - pos();
      const double angle = atan2(dir.x, dir.z);
      auto* rot = actions->mutable_rotate();

      const double kWorstAccuracyDegrees = 50.0;
      const double accuracy_degrees = kWorstAccuracyDegrees * (1.0 - accuracy_);
      const double accuracy_radians = accuracy_degrees * M_PI / 180.0;
      const double r_u = urand(-accuracy_radians, accuracy_radians);
      const double r_v = urand(-accuracy_radians, accuracy_radians);
      const nacb::Quaternion q = nacb::Quaternion::rod(nacb::Vec3d(0, angle, 0)) *
        nacb::Quaternion::rod(nacb::Vec3d(r_u, 0, 0)) *
        nacb::Quaternion::rod(nacb::Vec3d(0, 0, r_v));

      SetVec3(rot->mutable_v(), q.v);
      rot->set_a(q.a);

      if (urand() < shoot_rate_) {
        actions->mutable_shoot()->set_shoot(true);
      }
    }
  }
}

void SimpleAgent::GetActions(const ObservableState& state,
                             battle::Actions* actions) {
  std::vector<nacb::Vec3d> other_points;
  for (const auto& other_agents : state.visible_agents) {
    other_points.push_back(other_agents.pos);
  }
  const bool path_obstructed = plan_ &&
    plan_->IsPathObstructedByPoints(other_points, params_.replan_obstruction_distance());
  if (path_obstructed) {
    VLOG(3) << "Path obstructed";
  }
  const bool should_replan = urand() < params_.replan_rate();
  if (!plan_ || should_replan || path_obstructed) {
    // Look for an interesting spot to go to (e.g., a power-up) or
    // other region not explored for a while. And plan to get there.
    // Make up some priority.
    // What events require a replan? Change of combat mode?
    
    const double health_deficit = max_health() - health();
    const double armor_deficit = max_armor() - armor();
    std::deque<nacb::Vec3d> good_points;
    double good_score = 1e10;
    double good_dist = 0;
    if (urand() < params_.favor_powerups()) {
      for (const auto& pup : state.power_ups) {
        if (armor_deficit == 0 && pup->type() == state::PowerUp::POWER_UP_ARMOR) continue;
        if (health_deficit == 0 && pup->type() == state::PowerUp::POWER_UP_HEALTH) continue;
        
        std::deque<nacb::Vec3d> points;
        double pup_distance = Search(state.access_map, other_points, pos(), pup->pos(), &points);
        if (pup_distance < 1 && pup->time_until_respawn() > 20) continue;
        double score = pup_distance + pup->time_until_respawn() + 5 * double(rand()) / RAND_MAX;
        if (score < good_score) {
          good_points = points;
          good_score = score;
          good_dist = pup_distance;
        }
      }
    }
    if ((good_score > 5 && good_dist <= 1)) {
      const AccessibilityMap* access_map = state.access_map;
      std::vector<std::pair<double, nacb::Vec3d> > candidates;
      if (state.visible_agents.size() > 0 && params_.aggressiveness() > 0) {
        for (const auto& agent : state.visible_agents) {
          nacb::Vec2<int> pt = access_map->WorldToImage(agent.pos);
          int invalid_zone = 4; // * (1.0 - params_.aggressiveness());
          int search_zone = 6; //invalid_zone + 2;
          for (int x = -search_zone; x <= search_zone; ++x) {
            for (int y = -search_zone; y <= search_zone; ++y) {
              nacb::Vec2<int> c(pt.x + x, pt.y + y);
              if (c.len() <= invalid_zone) {
                continue;
              }
              if (c.x < 0 || c.y < 0 || c.x >= access_map->map().w || c.y >= access_map->map().h) {
                continue;
              }
              if (access_map->map()(c.x, c.y)) {
                auto pt = access_map->ImageToWorld(nacb::Vec2<int>(x, y));
                auto dir = pos() - agent.pos;
                dir = dir.normalize();
                candidates.push_back(std::make_pair((pt - agent.pos).dot(dir) + (rand() % 2), pt));
              }
            }
          }
          if (rand() % state.visible_agents.size() == 0) break;
        }
      } else {
        for (int x = 0; x < access_map->map().w; ++x) {
          for (int y = 0; y < access_map->map().h; ++y) {
            if (access_map->map()(x, y)) {
              auto pt = access_map->ImageToWorld(nacb::Vec2<int>(x, y));
              candidates.push_back(std::make_pair((pt - pos()).len() + urand(-3, 3), pt));
            }
          }
        }
      }
      good_points.clear();
        
      std::sort(candidates.begin(), candidates.end(), [](const std::pair<double, nacb::Vec3d>& p1,
                                                         const std::pair<double, nacb::Vec3d>& p2) {
                                                        return p1.first > p2.first;
                                                      });
      VLOG(3) << candidates.front().first << " " << candidates.back().first;
      Search(state.access_map, other_points, pos(), candidates[0].second, &good_points);
    }
    plan_.reset(new Plan);
    plan_->waypoints_ = SmoothPoints(good_points);
  }
  return PlanFollowingAgent::GetActions(state, actions);
}
