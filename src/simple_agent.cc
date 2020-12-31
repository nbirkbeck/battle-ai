#include "simple_agent.h"
#include <nmisc/heap.h>
#include <nmath/quaternion.h>

#include "src/math.h"

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

double Search(const AccessibilityMap* access_map,
              const std::vector<nacb::Vec3d>& other_points,
              const nacb::Vec3d& start,
              const nacb::Vec3d& target,
              std::deque<nacb::Vec3d>* points) {
  const nacb::Vec2<int> start_i = access_map->WorldToImage(start);
  const nacb::Vec2<int> target_i = access_map->WorldToImage(target);

  Heap heap(access_map->map().w * access_map->map().h);
  auto HeapIndex = [&](const nacb::Vec2<int>& p) {
    return p.y * access_map->map().w + p.x;
  };
  auto IndexToPoint = [&](int i) {
                       return nacb::Vec2<int>(i % access_map->map().w,
                                               i / access_map->map().w);
  };
  auto Heuristic = [&](const nacb::Vec2<int>& a, const nacb::Vec2<int>& b) {
    return (a - b).len();
  };
  heap.insert(HeapIndex(start_i), Heuristic(start_i, target_i));
  std::unordered_map<int, nacb::Vec2<int> > prev;
  std::unordered_map<int, bool> visited;
  std::unordered_map<int, bool> others;
  for (const auto& o : other_points) {
    if (HeapIndex(access_map->WorldToImage(o)) != HeapIndex(start_i)) {
      for (int x = -1; x <= 1; ++x) {
        for (int z = -1; z <= 1; ++z) {
          auto point = access_map->WorldToImage(o) + nacb::Vec2<int>(x, z);
          others[HeapIndex(point)] = true;
        }
      }
    }
  }

  double total_distance = -1;
  int explored = 0;
  while (heap.getSize() > 0) {
    double f = 0;
    int top = heap.remove(&f);
    visited[top] = 1;
    explored++;
    const nacb::Vec2<int> p = IndexToPoint(top);
    const nacb::Vec2<int> kNeigh[8] = {
                       {-1, -1},
                       { 0, -1},
                       { 1, -1},
                       {-1, 0},
                       { 1, 0},
                       {-1, 1},
                       { 0, 1},
                       { 1, 1},
    };
    const double h = Heuristic(p, target_i);
    const double g = f - h;
    if (h == 0) {
      VLOG(3) << "Found goal, explored = " << explored;
      total_distance = g;
      break;
    }
    
    for (int ni = 0; ni < 8; ++ni) {
      const nacb::Vec2<int> n = p + kNeigh[ni];
      if (n.x < 0 || n.y < 0 || n.x >= access_map->map().w || n.y >= access_map->map().h) continue;
      const double n_g = g + kNeigh[ni].len();
      const double n_f = n_g + Heuristic(n, target_i);
      const int n_i = HeapIndex(n);
      if (visited[n_i]) continue;
      if (others[n_i]) continue;
      if (!access_map->map()(n.x, n.y)) continue;
      
      if (heap.exists(n_i)) {
        const double old_f = heap.get(n_i);
        if (n_f < old_f) {
          heap.update(n_i, n_f);
          prev[n_i] = p;
        }
      } else {
        heap.insert(n_i, n_f);
        prev[n_i] = p;
      }
    }
  }
  points->clear();
  if (total_distance >= 0) {
    const int first_index = HeapIndex(start_i);
    int cur_index = HeapIndex(target_i);
    while (cur_index != first_index) {
      auto cur_point = IndexToPoint(cur_index);

      points->push_front(access_map->ImageToWorld(cur_point));
      if (!prev.count(cur_index)) {
        LOG(INFO) << "Missing cur_index = " << cur_index << " in prev";
        return -1;
      }
      points->front().y = 1;
      cur_index = HeapIndex(prev[cur_index]);
    }
  }
  return total_distance;
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
    LOG(INFO) << "Path obstructed";
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
    plan_->waypoints_ = good_points;
  }
  if (!plan_) return;

  if (plan_->HasWaypoints()) {
    nacb::Vec3d waypoint = plan_->GetFirstWaypoint();
    if ((waypoint - pos()).len() < 0.1) {
      plan_->waypoints_.pop_front();
      waypoint = plan_->GetFirstWaypoint();
    }
    auto* move = actions->mutable_move();
    auto* target = move->mutable_target();
    target->set_x(waypoint.x);
    target->set_y(waypoint.y);
    target->set_z(waypoint.z);

    nacb::Vec3d dir = waypoint - pos();
    const double angle = atan2(dir.x, dir.z);
    auto* rot = actions->mutable_rotate();
    nacb::Quaternion q = nacb::Quaternion::rod(nacb::Vec3d(0, angle, 0));
    rot->mutable_v()->set_x(q.v.x);
    rot->mutable_v()->set_y(q.v.y);
    rot->mutable_v()->set_z(q.v.z);
    rot->set_a(q.a);
  } else {
    plan_.reset(nullptr);
  }

  if (state.visible_agents.size() > 0) {
    for (const auto& agent : state.visible_agents) {
      if (agent.confidence < params_.shoot_confidence()) continue;
      const nacb::Vec3d dir = agent.pos - pos();
      const double angle = atan2(dir.x, dir.z);
      auto* rot = actions->mutable_rotate();

      const double kWorstAccuracyDegrees = 50.0;
      const double accuracy_degrees = kWorstAccuracyDegrees * (1.0 - params_.accuracy());
      const double accuracy_radians = accuracy_degrees * M_PI / 180.0;
      const double r_u = urand(-accuracy_radians, accuracy_radians);
      const double r_v = urand(-accuracy_radians, accuracy_radians);
      const nacb::Quaternion q = nacb::Quaternion::rod(nacb::Vec3d(0, angle, 0)) *
        nacb::Quaternion::rod(nacb::Vec3d(r_u, 0, 0)) *
        nacb::Quaternion::rod(nacb::Vec3d(0, 0, r_v));

      rot->mutable_v()->set_x(q.v.x);
      rot->mutable_v()->set_y(q.v.y);
      rot->mutable_v()->set_z(q.v.z);
      rot->set_a(q.a);

      // Only shoot if another agent is visible.
      if (urand() < params_.shoot_rate()) {
        actions->mutable_shoot()->set_shoot(true);
      }
    }
  }
}
