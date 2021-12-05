#include "search.h"

#include <nmisc/heap.h>

double Search(const AccessibilityMap* access_map,
              const std::vector<nacb::Vec3d>& other_points,
              const nacb::Vec3d& start, const nacb::Vec3d& target,
              std::deque<nacb::Vec3d>* points) {
  const nacb::Vec2<int> start_i = access_map->WorldToImage(start);
  const nacb::Vec2<int> target_i = access_map->WorldToImage(target);

  Heap heap(access_map->map().w * access_map->map().h);
  auto HeapIndex = [&](const nacb::Vec2<int>& p) {
    return p.y * access_map->map().w + p.x;
  };
  auto IndexToPoint = [&](int i) {
    return nacb::Vec2<int>(i % access_map->map().w, i / access_map->map().w);
  };
  auto Heuristic = [&](const nacb::Vec2<int>& a, const nacb::Vec2<int>& b) {
    return (a - b).len();
  };
  heap.insert(HeapIndex(start_i), Heuristic(start_i, target_i));
  std::unordered_map<int, nacb::Vec2<int>> prev;
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
        {-1, -1}, {0, -1}, {1, -1}, {-1, 0}, {1, 0}, {-1, 1}, {0, 1}, {1, 1},
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
      if (n.x < 0 || n.y < 0 || n.x >= access_map->map().w ||
          n.y >= access_map->map().h)
        continue;
      const double n_g = g + kNeigh[ni].len();
      const double n_f = n_g + Heuristic(n, target_i);
      const int n_i = HeapIndex(n);
      if (visited[n_i])
        continue;
      if (others[n_i])
        continue;
      if (!access_map->map()(n.x, n.y))
        continue;

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
