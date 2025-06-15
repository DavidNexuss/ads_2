#include <set>
#include <queue>
#include <optional>
#include "sweep.hpp"

struct Event {
  float x;
  int   type;
  Point p;
  int   segIndexA, segIndexB;

  bool operator<(const Event& other) const {
    if (!fequal(x, other.x)) return x > other.x;
    return type > other.type;
  }
};

struct SegmentCompare {
  float&                      sweepX;
  const std::vector<Segment>& segments;

  SegmentCompare(float& sweepX, const std::vector<Segment>& segments) :
    sweepX(sweepX), segments(segments) {}

  bool operator()(int i, int j) const {
    const auto& si = segments[i];
    const auto& sj = segments[j];

    auto eval = [](const Segment& s, float x) -> float {
      if (fequal(s.a.x, s.b.x)) return s.a.y;
      float t = (x - s.a.x) / (s.b.x - s.a.x);
      return s.a.y + t * (s.b.y - s.a.y);
    };

    return lessThan(eval(si, sweepX), eval(sj, sweepX));
  }
};

SweepResult findIntersections(const Sweepinfo& info) {
  SweepResult                result;
  std::priority_queue<Event> eventQueue;
  const auto&                segments = info.segments;

  for (int i = 0; i < (int)segments.size(); ++i) {
    Point left = segments[i].a, right = segments[i].b;
    if (left.x > right.x) std::swap(left, right);
    eventQueue.push({left.x, 0, left, i, -1});
    eventQueue.push({right.x, 1, right, i, -1});
  }

  float                               sweepX = 0.0f;
  std::set<int, SegmentCompare>       activeSet(SegmentCompare(sweepX, segments));
  std::map<std::pair<int, int>, bool> scheduled;

  auto tryAddIntersection = [&](int i, int j) {
    if (i > j) std::swap(i, j);
    if (scheduled[{i, j}]) return;
    auto pt = intersect(segments[i], segments[j]);
    if (pt.has_value()) {
      eventQueue.push({pt->x, 2, *pt, i, j});
      scheduled[{i, j}] = true;
    }
  };

  while (!eventQueue.empty()) {
    Event ev = eventQueue.top();
    eventQueue.pop();
    sweepX = ev.x;

    if (ev.type == 0) {
      auto it   = activeSet.insert(ev.segIndexA).first;
      auto prev = (it == activeSet.begin()) ? activeSet.end() : std::prev(it);
      auto next = std::next(it);
      if (prev != activeSet.end()) tryAddIntersection(*prev, *it);
      if (next != activeSet.end()) tryAddIntersection(*it, *next);
    } else if (ev.type == 1) {
      auto it = activeSet.find(ev.segIndexA);
      if (it == activeSet.end()) continue;
      auto prev = (it == activeSet.begin()) ? activeSet.end() : std::prev(it);
      auto next = std::next(it);
      if (prev != activeSet.end() && next != activeSet.end())
        tryAddIntersection(*prev, *next);
      activeSet.erase(it);
    } else {
      result.intersectionPOints.insert(ev.p);
      result.intersectionMaps[ev.segIndexA].insert(ev.segIndexB);
      result.intersectionMaps[ev.segIndexB].insert(ev.segIndexA);

      int seg1_idx = ev.segIndexA;
      int seg2_idx = ev.segIndexB;

      activeSet.erase(seg1_idx);
      activeSet.erase(seg2_idx);

      activeSet.insert(seg1_idx);
      activeSet.insert(seg2_idx);


      auto it1 = activeSet.find(seg1_idx);
      auto it2 = activeSet.find(seg2_idx);

      if (it1 != activeSet.begin()) {
        tryAddIntersection(*std::prev(it1), *it1);
      }
      if (std::next(it1) != activeSet.end()) {
        tryAddIntersection(*it1, *std::next(it1));
      }

      if (it2 != activeSet.begin()) {
        tryAddIntersection(*std::prev(it2), *it2);
      }
      if (std::next(it2) != activeSet.end()) {
        tryAddIntersection(*it2, *std::next(it2));
      }
    }
  }

  return result;
}
