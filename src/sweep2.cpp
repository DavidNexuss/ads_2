#include <set>
#include <queue>
#include <optional>
#include "sweep.hpp"

/*
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

  SegmentCompare(float& x, const std::vector<Segment>& s) :
    sweepX(x), segments(s) {}

  bool operator()(int i, int j) const {
    if (i == j) return false;

    auto eval = [](const Segment& s, float x) -> float {
      if (fequal(s.a.x, s.b.x)) return s.a.y;
      float t = (x - s.a.x) / (s.b.x - s.a.x);
      return s.a.y + t * (s.b.y - s.a.y);
    };

    float yi = eval(segments[i], sweepX);
    float yj = eval(segments[j], sweepX);

    if (!fequal(yi, yj))
      return yi < yj;

    return i < j;
  }
};

SweepResult findIntersections2(const Sweepinfo& info) {
  SweepResult                result;
  std::priority_queue<Event> eventQueue;
  const auto&                segments = info.segments;

  // Seed endpoints
  for (int i = 0; i < (int)segments.size(); ++i) {
    Point a = segments[i].a;
    Point b = segments[i].b;
    if (greaterThan(a.x, b.x)) std::swap(a, b);
    eventQueue.push({a.x, 0, a, i, -1});
    eventQueue.push({b.x, 1, b, i, -1});
  }

  float                         sweepX = 0.0f;
  SegmentCompare                comp(sweepX, segments);
  std::set<int, SegmentCompare> activeSet(comp);
  std::set<std::pair<int, int>> scheduled;

  auto tryAddIntersection = [&](int a, int b) {
    if (a == b) return;
    if (a > b) std::swap(a, b);
    if (scheduled.count({a, b})) return;

    auto ip = intersect(segments[a], segments[b]);
    if (ip && greaterThanOrEqual(ip->x, sweepX)) {
      scheduled.insert({a, b});
      eventQueue.push({ip->x, 2, *ip, a, b});
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
    } else if (ev.type == 2) {
      result.intersectionPOints.insert(ev.p);
      result.intersectionMaps[ev.segIndexA].insert(ev.segIndexB);
      result.intersectionMaps[ev.segIndexB].insert(ev.segIndexA);

      int a = ev.segIndexA;
      int b = ev.segIndexB;

      // Reorder
      auto it_a = activeSet.find(a);
      auto it_b = activeSet.find(b);
      if (it_a == activeSet.end() || it_b == activeSet.end()) continue;

      if (it_a == it_b) {
        // Segments coincide in set ordering; handle carefully
        // Remove one segment at a time and re-insert both to force proper reordering
        activeSet.erase(it_a);
        auto new_it_a = activeSet.insert(a).first;
        auto new_it_b = activeSet.insert(b).first;

        // Check neighbors for both
        for (auto it : {new_it_a, new_it_b}) {
          if (it != activeSet.begin())
            tryAddIntersection(*std::prev(it), *it);
          if (std::next(it) != activeSet.end())
            tryAddIntersection(*it, *std::next(it));
        }
      } else {
        // Different iterators — erase both and re-insert as usual
        activeSet.erase(it_a);
        activeSet.erase(it_b);

        auto new_it_b = activeSet.insert(b).first;
        auto new_it_a = activeSet.insert(a).first;

        for (auto it : {new_it_a, new_it_b}) {
          if (it != activeSet.begin())
            tryAddIntersection(*std::prev(it), *it);
          if (std::next(it) != activeSet.end())
            tryAddIntersection(*it, *std::next(it));
        }
      }
    }
  }
  return result;
}
*/


// Assume these types exist and are included:
// struct Point { float x, y; };
// struct Segment { Point a, b; int id; };
// bool segmentsIntersect(const Segment& s1, const Segment& s2, Point& out);

#include <set>
#include <queue>
#include <map>
#include <optional>
#include <vector>

struct Event {
  float x;
  int   type; // 0=start, 1=end, 2=intersection
  Point p;
  int   segA, segB; // segB == -1 if not intersection

  bool operator<(const Event& other) const {
    if (x != other.x) return x > other.x; // min-heap by x
    return type > other.type;             // start < intersection < end
  }
};

struct SegmentCompare {
  float&                      sweepX;
  const std::vector<Segment>& segments;

  SegmentCompare(float& sweepX, const std::vector<Segment>& segments) :
    sweepX(sweepX), segments(segments) {}

  float evalY(int idx) const {
    const Segment& s = segments[idx];
    if (std::fabs(s.a.x - s.b.x) < 1e-6f) {
      return std::min(s.a.y, s.b.y);
    }
    float t = (sweepX - s.a.x) / (s.b.x - s.a.x);
    return s.a.y + t * (s.b.y - s.a.y);
  }

  bool operator()(int i, int j) const {
    float y1 = evalY(i);
    float y2 = evalY(j);
    if (std::fabs(y1 - y2) > 1e-6f) return y1 < y2;
    return i < j; // tie-breaker by segment id
  }
};


SweepResult findIntersections2(const Sweepinfo& info) {
  auto&                      segments = info.segments;
  SweepResult                result;
  std::priority_queue<Event> eventQueue;

  for (int i = 0; i < (int)segments.size(); ++i) {
    const Segment& s     = segments[i];
    Point          left  = s.a.x < s.b.x ? s.a : s.b;
    Point          right = s.a.x < s.b.x ? s.b : s.a;

    eventQueue.push({left.x, 0, left, i, -1});
    eventQueue.push({right.x, 1, right, i, -1});
  }

  float                               sweepX = 0.0f;
  SegmentCompare                      comp(sweepX, segments);
  std::set<int, SegmentCompare>       activeSet(comp);
  std::map<std::pair<int, int>, bool> scheduledIntersections;

  auto tryAddIntersection = [&](int i, int j) {
    if (i > j) std::swap(i, j);
    if (scheduledIntersections[{i, j}]) return;

    Point ipt;
    if (segmentsIntersect(segments[i], segments[j], ipt)) {
      if (ipt.x > sweepX || std::fabs(ipt.x - sweepX) < 1e-6f) {
        eventQueue.push({ipt.x, 2, ipt, i, j});
        scheduledIntersections[{i, j}] = true;
      }
    }
  };

  while (!eventQueue.empty()) {
    Event ev = eventQueue.top();
    eventQueue.pop();
    sweepX = ev.x;

    if (ev.type == 0) {
      // Insert segment
      auto it = activeSet.insert(ev.segA).first;
      if (it != activeSet.begin()) tryAddIntersection(*std::prev(it), *it);
      if (std::next(it) != activeSet.end()) tryAddIntersection(*it, *std::next(it));
    } else if (ev.type == 1) {
      // Remove segment
      auto it = activeSet.find(ev.segA);
      if (it == activeSet.end()) continue;

      auto prev = (it == activeSet.begin()) ? activeSet.end() : std::prev(it);
      auto next = std::next(it);

      if (prev != activeSet.end() && next != activeSet.end()) {
        tryAddIntersection(*prev, *next);
      }

      activeSet.erase(it);
    } else {
      // Intersection event
      result.intersectionPOints.insert(ev.p);
      result.intersectionMaps[ev.segA].insert(ev.segB);
      result.intersectionMaps[ev.segB].insert(ev.segA);

      auto it_a = activeSet.find(ev.segA);
      auto it_b = activeSet.find(ev.segB);

      if (it_a == activeSet.end() || it_b == activeSet.end()) continue;
      if (it_a == it_b) continue;

      // Swap segments in active set
      int segA = *it_a;
      int segB = *it_b;

      // Erase in order to avoid invalidating iterators
      if (std::distance(it_a, it_b) > 0) {
        activeSet.erase(it_b);
        activeSet.erase(it_a);
      } else {
        activeSet.erase(it_a);
        activeSet.erase(it_b);
      }

      // Insert swapped
      auto it_new_b = activeSet.insert(segA).first;
      auto it_new_a = activeSet.insert(segB).first;

      // Check neighbors of segB (now at it_new_a)
      if (it_new_a != activeSet.begin()) tryAddIntersection(*std::prev(it_new_a), *it_new_a);
      if (std::next(it_new_a) != activeSet.end()) tryAddIntersection(*it_new_a, *std::next(it_new_a));

      // Check neighbors of segA (now at it_new_b)
      if (it_new_b != activeSet.begin()) tryAddIntersection(*std::prev(it_new_b), *it_new_b);
      if (std::next(it_new_b) != activeSet.end()) tryAddIntersection(*it_new_b, *std::next(it_new_b));
    }
  }

  return result;
}
