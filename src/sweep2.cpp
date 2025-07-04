#include "sweep.hpp"
#include <set>
#include <queue>
#include <map>
#include <vector>

struct Event {
  float x;
  int   type;
  Point p;
  int   segA, segB;

  bool operator<(const Event& other) const {
    if (x != other.x) return x > other.x;
    return type > other.type;
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
    return i < j;
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
      result.intersectionPOints.insert(ev.p);
      result.intersectionMaps[ev.segA].insert(ev.segB);
      result.intersectionMaps[ev.segB].insert(ev.segA);

      auto it_a = activeSet.find(ev.segA);
      auto it_b = activeSet.find(ev.segB);

      if (it_a == activeSet.end() || it_b == activeSet.end()) continue;
      if (it_a == it_b) continue;

      int segA = *it_a;
      int segB = *it_b;

      if (std::distance(it_a, it_b) > 0) {
        activeSet.erase(it_b);
        activeSet.erase(it_a);
      } else {
        activeSet.erase(it_a);
        activeSet.erase(it_b);
      }

      auto it_new_b = activeSet.insert(segA).first;
      auto it_new_a = activeSet.insert(segB).first;

      if (it_new_a != activeSet.begin()) tryAddIntersection(*std::prev(it_new_a), *it_new_a);
      if (std::next(it_new_a) != activeSet.end()) tryAddIntersection(*it_new_a, *std::next(it_new_a));

      if (it_new_b != activeSet.begin()) tryAddIntersection(*std::prev(it_new_b), *it_new_b);
      if (std::next(it_new_b) != activeSet.end()) tryAddIntersection(*it_new_b, *std::next(it_new_b));
    }
  }

  return result;
}
