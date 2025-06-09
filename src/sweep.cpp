#include "sweep.hpp"
#include <algorithm>
#include <cmath>
#include <optional>
#include <queue>
#include <set>

constexpr float EPS = 1e-6f;

inline bool fequal(float a, float b) { return std::fabs(a - b) < EPS; }

inline bool lessThan(float a, float b) { return a < b - EPS; }

struct Event {
  float x;
  int type; // 0 = segment start, 1 = segment end, 2 = intersection
  Point p;
  int seg1, seg2; // seg1 is always valid, seg2 is valid only for type == 2

  bool operator>(const Event &other) const {
    auto retVal = [this](const Event &other) {
      if (!fequal(x, other.x))
        return lessThan(x, other.x);
      return type < other.type;
    };

    return !retVal(other);
  }
};

struct SweepCompare {
  float sweepX;
  const std::vector<Segment> *segments;

  SweepCompare(float x, const std::vector<Segment> *segs)
      : sweepX(x), segments(segs) {}

  // Less than function for comparing segments at the current sweep line
  // position
  bool operator()(int i, int j) const {
    const Segment &s1 = (*segments)[i];
    const Segment &s2 = (*segments)[j];

    auto evalY = [](const Segment &s, float x) -> float {
      if (fequal(s.a.x, s.b.x))
        return std::min(s.a.y, s.b.y);
      float t = (x - s.a.x) / (s.b.x - s.a.x);
      return s.a.y + t * (s.b.y - s.a.y);
    };

    float y1 = evalY(s1, sweepX);
    float y2 = evalY(s2, sweepX);
    if (!fequal(y1, y2))
      return y1 < y2;
    return i < j;
  }
};

std::optional<Point> computeIntersection(const Segment &s1, const Segment &s2) {
  float x1 = s1.a.x, y1 = s1.a.y;
  float x2 = s1.b.x, y2 = s1.b.y;
  float x3 = s2.a.x, y3 = s2.a.y;
  float x4 = s2.b.x, y4 = s2.b.y;

  float denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
  if (fequal(denom, 0))
    return std::nullopt; // Parallel

  float px =
      ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) /
      denom;
  float py =
      ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) /
      denom;

  auto onSegment = [](Point p, Segment s) {
    return std::min(s.a.x, s.b.x) - EPS <= p.x &&
           p.x <= std::max(s.a.x, s.b.x) + EPS &&
           std::min(s.a.y, s.b.y) - EPS <= p.y &&
           p.y <= std::max(s.a.y, s.b.y) + EPS;
  };

  Point inter{px, py};
  if (onSegment(inter, s1) && onSegment(inter, s2))
    return inter;
  return std::nullopt;
}

SweepResult findIntersections(const Sweepinfo &info) {
  SweepResult result;
  std::priority_queue<Event, std::vector<Event>, std::greater<>> events;

  const auto &segments = info.segments;
  for (int i = 0; i < segments.size(); ++i) {
    const auto &s = segments[i];
    Point left = s.a.x < s.b.x ? s.a : s.b;
    Point right = s.a.x < s.b.x ? s.b : s.a;
    events.push(Event{left.x, 0, left, i, -1});
    events.push(Event{right.x, 1, right, i, -1});
  }

  SweepCompare cmp(0, &segments);
  std::set<int, SweepCompare> status(cmp);
  std::map<int, std::set<int>> intersectMap;

  auto tryInsertEvent = [&](int i, int j) {
    if (i > j)
      std::swap(i, j);
    auto inter = computeIntersection(segments[i], segments[j]);
    if (inter) {
      events.push(Event{inter->x, 2, *inter, i, j});
    }
  };

  while (!events.empty()) {
    Event e = events.top();
    events.pop();

    cmp.sweepX = e.x;

    if (e.type == 0) {
      // Insert
      auto it = status.insert(e.seg1).first;
      auto prev = (it == status.begin()) ? status.end() : std::prev(it);
      auto next = std::next(it);

      if (prev != status.end())
        tryInsertEvent(*prev, *it);
      if (next != status.end())
        tryInsertEvent(*it, *next);
    } else if (e.type == 1) {
      // Remove
      auto it = status.find(e.seg1);
      auto prev = (it == status.begin()) ? status.end() : std::prev(it);
      auto next = std::next(it);

      if (prev != status.end() && next != status.end())
        tryInsertEvent(*prev, *next);

      if (it != status.end())
        status.erase(it);
    } else {
      // Intersection
      result.intersectionPOints.push_back(e.p);
      result.intersectionMaps[e.seg1].push_back(e.seg2);
      result.intersectionMaps[e.seg2].push_back(e.seg1);

      Segment s1 = segments[e.seg1];
      Segment s2 = segments[e.seg2];

      result.intersectionSegments.push_back(Segment(s1.a, e.p));
      result.intersectionSegments.push_back(Segment(e.p, s1.b));
      result.intersectionSegments.push_back(Segment(s2.a, e.p));
      result.intersectionSegments.push_back(Segment(e.p, s2.b));

      auto it1 = status.find(e.seg1);
      auto it2 = status.find(e.seg2);
      if (it1 == status.end() || it2 == status.end())
        continue;

      if (cmp(*it2, *it1))
        std::swap(it1, it2);

      int s1_idx = *it1;
      int s2_idx = *it2;

      status.erase(it1);
      status.erase(it2);

      status.insert(s2_idx);
      status.insert(s1_idx);

      auto itA = status.find(s2_idx);
      auto itB = status.find(s1_idx);

      auto before = (itA == status.begin()) ? status.end() : std::prev(itA);
      auto after = std::next(itB);

      if (before != status.end())
        tryInsertEvent(*before, *itA);
      if (after != status.end())
        tryInsertEvent(*after, *itB);
    }
  }

  return result;
}
