#pragma once
#include <cmath>
#include <map>
#include <vector>
#include <algorithm>
#include <set>
#include <optional>
/*

inline constexpr float EPS = 1e-6f;

inline bool fequal(float a, float b) { return std::fabs(a - b) < EPS; }

inline bool lessThan(float a, float b) { return a < b - EPS; }

inline bool greaterThan(float a, float b) {
  return a - b > EPS;
}

inline bool lessThanOrEqual(float a, float b) {
  return lessThan(a, b) || fequal(a, b);
}

inline bool greaterThanOrEqual(float a, float b) {
  return greaterThan(a, b) || fequal(a, b);
}


struct Point {
  float x, y;

  inline bool operator==(const Point& other) const {
    return fequal(x, other.x) && fequal(y, other.y);
  }

  inline bool operator!=(const Point& other) const { return !(*this == other); }

  inline bool operator<(const Point& other) const {
    if (!fequal(x, other.x)) return x < other.x;
    return y < other.y;
  }
};

struct Segment {
  Point a, b;
  int   id;

  Segment(Point a, Point b) {
    if (a.y > b.y)
      std::swap(a, b);

    this->a = a;
    this->b = b;
  }

  Segment(Point p1, Point p2, int segment_id) :
    id(segment_id) {
    if (lessThan(p2.x, p1.x) || (fequal(p1.x, p2.x) && lessThan(p2.y, p1.y))) {
      a = p2;
      b = p1;
    } else {
      a = p1;
      b = p2;
    }
  }

  Segment() :
    a(Point()), b(Point()), id(-1) {}

  inline bool operator==(const Segment& other) const {
    return a == other.a && b == other.b;
  }

  inline bool operator!=(const Segment& other) const {
    return !(*this == other);
  }

  inline bool operator<(const Segment& other) const {
    if (a < other.a)
      return true;
    return b < other.b;
  }
};
*/
inline constexpr float EPS = 1e-6f;

inline bool fequal(float a, float b) {
  return std::fabs(a - b) < EPS;
}

inline bool lessThan(float a, float b) {
  return a < b - EPS;
}

inline bool greaterThan(float a, float b) {
  return a - b > EPS;
}

inline bool lessThanOrEqual(float a, float b) {
  return lessThan(a, b) || fequal(a, b);
}

inline bool greaterThanOrEqual(float a, float b) {
  return greaterThan(a, b) || fequal(a, b);
}

struct Point {
  float x, y;

  inline bool operator==(const Point& other) const {
    return fequal(x, other.x) && fequal(y, other.y);
  }

  inline bool operator!=(const Point& other) const {
    return !(*this == other);
  }

  inline bool operator<(const Point& other) const {
    if (!fequal(x, other.x)) return lessThan(x, other.x);
    return lessThan(y, other.y);
  }
};

struct Segment {
  Point a, b;
  int   id;

  // Unified constructor: order points from left to right (increasing x),
  // breaking ties by y ascending.
  Segment(Point p1, Point p2, int segment_id = -1) :
    id(segment_id) {
    if (lessThan(p2.x, p1.x) || (fequal(p1.x, p2.x) && lessThan(p2.y, p1.y))) {
      a = p2;
      b = p1;
    } else {
      a = p1;
      b = p2;
    }
  }

  // Default constructor for convenience
  Segment() :
    a(Point{0, 0}), b(Point{0, 0}), id(-1) {}

  inline bool operator==(const Segment& other) const {
    return a == other.a && b == other.b;
  }

  inline bool operator!=(const Segment& other) const {
    return !(*this == other);
  }

  // Segment comparison by endpoints with epsilon-aware ordering.
  inline bool operator<(const Segment& other) const {
    if (a < other.a) return true;
    if (other.a < a) return false;
    return b < other.b;
  }
};

struct Sweepinfo {
  std::vector<Segment> segments;
};

bool segmentsIntersect(const Segment& s1, const Segment& s2, Point& out);

struct SweepResult {
  // Contains all points of intrsection
  std::set<Point> intersectionPOints;

  // Contains all segments resulting of intersection, that is, if segment A-B
  // intersects with C-D it returns this vector includes the resulting for
  // different segments containing the intersection point

  std::set<Segment> intersectionSegments;

  // Contains a map that maps segment_i with all its intersecting segment
  // indices
  std::map<int, std::set<int>> intersectionMaps;

  inline bool operator==(const SweepResult& other) {
    return intersectionPOints == other.intersectionPOints;
  }
};

SweepResult findIntersectionsInterval(const Sweepinfo& info);
SweepResult findIntersections(const Sweepinfo& info);
SweepResult findIntersections2(const Sweepinfo& info);
SweepResult findIntersectionsLibrary(const Sweepinfo& info);
SweepResult findIntersectionsNaive(const Sweepinfo& info);

inline std::optional<Point> intersect(const Segment& a, const Segment& b) {
  Point r     = {a.b.x - a.a.x, a.b.y - a.a.y};
  Point s     = {b.b.x - b.a.x, b.b.y - b.a.y};
  float denom = r.x * s.y - r.y * s.x;
  if (fequal(denom, 0)) return std::nullopt;

  Point diff = {b.a.x - a.a.x, b.a.y - a.a.y};
  float t    = (diff.x * s.y - diff.y * s.x) / denom;
  float u    = (diff.x * r.y - diff.y * r.x) / denom;
  if (t < -EPS || t > 1 + EPS || u < -EPS || u > 1 + EPS)
    return std::nullopt;

  return Point{a.a.x + t * r.x, a.a.y + t * r.y};
}

inline int orientation(Point p, Point q, Point r) {
  float val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
  if (fequal(val, 0.0f)) return 0;
  return (val > 0) ? 1 : 2;
}

// Checks if point q lies on segment pr
inline bool onSegment(Point p, Point q, Point r) {
  return fequal(orientation(p, q, r), 0) &&
    lessThanOrEqual(std::min(p.x, r.x), q.x) &&
    greaterThanOrEqual(std::max(p.x, r.x), q.x) &&
    lessThanOrEqual(std::min(p.y, r.y), q.y) &&
    greaterThanOrEqual(std::max(p.y, r.y), q.y);
}
