#pragma once
#include <cmath>
#include <map>
#include <vector>

inline constexpr float EPS = 1e-6f;

inline bool fequal(float a, float b) { return std::fabs(a - b) < EPS; }

inline bool lessThan(float a, float b) { return a < b - EPS; }

struct Point {
  float x, y;

  inline bool operator==(const Point &other) const {
    return fequal(x, other.x) && fequal(y, other.y);
  }

  inline bool operator!=(const Point &other) const { return !(*this == other); }

  inline bool operator<(const Point &other) const {
    if (lessThan(x, other.x))
      return true;
    return lessThan(y, other.y);
  }
};

struct Segment {
  Point a, b;

  Segment(Point a, Point b) {
    if (a.y > b.y)
      std::swap(a, b);

    this->a = a;
    this->b = b;
  }

  inline bool operator==(const Segment &other) const {
    return a == other.a && b == other.b;
  }

  inline bool operator!=(const Segment &other) const {
    return !(*this == other);
  }

  inline bool operator<(const Segment &other) const {
    if (a < other.a)
      return true;
    return b < other.b;
  }
};

struct Sweepinfo {
  std::vector<Segment> segments;
};

struct SweepResult {
  // Contains all points of intrsection
  std::vector<Point> intersectionPOints;

  // Contains all segments resulting of intersection, that is, if segment A-B
  // intersects with C-D it returns this vector includes the resulting for
  // different segments containing the intersection point

  std::vector<Segment> intersectionSegments;

  // Contains a map that maps segment_i with all its intersecting segment
  // indices
  std::map<int, std::vector<int>> intersectionMaps;

  inline bool operator==(const SweepResult &other) {
    return other.intersectionMaps == intersectionMaps &&
           other.intersectionSegments == intersectionSegments &&
           other.intersectionPOints == intersectionPOints;
  }
};

SweepResult findIntersections(const Sweepinfo &info);
SweepResult findIntersectionsNaive(const Sweepinfo &info);
