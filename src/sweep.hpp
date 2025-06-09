#pragma once
#include <map>
#include <vector>

struct Point {
  float x, y;
};

struct Segment {
  Point a, b;

  Segment(Point a, Point b) {
    if (a.y > b.y)
      std::swap(a, b);

    this->a = a;
    this->b = b;
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
};

SweepResult findIntersections(const Sweepinfo &info);
SweepResult findIntersectionsNaive(const Sweepinfo &info);
