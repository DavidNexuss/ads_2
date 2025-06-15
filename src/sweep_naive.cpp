#include "sweep.hpp"
#include <set>

bool segmentsIntersect(const Segment& s1, const Segment& s2, Point& out) {
  auto det = [](float a, float b, float c, float d) {
    return a * d - b * c;
  };

  float x1 = s1.a.x, y1 = s1.a.y;
  float x2 = s1.b.x, y2 = s1.b.y;
  float x3 = s2.a.x, y3 = s2.a.y;
  float x4 = s2.b.x, y4 = s2.b.y;

  float denom = det(x1 - x2, y1 - y2, x3 - x4, y3 - y4);
  if (fequal(denom, 0))
    return false;

  float px =
    det(det(x1, y1, x2, y2), x1 - x2, det(x3, y3, x4, y4), x3 - x4) / denom;
  float py =
    det(det(x1, y1, x2, y2), y1 - y2, det(x3, y3, x4, y4), y3 - y4) / denom;

  if (std::min(x1, x2) - EPS <= px && px <= std::max(x1, x2) + EPS &&
      std::min(y1, y2) - EPS <= py && py <= std::max(y1, y2) + EPS &&
      std::min(x3, x4) - EPS <= px && px <= std::max(x3, x4) + EPS &&
      std::min(y3, y4) - EPS <= py && py <= std::max(y3, y4) + EPS) {
    out = {px, py};
    return true;
  }
  return false;
}

SweepResult findIntersectionsNaive(const Sweepinfo& info) {
  SweepResult result;

  for (int i = 0; i < info.segments.size(); i++) {
    for (int j = i + 1; j < info.segments.size(); j++) {

      Point intersectionPoint;
      if (!segmentsIntersect(info.segments[i], info.segments[j], intersectionPoint))
        continue;

      result.intersectionPOints.insert(intersectionPoint);

      result.intersectionMaps[i].insert(j);
      result.intersectionMaps[j].insert(i);

      Segment a(info.segments[i].a, intersectionPoint);
      Segment b(info.segments[i].b, intersectionPoint);
      Segment c(info.segments[j].a, intersectionPoint);
      Segment d(info.segments[j].b, intersectionPoint);

      result.intersectionSegments.insert(a);
      result.intersectionSegments.insert(b);
      result.intersectionSegments.insert(c);
      result.intersectionSegments.insert(d);
    }
  }
  return result;
}
