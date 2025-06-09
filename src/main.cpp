#include "sweep.hpp"
#include <iomanip>
#include <iostream>
#include <stdlib.h>

Sweepinfo test1() {
  Sweepinfo info;

  // Example: Intersecting X shape
  info.segments.emplace_back(Point{0, 0}, Point{10, 10}); // Segment 0
  info.segments.emplace_back(Point{0, 10}, Point{10, 0}); // Segment 1

  // Parallel but non-overlapping
  info.segments.emplace_back(Point{2, 0}, Point{8, 6}); // Segment 2
  info.segments.emplace_back(Point{2, 8}, Point{8, 2}); // Segment 3

  return info;
}

void cliSolution(const Sweepinfo &info) {
  SweepResult result = findIntersections(info);

  std::cout << "===[ Sweep Info ]===\n";
  for (int i = 0; i < info.segments.size(); ++i) {
    const auto &s = info.segments[i];
    std::cout << "Segment " << i << ": (" << s.a.x << ", " << s.a.y << ") -> ("
              << s.b.x << ", " << s.b.y << ")\n";
  }

  std::cout << "\n===[ Intersections Points ]===\n";
  for (size_t i = 0; i < result.intersectionPOints.size(); ++i) {
    const auto &p = result.intersectionPOints[i];
    std::cout << "Intersection " << i << ": (" << std::fixed
              << std::setprecision(3) << p.x << ", " << p.y << ")\n";
  }

  std::cout << "\n===[ Intersected Segments ]===\n";
  for (size_t i = 0; i < result.intersectionSegments.size(); ++i) {
    const auto &s = result.intersectionSegments[i];
    std::cout << "Segment " << i << ": (" << s.a.x << ", " << s.a.y << ") -> ("
              << s.b.x << ", " << s.b.y << ")\n";
  }

  std::cout << "\n===[ Segment Intersection Map ]===\n";
  for (const auto &[seg, hits] : result.intersectionMaps) {
    std::cout << "Segment " << seg << " intersects with: ";
    for (int j : hits) {
      std::cout << j << " ";
    }
    std::cout << "\n";
  }
}

int main() { cliSolution(test1()); }
