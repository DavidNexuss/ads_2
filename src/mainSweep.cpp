#include "sweep.hpp"
#include <iomanip>
#include <iostream>
#include <random>
#include <stdlib.h>
#include <math.h>
#include <functional>


#define RESET "\033[0m"
#define RED   "\033[31m"
#define GREEN "\033[32m"

Point rotate(float a, Point b) {
  return b;
}

bool verbose = false;

//Simple toy example without anything difficult
Sweepinfo test0() {
  Sweepinfo info;
  float     a = 0.4;

  info.segments.emplace_back(rotate(a, Point{1, 1}), rotate(a, Point{9, 9}));
  info.segments.emplace_back(rotate(a, Point{1, 9}), rotate(a, Point{9, 1}));

  info.segments.emplace_back(rotate(a, Point{3, 5}), rotate(a, Point{7, 5}));
  info.segments.emplace_back(rotate(a, Point{5, 3}), rotate(a, Point{5, 7}));

  return info;
}


/**
 * Simple test with colinear segment Intersections
 */
Sweepinfo test1() {
  Sweepinfo info;
  float     a = 0.0;

  info.segments.emplace_back(rotate(a, Point{0, 0}), rotate(a, Point{10, 10}));
  info.segments.emplace_back(rotate(a, Point{0, 10}), rotate(a, Point{10, 0}));

  info.segments.emplace_back(rotate(a, Point{2, 0}), rotate(a, Point{8, 6}));
  info.segments.emplace_back(rotate(a, Point{2, 8}), rotate(a, Point{8, 2}));

  return info;
}

// Generate N points randomly and connect them one to one
Sweepinfo test2(int n) {
  std::vector<Segment> segments;

  std::random_device                    rd;
  std::mt19937                          gen(rd());
  std::uniform_real_distribution<float> dis(0.0f, 100.0f);

  for (int i = 0; i < n; ++i) {
    Point a{dis(gen), dis(gen)};
    Point b{dis(gen), dis(gen)};
    segments.push_back(Segment{a, b});
  }

  return {.segments = std::move(segments)};
}

// Same segment
Sweepinfo testDegenerate1() {
  Segment a = {{2, 3}, {4, 4}};
  return {{a, a}};
}

// Vertex sharing
Sweepinfo testDegenerate2() {
  Segment a = {{2, 3}, {4, 4}};
  Segment b = {{0, 0}, {4, 4}};
  return {{a, b}};
}

// Intersection is a point
Sweepinfo testDegenerate3() {
  Segment a = {{0, 0}, {4, 0}};
  Segment b = {{3, 0}, {3, 4}};
  return {{a, b}};
}

void cliSolution(const Sweepinfo& info, bool compare, std::function<SweepResult(const Sweepinfo& info)> function, bool showDifference = false) {
  SweepResult result = function(info);

  auto printResult = [](const SweepResult& result) {
    std::cerr << "\n===[ Intersections Points ]===\n";
    int i = 0;
    for (auto& p : result.intersectionPOints) {
      std::cerr << "Intersection " << i++ << ": (" << std::fixed
                << std::setprecision(3) << p.x << ", " << p.y << ")\n";
    }

    std::cerr << "\n===[ Intersected Segments ]===\n";
    for (auto& s : result.intersectionSegments) {
      std::cerr << "Segment " << i << ": (" << s.a.x << ", " << s.a.y
                << ") -> (" << s.b.x << ", " << s.b.y << ")\n";
    }

    std::cerr << "\n===[ Segment Intersection Map ]===\n";
    for (const auto& [seg, hits] : result.intersectionMaps) {
      std::cerr << "Segment " << seg << " intersects with: ";
      for (int j : hits) {
        std::cerr << j << " ";
      }
      std::cerr << "\n";
    }
  };

  if (verbose) {
    std::cerr << "===[ Sweep Info ]===\n";
    for (int i = 0; i < info.segments.size(); ++i) {
      const auto& s = info.segments[i];
      std::cerr << "Segment " << i << ": (" << s.a.x << ", " << s.a.y << ") -> ("
                << s.b.x << ", " << s.b.y << ")\n";
    }
  }

  if (verbose)
    printResult(result);

  if (compare) {
    auto naiveResult = findIntersectionsNaive(info);
    bool same        = result == naiveResult;
    if (same) {
      std::cout << GREEN << ">> Validation OK!!" << RESET << std::endl;
    } else {
      std::cout << RED << ">> Validation ERROR!!      -> " << RESET;
      std::cout << "Expected: " << naiveResult.intersectionPOints.size() << " Got: " << result.intersectionPOints.size() << std::endl;
      if (verbose || showDifference) {
        std::cout << "Naive Result:" << std::endl;
        printResult(naiveResult);
        std::cout << "Result" << std::endl;
        printResult(result);
      }
    }
  }
}

int main(int argc, char** argv) {
  if (argc > 1 && std::string(argv[1]) == "-verbose") {
    verbose = true;
  }

  int count = 4;

  std::function<SweepResult(const Sweepinfo& info)> functions[] = {
    findIntersectionsNaive, findIntersections, findIntersections2, findIntersectionsInterval};

  std::string functionsName[] = {
    "findIntersectionsNaive", "findIntersections", "findIntersections2", "findIntersectionsInterval"};


  for (int i = 0; i < count; i++) {
    std::cout << "Probing: " << functionsName[i] << std::endl;
    std::cout << "test0\t";
    cliSolution(test0(), true, functions[i]);
    std::cout << "test1\t";
    cliSolution(test1(), true, functions[i]);
    std::cout << "degenerate1\t";
    cliSolution(testDegenerate1(), true, functions[i]);
    std::cout << "degenerate2\t";
    cliSolution(testDegenerate2(), true, functions[i]);
    std::cout << "degenerate3\t";
    cliSolution(testDegenerate3(), true, functions[i]);
    std::cout << "test2\t";
    cliSolution(test2(200), true, functions[i]);
    std::cout << std::endl;
  }
}
