#include "sweep.hpp"
#include <map>

struct IntervalNode {
  Segment       segment;
  float         low, high;
  IntervalNode* left  = nullptr;
  IntervalNode* right = nullptr;
  float         maxHigh;

  IntervalNode(const Segment& seg) :
    segment(seg), low(std::min(seg.a.x, seg.b.x)), high(std::max(seg.a.x, seg.b.x)), maxHigh(high) {}
};

class IntervalTree {
  public:
  IntervalTree() :
    root(nullptr) {}
  ~IntervalTree() { clear(root); }

  void insert(const Segment& segment) {
    root = insert(root, segment);
  }

  void search(const Segment& segment, std::vector<Segment>& result) const {
    search(root, segment, result);
  }

  private:
  IntervalNode* root;

  void clear(IntervalNode* node) {
    if (!node) return;
    clear(node->left);
    clear(node->right);
    delete node;
  }

  IntervalNode* insert(IntervalNode* node, const Segment& segment) {
    if (!node) return new IntervalNode(segment);

    float low = std::min(segment.a.x, segment.b.x);
    if (low < node->low) {
      node->left = insert(node->left, segment);
    } else {
      node->right = insert(node->right, segment);
    }

    node->maxHigh = std::max(node->maxHigh, std::max(segment.a.x, segment.b.x));
    return node;
  }

  bool overlaps(float low1, float high1, float low2, float high2) const {
    return low1 <= high2 + EPS && low2 <= high1 + EPS;
  }

  void search(IntervalNode* node, const Segment& segment, std::vector<Segment>& result) const {
    if (!node) return;

    float low  = std::min(segment.a.x, segment.b.x);
    float high = std::max(segment.a.x, segment.b.x);

    if (overlaps(low, high, node->low, node->high)) {
      result.push_back(node->segment);
    }

    if (node->left && node->left->maxHigh >= low - EPS) {
      search(node->left, segment, result);
    }

    search(node->right, segment, result);
  }
};

SweepResult findIntersectionsInterval(const Sweepinfo& info) {
  SweepResult          result;
  IntervalTree         tree;
  std::map<Point, int> pointIndexMap;

  for (int i = 0; i < info.segments.size(); ++i) {
    const Segment&       seg = info.segments[i];
    std::vector<Segment> candidates;
    tree.search(seg, candidates);

    for (const Segment& other : candidates) {
      Point ip;
      if (segmentsIntersect(seg, other, ip)) {
        if (pointIndexMap.count(ip) == 0) {
          pointIndexMap[ip] = result.intersectionPOints.size();
          result.intersectionPOints.insert(ip);
        }

        for (const Segment& s : {seg, other}) {
          if (s.a != ip && s.b != ip) {
            result.intersectionSegments.insert(Segment(s.a, ip));
            result.intersectionSegments.insert(Segment(ip, s.b));
          } else {
            result.intersectionSegments.insert(s);
          }
        }

        int i1 = std::find(info.segments.begin(), info.segments.end(), seg) - info.segments.begin();
        int i2 = std::find(info.segments.begin(), info.segments.end(), other) - info.segments.begin();

        result.intersectionMaps[i1].insert(i2);
        result.intersectionMaps[i2].insert(i1);
      }
    }
    tree.insert(seg);
  }

  return result;
}
