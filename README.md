# Sweep Line Segment Intersection

A C++ project implementing and testing 2D line segment intersection algorithms, including a sweep line approach and an interval tree-based fallback.

## Overview

This project evaluates sweep line algorithms for detecting segment intersections in 2D, highlighting limitations such as numerical precision and comparator stability. When misclassification occurs, an alternative interval tree method is used to improve reliability.

## Features

- Sweep line algorithm for segment intersection detection
- Interval tree fallback approach
- Multiple test cases covering degenerate and random inputs
- No external dependencies — pure C++ with CMake build system

## Build Instructions

```bash
cmake . -B build
cd build
make -j4
./mainSweep [-verbose]
