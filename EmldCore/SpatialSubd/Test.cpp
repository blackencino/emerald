//-*****************************************************************************
// Copyright (c) 2001-2013, Christopher Jon Horvath. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of Christopher Jon Horvath nor the names of his
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//-*****************************************************************************

#include "All.h"

#include <OpenEXR/ImathVec.h>

#include <iostream>
#include <random>
#include <vector>

namespace EmldCore {
namespace SpatialSubd {

using namespace Imath;

//-*****************************************************************************
void Test1() {
    // Make some points
    std::mt19937_64 rng(12345);
    std::uniform_real_distribution<float> dst(-1.0f, 1.0f);
    auto gen = [&rng, &dst](){ return dst(rng); };

    std::vector<V2f> points(5000);
    for (int i = 0; i < 5000; ++i) { points[i] = 50.0f * V2f(gen(), gen()); }
    std::cout << "Made " << points.size() << "random points." << std::endl;

    // Make the tree
    PointTree2f pointTree(points);
    std::cout << "Made a point tree." << std::endl;

    // Choose a point within the bounding box of the tree
    V2f testPt(50.0f * gen(), 50.0f * gen());
    float testRad = 5.0f;
    std::cout << "Chose test point:" << testPt << std::endl;

    // Find the closest point.
    V2f bestPoint;
    float bestDist2;
    if (pointTree.closestPoint(testPt, bestPoint, bestDist2)) {
        std::cout << "Closest point: " << bestPoint
                  << " to test point: " << testPt
                  << " with dist2 of: " << bestDist2 << std::endl;
    } else {
        std::cout << "No closest point found. Weird." << std::endl;
    }

    // Find the closest point squared.
    float maxDist2 = testRad * testRad;
    if (pointTree.closestPointWithinSquaredDistance(
          testPt, maxDist2, bestPoint, bestDist2)) {
        std::cout << "Closest point: " << bestPoint
                  << " to test point: " << testPt
                  << " with dist2 of: " << bestDist2
                  << ", max dist2 was: " << maxDist2 << std::endl;
    } else {
        std::cout << "No closest point found within maxDist2 of: " << maxDist2
                  << std::endl;
    }
}

//-*****************************************************************************
void Test2() {
    // Make some points
    std::mt19937_64 rng(54321);
    std::uniform_real_distribution<float> dst(-1.0f, 1.0f);
    auto gen = [&rng, &dst](){ return dst(rng); };

    std::vector<V3f> points(5000);
    for (int i = 0; i < 5000; ++i) {
        points[i] = 50.0f * V3f(gen(), gen(), gen());
    }
    std::cout << "Made " << points.size() << "random points." << std::endl;

    // Make the tree
    PointTree3f pointTree(points);
    std::cout << "Made a point tree." << std::endl;

    // Choose a point within the bounding box of the tree
    V3f testPt(50.f * gen(), 50.0f * gen(), 50.0f * gen());
    float testRad = 5.0f;
    std::cout << "Chose test point:" << testPt << std::endl;

    // Find the closest point.
    V3f bestPoint;
    float bestDist2;
    if (pointTree.closestPoint(testPt, bestPoint, bestDist2)) {
        std::cout << "Closest point: " << bestPoint
                  << " to test point: " << testPt
                  << " with dist2 of: " << bestDist2 << std::endl;
    } else {
        std::cout << "No closest point found. Weird." << std::endl;
    }

    // Find the closest point squared.
    float maxDist2 = testRad * testRad;
    if (pointTree.closestPointWithinSquaredDistance(
          testPt, maxDist2, bestPoint, bestDist2)) {
        std::cout << "Closest point: " << bestPoint
                  << " to test point: " << testPt
                  << " with dist2 of: " << bestDist2
                  << ", max dist2 was: " << maxDist2 << std::endl;
    } else {
        std::cout << "No closest point found within maxDist2 of: " << maxDist2
                  << std::endl;
    }
}

}  // End namespace SpatialSubd
}  // End namespace EmldCore
