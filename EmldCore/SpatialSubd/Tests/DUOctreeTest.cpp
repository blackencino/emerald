#include <EmldCore/SpatialSubd/DUOctree.h>

#include <OpenEXR/ImathBox.h>
#include <OpenEXR/ImathVec.h>

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <map>
#include <random>

using Imath::Box3i;
using Imath::V3f;
using Imath::V3i;

struct Comparator {
    bool operator()(const V3i& iA, const V3i& iB) const {
        if (iA.z < iB.z) {
            return true;
        } else if (iA.z > iB.z) {
            return false;
        } else {
            if (iA.y < iB.y) {
                return true;
            } else if (iA.y > iB.y) {
                return false;
            } else {
                return iA.x < iB.x;
            }
        }
    }
};

typedef std::map<V3i, int, Comparator> V3iIntMap;
typedef EmldCore::SpatialSubd::DUOctree<int> V3iIntTree;

int main(int argc, char* argv[]) {
    std::mt19937_64 rng(12345);
    std::uniform_int_distribution<int> dst(1700, 1900);
    auto die = [&rng, dst]() { return dst(rng); };

    V3iIntMap iMap;
    V3iIntTree iTree;

    for (int j = 0; j < 5000; ++j) {
        V3i next(die(), die(), die());
        iMap[next] = j;
        iTree.set(next, j);
    }

    for (V3iIntMap::iterator iter = iMap.begin(); iter != iMap.end(); ++iter) {
        const V3i& point = (*iter).first;
        int val = (*iter).second;
        int treeVal = 0;
        bool found = iTree.get(point, treeVal);

        if (!found || val != treeVal) {
            std::cerr << "ERROR: Bad tree. " << std::endl
                      << "point: " << point << std::endl
                      << "val: " << val << std::endl
                      << "treeVal: " << treeVal << std::endl
                      << "found: " << found << std::endl;
            exit(-1);
        }

        std::cout << "\tpoint: " << point << "\tval: " << val
                  << "\ttreeVal: " << treeVal << std::endl;
    }

    std::cout << "All good." << std::endl;

    return 0;
}
