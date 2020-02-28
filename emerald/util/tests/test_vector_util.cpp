#include <emerald/util/foundation.h>
#include <emerald/util/random.h>
#include <emerald/util/vector_util.h>

#include <vector>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

namespace eu = emerald::util;

int main(int argc, char* argv[]) {
    eu::V3d g;
    eu::set_zero(g);

    eu::UniformRand rand;
    std::vector<float> v;
    for (int i = 0; i < 100; ++i) { v.push_back(rand()); }

    const float* vdata = v.data();
    for (int i = 0; i < 100; ++i) { std::cout << vdata[i] << std::endl; }

    eu::VectorHashKey vkey = eu::ComputeVectorHashKey(v);
    std::cout << "Vector hash key: " << eu::FormatHashKey(vkey) << std::endl;

    return 0;
}
