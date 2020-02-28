#include <emerald/noise/simplex_noise.h>
#include <emerald/util/random.h>

#include <iostream>

using namespace emerald::noise;

//-*****************************************************************************
void noiseTest1() {
    emerald::util::UniformRand gen{0.0, 1.0};
    SimplexNoised snd;

    for (int i = 0; i < 100; ++i) {
        V3d pt(300.0 * (gen() - 0.5),
               300.0 * (gen() - 0.5),
               300.0 * (gen() - 0.5));

        double t = snd(pt);

        std::cout << "Point: " << pt << ", noise val: " << t << std::endl;
    }
}

//-*****************************************************************************
int main(int, char*[]) {
    noiseTest1();
    return 0;
}
