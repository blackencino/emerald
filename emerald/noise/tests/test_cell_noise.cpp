#include <emerald/noise/cell_noise.h>
#include <iostream>

int main(int, char*[]) {
    for (float f = 0.1f; f <= 100.0f; f += 0.371773f) {
        std::cout << "CellNoise( " << f << " ) = \t"
                  << emerald::noise::CellNoise3(Imath::V3f(f, 0, 0))
                  << std::endl;
    }

    return 0;
}
