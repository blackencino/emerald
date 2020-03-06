#include <emerald/wisp/bin/parse_parameters.h>
#include <emerald/wisp/simulation.h>

#include <iostream>

using namespace emerald::wisp;

int main(int argc, char* argv[]) {
    Simulation simulation{parse_parameters(argc, argv)};

    for (int frame = 0; frame < simulation.parameters().num_batch_frames;
         ++frame) {
        simulation.step();
        std::cout << "Frame: " << frame << std::endl;
    }

    return 0;
}
