#pragma once

#include <emerald/wisp/foundation.h>
#include <emerald/shallow_weaver/slab_ops.h>
#include <emerald/wisp/state.h>

#include <memory>
#include <optional>
#include <utility>

namespace emerald::wisp {

class Simulation_old;

class Simulation {
public:
    struct Parameters {
        float world_size = 100.0f;
        int resolution = 256;
        float emission_rate = 2.0f;
        float density_emission_radius = 15.0f;
        float velocity_emission_radius = 20.0f;
        float density_viscosity = 1.0e-5f;
        float velocity_viscosity = 1.0e-5f;
        float density_damping = 0.01f;
        float velocity_damping = 0.01f;
        float velocity_input_scaling = 0.75f;
        float virtual_time_step = 2.0f;
        float frames_per_second = 24.0f;
        int num_batch_frames = 100;
    };

    Simulation(Parameters const& params);
    ~Simulation();

    void set_input(V2f const ndc_point, V3f const rgb);
    void set_input() {
        m_input = std::nullopt;
    }

    void step();

    Parameters const& parameters() const {
        return m_params;
    }

    State const& state() const {
        return m_state;
    }

private:
    Parameters m_params;
    float m_dxy = 0.0f;

    State m_state;

    std::optional<std::pair<int2, float3>> m_input = std::nullopt;

    std::unique_ptr<Simulation_old> m_old;
};

}  // namespace emerald::wisp
