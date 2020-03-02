#pragma once

#include <emerald/shallow_weaver/foundation.h>
#include <emerald/shallow_weaver/slab_ops.h>
#include <emerald/shallow_weaver/state.h>

#include <memory>
#include <optional>
#include <utility>

namespace emerald::shallow_weaver {

class Simulation_old;

class Simulation {
public:
    struct Parameters {
        float world_size = 37.175f;
        int resolution = 512;
        float wave_speed = 1.0f;
        float frames_per_second = 24.0f;
        float damping = 1.0e-6f;
        int num_batch_frames = 100;
        Time_integration time_integration = Time_integration::RUNGE_KUTTA_4;
    };

    Simulation(Parameters const& params);
    ~Simulation();

    void set_input(V2f const ndc_point, float const input_height);
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
    Nullary_timeless_slab_op m_velocity_boundary;
    Binary_slab_op m_jacobi_iter;
    Jacobi_solve_op m_jacobi_solve;

    Time_step_op m_time_step;

    State m_state;

    std::optional<std::pair<int2, float>> m_input = std::nullopt;

    std::unique_ptr<Simulation_old> m_old;
};

}  // namespace emerald::shallow_weaver
