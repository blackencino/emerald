#include <emerald/shallow_weaver/simulation.h>

#include <emerald/noise/fbm.h>
#include <emerald/noise/simplex_noise.h>
#include <emerald/shallow_weaver/simulation_old.h>
#include <emerald/util/format.h>
#include <emerald/util/timer.h>

namespace emerald::shallow_weaver {

static float cell_size(Simulation::Parameters const& params) {
    return params.world_size / static_cast<float>(params.resolution);
}

Simulation::Simulation(Parameters const& params)
  : m_params(params)
  , m_velocity_boundary(enforce_neumann_boundary_conditions)
  , m_jacobi_iter(
        make_jacobi_iteration_accel(m_params.wave_speed, cell_size(params)))
  , m_jacobi_solve(make_jacobi_solve_accel(m_jacobi_iter))
  , m_time_step(
        make_time_step(make_enforce_height_boundary_conditions(std::nullopt),
                       m_velocity_boundary,
                       m_jacobi_solve,
                       m_params.time_integration))
  , m_state({params.resolution, params.resolution}) {
    auto const NX = params.resolution;
    auto const NY = params.resolution;
    auto const DXY = cell_size(params);

    emerald::noise::StdFbmf fbm{4, 2.17f, 0.75f};
    constexpr float noise_scale = 50.3f;
    for (int j = 0; j < NY; ++j) {
        auto const worldY = 9911.44f + DXY * static_cast<float>(j);
        for (int i = 0; i < NX; ++i) {
            auto const worldX = 2341.17f + DXY * static_cast<float>(i);

            auto n = fbm(worldX / noise_scale, worldY / noise_scale);
            n = 1.0f - std::abs(n);
            n = (2.0f * n) - 1.0f;

            m_state.height.value(i, j) = 0.5f * n;
        }
    }
    enforce_neumann_boundary_conditions(m_state.height);

    // m_state.height_prev = m_state.height;
    // m_state.velocity_prev = m_state.velocity;

    m_old = std::make_unique<Simulation_old>(params, m_state);

    // Let's make some default terrain.
    emerald::noise::StdFbmf fbm2{6, 1.67f, 0.57f};
    constexpr float terrain_noise_scale = 43.3f;
    constexpr float terrain_amp = 7.1f;
    for (int j = 0; j < NY; ++j) {
        auto const worldY = 17133.77f + DXY * static_cast<float>(j);
        for (int i = 0; i < NX; ++i) {
            auto const worldX = -9932.31f + DXY * static_cast<float>(i);

            auto const n =
                fbm(worldX / terrain_noise_scale, worldY / terrain_noise_scale);
            m_state.terrain_height.value(i, j) = terrain_amp * n;
        }
    }

    //for (int i = 0; i < 50; ++i) { step(); }
}

Simulation::~Simulation() {
}

void Simulation::set_input(V2f const ndc_point, float const input_height) {
    m_input = {{std::clamp(static_cast<int>(ndc_point.x * m_params.resolution),
                           0,
                           m_params.resolution - 1),
                std::clamp(static_cast<int>(ndc_point.y * m_params.resolution),
                           0,
                           m_params.resolution - 1)},
               input_height};
}

void Simulation::step() {
    auto const dt = 1.0f / m_params.frames_per_second;
    // Caliper caliper{"full step"};
#if 0
    //auto height_boundary = make_enforce_height_boundary_conditions(m_input);
    //auto time_step = make_time_step(height_boundary,
    //                               m_velocity_boundary,
    //                                m_jacobi_solve,
    //                                m_params.time_integration);

    // m_state = time_step(std::move(m_state), dt);
    m_time_step(m_state, dt);

    m_input = std::nullopt;
#else
    if (m_input) {
        m_old->InputActive = true;
        auto const ij = m_input.value().first;
        auto const h = m_input.value().second;
        m_old->InputIndexX = ij[0];
        m_old->InputIndexY = ij[1];
        m_old->InputHeight = h;
    } else {
        m_old->InputActive = false;
    }
    m_old->TimeStepRK4(dt);
    m_old->InputActive = false;
#endif
}

}  // namespace emerald::shallow_weaver
