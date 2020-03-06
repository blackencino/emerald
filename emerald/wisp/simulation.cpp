#include <emerald/wisp/simulation.h>

// #include <emerald/noise/fbm.h>
// #include <emerald/noise/simplex_noise.h>
#include <emerald/wisp/simulation_old.h>
#include <emerald/util/format.h>
#include <emerald/util/timer.h>

namespace emerald::wisp {

static float cell_size(Simulation::Parameters const& params) {
    return params.world_size / static_cast<float>(params.resolution);
}

Simulation::Simulation(Parameters const& params)
  : m_params(params)
  , m_state({params.resolution, params.resolution}) {
    auto const NX = params.resolution;
    auto const NY = params.resolution;
    auto const DXY = cell_size(params);

    // emerald::noise::StdFbmf fbm{4, 2.17f, 0.75f};
    // constexpr float noise_scale = 50.3f;
    // for (int j = 0; j < NY; ++j) {
    //     auto const worldY = 9911.44f + DXY * static_cast<float>(j);
    //     for (int i = 0; i < NX; ++i) {
    //         auto const worldX = 2341.17f + DXY * static_cast<float>(i);

    //         auto n = fbm(worldX / noise_scale, worldY / noise_scale);
    //         n = 1.0f - std::abs(n);
    //         n = (2.0f * n) - 1.0f;

    //         m_state.height.value(i, j) = 0.5f * n;
    //     }
    // }
    // enforce_neumann_boundary_conditions(m_state.height);

    // m_state.height_prev = m_state.height;
    // m_state.velocity_prev = m_state.velocity;

    m_old = std::make_unique<Simulation_old>(params, m_state);

    //for (int i = 0; i < 50; ++i) { step(); }
}

Simulation::~Simulation() {
}

void Simulation::set_input(V2f const prev_ndc_point,
                           V2f const ndc_point, V3f const rgb) {
    m_input = {{std::clamp(static_cast<int>(prev_ndc_point.x * m_params.resolution),
                           0,
                           m_params.resolution - 1),
                std::clamp(static_cast<int>(prev_ndc_point.y * m_params.resolution),
                           0,
                           m_params.resolution - 1)},
                {std::clamp(static_cast<int>(ndc_point.x * m_params.resolution),
                           0,
                           m_params.resolution - 1),
                std::clamp(static_cast<int>(ndc_point.y * m_params.resolution),
                           0,
                           m_params.resolution - 1)},
               {rgb.x, rgb.y, rgb.z}};
}

void Simulation::step() {
    //auto const dt = 1.0f / m_params.frames_per_second;
    // Caliper caliper{"full step"};
    if (m_input) {
        m_old->InputActive = true;
        auto const [prev_ij, ij, rgb] = m_input.value();
        m_old->PrevInputIndexX = prev_ij[0];
        m_old->PrevInputIndexY = prev_ij[1];
        m_old->InputIndexX = ij[0];
        m_old->InputIndexY = ij[1];
        m_old->InputColorR = rgb[0];
        m_old->InputColorG = rgb[1];
        m_old->InputColorB = rgb[2];
    } else {
        m_old->InputActive = false;
    }
    m_old->FluidTimeStep();
    m_old->InputActive = false;
}

}  // namespace emerald::wisp
