#include <emerald/shallow_weaver/simulation_old.h>

#include <emerald/shallow_weaver/slab_ops.h>
#include <emerald/util/functions.h>
#include <emerald/util/timer.h>

#include <tbb/parallel_for.h>

#include <iostream>

namespace emerald::shallow_weaver {

Simulation_old::Simulation_old(Simulation::Parameters const& params,
                               State& _in_state)
  : _state(_in_state)
  , WaveSpeed(params.wave_speed)
  , WorldSize(params.world_size)
  , NX(params.resolution)
  , NY(params.resolution)
  , ArraySize(params.resolution * params.resolution)
  , DXY(WorldSize / NX)
  , DAMPING(params.damping) {
}

Float_slab& Simulation_old::AState(StateLabel const index) {
    return reinterpret_cast<Float_slab*>(&_state.height)[index];
}
Float_slab const& Simulation_old::AState(StateLabel const index) const {
    return reinterpret_cast<Float_slab const*>(&_state.height)[index];
}

// Index an element of a grid in the state array
int Simulation_old::IX(int i, int j) const {
    return (i + NX * j);
}

static void enforce_dirichlet_boundary_conditions(int2 const size,
                                                  float* const field) {
    do_slab_op_2d(
        size,
        [=](auto const i, auto const j, auto const index) {
            if (i == 0 || i == (size[0] - 0) || j == 0 || j == (size[1] - 1)) {
                field[index] = 0.0f;
            }
        },
        0);
}

void Simulation_old::EnforceDirichletBoundaryConditions(StateLabel const io_a) {
    enforce_dirichlet_boundary_conditions({NX, NY}, AState(io_a).data());
}

static void enforce_neumann_boundary_conditions(int2 const size,
                                                float* const field) {
    do_slab_op_2d(
        size,
        [=](auto const i, auto const j, auto const index) {
            if (j == 0) {
                field[index] = field[index + size[0]];
            } else if (j == size[1] - 1) {
                field[index] = field[index - size[0]];
            } else if (i == 0) {
                field[index] = field[index + 1];
            } else if (i == size[0] - 1) {
                field[index] = field[index - 1];
            }
        },
        0);
}

void Simulation_old::EnforceNeumannBoundaryConditions(StateLabel const io_v) {
    enforce_neumann_boundary_conditions({NX, NY}, AState(io_v).data());
}

void Simulation_old::EnforceHeightBoundaryConditions(StateLabel const io_h) {
    EnforceNeumannBoundaryConditions(io_h);

    if (InputActive) {
        const int radius_pixels = std::max(1, (5 * NX) / 512);
        int const minX = std::clamp(InputIndexX - radius_pixels, 0, NX - 1);
        int const maxX = std::clamp(InputIndexX + radius_pixels, 0, NX - 1);
        int const minY = std::clamp(InputIndexY - radius_pixels, 0, NY - 1);
        int const maxY = std::clamp(InputIndexY + radius_pixels, 0, NY - 1);
        for (int j = minY; j <= maxY; ++j) {
            for (int i = minX; i <= maxX; ++i) {
                auto const r = std::hypot(static_cast<float>(i - InputIndexX),
                                          static_cast<float>(j - InputIndexY));
                if (r <= static_cast<float>(radius_pixels)) {
                    AState(io_h)[IX(i, j)] = InputHeight;
                }
            }
        }
    }
}

void Simulation_old::CopyArray(StateLabel const i_src, StateLabel const o_dst) {
    AState(o_dst) = AState(i_src);
}

void Simulation_old::FillArray(StateLabel const o_a, float i_val) {
    AState(o_a).fill(i_val);
}

void Simulation_old::SwapHeight() {
    AState(StateLabel::Height).swap(AState(StateLabel::HeightPrev));
}

void Simulation_old::SwapVel() {
    AState(StateLabel::Velocity).swap(AState(StateLabel::VelocityPrev));
}

void Simulation_old::SwapState() {
    SwapHeight();
    SwapVel();
}

static void accumulate_here(int2 const size,
                            float* const into,
                            float const* const from,
                            float const gain) {
    do_slab_op_1d(size, [=](auto const i) { into[i] += gain * from[i]; });
}

// Estimate height star
void Simulation_old::EstimateHeightStar(float i_dt) {
    CopyArray(StateLabel::HeightPrev, StateLabel::HeightStar);
    accumulate_here({NX, NY},
                    AState(StateLabel::HeightStar).data(),
                    AState(StateLabel::VelocityStar).data(),
                    i_dt);
    EnforceHeightBoundaryConditions(StateLabel::HeightStar);
}

// Estimate vel star
void Simulation_old::EstimateVelStar(float i_dt) {
    CopyArray(StateLabel::VelocityPrev, StateLabel::VelocityStar);
    accumulate_here({NX, NY},
                    AState(StateLabel::VelocityStar).data(),
                    AState(StateLabel::AccelerationStar).data(),
                    i_dt);
    EnforceNeumannBoundaryConditions(StateLabel::VelocityStar);
}

static void jacobi_compute_bias(Float_slab& bias_slab,
                                Float_slab const& h_slab,
                                float const kappa,
                                float const gamma) {
    do_slab_op_2d(
        bias_slab,
        [&h_slab, kappa, gamma](
            auto& bias_slab, auto const i, auto const j, auto const index) {
            auto const h_star_left = h_slab.value(i - 1, j);
            auto const h_star_right = h_slab.value(i + 1, j);
            auto const h_star_down = h_slab.value(i, j - 1);
            auto const h_star_up = h_slab.value(i, j + 1);
            auto const h_star_cen = h_slab[index];

            auto const b = gamma * (h_star_left + h_star_right + h_star_down +
                                    h_star_up - (4 * h_star_cen));

            bias_slab[index] = b / (1 + 4 * kappa);
        },
        1);
}

static void jacobi_iter_from_bias(Float_slab& a_slab,
                                  Float_slab const& a_old_slab,
                                  Float_slab const& bias_slab,
                                  float const kappa) {
    auto const gain = kappa / (1 + 4 * kappa);
    do_slab_op_2d(
        a_slab,
        [&a_old_slab, &bias_slab, gain](
            auto& a_slab, auto const i, auto const j, auto const index) {
            auto const a_down = a_old_slab.value(i, j - 1);
            auto const a_left = a_old_slab.value(i - 1, j);
            auto const a_right = a_old_slab.value(i + 1, j);
            auto const a_up = a_old_slab.value(i, j + 1);
            auto const c = gain * (a_left + a_right + a_down + a_up);

            a_slab[index] = bias_slab[index] + c;
        },
        1);
}

void Simulation_old::JacobiComputeBias(StateLabel const i_hStar,
                                       StateLabel const o_bias,
                                       float i_dt) {
    float const kappa = sqr(WaveSpeed) * sqr(i_dt) / sqr(DXY);
    float const gamma = sqr(WaveSpeed) / sqr(DXY);
    jacobi_compute_bias(AState(o_bias), AState(i_hStar), kappa, gamma);
}

void Simulation_old::JacobiIterationAccelBias(StateLabel const i_aOld,
                                              StateLabel const o_aNew,
                                              StateLabel const i_Bias,
                                              float i_dt) {
    float const kappa = sqr(WaveSpeed) * sqr(i_dt) / sqr(DXY);
    float const gamma = sqr(WaveSpeed) / sqr(DXY);
    jacobi_iter_from_bias(
        AState(o_aNew), AState(i_aOld), AState(i_Bias), kappa);
    EnforceDirichletBoundaryConditions(o_aNew);
}

// Solve for acceleration.
void Simulation_old::JacobiSolveAccel(StateLabel const i_hStar, float i_dt) {
    // Initialize acceleration to zero.
    FillArray(StateLabel::AccelerationStar, 0.0);

    JacobiComputeBias(i_hStar, StateLabel::JacobiTmp2, i_dt);

    // Solve from StateLabel::JacobiTmp into StateAccel
    for (int iter = 0; iter < 20; ++iter) {
        AState(StateLabel::AccelerationStar)
            .swap(AState(StateLabel::JacobiTmp));

        JacobiIterationAccelBias(StateLabel::JacobiTmp,
                                 StateLabel::AccelerationStar,
                                 StateLabel::JacobiTmp2,
                                 i_dt);
    }
}

void Simulation_old::EstimateAccelStar(float i_dt) {
    JacobiSolveAccel(StateLabel::HeightStar, i_dt);
}

// Accumulate estimate
void Simulation_old::AccumulateEstimate(float i_dt) {
    accumulate_here({NX, NY},
                    AState(StateLabel::Height).data(),
                    AState(StateLabel::VelocityStar).data(),
                    i_dt);
    accumulate_here({NX, NY},
                    AState(StateLabel::Velocity).data(),
                    AState(StateLabel::AccelerationStar).data(),
                    i_dt);
}

static void gain_in_place(Float_slab& into, float const gain) {
    do_slab_op_1d(into, [gain](auto& into, auto const i) { into[i] *= gain; });
}

void Simulation_old::ApplyDamping(float i_dt) {
    // compute gain
    float const gain = std::pow((1.0f - DAMPING), i_dt);
    // std::cout << "Damping = " << DAMPING << ", gain = " << gain << std::endl;
    gain_in_place(AState(StateLabel::Height), gain);
    gain_in_place(AState(StateLabel::Velocity), gain);
}

// First-Order Symplectic Time Step function.
void Simulation_old::TimeStepFirstOrder(float i_dt) {
    // Swap state
    SwapState();

    // Initialize estimate. This just amounts to copying
    // The previous values into the current values.
    CopyArray(StateLabel::HeightPrev, StateLabel::Height);
    CopyArray(StateLabel::VelocityPrev, StateLabel::Velocity);

    // 1
    CopyArray(StateLabel::Velocity, StateLabel::VelocityStar);
    EstimateHeightStar(i_dt);
    EstimateAccelStar(i_dt);
    AccumulateEstimate(i_dt);

    // Final boundary conditions on height and vel
    EnforceHeightBoundaryConditions(StateLabel::Height);
    EnforceNeumannBoundaryConditions(StateLabel::Velocity);

    ApplyDamping(i_dt);
}

// Second-Order Runge-Kutta Time Step function.
void Simulation_old::TimeStepRK2(float i_dt) {
    // Swap state
    SwapState();

    // Initialize estimate. This just amounts to copying
    // The previous values into the current values.
    CopyArray(StateLabel::HeightPrev, StateLabel::Height);
    CopyArray(StateLabel::VelocityPrev, StateLabel::Velocity);

    // 1
    CopyArray(StateLabel::Velocity, StateLabel::VelocityStar);
    EstimateHeightStar(i_dt);
    EstimateAccelStar(i_dt);
    AccumulateEstimate(i_dt / 2.0);

    // 2
    EstimateVelStar(i_dt);
    EstimateHeightStar(i_dt);
    EstimateAccelStar(i_dt);
    AccumulateEstimate(i_dt / 2.0);

    // Final boundary conditions on height and vel
    EnforceHeightBoundaryConditions(StateLabel::Height);
    EnforceNeumannBoundaryConditions(StateLabel::Velocity);

    ApplyDamping(i_dt);
}

// Fourth-Order Runge-Kutta Time Step function.
void Simulation_old::TimeStepRK4(float i_dt) {
    // Swap state
    SwapState();

    // Initialize estimate. This just amounts to copying
    // The previous values into the current values.
    CopyArray(StateLabel::HeightPrev, StateLabel::Height);
    CopyArray(StateLabel::VelocityPrev, StateLabel::Velocity);

    // 1
    CopyArray(StateLabel::Velocity, StateLabel::VelocityStar);
    EstimateHeightStar(i_dt);
    EstimateAccelStar(i_dt);
    AccumulateEstimate(i_dt / 6.0);

    // 2
    EstimateVelStar(i_dt / 2.0);
    EstimateHeightStar(i_dt / 2.0);
    EstimateAccelStar(i_dt / 2.0);
    AccumulateEstimate(i_dt / 3.0);

    // 3
    EstimateVelStar(i_dt / 2.0);
    EstimateHeightStar(i_dt / 2.0);
    EstimateAccelStar(i_dt / 2.0);
    AccumulateEstimate(i_dt / 3.0);

    // 4
    EstimateVelStar(i_dt);
    EstimateHeightStar(i_dt);
    EstimateAccelStar(i_dt);
    AccumulateEstimate(i_dt / 6.0);

    // Final boundary conditions on height and vel
    EnforceHeightBoundaryConditions(StateLabel::Height);
    EnforceNeumannBoundaryConditions(StateLabel::Velocity);

    ApplyDamping(i_dt);
}

}  // namespace emerald::shallow_weaver
