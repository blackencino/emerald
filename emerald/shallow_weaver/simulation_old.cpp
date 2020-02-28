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
  , DXY(WorldSize / NX) {
}

Float_slab& Simulation_old::AState(int index) {
    return reinterpret_cast<Float_slab*>(&_state.height)[index];
}
Float_slab const& Simulation_old::AState(int index) const {
    return reinterpret_cast<Float_slab const*>(&_state.height)[index];
}

// Index an element of a grid in the state array
int Simulation_old::IX(int i, int j) const {
    return (i + NX * j);
}

void enforce_dirichlet_boundary_conditions(int2 const size,
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

void Simulation_old::EnforceDirichletBoundaryConditions(int io_a) {
    for (int j = 0; j < NY; ++j) {
        if (j == 0 || j == (NY - 1)) {
            for (int i = 0; i < NX; ++i) { AState(io_a)[IX(i, j)] = 0.0f; }
        } else {
            AState(io_a)[IX(0, j)] = 0.0f;
            AState(io_a)[IX(NX - 1, j)] = 0.0f;
        }
    }
}

void enforce_neumann_boundary_conditions(int2 const size, float* const field) {
    do_slab_op_2d(
        size,
        [=](auto const i, auto const j, auto const index) {
            if (j == 0) {}
        },
        0);
}

void Simulation_old::EnforceNeumannBoundaryConditions(int io_v) {
    for (int j = 0; j < NY; ++j) {
        if (j == 0) {
            for (int i = 0; i < NX; ++i) {
                AState(io_v)[IX(i, 0)] = AState(io_v)[IX(i, 1)];
            }
        } else if (j == (NY - 1)) {
            for (int i = 0; i < NX; ++i) {
                AState(io_v)[IX(i, NY - 1)] = AState(io_v)[IX(i, NY - 2)];
            }
        }

        AState(io_v)[IX(0, j)] = AState(io_v)[IX(1, j)];
        AState(io_v)[IX(NX - 1, j)] = AState(io_v)[IX(NX - 2, j)];
    }
}

void Simulation_old::EnforceHeightBoundaryConditions(int io_h) {
    EnforceNeumannBoundaryConditions(io_h);

    if (InputActive) {
        constexpr int radius_pixels = 5;
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

        // AState(io_h)[IX(InputIndexX, InputIndexY)] = InputHeight;
    }
}

void Simulation_old::CopyArray(int i_src, int o_dst) {
    // for (int i = 0; i < ArraySize; ++i) { AState(o_dst)[i] =
    // AState(i_src)[i]; }

    // auto& src = AState(i_src);
    // do_slab_op_1d_range(
    //     AState(o_dst), [&src](Float_slab& dst, int const begin, int const
    //     end) {
    //         float* const dst_ptr = dst.data();
    //         float const* const src_ptr = src.data();
    //         std::copy(src_ptr + begin, src_ptr + end, dst_ptr + begin);
    //    });
    AState(o_dst) = AState(i_src);
}

void Simulation_old::FillArray(int o_a, float i_val) {
    // for (int i = 0; i < ArraySize; ++i) { AState(o_a)[i] = i_val; }
    AState(o_a).fill(i_val);
    // do_slab_op_1d_range(
    //     AState(o_a), [i_val](Float_slab& dst, int const begin, int const end)
    //     {
    //         float* const dst_ptr = dst.data();
    //         std::fill(dst_ptr + begin, dst_ptr + end, i_val);
    //     });
}

void Simulation_old::SwapHeight() {
    AState(StateHeight).swap(AState(StateHeightPrev));
}

void Simulation_old::SwapVel() {
    AState(StateVel).swap(AState(StateVelPrev));
}

void Simulation_old::SwapState() {
    SwapHeight();
    SwapVel();
}

static void accumulate_here(Float_slab& into,
                            Float_slab const& from,
                            float const gain) {
    do_slab_op_1d(into, [&from, gain](auto& into, auto const i) {
        into[i] += gain * from[i];
    });
}

// Estimate height star
void Simulation_old::EstimateHeightStar(float i_dt) {
#if 0
    for (int i = 0; i < ArraySize; ++i) {
        AState(StateHeightStar)[i] =
            AState(StateHeightPrev)[i] + (i_dt * AState(StateVelStar)[i]);
    }
#else
    CopyArray(StateHeightPrev, StateHeightStar);
    accumulate_here(AState(StateHeightStar), AState(StateVelStar), i_dt);
#endif
    EnforceHeightBoundaryConditions(StateHeightStar);
}

// Estimate vel star
void Simulation_old::EstimateVelStar(float i_dt) {
#if 0
    for (int i = 0; i < ArraySize; ++i) {
        AState(StateVelStar)[i] =
            AState(StateVelPrev)[i] + (i_dt * AState(StateAccelStar)[i]);
    }
#else
    CopyArray(StateVelPrev, StateVelStar);
    accumulate_here(AState(StateVelStar), AState(StateAccelStar), i_dt);
#endif
    EnforceNeumannBoundaryConditions(StateVelStar);
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

void Simulation_old::JacobiComputeBias(int i_hStar, int o_bias, float i_dt) {
    float const kappa = sqr(WaveSpeed) * sqr(i_dt) / sqr(DXY);
    float const gamma = sqr(WaveSpeed) / sqr(DXY);
    jacobi_compute_bias(AState(o_bias), AState(i_hStar), kappa, gamma);
}

void Simulation_old::JacobiIterationAccelBias(int i_aOld,
                                              int o_aNew,
                                              int i_Bias,
                                              float i_dt) {
    float const kappa = sqr(WaveSpeed) * sqr(i_dt) / sqr(DXY);
    float const gamma = sqr(WaveSpeed) / sqr(DXY);
    jacobi_iter_from_bias(
        AState(o_aNew), AState(i_aOld), AState(i_Bias), kappa);
    EnforceDirichletBoundaryConditions(o_aNew);
}

// Solve for acceleration.
void Simulation_old::JacobiSolveAccel(int i_hStar, float i_dt) {
    // Initialize acceleration to zero.
    FillArray(StateAccelStar, 0.0);

    JacobiComputeBias(i_hStar, StateJacobiTmp2, i_dt);

    // Solve from StateJacobiTmp into StateAccel
    for (int iter = 0; iter < 20; ++iter) {
        AState(StateAccelStar).swap(AState(StateJacobiTmp));

        JacobiIterationAccelBias(
            StateJacobiTmp, StateAccelStar, StateJacobiTmp2, i_dt);
    }
}

void Simulation_old::EstimateAccelStar(float i_dt) {
    JacobiSolveAccel(StateHeightStar, i_dt);
}

// Accumulate estimate
void Simulation_old::AccumulateEstimate(float i_dt) {
#if 0
    for (int i = 0; i < ArraySize; ++i) {
        AState(StateHeight)[i] += i_dt * AState(StateVelStar)[i];
        AState(StateVel)[i] += i_dt * AState(StateAccelStar)[i];
    }
#else
    accumulate_here(AState(StateHeight), AState(StateVelStar), i_dt);
    accumulate_here(AState(StateVel), AState(StateAccelStar), i_dt);
#endif
}

// First-Order Symplectic Time Step function.
void Simulation_old::TimeStepFirstOrder(float i_dt) {
    // Swap state
    SwapState();

    // Initialize estimate. This just amounts to copying
    // The previous values into the current values.
    CopyArray(StateHeightPrev, StateHeight);
    CopyArray(StateVelPrev, StateVel);

    // 1
    CopyArray(StateVel, StateVelStar);
    EstimateHeightStar(i_dt);
    EstimateAccelStar(i_dt);
    AccumulateEstimate(i_dt);

    // Final boundary conditions on height and vel
    EnforceHeightBoundaryConditions(StateHeight);
    EnforceNeumannBoundaryConditions(StateVel);
}

// Second-Order Runge-Kutta Time Step function.
void Simulation_old::TimeStepRK2(float i_dt) {
    // Swap state
    SwapState();

    // Initialize estimate. This just amounts to copying
    // The previous values into the current values.
    CopyArray(StateHeightPrev, StateHeight);
    CopyArray(StateVelPrev, StateVel);

    // 1
    CopyArray(StateVel, StateVelStar);
    EstimateHeightStar(i_dt);
    EstimateAccelStar(i_dt);
    AccumulateEstimate(i_dt / 2.0);

    // 2
    EstimateVelStar(i_dt);
    EstimateHeightStar(i_dt);
    EstimateAccelStar(i_dt);
    AccumulateEstimate(i_dt / 2.0);

    // Final boundary conditions on height and vel
    EnforceHeightBoundaryConditions(StateHeight);
    EnforceNeumannBoundaryConditions(StateVel);
}

// Fourth-Order Runge-Kutta Time Step function.
void Simulation_old::TimeStepRK4(float i_dt) {
    // Swap state
    SwapState();

    // Initialize estimate. This just amounts to copying
    // The previous values into the current values.
    CopyArray(StateHeightPrev, StateHeight);
    CopyArray(StateVelPrev, StateVel);

    // 1
    CopyArray(StateVel, StateVelStar);
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
    EnforceHeightBoundaryConditions(StateHeight);
    EnforceNeumannBoundaryConditions(StateVel);
}

}  // namespace emerald::shallow_weaver
