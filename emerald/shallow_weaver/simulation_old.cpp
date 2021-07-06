#include <emerald/shallow_weaver/simulation_old.h>

#include <emerald/shallow_weaver/slab_ops.h>
#include <emerald/util/functions.h>
#include <emerald/util/timer.h>

#include <tbb/parallel_for.h>

#include <iostream>

namespace emerald::shallow_weaver {

static constexpr float WALL_DEPTH = 10.5f;

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

    do_slab_op_2d(
      {NX, NY},
      [stride = NX,
       height = AState(io_h).data(),
       terrain_height = _state.terrain_height.data()](
        auto const /*i*/, auto const /*j*/, auto const index) {
          auto const h = height[index];
          auto const t = terrain_height[index];
          if (t > h) {
              height[index] = 0.0f;
              // height[index] = std::max(0.0f, t);
          }

          // auto const d = h - t;
          // if (d >= 0.0f) { return; }
          // if (d > -WALL_DEPTH) {
          //     height[index] = t;
          //     return;
          // }

          // float h_numer = 0.001f * t;
          // float h_denom = 0.001f;

          // float const h_bottom = height[index - stride];
          // float const t_bottom = terrain_height[index - stride];
          // if (h_bottom > t_bottom) {
          //     h_numer += h_bottom;
          //     h_denom += 1.0f;
          // }

          // float const h_left = height[index - 1];
          // float const t_left = terrain_height[index - 1];
          // if (h_left > t_left) {
          //     h_numer += h_left;
          //     h_denom += 1.0f;
          // }

          // float const h_right = height[index + 1];
          // float const t_right = terrain_height[index + 1];
          // if (h_right > t_right) {
          //     h_numer += h_right;
          //     h_denom += 1.0f;
          // }

          // float const h_top = height[index + stride];
          // float const t_top = terrain_height[index + stride];
          // if (h_top > t_top) {
          //     h_numer += h_top;
          //     h_denom += 1.0f;
          // }

          // height[index] = h_numer / h_denom;
      },
      0);
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

// each point that we solve for has a diagonal value and 0-4 off-diagonal
// values, as well as 0-4 off-diagonal indices. when considering a neighboring
// point, we take the relative distance to that point (dxy for left and down,
// -dxy for right and up). if the point is on the other side of a wall, we have
// neumann boundary conditions which means the value on the other side is the
// same as us.

// building one row of the Ax = b solver.
static void compute_coeff_other(
  int2 const size,
  float* const coeff_cen,
  float* const coeff_other,
  float* const rhs,
  float const* const terrain_height,
  float const* const height_star,
  float const wave_speed,
  float const DXY,
  float const DT,
  int const stride,
  std::function<bool(int, int)> const coord_test) {
    auto const k = sqr(wave_speed) / sqr(DXY);

    do_slab_op_2d(
      size,
      [=](auto const i, auto const j, auto const index) {
          // quick out for invalid coordinates.
          if (!coord_test(i, j)) {
              coeff_other[index] = 0.0f;
              return;
          }

          auto const terrain_height_cen = terrain_height[index];
          auto height_star_cen = height_star[index];
          auto depth_cen = std::max(0.0f, height_star_cen - terrain_height_cen);

          // // If the water depth is significantly beneath the terrain depth,
          // // exit!
          // if (depth_cen < -WALL_DEPTH) {
          //     coeff_other[index] = 0.0f;
          //     return;
          // }

          // // If the water depth is only slightly beneath the terrain depth,
          // // set the water height to the terrain height
          // if (depth_cen < 0.0f) {
          //     depth_cen = 0.0f;
          //     height_star_cen = terrain_height_cen;
          // }

          auto const index_other = index + stride;
          auto const terrain_height_other = terrain_height[index_other];
          auto const height_star_other = height_star[index_other];
          auto const depth_other =
            std::max(0.0f, height_star_other - terrain_height_other);
          // if (depth_other > 0.0f) {
          auto const gamma_other = k * (depth_other + depth_cen) / 2.0f;
          coeff_cen[index] += gamma_other * sqr(DT);
          coeff_other[index] = -gamma_other * sqr(DT);
          rhs[index] += gamma_other * (height_star_other - height_star_cen);
          // } else if (depth_other > -WALL_DEPTH) {
          //     // if the depth is strongly negative,
          //     // it means that it is effectively a wall, so we
          //     // use a neumann boundary condition that the acceleration is
          //     the
          //     // same at the left neighbor.
          //     // We should do a second-order boundary condition based on the
          //     // gradient of the terrain, but we can save that for later.
          //     //
          //     // That amounts here to doing nothing.
          //     // If the terrain depth change is slight, treat it as a
          //     // dirichlet boundary condition

          //     // when the depth is small, what do we do? The water height
          //     // can be considered the terrain height at the border.
          //     auto const gamma_other = k * depth_cen / 2.0f;
          //     coeff_cen[index] += gamma_other * sqr(DT);
          //     coeff_other[index] = -gamma_other * sqr(DT);
          //     rhs[index] +=
          //         gamma_other * (terrain_height_other - height_star_cen);
          // }
      },
      0);
}

static void fill(int2 const size, float* const data, float const val) {
    std::fill(data, data + (size[0] * size[1]), val);
    // do_slab_op_1d(size, [=](auto const i) { data[i] = val; });
}

static void init_jacobi_solve(int2 const size,
                              float* const coeff_cen,
                              float* const coeff_bottom,
                              float* const coeff_left,
                              float* const coeff_right,
                              float* const coeff_top,
                              float* const rhs,
                              float const* const terrain_height,
                              float const* const height_star,
                              float const wave_speed,
                              float const DXY,
                              float const DT) {
    fill(size, coeff_cen, 1.0f);
    fill(size, coeff_bottom, 0.0f);
    fill(size, coeff_left, 0.0f);
    fill(size, coeff_right, 0.0f);
    fill(size, coeff_top, 0.0f);
    fill(size, rhs, 0.0f);

    compute_coeff_other(size,
                        coeff_cen,
                        coeff_bottom,
                        rhs,
                        terrain_height,
                        height_star,
                        wave_speed,
                        DXY,
                        DT,
                        -size[0],
                        [](int, int j) -> bool { return j > 0; });

    compute_coeff_other(size,
                        coeff_cen,
                        coeff_left,
                        rhs,
                        terrain_height,
                        height_star,
                        wave_speed,
                        DXY,
                        DT,
                        -1,
                        [](int i, int) -> bool { return i > 0; });

    compute_coeff_other(
      size,
      coeff_cen,
      coeff_right,
      rhs,
      terrain_height,
      height_star,
      wave_speed,
      DXY,
      DT,
      1,
      [size](int i, int) -> bool { return i < (size[0] - 1); });

    compute_coeff_other(
      size,
      coeff_cen,
      coeff_top,
      rhs,
      terrain_height,
      height_star,
      wave_speed,
      DXY,
      DT,
      size[0],
      [size](int, int j) -> bool { return j < (size[1] - 1); });
}

static void apply_coeff_other(int2 const size,
                              float* const accel,
                              float const* const accel_old,
                              float const* const coeff_other,
                              int const stride) {
    do_slab_op_1d(size, [=](auto const i) {
        auto const c = coeff_other[i];
        if (c != 0.0f) { accel[i] -= c * accel_old[i + stride]; }
    });
}

static void copy_from(int2 const size,
                      float* const dst,
                      float const* const src) {
    size_t const num = size[0] * size[1];
    std::copy(src, src + num, dst);
}

static void divide_by_coeff_cen(int2 const size,
                                float* const accel,
                                float const* const coeff_cen) {
    // coeff cen is calculated such that it is always >= 1.0f
    do_slab_op_1d(size, [=](auto const i) { accel[i] /= coeff_cen[i]; });
}

static void jacobi_iteration_from_coeffs(int2 const size,
                                         float* const accel,
                                         float const* const accel_old,
                                         float const* const coeff_cen,
                                         float const* const coeff_bottom,
                                         float const* const coeff_left,
                                         float const* const coeff_right,
                                         float const* const coeff_top,
                                         float const* const rhs) {
    copy_from(size, accel, rhs);
    apply_coeff_other(size, accel, accel_old, coeff_bottom, -size[0]);
    apply_coeff_other(size, accel, accel_old, coeff_left, -1);
    apply_coeff_other(size, accel, accel_old, coeff_right, 1);
    apply_coeff_other(size, accel, accel_old, coeff_top, size[0]);
    divide_by_coeff_cen(size, accel, coeff_cen);
}

static void jacobi_solve(int2 const size,
                         float* const accel,
                         float* const coeff_cen,
                         float* const coeff_bottom,
                         float* const coeff_left,
                         float* const coeff_right,
                         float* const coeff_top,
                         float* const rhs,
                         float* const tmp,
                         float const* const terrain_height,
                         float const* const height_star,
                         float const wave_speed,
                         float const DXY,
                         float const DT,
                         int const num_iters) {
    init_jacobi_solve(size,
                      coeff_cen,
                      coeff_bottom,
                      coeff_left,
                      coeff_right,
                      coeff_top,
                      rhs,
                      terrain_height,
                      height_star,
                      wave_speed,
                      DXY,
                      DT);

    fill(size, accel, 0.0f);

    for (int iter2 = 0; iter2 < num_iters; iter2 += 2) {
        jacobi_iteration_from_coeffs(size,
                                     tmp,
                                     accel,
                                     coeff_cen,
                                     coeff_bottom,
                                     coeff_left,
                                     coeff_right,
                                     coeff_top,
                                     rhs);

        jacobi_iteration_from_coeffs(size,
                                     accel,
                                     tmp,
                                     coeff_cen,
                                     coeff_bottom,
                                     coeff_left,
                                     coeff_right,
                                     coeff_top,
                                     rhs);
    }
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
    // JacobiSolveAccel(StateLabel::HeightStar, i_dt);

    jacobi_solve({NX, NY},
                 _state.acceleration_star.data(),
                 _state.jacobi_tmp.data(),
                 _state.jacobi_tmp_2.data(),
                 _state.jacobi_tmp_3.data(),
                 _state.jacobi_tmp_4.data(),
                 _state.jacobi_tmp_5.data(),
                 _state.jacobi_tmp_6.data(),
                 _state.jacobi_tmp_7.data(),
                 _state.terrain_height.data(),
                 _state.height_star.data(),
                 WaveSpeed,
                 DXY,
                 i_dt,
                 20);
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
    AccumulateEstimate(i_dt / 2.0f);

    // 2
    EstimateVelStar(i_dt);
    EstimateHeightStar(i_dt);
    EstimateAccelStar(i_dt);
    AccumulateEstimate(i_dt / 2.0f);

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
    AccumulateEstimate(i_dt / 6.0f);

    // 2
    EstimateVelStar(i_dt / 2.0f);
    EstimateHeightStar(i_dt / 2.0f);
    EstimateAccelStar(i_dt / 2.0f);
    AccumulateEstimate(i_dt / 3.0f);

    // 3
    EstimateVelStar(i_dt / 2.0f);
    EstimateHeightStar(i_dt / 2.0f);
    EstimateAccelStar(i_dt / 2.0f);
    AccumulateEstimate(i_dt / 3.0f);

    // 4
    EstimateVelStar(i_dt);
    EstimateHeightStar(i_dt);
    EstimateAccelStar(i_dt);
    AccumulateEstimate(i_dt / 6.0f);

    // Final boundary conditions on height and vel
    EnforceHeightBoundaryConditions(StateLabel::Height);
    EnforceNeumannBoundaryConditions(StateLabel::Velocity);

    ApplyDamping(i_dt);
}

}  // namespace emerald::shallow_weaver
