#include <emerald/shallow_weaver/slab_ops.h>

#include <emerald/util/format.h>
#include <emerald/util/functions.h>
#include <emerald/util/timer.h>

#include <tbb/parallel_for.h>

#include <chrono>

namespace emerald::shallow_weaver {

#if 0

Nullary_timeless_slab_op make_enforce_dirichlet_boundary_conditions() {
    return [](Float_slab&& v) -> Float_slab {
        for (int j = 0; j < v.size()[1]; ++j) {
            if (j == 0 || j == (v.size()[1] - 1)) {
                for (int i = 0; i < v.size()[0]; ++i) { v.value(i, j) = 0.0f; }
            } else {
                v.value(0, j) = 0.0f;
                v.value(v.size()[0] - 1, j) = 0.0f;
            }
        }
        return std::move(v);
    };
}

Nullary_timeless_slab_op make_enforce_neumann_boundary_conditions() {
    return [](Float_slab&& v) -> Float_slab {
        for (int j = 0; j < v.size()[1]; ++j) {
            if (j == 0) {
                for (int i = 0; i < v.size()[0]; ++i) {
                    v.value(i, 0) = v.value(i, 1);
                }
            } else if (j == (v.size()[1] - 1)) {
                for (int i = 0; i < v.size()[0]; ++i) {
                    v.value(i, v.size()[1] - 1) = v.value(i, v.size()[1] - 2);
                }
            }

            v.at(0, j) = v.at(1, j);
            v.at(v.size()[0] - 1, j) = v.at(v.size()[0] - 2, j);
        }

        return std::move(v);
    };
}

Nullary_timeless_slab_op make_enforce_height_boundary_conditions(
    std::optional<std::pair<int2, float>> const input) {
    if (!input) {
        return make_enforce_neumann_boundary_conditions();
    } else {
        return [ij = input.value().first,
                input_height = input.value().second,
                boundary = make_enforce_neumann_boundary_conditions()](
                   Float_slab&& h) -> Float_slab {
            Float_slab fixed_h = boundary(std::move(h));
            fixed_h.value(ij) = input_height;
            return fixed_h;
        };
    }
}

static float neighborhood_value_sum(Float_slab const& v,
                                    int const i,
                                    int const j) {
    return v.value(i - 1, j) +  // left
           v.value(i + 1, j) +  // right
           v.value(i, j - 1) +  // down
           v.value(i, j + 1);   // up
}

// Jacobi iteration to get temp acceleration
Binary_slab_op make_jacobi_iteration_accel(float const wave_speed,
                                           float const DXY) {
    return [wave_speed,
            DXY,
            dirichlet = make_enforce_dirichlet_boundary_conditions()](
               Float_slab&& into,
               Float_slab const& accel_old,
               Float_slab const& height,
               float const dt) -> Float_slab {
        auto const kappa = sqr(wave_speed) * sqr(dt) / sqr(DXY);
        auto const gamma = sqr(wave_speed) / sqr(DXY);

        for (int j = 1; j < into.size()[1] - 1; ++j) {
            for (int i = 1; i < into.size()[0] - 1; ++i) {
                auto const a_nbhd_sum = neighborhood_value_sum(accel_old, i, j);
                auto const h_nbhd_sum = neighborhood_value_sum(height, i, j);
                auto const h_cen = height.value(i, j);

                auto const b = gamma * (h_nbhd_sum - (4 * h_cen));
                auto const c = kappa * a_nbhd_sum;

                into.value(i, j) = (b + c) / (1 + 4 * kappa);
            }
        }

        return dirichlet(std::move(into));
    };
}

Jacobi_solve_op make_jacobi_solve_accel(Binary_slab_op jacobi_iter) {
    return [jacobi_iter](Float_slab&& into,
                         Float_slab&& tmp,
                         Float_slab const& height,
                         float const dt) -> std::tuple<Float_slab, Float_slab> {
        // Initialize acceleration to zero
        into.fill(0.0f);

        // Solve from state_jacobi_tmp into StateAccel
        for (int iter2 = 0; iter2 < 10; ++iter2) {
            // solve into tmp
            tmp = jacobi_iter(std::move(tmp), into, height, dt);

            // tmp now contains the best guess at accel. solve back into into
            into = jacobi_iter(std::move(into), tmp, height, dt);
        }

        return {std::move(into), std::move(tmp)};
    };
}

Unary_slab_op make_accumulate() {
    return [](Float_slab&& into,
              Float_slab const& derivative,
              float const dt) -> Float_slab {
        auto const derivative_data = derivative.data();
        auto into_data = into.data();
        auto const array_size = static_cast<int>(into_data.size());

        for (int i = 0; i < array_size; ++i) {
            into_data[i] += dt * derivative_data[i];
        }
        return std::move(into);
    };
}

#define DEFINE_LOCAL_LAMBDAS                                               \
    auto swap_state = [&]() {                                              \
        using std::swap;                                                   \
        swap(state.velocity, state.velocity_prev);                         \
        swap(state.height, state.height_prev);                             \
    };                                                                     \
                                                                           \
    auto estimate_height_star = [&](float const dt) {                      \
        state.height_star = state.height_prev;                             \
        state.height_star = height_boundary(accumulate(                    \
            std::move(state.height_star), state.velocity_star, dt));       \
    };                                                                     \
                                                                           \
    auto estimate_velocity_star = [&](float const dt) {                    \
        state.velocity_star = state.velocity_prev;                         \
        state.velocity_star = velocity_boundary(accumulate(                \
            std::move(state.velocity_star), state.acceleration_star, dt)); \
    };                                                                     \
                                                                           \
    auto estimate_acceleration_star = [&](float const dt) {                \
        std::tie(state.acceleration_star, state.jacobi_tmp) =              \
            jacobi_solve_accel(std::move(state.acceleration_star),         \
                               std::move(state.jacobi_tmp),                \
                               state.height_star,                          \
                               dt);                                        \
    };                                                                     \
                                                                           \
    auto accumulate_estimate = [&](float const dt) {                       \
        state.height =                                                     \
            accumulate(std::move(state.height), state.velocity_star, dt);  \
        state.velocity = accumulate(                                       \
            std::move(state.velocity), state.acceleration_star, dt);       \
    };                                                                     \
                                                                           \
    auto enforce_boundary = [&]() {                                        \
        state.height = height_boundary(std::move(state.height));           \
        state.velocity = velocity_boundary(std::move(state.velocity));     \
    };

// First-Order Symplectic Time Step function.
Time_step_op make_time_step_first_order(
    Nullary_timeless_slab_op height_boundary,
    Nullary_timeless_slab_op velocity_boundary,
    Unary_slab_op accumulate,
    Jacobi_solve_op jacobi_solve_accel) {
    return [=](State&& state, float const dt) {
        DEFINE_LOCAL_LAMBDAS;

        // Swap the states.
        swap_state();

        // Initialize estimate. This just amounts to copying
        // The previous values into the current values.
        state.velocity = state.velocity_prev;
        state.height = state.height_prev;

        // 1
        state.velocity_star = state.velocity;
        estimate_height_star(dt);
        estimate_acceleration_star(dt);
        accumulate_estimate(dt);

        // Final boundary conditions on height and vel
        enforce_boundary();

        return std::move(state);
    };
}

Time_step_op make_time_step_rk2(Nullary_timeless_slab_op height_boundary,
                                Nullary_timeless_slab_op velocity_boundary,
                                Unary_slab_op accumulate,
                                Jacobi_solve_op jacobi_solve_accel) {
    return [=](State&& state, float const dt) {
        DEFINE_LOCAL_LAMBDAS;

        // Swap the states.
        swap_state();

        // Initialize estimate. This just amounts to copying
        // The previous values into the current values.
        state.velocity = state.velocity_prev;
        state.height = state.height_prev;

        // 1
        state.velocity_star = state.velocity;
        estimate_height_star(dt);
        estimate_acceleration_star(dt);
        accumulate_estimate(dt / 2.0f);

        // 2
        estimate_velocity_star(dt);
        estimate_height_star(dt);
        estimate_acceleration_star(dt);
        accumulate_estimate(dt / 2.0);

        // Final boundary conditions on height and vel
        enforce_boundary();

        return std::move(state);
    };
}

// Fourth-Order Runge-Kutta Time Step function.
Time_step_op make_time_step_rk4(Nullary_timeless_slab_op height_boundary,
                                Nullary_timeless_slab_op velocity_boundary,
                                Unary_slab_op accumulate,
                                Jacobi_solve_op jacobi_solve_accel) {
    return [=](State&& state, float const dt) {
        DEFINE_LOCAL_LAMBDAS;

        // Swap the states.
        swap_state();

        // Initialize estimate. This just amounts to copying
        // The previous values into the current values.
        state.velocity = state.velocity_prev;
        state.height = state.height_prev;

        // 1
        state.velocity_star = state.velocity;
        estimate_height_star(dt);
        estimate_acceleration_star(dt);
        accumulate_estimate(dt / 6.0);

        // 2
        estimate_velocity_star(dt / 2.0);
        estimate_height_star(dt / 2.0);
        estimate_acceleration_star(dt / 2.0);
        accumulate_estimate(dt / 3.0);

        // 3
        estimate_velocity_star(dt / 2.0);
        estimate_height_star(dt / 2.0);
        estimate_acceleration_star(dt / 2.0);
        accumulate_estimate(dt / 3.0);

        // 4
        estimate_velocity_star(dt);
        estimate_height_star(dt);
        estimate_acceleration_star(dt);
        accumulate_estimate(dt / 6.0);

        // Final boundary conditions on height and vel
        enforce_boundary();

        return std::move(state);
    };
}

Time_step_op make_time_step(Nullary_timeless_slab_op height_boundary,
                            Nullary_timeless_slab_op velocity_boundary,
                            Unary_slab_op accumulate,
                            Jacobi_solve_op jacobi_solve_accel,
                            Time_integration const time_integration) {
    switch (time_integration) {
    case Time_integration::FIRST_ORDER:
        return make_time_step_first_order(
            height_boundary, velocity_boundary, accumulate, jacobi_solve_accel);
    case Time_integration::RUNGE_KUTTA_2:
        return make_time_step_rk2(
            height_boundary, velocity_boundary, accumulate, jacobi_solve_accel);
    case Time_integration::RUNGE_KUTTA_4:
    default:
        return make_time_step_rk4(
            height_boundary, velocity_boundary, accumulate, jacobi_solve_accel);
    }
}

#else

#define IX(i, j) ((i) + ((j)*NX))

void enforce_dirichlet_boundary_conditions(Float_slab& slab) {
    tbb::parallel_for(tbb::blocked_range<int>{0, slab.size()[1]},
                      [&slab](tbb::blocked_range<int> const& range) {
                          int const NX = slab.size()[0];
                          int const NY = slab.size()[1];
                          for (int j = range.begin(); j < range.end(); ++j) {
                              if (j == 0 || j == (NY - 1)) {
                                  for (int i = 0; i < NX; ++i) {
                                      slab.value(i, j) = 0.0f;
                                  }
                              } else {
                                  slab.value(0, j) = 0.0f;
                                  slab.value(NX - 1, j) = 0.0f;
                              }
                          }
                      });
}

void enforce_neumann_boundary_conditions(Float_slab& slab) {
    tbb::parallel_for(tbb::blocked_range<int>{0, slab.size()[1]},
                      [&slab](tbb::blocked_range<int> const& range) {
                          int const NX = slab.size()[0];
                          int const NY = slab.size()[1];
                          for (int j = range.begin(); j < range.end(); ++j) {
                              if (j == 0) {
                                  for (int i = 0; i < NX; ++i) {
                                      slab.value(i, 0) = slab.value(i, 1);
                                  }
                              } else if (j == (NY - 1)) {
                                  for (int i = 0; i < NX; ++i) {
                                      slab.value(i, NY - 1) =
                                          slab.value(i, NY - 2);
                                  }
                              }

                              slab.value(0, j) = slab.value(1, j);
                              slab.value(NX - 1, j) = slab.value(NX - 2, j);
                          }
                      });
}

Nullary_timeless_slab_op make_enforce_height_boundary_conditions(
    std::optional<std::pair<int2, float> > const input) {
    if (!input) {
        return enforce_neumann_boundary_conditions;
    } else {
        return [ij = input.value().first,
                input_height = input.value().second](Float_slab& h) {
            enforce_neumann_boundary_conditions(h);
            h.value(ij) = input_height;
        };
    }
}

// Jacobi iteration to get temp acceleration
Binary_slab_op make_jacobi_iteration_accel(float const wave_speed,
                                           float const DXY) {
    auto const gamma = sqr(wave_speed / DXY);
    return [gamma](Float_slab& a_slab,
                   Float_slab const& a_old_slab,
                   Float_slab const& h_slab,
                   float const dt) {
        tbb::parallel_for(
            tbb::blocked_range<int>{1, a_slab.size()[1] - 1},
            [&a_slab, &a_old_slab, &h_slab, dt, gamma](
                tbb::blocked_range<int> const& range) {
                auto const kappa = gamma * dt;
                int const NX = a_slab.size()[0];
                int const NY = a_slab.size()[1];
                for (int j = range.begin(); j < range.end(); ++j) {
                    for (int i = 1; i < NX - 1; ++i) {
                        auto const a_nbhd_sum =
                            a_old_slab.value(i, j - 1) +  // down
                            a_old_slab.value(i - 1, j) +  // left
                            a_old_slab.value(i + 1, j) +  // right
                            a_old_slab.value(i, j + 1);   // up;

                        auto const h_nbhd_sum_lo =
                            h_slab.value(i, j - 1) +            // down
                            h_slab.value(i - 1, j);             // left
                        auto const h_cen = h_slab.value(i, j);  // center
                        auto const h_nbhd_sum_hi =
                            h_slab.value(i + 1, j) +  // right
                            h_slab.value(i, j + 1);   // up;

                        auto const b =
                            gamma *
                            ((h_nbhd_sum_lo + h_nbhd_sum_hi) - (4.0f * h_cen));
                        auto const c = kappa * a_nbhd_sum;

                        a_slab.value(i, j) = (b + c) / (1.0f + 4.0f * kappa);
                    }
                }

                enforce_dirichlet_boundary_conditions(a_slab);
            });
    };
}

Jacobi_solve_op make_jacobi_solve_accel(Binary_slab_op jacobi_iter) {
    return [jacobi_iter](Float_slab& into,
                         Float_slab& tmp,
                         Float_slab const& height,
                         float const dt) {
        // Initialize acceleration to zero
        into.fill(0.0f);

        // Solve from state_jacobi_tmp into StateAccel
        for (int iter2 = 0; iter2 < 10; ++iter2) {
            // solve into tmp
            jacobi_iter(tmp, into, height, dt);

            // tmp now contains the best guess at accel. solve back into
            // into
            jacobi_iter(into, tmp, height, dt);
        }
    };
}

void copy_from(Float_slab& into, Float_slab const& from) {
    std::copy(
        from.as_span().begin(), from.as_span().end(), into.as_span().begin());
}

void accumulate(Float_slab& into,
                Float_slab const& derivative,
                float const dt) {
    auto const* const derivative_data = derivative.data();
    auto* const into_data = into.data();
    auto const array_size = static_cast<int>(into.size()[0] * into.size()[1]);

    for (int i = 0; i < array_size; ++i) {
        into_data[i] += dt * derivative_data[i];
    }
}

#define DEFINE_LOCAL_LAMBDAS                                                   \
    auto swap_state = [&]() {                                                  \
        state.velocity.swap(state.velocity_prev);                              \
        state.height.swap(state.height_prev);                                  \
    };                                                                         \
                                                                               \
    auto estimate_height_star = [&](float const dt) {                          \
        copy_from(state.height_star, state.height_prev);                       \
        accumulate(state.height_star, state.velocity_star, dt);                \
        height_boundary(state.height_star);                                    \
    };                                                                         \
                                                                               \
    auto estimate_velocity_star = [&](float const dt) {                        \
        copy_from(state.velocity_star, state.velocity_prev);                   \
        accumulate(state.velocity_star, state.acceleration_star, dt);          \
        velocity_boundary(state.velocity_star);                                \
    };                                                                         \
                                                                               \
    auto estimate_acceleration_star = [&](float const dt) {                    \
        jacobi_solve_accel(                                                    \
            state.acceleration_star, state.jacobi_tmp, state.height_star, dt); \
    };                                                                         \
                                                                               \
    auto accumulate_estimate = [&](float const dt) {                           \
        accumulate(state.height, state.velocity_star, dt);                     \
        accumulate(state.velocity, state.acceleration_star, dt);               \
    };                                                                         \
                                                                               \
    auto enforce_boundary = [&]() {                                            \
        height_boundary(state.height);                                         \
        velocity_boundary(state.velocity);                                     \
    };

// First-Order Symplectic Time Step function.
Time_step_op make_time_step_first_order(
    Nullary_timeless_slab_op height_boundary,
    Nullary_timeless_slab_op velocity_boundary,
    Jacobi_solve_op jacobi_solve_accel) {
    return [=](State& state, float const dt) {
        DEFINE_LOCAL_LAMBDAS;

        // Swap the states.
        swap_state();

        // Initialize estimate. This just amounts to copying
        // The previous values into the current values.
        copy_from(state.velocity, state.velocity_prev);
        copy_from(state.height, state.height_prev);

        // 1
        copy_from(state.velocity_star, state.velocity);
        estimate_height_star(dt);
        estimate_acceleration_star(dt);
        accumulate_estimate(dt);

        // Final boundary conditions on height and vel
        enforce_boundary();
    };
}

Time_step_op make_time_step_rk2(Nullary_timeless_slab_op height_boundary,
                                Nullary_timeless_slab_op velocity_boundary,
                                Jacobi_solve_op jacobi_solve_accel) {
    return [=](State& state, float const dt) {
        DEFINE_LOCAL_LAMBDAS;

        // Swap the states.
        swap_state();

        // Initialize estimate. This just amounts to copying
        // The previous values into the current values.
        copy_from(state.velocity, state.velocity_prev);
        copy_from(state.height, state.height_prev);

        // 1
        copy_from(state.velocity_star, state.velocity);
        estimate_height_star(dt);
        estimate_acceleration_star(dt);
        accumulate_estimate(dt / 2.0f);

        // 2
        estimate_velocity_star(dt);
        estimate_height_star(dt);
        estimate_acceleration_star(dt);
        accumulate_estimate(dt / 2.0);

        // Final boundary conditions on height and vel
        enforce_boundary();
    };
}

// Fourth-Order Runge-Kutta Time Step function.
Time_step_op make_time_step_rk4(Nullary_timeless_slab_op height_boundary,
                                Nullary_timeless_slab_op velocity_boundary,
                                Jacobi_solve_op jacobi_solve_accel) {
    return [=](State& state, float const dt) {
        DEFINE_LOCAL_LAMBDAS;

        // Swap the states.
        {
            auto _ = Caliper{"A"};
            swap_state();

            // Initialize estimate. This just amounts to copying
            // The previous values into the current values.
            copy_from(state.velocity, state.velocity_prev);
            copy_from(state.height, state.height_prev);
        }

        {
            auto _ = Caliper{"B"};
            // 1
            {
                auto c2 = Caliper{"B.0"};
                copy_from(state.velocity_star, state.velocity);
            }
            {
                auto c2 = Caliper{"B.1"};
                estimate_height_star(dt);
            }
            {
                auto c2 = Caliper{"B.2"};
                estimate_acceleration_star(dt);
            }

            {
                auto c2 = Caliper{"B.3"};
                accumulate_estimate(dt / 6.0);
            }
        }

        {
            auto _ = Caliper{"C"};
            // 2
            estimate_velocity_star(dt / 2.0);
            estimate_height_star(dt / 2.0);
            estimate_acceleration_star(dt / 2.0);
            accumulate_estimate(dt / 3.0);
        }

        {
            auto _ = Caliper{"D"};
            // 3
            estimate_velocity_star(dt / 2.0);
            estimate_height_star(dt / 2.0);
            estimate_acceleration_star(dt / 2.0);
            accumulate_estimate(dt / 3.0);
        }

        {
            auto _ = Caliper{"E"};
            // 4
            estimate_velocity_star(dt);
            estimate_height_star(dt);
            estimate_acceleration_star(dt);
            accumulate_estimate(dt / 6.0);
        }

        {
            auto _ = Caliper{"F"};
            // Final boundary conditions on height and vel
            enforce_boundary();
        }
    };
}

Time_step_op make_time_step(Nullary_timeless_slab_op height_boundary,
                            Nullary_timeless_slab_op velocity_boundary,
                            Jacobi_solve_op jacobi_solve_accel,
                            Time_integration const time_integration) {
    switch (time_integration) {
    case Time_integration::FIRST_ORDER:
        return make_time_step_first_order(
            height_boundary, velocity_boundary, jacobi_solve_accel);
    case Time_integration::RUNGE_KUTTA_2:
        return make_time_step_rk2(
            height_boundary, velocity_boundary, jacobi_solve_accel);
    case Time_integration::RUNGE_KUTTA_4:
    default:
        return make_time_step_rk4(
            height_boundary, velocity_boundary, jacobi_solve_accel);
    }
}

#endif

}  // namespace emerald::shallow_weaver
