#pragma once

#include <emerald/shallow_weaver/foundation.h>
#include <emerald/shallow_weaver/slab.h>
#include <emerald/shallow_weaver/state.h>

#include <tbb/parallel_for.h>

#include <functional>
#include <optional>
#include <tuple>
#include <utility>

namespace emerald::shallow_weaver {

#if 0

using Nullary_timeless_slab_op = std::function<Float_slab(Float_slab&&)>;
using Nullary_slab_op = std::function<Float_slab(Float_slab&&, float const)>;
using Unary_slab_op =
    std::function<Float_slab(Float_slab&&, Float_slab const&, float const)>;
using Binary_slab_op = std::function<Float_slab(
    Float_slab&&, Float_slab const&, Float_slab const&, float const)>;
using Jacobi_solve_op = std::function<std::tuple<Float_slab, Float_slab>(
    Float_slab&&, Float_slab&&, Float_slab const&, float const)>;

using Time_step_op = std::function<State(State&&, float const)>;

Nullary_timeless_slab_op make_enforce_dirichlet_boundary_conditions();

Nullary_timeless_slab_op make_enforce_neumann_boundary_conditions();

Nullary_timeless_slab_op make_enforce_height_boundary_conditions(
    std::optional<std::pair<int2, float>> const input);

// Jacobi iteration to get temp acceleration
Binary_slab_op make_jacobi_iteration_accel(float const wave_speed,
                                           float const DXY);

Jacobi_solve_op make_jacobi_solve_accel(Binary_slab_op jacobi_iter);

Unary_slab_op make_accumulate();

// First-Order Symplectic Time Step function.
Time_step_op make_time_step_first_order(
    Nullary_timeless_slab_op height_boundary,
    Nullary_timeless_slab_op velocity_boundary,
    Unary_slab_op accumulate,
    Jacobi_solve_op jacobi_solve_accel);

Time_step_op make_time_step_rk2(Nullary_timeless_slab_op height_boundary,
                                Nullary_timeless_slab_op velocity_boundary,
                                Unary_slab_op accumulate,
                                Jacobi_solve_op jacobi_solve_accel);

// Fourth-Order Runge-Kutta Time Step function.
Time_step_op make_time_step_rk4(Nullary_timeless_slab_op height_boundary,
                                Nullary_timeless_slab_op velocity_boundary,
                                Unary_slab_op accumulate,
                                Jacobi_solve_op jacobi_solve_accel);

Time_step_op make_time_step(Nullary_timeless_slab_op height_boundary,
                            Nullary_timeless_slab_op velocity_boundary,
                            Unary_slab_op accumulate,
                            Jacobi_solve_op jacobi_solve_accel,
                            Time_integration const time_integration);

#else

using Nullary_timeless_slab_op = std::function<void(Float_slab&)>;
using Nullary_slab_op = std::function<void(Float_slab&, float const)>;
using Unary_slab_op =
    std::function<void(Float_slab&, Float_slab const&, float const)>;
using Binary_slab_op = std::function<void(
    Float_slab&, Float_slab const&, Float_slab const&, float const)>;
using Jacobi_solve_op = std::function<void(
    Float_slab&, Float_slab&, Float_slab const&, float const)>;

using Time_step_op = std::function<void(State&, float const)>;

void enforce_dirichlet_boundary_conditions(Float_slab& slab);
void enforce_neumann_boundary_conditions(Float_slab& v);
void copy_from(Float_slab& into, Float_slab const& from);
void accumulate(Float_slab& into, Float_slab const& derivative, float const dt);

Nullary_timeless_slab_op make_enforce_height_boundary_conditions(
    std::optional<std::pair<int2, float> > const input);

// Jacobi iteration to get temp acceleration
Binary_slab_op make_jacobi_iteration_accel(float const wave_speed,
                                           float const DXY);

Jacobi_solve_op make_jacobi_solve_accel(Binary_slab_op jacobi_iter);

// First-Order Symplectic Time Step function.
Time_step_op make_time_step_first_order(
    Nullary_timeless_slab_op height_boundary,
    Nullary_timeless_slab_op velocity_boundary,
    Jacobi_solve_op jacobi_solve_accel);

Time_step_op make_time_step_rk2(Nullary_timeless_slab_op height_boundary,
                                Nullary_timeless_slab_op velocity_boundary,
                                Jacobi_solve_op jacobi_solve_accel);

// Fourth-Order Runge-Kutta Time Step function.
Time_step_op make_time_step_rk4(Nullary_timeless_slab_op height_boundary,
                                Nullary_timeless_slab_op velocity_boundary,
                                Jacobi_solve_op jacobi_solve_accel);

Time_step_op make_time_step(Nullary_timeless_slab_op height_boundary,
                            Nullary_timeless_slab_op velocity_boundary,
                            Jacobi_solve_op jacobi_solve_accel,
                            Time_integration const time_integration);

#endif

//------------------------------------------------------------------------------
// parallel slab operations

template <typename T, typename Function>
void do_slab_op_1d(Slab<T>& slab, Function&& func) {
    tbb::parallel_for(
        tbb::blocked_range<int>{0, slab.size()[0] * slab.size()[1]},
        [&slab, func = std::forward<Function>(func)](
            tbb::blocked_range<int> const& range) {
            for (int i = range.begin(); i < range.end(); ++i) { func(slab, i); }
        });
}

template <typename T, typename Function>
void do_slab_op_1d_range(Slab<T>& slab, Function&& func) {
    tbb::parallel_for(
        tbb::blocked_range<int>{0, slab.size()[0] * slab.size()[1]},
        [&slab, func = std::forward<Function>(func)](
            tbb::blocked_range<int> const& range) {
            func(slab, range.begin(), range.end());
        });
}

template <typename T, typename Function>
void do_slab_op_2d(Slab<T>& slab, Function&& func, int const border) {
    tbb::parallel_for(tbb::blocked_range<int>{border, slab.size()[1] - border},
                      [&slab, func = std::forward<Function>(func), border](
                          tbb::blocked_range<int> const& range) {
                          for (int j = range.begin(); j < range.end(); ++j) {
                              int index = border + (slab.size()[0] * j);
                              for (int i = border; i < slab.size()[1] - border;
                                   ++i, ++index) {
                                  func(slab, i, j, index);
                              }
                          }
                      });
}

////////////////////
// These ones don't take a slab, just a size. The function is assumed to bind
// all the arrays it uses.

template <typename Function>
void do_slab_op_1d(int2 const size, Function&& func) {
    tbb::parallel_for(int{0}, size[0] * size[1], std::forward<Function>(func));
}

template <typename Function>
void do_slab_op_1d_range(int2 const size, Function&& func) {
    tbb::parallel_for(tbb::blocked_range<int>{0, size[0] * size[1]},
                      [func = std::forward<Function>(func)](
                          tbb::blocked_range<int> const& range) {
                          func(range.begin(), range.end());
                      });
}

template <typename Function>
void do_slab_op_2d(int2 const size, Function&& func, int const border) {
    tbb::parallel_for(tbb::blocked_range<int>{border, size[1] - border},
                      [func = std::forward<Function>(func), size, border](
                          tbb::blocked_range<int> const& range) {
                          for (int j = range.begin(); j < range.end(); ++j) {
                              int index = border + (size[0] * j);
                              for (int i = border; i < size[1] - border;
                                   ++i, ++index) {
                                  func(i, j, index);
                              }
                          }
                      });
}

}  // namespace emerald::shallow_weaver
