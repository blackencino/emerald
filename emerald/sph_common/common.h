#pragma once

#include <emerald/sph_common/types.h>
#include <emerald/util/safe_divide.h>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <utility>

namespace emerald::sph_common {

using namespace emerald::util;

// HACK
constexpr bool DO_PARALLEL = true;

//------------------------------------------------------------------------------
// Generics
template <typename Function>
void for_each_iota(size_t const count,
                   Function&& func,
                   size_t const grainsize = 512) {
    if constexpr (DO_PARALLEL) {
        tbb::parallel_for(
          tbb::blocked_range<size_t>{size_t{0}, count, grainsize},
          [func = std::forward<Function>(func)](
            tbb::blocked_range<size_t> const& range) {
              for (size_t i = range.begin(); i != range.end(); ++i) { func(i); }
          });
    } else {
        for (size_t i = 0; i < count; ++i) { func(i); }
    }
}

template <typename T, bool PARALLEL = DO_PARALLEL>
void fill_array(size_t const count, T const value, T* const values) {
    if (!count) { return; }

    if constexpr (PARALLEL) {
        // for_each_iota(count, [=](auto const i) { values[i] = value; });
        constexpr size_t grainsize = 512;
        tbb::parallel_for(
          tbb::blocked_range<size_t>{size_t{0}, count, grainsize},
          [=](tbb::blocked_range<size_t> const& range) {
              std::fill(values + range.begin(), values + range.end(), value);
          });
    } else {
        std::fill(values, values + count, value);
    }
}

template <typename T, bool PARALLEL = DO_PARALLEL>
void copy_array(size_t const count, T* const dst, T const* const src) {
    if (!count || dst == src) { return; }

    if constexpr (PARALLEL) {
        // for_each_iota(count, [=](auto const i) { dst[i] = src[i]; });
        constexpr size_t grainsize = 512;
        tbb::parallel_for(
          tbb::blocked_range<size_t>{size_t{0}, count, grainsize},
          [=](tbb::blocked_range<size_t> const& range) {
              std::copy(
                src + range.begin(), src + range.end(), dst + range.begin());
          });
    } else {
        std::copy(src, src + count, dst);
    }
}

template <typename Value, typename Multiplier>
void accumulate(size_t const count,
                Multiplier const mult,
                Value* const dst,
                Value const* const src) {
    for_each_iota(count, [=](auto const i) { dst[i] += mult * src[i]; });
}

template <typename Value>
void accumulate(size_t const count, Value* const dst, Value const* const src) {
    for_each_iota(count, [=](auto const i) { dst[i] += src[i]; });
}

//------------------------------------------------------------------------------
Box2f compute_bounds(size_t const particle_count, V2f const* const positions);

// origin is at 0, cell size is 1
V2i compute_grid_coord(V2f const p);

// origin is at 0, cell size given
V2i compute_grid_coord(V2f const p, float const cell_size);

// origin and cell size given
V2i compute_grid_coord(V2f const p,
                       V2f const bounds_min,
                       float const cell_size);

void compute_grid_coords(size_t const particle_count,
                         float const cell_size,
                         V2f const bounds_min,
                         V2i* const grid_coords,
                         V2f const* const positions);

void compute_z_indices(size_t const particle_count,
                       uint64_t* const z_indices,
                       V2i const* const grid_coords);

//------------------------------------------------------------------------------
template <typename Return_type, typename Value_type, typename Modifier>
Return_type max_element(size_t const element_count,
                        Return_type const initial_value,
                        Value_type const* const values,
                        Modifier&& modifier) {
    if constexpr (DO_PARALLEL) {
        return tbb::parallel_reduce(
          tbb::blocked_range<Value_type const*>{values, values + element_count},
          initial_value,
          [modifier = std::forward<Modifier>(modifier)](
            tbb::blocked_range<Value_type const*> const& range,
            Return_type const value) -> Return_type {
              Return_type max_value = value;
              for (auto const input : range) {
                  max_value = std::max(max_value, modifier(input));
              }
              return max_value;
          },
          [](Return_type const a, Return_type const b) -> Return_type {
              return std::max(a, b);
          });
    } else {
        Return_type max_value{0};
        for (size_t i = 0; i < element_count; ++i) {
            max_value = std::max(max_value, modifier(values[i]));
        }
        return max_value;
    }
}

float max_density_error(size_t const particle_count,
                        float const target_density,
                        float const* const densities);

float max_vector_squared_magnitude(size_t const count,
                                   V2f const* const vectors);

float average_value(size_t const particle_count,
                    int64_t const discretization,
                    float const normalized_value,
                    float const* const values);

template <typename T, typename Function>
float average_func_value(size_t const particle_count,
                         int64_t const discretization,
                         float const normalized_value,
                         T const* const values,
                         Function&& func) {
    if (!particle_count) { return 0.0f; }

    int64_t discretized_sum = 0;
    auto const discretization_f = static_cast<float>(discretization);
    if constexpr (DO_PARALLEL) {
        discretized_sum = tbb::parallel_reduce(
          tbb::blocked_range<T const*>{values, values + particle_count},
          discretized_sum,
          [normalized_value,
           discretization_f,
           func = std::forward<Function>(func)](
            tbb::blocked_range<T const*> const& range,
            int64_t const incoming_sum) -> int64_t {
              int64_t sum = incoming_sum;
              for (T const& value : range) {
                  sum += static_cast<int64_t>((func(value) / normalized_value) *
                                              discretization_f);
              }
              return sum;
          },
          [](int64_t const a, int64_t const b) -> int64_t { return a + b; });
    } else {
        for (size_t i = 0; i < particle_count; ++i) {
            discretized_sum += static_cast<int64_t>(
              (func(values[i]) / normalized_value) * discretization_f);
        }
    }

    double const numer = static_cast<double>(discretized_sum) *
                         static_cast<double>(normalized_value);
    double const denom =
      static_cast<double>(particle_count) * static_cast<double>(discretization);

    if (is_safe_divide(numer, denom)) {
        return static_cast<float>(numer / denom);
    } else {
        return 0.0f;
    }
}

//------------------------------------------------------------------------------
template <typename T>
inline void update_atomic_max(std::atomic<T>& atom, T const val) {
    for (T atom_val = atom;
         atom_val < val && !atom.compare_exchange_weak(
                             atom_val, val, std::memory_order_relaxed);) {}
}

template <typename InputValue, typename TransformFunction>
std::pair<float, float> compute_transformed_scalar_average_and_max(
  size_t const particle_count,
  float const discretization,
  InputValue const* const input_values,
  TransformFunction&& trans_func) {
    if (particle_count < 1) { return {0.0f, 0.0f}; }

    std::atomic<int64_t> integral_value_sum = 0;
    std::atomic<float> value_max = std::numeric_limits<float>::lowest();

    for_each_iota(
      particle_count,
      [=, &integral_value_sum, &value_max](auto const particle_index) {
          auto const value = trans_func(input_values[particle_index]);
          update_atomic_max(value_max, value);
          integral_value_sum += static_cast<int64_t>(value * discretization);
      });

    auto const value_average_numer = static_cast<double>(integral_value_sum);
    auto const value_average_denom = static_cast<double>(1000 * particle_count);
    return {static_cast<float>(value_average_numer / value_average_denom),
            value_max};
}

inline std::pair<float, float> compute_scalar_average_and_max(
  size_t const particle_count,
  float const discretization,
  float const* const values) {
    return compute_transformed_scalar_average_and_max(
      particle_count, discretization, values, [](float const f) { return f; });
}

}  // namespace emerald::sph_common
