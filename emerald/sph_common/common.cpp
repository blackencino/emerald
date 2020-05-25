#include <emerald/sph_common/common.h>

#include <emerald/util/functions.h>
#include <emerald/util/safe_divide.h>
#include <emerald/z_index/z_index.h>

#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>

namespace emerald::sph_common {

using namespace emerald::util;

Box2f compute_bounds(size_t const particle_count, V2f const* const positions) {
    if (particle_count < 1) { return Box2f{}; }

    if constexpr (DO_PARALLEL) {
        return tbb::parallel_reduce(
          // Range for reduction
          tbb::blocked_range<V2f const*>{positions, positions + particle_count},
          // Identity element
          Box2f{},
          // Reduce a subrange and partial bounds
          [](tbb::blocked_range<V2f const*> const& range,
             Box2f partial_bounds) {
              for (auto const& p : range) { partial_bounds.extendBy(p); }
              return partial_bounds;
          },
          // Reduce two partial bounds
          [](Box2f bounds_a, Box2f const& bounds_b) {
              bounds_a.extendBy(bounds_b);
              return bounds_a;
          });
    } else {
        Box2f bounds;
        for (size_t i = 0; i < particle_count; ++i) {
            bounds.extendBy(positions[i]);
        }
        return bounds;
    }
}

V2i compute_grid_coord(V2f const p) {
    return {static_cast<int32_t>(std::floor(p[0])),
            static_cast<int32_t>(std::floor(p[1]))};
}

V2i compute_grid_coord(V2f const position, float const cell_size) {
    return compute_grid_coord(position / cell_size);
}

V2i compute_grid_coord(V2f const position,
                       V2f const bounds_min,
                       float const cell_size) {
    return compute_grid_coord(position - bounds_min, cell_size);
}

void compute_grid_coords(size_t const particle_count,
                         float const cell_size,
                         V2f const bounds_min,
                         V2i* const grid_coords,
                         V2f const* const positions) {
    for_each_iota(particle_count, [=](auto const i) {
        grid_coords[i] =
          compute_grid_coord(positions[i], bounds_min, cell_size);
    });
}

void compute_z_indices(size_t const particle_count,
                       uint64_t* const z_indices,
                       V2i const* const grid_coords) {
    for_each_iota(particle_count, [=](auto const i) {
        auto const [x, y] = grid_coords[i];
        z_indices[i] = z_index::z_index(x, y);
    });
}

float max_density_error(size_t const count,
                        float const target_density,
                        float const* const densities) {
    return max_element(
      count, 0.0f, densities, [target_density](float const density) -> float {
          return std::max(0.0f, density - target_density);
      });
}

float max_vector_squared_magnitude(size_t const count,
                                   V2f const* const vectors) {
    return max_element(count, 0.0f, vectors, [](V2f const vec) -> float {
        return vec.dot(vec);
    });
}

float average_value(size_t const particle_count,
                    int64_t const discretization,
                    float const normalized_value,
                    float const* const values) {
    return average_func_value(particle_count,
                              discretization,
                              normalized_value,
                              values,
                              [](float const f) { return f; });
}

float average_abs_value(size_t const particle_count,
                        int64_t const discretization,
                        float const normalized_value,
                        float const* const values) {
    return average_func_value(particle_count,
                              discretization,
                              normalized_value,
                              values,
                              [](float const f) { return std::abs(f); });
}

}  // namespace emerald::sph_common
