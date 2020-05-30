#include <emerald/sph_common/common.h>
#include <emerald/sph_common/dynamics.h>
#include <emerald/sph_common/neighborhood.h>

#include <emerald/util/format.h>
#include <fmt/format.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <random>
#include <vector>

namespace emerald::sph_common {

using namespace emerald::util;

struct Particle_set {
    Particle_set()
      : Particle_set(17272,
                     79270,
                     std::uniform_real_distribution<float>{-100.0f, 100.0f},
                     std::uniform_real_distribution<float>{-10.0f, 10.0f},
                     std::uniform_real_distribution<float>{-1.0f, 1.0f},
                     std::uniform_real_distribution<float>{0.0f, 1.0e6f},
                     2.1f) {
    }
    Particle_set(uint64_t const seed,
                 size_t const count,
                 std::uniform_real_distribution<float> pos_dist,
                 std::uniform_real_distribution<float> vel_dist,
                 std::uniform_real_distribution<float> force_dist,
                 std::uniform_real_distribution<float> inv_mass_dist,
                 float const cell_size) {
        positions.resize(count);
        velocities.resize(count);
        forces.resize(count);
        inv_masses.resize(count);
        new_velocities.resize(count);
        new_positions.resize(count);

        std::mt19937_64 gen{seed};
        for (auto& [x, y] : positions) {
            x = pos_dist(gen);
            y = pos_dist(gen);
        }

        for (auto& [x, y] : velocities) {
            x = vel_dist(gen);
            y = vel_dist(gen);
        }

        for (auto& [x, y] : forces) {
            x = force_dist(gen);
            y = force_dist(gen);
        }

        for (auto& inv_mass : inv_masses) { inv_mass = inv_mass_dist(gen); }

        V2f const bounds_min{0.0f, 0.0f};
        grid_coords.resize(count);
        compute_grid_coords(
          count, cell_size, bounds_min, grid_coords.data(), positions.data());

        z_indices.resize(count);
        compute_z_indices(count, z_indices.data(), grid_coords.data());

        index_pairs.resize(count);
        compute_index_pairs(count, index_pairs.data(), z_indices.data());
        sort_index_pairs(count, index_pairs.data());

        block_indices.resize(count);
        compute_block_indices(count, block_indices.data(), index_pairs.data());

        size_t const block_count = block_indices.back() + 1;
        blocks.resize(block_count);
        fill_blocks(count, blocks.data(), block_indices.data());

        block_map = create_block_map(
          block_count, std::move(block_map), blocks.data(), index_pairs.data());

        neighborhood.resize(count);
        create_regular_neighborhoods(count,
                                     cell_size,
                                     neighborhood.counts.data(),
                                     neighborhood.indices.data(),
                                     neighborhood.distances.data(),
                                     neighborhood.vectors_to.data(),
                                     positions.data(),
                                     grid_coords.data(),
                                     positions.data(),
                                     index_pairs.data(),
                                     block_map);
        compute_neighbor_kernels(count,
                                 cell_size / 2.0f,
                                 neighborhood.kernels.data(),
                                 neighborhood.counts.data(),
                                 neighborhood.distances.data());
        compute_neighbor_kernel_gradients(count,
                                          cell_size / 2.0f,
                                          neighborhood.kernel_gradients.data(),
                                          neighborhood.counts.data(),
                                          neighborhood.vectors_to.data());
    }

    std::vector<V2f> positions;
    std::vector<V2f> velocities;
    std::vector<V2f> forces;
    std::vector<float> inv_masses;
    std::vector<V2f> new_velocities;
    std::vector<V2f> new_positions;

    std::vector<V2i> grid_coords;
    std::vector<uint64_t> z_indices;
    std::vector<std::pair<uint64_t, size_t>> index_pairs;
    std::vector<uint32_t> block_indices;
    std::vector<std::pair<size_t, size_t>> blocks;
    Block_map block_map;
    Neighborhood_vectors neighborhood;
};

struct Performance_test : public ::testing::Test {
    Performance_test()
      : particles()
      , solid_particles(54321,
                        50000,
                        std::uniform_real_distribution<float>{-100.0f, 100.0f},
                        std::uniform_real_distribution<float>{-10.0f, 10.0f},
                        std::uniform_real_distribution<float>{-1.0f, 1.0f},
                        std::uniform_real_distribution<float>{0.0f, 1.0e6f},
                        2.1f) {
        auto const count = particles.positions.size();
        float const cell_size = 2.1f;

        solid_neighborhood.resize(count);
        create_regular_neighborhoods(count,
                                     cell_size,
                                     solid_neighborhood.counts.data(),
                                     solid_neighborhood.indices.data(),
                                     solid_neighborhood.distances.data(),
                                     solid_neighborhood.vectors_to.data(),
                                     particles.positions.data(),
                                     particles.grid_coords.data(),
                                     solid_particles.positions.data(),
                                     solid_particles.index_pairs.data(),
                                     solid_particles.block_map);
        compute_neighbor_kernels(count,
                                 cell_size / 2.0f,
                                 solid_neighborhood.kernels.data(),
                                 solid_neighborhood.counts.data(),
                                 solid_neighborhood.distances.data());
        compute_neighbor_kernel_gradients(
          count,
          cell_size / 2.0f,
          solid_neighborhood.kernel_gradients.data(),
          solid_neighborhood.counts.data(),
          solid_neighborhood.vectors_to.data());
    }

    Particle_set particles;
    Particle_set solid_particles;

    Neighborhood_vectors solid_neighborhood;
};

TEST_F(Performance_test, Non_separated) {
    auto const count = particles.positions.size();

    auto const start = std::chrono::high_resolution_clock::now();
    auto const* const pos_data = particles.positions.data();
    auto const* const vel_data = particles.velocities.data();
    auto const* const inv_mass_data = particles.inv_masses.data();
    auto const* const force_data = particles.forces.data();
    auto* const new_pos_data = particles.new_positions.data();
    float const dt = 1.0f / 24.0f;
    for (int i = 0; i < 10000; ++i) {
        for_each_iota(count, [=](auto const particle_index) {
            new_pos_data[particle_index] =
              pos_data[particle_index] +
              dt *
                (vel_data[particle_index] + dt * inv_mass_data[particle_index] *
                                              force_data[particle_index]);
        });
    }
    auto const end = std::chrono::high_resolution_clock::now();

    fmt::print("Non-separated: {}\n",
               std::chrono::duration<double>{end - start}.count());
}

TEST_F(Performance_test, Separated) {
    auto const count = particles.positions.size();

    auto const start = std::chrono::high_resolution_clock::now();
    auto const* const pos_data = particles.positions.data();
    auto const* const vel_data = particles.velocities.data();
    auto const* const inv_mass_data = particles.inv_masses.data();
    auto const* const force_data = particles.forces.data();
    auto* const new_pos_data = particles.new_positions.data();
    auto* const new_vel_data = particles.new_velocities.data();
    float const dt = 1.0f / 24.0f;
    for (int i = 0; i < 10000; ++i) {
        copy_array(count, new_vel_data, force_data);
        for_each_iota(count, [=](auto const particle_index) {
            new_vel_data[particle_index] *= dt * inv_mass_data[particle_index];
        });
        for_each_iota(count, [=](auto const particle_index) {
            new_vel_data[particle_index] += vel_data[particle_index];
        });
        copy_array(count, new_pos_data, pos_data);
        accumulate(count, dt, new_pos_data, new_vel_data);
    }
    auto const end = std::chrono::high_resolution_clock::now();

    fmt::print("Separated: {}\n",
               std::chrono::duration<double>{end - start}.count());
}

void density_non_separated(size_t const count,
                           float* const densities,
                           Neighborhood_pointers const neighbors,
                           Neighborhood_pointers const solid_neighbors) {
    for_each_iota(count, [=](auto const particle_index) {
        float density = 0.0f;
        auto const nbhd_count = neighbors.counts[particle_index];
        if (nbhd_count) {
            auto const& nbhd_kernels = neighbors.kernels[particle_index];
            for (uint8_t j = 0; j < nbhd_count; ++j) {
                density += nbhd_kernels[j];
            }
        }

        auto const solid_nbhd_count = solid_neighbors.counts[particle_index];
        if (solid_nbhd_count) {
            auto const& solid_nbhd_kernels =
              solid_neighbors.kernels[particle_index];
            for (uint8_t j = 0; j < solid_nbhd_count; ++j) {
                density += solid_nbhd_kernels[j];
            }
        }

        densities[particle_index] = density;
    });
}

TEST_F(Performance_test, Density_non_separated) {
    auto const count = particles.positions.size();

    std::vector<float> densities;
    densities.resize(count);

    auto const start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 1000; ++i) {
        density_non_separated(count,
                              densities.data(),
                              particles.neighborhood.pointers(),
                              solid_neighborhood.pointers());
    }
    auto const end = std::chrono::high_resolution_clock::now();

    fmt::print("Density non-separated: {}\n",
               std::chrono::duration<double>{end - start}.count());
}

void density_separated_partial(bool const accumulate,
                               size_t const count,
                               float* const densities,
                               Neighborhood_pointers const neighborhood) {
    for_each_iota(count, [=](auto const particle_index) {
        float density = 0.0f;
        auto const nbhd_count = neighborhood.counts[particle_index];
        if (nbhd_count) {
            auto const& nbhd_kernels = neighborhood.kernels[particle_index];
            for (uint8_t j = 0; j < nbhd_count; ++j) {
                density += nbhd_kernels[j];
            }
        }

        if (accumulate) {
            densities[particle_index] += density;
        } else {
            densities[particle_index] = density;
        }
    });
}

void density_separated(size_t const count,
                       float* const densities,
                       Neighborhood_pointers const neighborhood,
                       Neighborhood_pointers const solid_neighborhood) {
    density_separated_partial(false, count, densities, neighborhood);
    density_separated_partial(true, count, densities, solid_neighborhood);
}

TEST_F(Performance_test, Density_separated) {
    auto const count = particles.positions.size();

    std::vector<float> densities;
    densities.resize(count);

    auto const start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 1000; ++i) {
        density_separated(count,
                          densities.data(),
                          particles.neighborhood.pointers(),
                          solid_neighborhood.pointers());
    }
    auto const end = std::chrono::high_resolution_clock::now();

    fmt::print("Density separated: {}\n",
               std::chrono::duration<double>{end - start}.count());
}

void divergence_non_separated(size_t const count,
                              float const mass_per_particle,
                              float* const divergences,

                              V2f const* const velocities,
                              Neighborhood_pointers const neighborhood,

                              V2f const* const solid_velocities,
                              Neighborhood_pointers const solid_neighborhood) {
    for_each_iota(count, [=](auto const particle_index) {
        float divergence = 0.0f;

        auto const velocity = velocities[particle_index];

        auto const nbhd_count = neighborhood.counts[particle_index];
        if (nbhd_count) {
            auto const& nbhd_indices = neighborhood.indices[particle_index];
            auto const& nbhd_kernel_gradients =
              neighborhood.kernel_gradients[particle_index];
            for (uint8_t j = 0; j < nbhd_count; ++j) {
                auto const other_particle_index = nbhd_indices[j];
                divergence += mass_per_particle *
                              ((velocity - velocities[other_particle_index])
                                 .dot(nbhd_kernel_gradients[j]));
            }
        }

        auto const solid_nbhd_count = solid_neighborhood.counts[particle_index];
        if (solid_nbhd_count) {
            auto const& nbhd_indices =
              solid_neighborhood.indices[particle_index];
            auto const& nbhd_kernel_gradients =
              solid_neighborhood.kernel_gradients[particle_index];
            for (uint8_t j = 0; j < solid_nbhd_count; ++j) {
                auto const other_particle_index = nbhd_indices[j];
                divergence +=
                  mass_per_particle *
                  ((velocity - solid_velocities[other_particle_index])
                     .dot(nbhd_kernel_gradients[j]));
            }
        }

        divergences[particle_index] = divergence;
    });
}

TEST_F(Performance_test, Divergence_non_separated) {
    auto const count = particles.positions.size();

    std::vector<float> divergences;
    divergences.resize(count);

    auto const start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 100; ++i) {
        divergence_non_separated(count,

                                 1.0f,
                                 divergences.data(),

                                 particles.velocities.data(),
                                 particles.neighborhood.pointers(),

                                 solid_particles.velocities.data(),
                                 solid_neighborhood.pointers());
    }
    auto const end = std::chrono::high_resolution_clock::now();

    fmt::print("Divergence non-separated: {}\n",
               std::chrono::duration<double>{end - start}.count());
}

void divergence_separated_part(bool const accumulate,
                               size_t const count,
                               float const mass_per_particle,
                               float* const divergences,

                               V2f const* const velocities,
                               V2f const* const other_velocities,
                               Neighborhood_pointers const neighborhood) {
    for_each_iota(count, [=](auto const particle_index) {
        float divergence = 0.0f;

        auto const velocity = velocities[particle_index];

        auto const nbhd_count = neighborhood.counts[particle_index];
        if (nbhd_count) {
            auto const& nbhd_indices = neighborhood.indices[particle_index];
            auto const& nbhd_kernel_gradients =
              neighborhood.kernel_gradients[particle_index];
            for (uint8_t j = 0; j < nbhd_count; ++j) {
                auto const other_particle_index = nbhd_indices[j];
                divergence +=
                  mass_per_particle *
                  ((velocity - other_velocities[other_particle_index])
                     .dot(nbhd_kernel_gradients[j]));
            }
        }

        if (accumulate) {
            divergences[particle_index] += divergence;
        } else {
            divergences[particle_index] = divergence;
        }
    });
}

void divergence_separated(size_t const count,
                          float const mass_per_particle,
                          float* const divergences,

                          V2f const* const velocities,
                          Neighborhood_pointers const neighborhood,

                          V2f const* const solid_velocities,
                          Neighborhood_pointers const solid_neighborhood) {
    divergence_separated_part(false,
                              count,
                              mass_per_particle,
                              divergences,
                              velocities,
                              velocities,
                              neighborhood);
    divergence_separated_part(true,
                              count,
                              mass_per_particle,
                              divergences,
                              velocities,
                              solid_velocities,
                              solid_neighborhood);
}

TEST_F(Performance_test, Divergence_separated) {
    auto const count = particles.positions.size();

    std::vector<float> divergences;
    divergences.resize(count);

    auto const start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 100; ++i) {
        divergence_separated(count,

                             1.0f,
                             divergences.data(),

                             particles.velocities.data(),
                             particles.neighborhood.pointers(),

                             solid_particles.velocities.data(),
                             solid_neighborhood.pointers());
    }
    auto const end = std::chrono::high_resolution_clock::now();

    fmt::print("Divergence separated: {}\n",
               std::chrono::duration<double>{end - start}.count());
}

TEST_F(Performance_test, Copy_parallel) {
    auto const count = particles.positions.size();

    auto const start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 10000; ++i) {
        copy_array<V2f, true>(
          count, particles.new_positions.data(), particles.positions.data());
    }
    auto const end = std::chrono::high_resolution_clock::now();

    fmt::print("Copy parallel: {}\n",
               std::chrono::duration<double>{end - start}.count());
}

TEST_F(Performance_test, Copy_serial) {
    auto const count = particles.positions.size();

    auto const start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 10000; ++i) {
        copy_array<V2f, false>(
          count, particles.new_positions.data(), particles.positions.data());
    }
    auto const end = std::chrono::high_resolution_clock::now();

    fmt::print("Copy serial: {}\n",
               std::chrono::duration<double>{end - start}.count());
}

TEST_F(Performance_test, Copy_parallel_std) {
    auto const count = particles.positions.size();

    auto const start = std::chrono::high_resolution_clock::now();
    auto const* const pos_data = particles.positions.data();
    auto* const new_pos_data = particles.new_positions.data();
    constexpr size_t grainsize = 512;
    for (int i = 0; i < 10000; ++i) {
        tbb::parallel_for(
          tbb::blocked_range<size_t>{size_t{0}, count, grainsize},
          [=](tbb::blocked_range<size_t> const& range) {
              std::copy(pos_data + range.begin(),
                        pos_data + range.end(),
                        new_pos_data + range.begin());
          });
    }
    auto const end = std::chrono::high_resolution_clock::now();

    fmt::print("Copy parallel std: {}\n",
               std::chrono::duration<double>{end - start}.count());
}

}  // namespace emerald::sph_common