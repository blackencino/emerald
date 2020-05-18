//------------------------------------------------------------------------------
// It's going to work. :)
// It's just big. Don't let it get overwhelming.

static constexpr uint8_t NEIGHBORHOOD_MAX_COUNT = 64;

template <typename T>
using Neighbor_values = std::array<T, NEIGHBORHOOD_MAX_COUNT>;

void compute_particle_body_indices(
  size_t const body_count,
  size_t* const particle_body_indices,
  std::pair<size_t, size_t> const* const body_particle_index_ranges) {
    for_each_iota(body_count, [=](auto const body_i) {
        auto const body_particle_index_range =
          body_particle_index_ranges[body_i];
        for (size_t part_i = body_particle_index_range.first;
             part_i != body_particle_index_range.second;
             ++part_i) {
            particle_body_indices[part_i] = body_i;
        }
    });
}

void compute_body_centroids_and_masses(
  size_t const body_count,
  V2f* const body_centroids,
  float* const body_masses,
  std::pair<size_t, size_t> const* const body_particle_index_ranges,
  V2f const* const particle_positions,
  float const* const particle_masses) {
    for_each_iota(body_count, [=](auto const body_i) {
        auto const body_particle_index_range =
          body_particle_index_ranges[body_i];
        V2f body_centroid{0.0f, 0.0f};
        float body_mass = 0.0f;

        for (size_t part_i = body_particle_index_range.first;
             part_i != body_particle_index_range.second;
             ++part_i) {
            auto const mass_i = particle_masses[part_i];
            auto const pos_i = particle_positions[part_i];

            body_centroid += mass_i * pos_i;
            body_mass += mass_i;
        }

        if (is_safe_divide(body_centroid, body_mass)) {
            body_centroid /= body_mass;
        }

        body_centroids[body_i] = body_centroid;
        body_masses[body_i] = body_mass;
    });
}

void compute_particle_centroid_offsets(
  size_t const body_count,
  V2f* const particle_centroid_offsets,
  V2f const* const body_centroids,
  std::pair<size_t, size_t> const* const body_particle_index_ranges,
  V2f const* const particle_positions) {
    for_each_iota(body_count, [=](auto const body_i) {
        auto const body_particle_index_range =
          body_particle_index_ranges[body_i];
        auto const body_centroid = body_centroids[body_i];

        for (size_t part_i = body_particle_index_range.first;
             part_i != body_particle_index_range.second;
             ++part_i) {
            particle_centroid_offset[part_i] =
              particle_positions[part_i] - body_centroid;
        }
    });
}

void compute_body_inertial_moments(
  size_t const body_count,
  M33f* const body_inertial_moments,
  std::pair<size_t, size_t> const* const body_particle_index_ranges,
  V2f const* const particle_centroid_offsets,
  float const* const particle_masses,
  float const* const particle_radii) {
    for_each_iota(body_count, [=](auto const body_i) {
        auto const body_particle_index_range =
          body_particle_index_ranges[body_i];

        M33ff I_body{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

        for (size_t part_i = body_particle_index_range.first;
             part_i != body_particle_index_range.second;
             ++part_i) {
            // We use the parallel axis theorem to compute the
            // inertial tensor of a displaced sphere.
            // the Inertial tensor of a sphere is 2/5 m r^2 on the
            // diagonal of each of the x, y, z axes.
            // the parallel axis theorem says that the inertial tensor
            // displaced by vector d is Isphere - M (cross_product_matrix(d)^2)
            // the negative square of the cross-product matrix is this:
            //
            // y^2 + z^2     -xy     -xz
            // -xy     x2 + z^2      -yz
            // -xz     -yz     x^2 + y^2
            //
            // since our displacement d is zero in z,
            //
            // y^2       -xy      0
            // -xy       x^2      0
            // 0          0     x^2 + y^2
            //
            // the whole thing is
            // m *    2/5 r^2 + y^2       -xy      0
            //        -xy       2/5 r^2 + x^2      0
            //        0         0       2/5 r^2 + x^2 + y^2

            auto const mass_i = particle_masses[part_i];
            auto const offset_i = particle_centroid_offsets[part_i];
            auto const radius_i = particle_radii[part_i];

            auto const sphere_diagonal = ((2 * mass_i) * sqr(radius_i)) / 5;
            auto const dx_sqr = sqr(offset_i[0]);
            auto const dy_sqr = sqr(offset_i[1]);

            I_body[0][0] += sphere_diagonal + mass_i * dy_sqr;
            I_body[1][1] += sphere_diagonal + mass_i * dx_sqr;
            I_body[2][2] += sphere_diagonal + mass_i * (dx_sqr + dy_sqr);

            I_body[0][1] += -mass_i * offset_i[0] * offset_i[1];
        }

        // Result is symmetric.

        I_body[1][0] = I_body[0][1];
        I_body[2][0] = I_body[0][2];
        I_body[2][1] = I_body[1][2];

        body_inertial_moments[body_i] = I_body;
    });
}

void compute_body_inverse_inertial_moments(
  size_t const body_count,
  M33f* const body_inverse_inertial_moments,
  M33f const* const body_inertial_moments) {
    copy_array(
      body_count, body_inverse_inertial_moments, body_inertial_moments);

    for_each_iota(body_count, [=](auto const body_i) {
        body_inverse_inertial_moments[body_i].gjInvert(true);
    });
};

//------------------------------------------------------------------------------
// How do we build rigid neighborhoods?
// Let's not overthink it. The solution in the multi-fluids paper is
// too complex and I don't even understand it now.
// so what we need here is that the neighborhoods of the particles
// contain the neighbors of just the same rigid body first
// and then contain the neighbors of other rigid bodies next.
// we will need to loop over the bodies one by one.
//
// Because we don't want to keep track of which particles live in which
// neighboring bodie

struct Neighbor {
    size_t index = 0;
    float distance = 0.0f;
    V2f vector_to = V2f{0.0f, 0.0f};
};

bool neighbor_order(Neighbor const& a, Neighbor const& b) {
    if (a.distance < b.distance) {
        return true;
    } else if (a.distance > b.distance) {
        return false;
    } else {
        return a.index < b.index;
    }
}

using Neighbors = std::array<Neighbor, NEIGHBORHOOD_MAX_COUNT>;

void sort_neighbors_in_place(Neighbors& neighbors, uint8_t const count) {
    std::sort(neighbors.begin(), neighbors.begin() + count, neighbor_order);
}

// Pass in an accept function that takes two indices and determines whether the
// particle at the second index can be a neighbor (of this type) of the particle
// at the first index
template <typename AcceptConditionFunction>
void create_neighborhoods(
  size_t const particle_count,
  float const max_distance,
  uint8_t* const neighbor_counts,
  Neighbor_values<size_t>* const neighbor_indices,
  Neighbor_values<float>* const neighbor_distances,
  Neighbor_values<V2f>* const neighbor_vectors_to,

  V2f const* const positions,
  V2i const* const grid_coords,
  std::pair<uint64_t, size_t> const* const sorted_index_pairs,
  Block_map const& block_map,
  AcceptConditionFunction&& accept_condition_function) {
    for_each_iota(
      particle_count,
      [max_distance,
       neighbor_counts,
       neighbor_indices,
       neighbor_distances,
       neighbor_vectors_to,
       positions,
       grid_coords,
       sorted_index_pairs,
       &block_map,
       accept = std::forward<AcceptConditionFunction>(
         accept_condition_function)](auto const particle_index) {
          auto const pos = positions[particle_index];
          auto const grid_coord = grid_coords[particle_index];

          Neighbors neighbors;
          uint8_t nbhd_count = 0;
          for (int32_t j = grid_coord[1] - 1;
               (nbhd_count < NEIGHBORHOOD_MAX_COUNT) &&
               (j <= grid_coord[1] + 1);
               ++j) {
              for (int32_t i = grid_coord[0] - 1;
                   (nbhd_count < NEIGHBORHOOD_MAX_COUNT) &&
                   (i <= grid_coord[0] + 1);
                   ++i) {
                  auto const other_z_index = z_index::z_index(i, j);
                  auto const found_iter = block_map.find(other_z_index);
                  if (found_iter == block_map.end()) { continue; }

                  // sip == sorted_index_pair
                  auto const [sip_begin, sip_end] = (*found_iter).second;
                  for (auto sip = sip_begin; sip != sip_end; ++sip) {
                      auto const other_particle_index =
                        sorted_index_pairs[sip].second;
                      if ((other_particle_index == particle_index) ||
                          !accept(particle_index, other_particle_index)) {
                          continue;
                      }

                      auto& neighbor = neighbors.at(nbhd_count);
                      neighbor.index = other_particle_index;
                      neighbor.vector_to =
                        positions[other_particle_index] - pos;
                      neighbor.distance = neighbor.vector_to.length();

                      if (neighbor.distance >= max_distance) { continue; }
                      ++nbhd_count;
                  }
              }
          }
          sort_neighbors_in_place(neighbors, nbhd_count);
          neighbor_counts[particle_index] = nbhd_count;
          auto& nbr_indices = neighbor_indices[particle_index];
          auto& nbr_distances = neighbor_distances[particle_index];
          auto& nbr_vectors_to = neighbor_vectors_to[particle_index];
          for (uint8_t neighbor_i = 0; neighbor_i < nbhd_count; ++neighbor_i) {
              auto const& neighbor = neighbors[neighbor_i];
              nbr_indices[neighbor_i] = neighbor.index;
              nbr_distances[neighbor_i] = neighbor.distance;
              nbr_vectors_to[neighbor_i] = neighbor.vector_to;
          }
      });
}

void create_rigid_self_neighborhoods(
  size_t const particle_count,
  float const max_distance,
  uint8_t* const rigid_self_neighbor_counts,
  Neighbor_values<size_t>* const rigid_self_neighbor_indices,
  Neighbor_values<float>* const rigid_self_neighbor_distances,
  Neighbor_values<V2f>* const rigid_self_neighbor_vectors_to,

  V2f const* const positions,
  V2i const* const grid_coords,
  size_t const* const particle_body_indices,
  std::pair<uint64_t, size_t> const* const sorted_index_pairs,
  Block_map const& block_map) {
    create_neighborhoods(particle_count,
                         max_distance,
                         rigid_self_neighbor_counts,
                         rigid_self_neighbor_indices,
                         rigid_self_neighbor_distances,
                         rigid_self_neighbor_vectors_to,

                         positions,
                         grid_coords,
                         sorted_index_pairs,
                         block_map,
                         [particle_body_indices](auto const i, auto const j) {
                             return particle_body_indices[i] ==
                                    particle_body_indices[j];
                         });
}

void create_rigid_other_neighborhoods(
  size_t const particle_count,
  float const max_distance,
  uint8_t* const rigid_other_neighbor_counts,
  Neighbor_values<size_t>* const rigid_other_neighbor_indices,
  Neighbor_values<float>* const rigid_other_neighbor_distances,
  Neighbor_values<V2f>* const rigid_other_neighbor_vectors_to,

  V2f const* const positions,
  V2i const* const grid_coords,
  size_t const* const particle_body_indices,
  std::pair<uint64_t, size_t> const* const sorted_index_pairs,
  Block_map const& block_map) {
    create_neighborhoods(particle_count,
                         max_distance,
                         rigid_other_neighbor_counts,
                         rigid_other_neighbor_indices,
                         rigid_other_neighbor_distances,
                         rigid_other_neighbor_vectors_to,

                         positions,
                         grid_coords,
                         sorted_index_pairs,
                         block_map,
                         [particle_body_indices](auto const i, auto const j) {
                             return particle_body_indices[i] !=
                                    particle_body_indices[j];
                         });
}

void compute_neighborhood_kernels(
  size_t const particle_count,
  float const support,
  Neighbor_values<float>* const neighbor_kernels,
  uint8_t const* const neighbor_counts,
  Neighbor_values<float> const* const neighbor_distances) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto& neighbor_kernel = neighor_kernels[particle_index];
        auto const neighbor_count = neighbor_counts[particle_index];
        auto const& neighbor_distance = neighbor_distances[particle_index];
        for (uint8_t j = 0; j < neighbor_count; ++j) {
            neighbor_kernel[j] = kernels::W(neighbor_distance[j], support);
        }
    });
}

void compute_neighborhood_kernel_gradients(
  size_t const particle_count,
  float const support,
  Neighbor_values<V2f>* const neighbor_kernel_gradients,
  uint8_t const* const neighbor_counts,
  Neighbor_values<V2f> const* const neighbor_vectors_to) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto& neighbor_kernel_gradient =
          neighor_kernel_gradients[particle_index];
        auto const neighbor_count = neighbor_counts[particle_index];
        auto const& neighbor_vector_to = neighbor_vectors_to[particle_index];
        for (uint8_t j = 0; j < neighbor_count; ++j) {
            neighbor_kernel_gradient[j] =
              kernels::GradW(neighbor_vector_to[j], support);
        }
    });
}

// Once we have neighborhoods, we can compute the number of contacts
// each body has by the number of rigid other neighbors
void compute_body_num_contacts(
  size_t const body_count,
  size_t* const body_num_contacts,
  std::pair<size_t, size_t> const* const body_particle_index_ranges,
  uint8_t const* const rigid_self_neighbor_counts) {
    for_each_iota(body_count, [=](auto const body_index) {
        auto const body_particle_index_range =
          body_particle_index_ranges[body_index];
        size_t contact_count = 0;
        for (size_t particle_index = body_particle_index_range.first;
             particle_index != body_particle_index_range.second;
             ++particle_index) {
            if (rigid_self_neighbor_counts[particle_index]) { ++contact_count; }
        }
        body_num_contacts[body_index] = contact_count;
    });
}

// Only needs to be computed once per rigid formulation
void compute_artificial_rest_volumes(
  size_t const particle_count,
  float const support,
  float* const artificial_rest_volumes,
  uint8_t const* const rigid_self_neighbor_counts,
  Neighbor_values<float> const* const rigid_self_neighbor_kernels) {
    constexpr float numer = 0.7f;
    auto const denom_init = kernels::W(0.0f, support);
    // Divide always safe because denom is nonzero even with zero particles.
    EMLD_ASSERT(is_safe_divide(numer, denom_init), "Bad divide");
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const nbhd_count = rigid_self_neighbor_counts[particle_index];
        auto const& nbhd_kernels = rigid_self_neighbor_kernels[particle_index];
        float denom = denom_init;
        for (uint8_t j = 0; j < nbhd_count; ++j) { denom += nbhd_kernels[j]; }

        artificial_rest_volumes[particle_index] = numer / denom;
    });
}

void init_artificial_densities(size_t const particle_count,
                               float const support,
                               float* const artificial_densities,
                               float const* const artificial_rest_volumes) {
    auto const W_0_h = kernels::W(0.0f, support);
    for_each_iota(particle_count, [=](auto const particle_index) {
        artificial_densities[particle_index] =
          W_0_h * artificial_rest_volumes[particle_index];
    });
}

void accumulate_artificial_densities(
  size_t const particle_count,
  float* const artificial_densities,
  float const* const artificial_rest_volumes,
  uint8_t const* const neighbor_counts,
  Neighbor_values<size_t> const* const neighbor_indices,
  Neighbor_values<float> const* const neighbor_kernels) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const nbhd_count = neighbor_counts[particle_index];
        if (!nbhd_count) { return; }

        auto const& nbhd_indices = neighbor_indices[particle_index];
        auto const& nbhd_kernels = neighbor_kernels[particle_index];
        float density_sum = 0.0f;
        for (int j = 0; j < nbhd_count; ++j) {
            auto const other_particle_index = nbhd_indices[j];
            density_sum +=
              artificial_rest_volumes[other_particle_index] * nbhd_kernels[j];
        }

        artificial_densities[particle_index] += density_sum;
    });
}

void compute_artificial_densities(
  size_t const particle_count,
  float const support,
  float* const artificial_densities,
  float const* const artificial_rest_volumes,
  uint8_t const* const rigid_self_neighbor_counts,
  Neighbor_values<size_t> const* const rigid_self_neighbor_indices,
  Neighbor_values<float> const* const rigid_self_neighbor_kernels,
  uint8_t const* const rigid_other_neighbor_counts,
  Neighbor_values<size_t> const* const rigid_other_neighbor_indices,
  Neighbor_values<float> const* const rigid_other_neighbor_kernels) {
    init_artificial_densities(
      particle_count, support, artificial_densities, artificial_rest_volumes);

    accumulate_artificial_densities(particle_count,
                                    artificial_densities,
                                    artificial_rest_volumes,
                                    rigid_self_neighbor_counts,
                                    rigid_self_neighbor_indices,
                                    rigid_self_neighbor_kernels);

    accumulate_artificial_densities(particle_count,
                                    artificial_densities,
                                    artificial_rest_volumes,
                                    rigid_other_neighbor_counts,
                                    rigid_other_neighbor_indices,
                                    rigid_other_neighbor_kernels);
}

void compute_artificial_volumes(size_t const particle_count,
                                float* const artificial_volumes,
                                float const* const artificial_rest_volumes,
                                float const* const artificial_densities) {
    copy_array(particle_count, artificial_volumes, artificial_rest_volumes);
    for_each_iota(particle_count, [=](auto const particle_index) {
        artificial_volumes[particle_index] /=
          artificial_densities[particle_index];
    });
}

void forward_euler_integrate_body_linear_velocities(
  size_t const body_count,
  float const dt,
  V2f* const body_linear_velocities_next,
  V2f const* const body_linear_velocities,
  float const* const body_masses,
  V2f const* const body_external_forces) {
    // Accommodate in-place usage
    if (body_linear_velocities != body_linear_velocities_next) {
        copy_array(
          body_count, body_linear_velocities_next, body_linear_velocities);
    }
    for_each_iota(body_count, [=](auto const body_index) {
        body_linear_velocities_next[body_index] +=
          (dt / body_masses[body_index]) * body_external_forces[body_index];
    });
}

void forward_euler_integrate_body_angular_velocities(
  size_t const body_count,
  float const dt,
  float* const body_angular_velocities_next,
  float const* const body_angular_velocities,
  M33f const* const body_inertial_moments,
  M33f const* const body_inverse_inertial_moments,
  float const* const body_external_torques) {
    // In 3d, there's a term related to
    // dt * (I omega) x omega, where x is cross product.
    // However, if the rotations are always about the z axis (we're in a plane),
    // then I is always [k1  k2  0
    //                   k2  k3  0
    //                   0   0  k4]
    // and omega is always [0 0 w]^T
    // which means I omega is always just [0 0 w * k4]^T
    // a vector parallel to the z axis crossed with another vector parallel to
    // the z axis is always zero, so we can skip this term. Leaving it here for
    // 3d case to come. for_each_iota(body_count, [=](auto const body_index) {
    //     V3f const omega{0.0f, 0.0f, body_angular_velocities[body_index]};
    //     V3f const I_times_omega = omega * body_inertial_moments[body_index];
    //     body_angular_velocities_star[body_index] = dt *
    //     I_times_omega.cross(omega);
    // });

    // Accommodate in-place usage
    if (body_angular_velocities != body_angular_velocities_next) {
        copy_array(
          body_count, body_angular_velocities_next, body_angular_velocities);
    }

    for_each_iota(body_count, [=](auto const body_index) {
        V3f const torque{0.0f, 0.0f, body_external_torques[body_index]};
        // multiplication transposed because imath
        auto const dt_Iinv_torque =
          dt * (torque * body_inertial_moments[body_index]);
        body_angular_velocities_star[body_index] += dt_Iinv_torque[2];
    });
}

void compute_particle_velocities_from_rigids(
  size_t const body_count,
  V2f* const particle_velocities,
  std::pair<size_t, size_t> const* const body_particle_index_ranges,
  V2f const* const particle_centroid_offsets,
  V2f const* const body_linear_velocities,
  float const* const body_angular_velocities) {
    for_each_iota(body_count, [=](auto const body_i) {
        auto const body_particle_index_range =
          body_particle_index_ranges[body_i];
        auto const body_v = body_linear_velocities[body_i];
        V3f const body_omega = {0.0f, 0.0f, body_angular_velocities[body_i]};
        for (size_t particle_index = body_particle_index_range.first;
             particle_index != body_particle_index_range.second;
             ++particle_index) {
            auto const r2 = particle_centroid_offsets[particle_index];
            V3f const r3{r2[0], r2[1], 0.0f};
            auto const spin_v3 = body_omega.cross(r3);
            particle_velocities[particle_index] =
              body_v + V2f{spin_v3[0], spin_v3[1]};
        }
    });
}

void compute_density_times_divergences(
  size_t const particle_count,
  float const support,
  float* const divergences,
  float const* const artificial_densities,
  float const* const artificial_volumes,
  V2f const* const positions,
  V2f const* const velocities,
  uint8_t const* const rigid_other_neighbor_counts,
  Neighbor_values<size_t> const* const rigid_other_neighbor_indices,
  Neighbor_values<V2f> const* const rigid_other_neighbor_kernel_gradients) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const nbhd_count = rigid_other_neighbor_counts[particle_index];
        if (!nbhd_count) {
            divergences[particle_index] = 0.0f;
            return;
        }

        auto const velocity = velocities[particle_index];
        auto const& nbhd_indices = rigid_other_neighbor_indices[particle_index];
        auto const& nbhd_kernel_gradients =
          rigid_other_neighbor_kernel_gradients[particle_index];
        float divergence = 0.0f;
        for (uint32_t j = 0; j < nbhd_count; ++j) {
            auto const other_particle_index = nbhd_indices[j];
            auto const delta_vel = velocities[other_particle_index] - velocity;
            auto const other_density =
              artificial_densities[other_particle_index];
            auto const other_volume = artificial_volumes[other_particle_index];
            auto const grad_w = nbhd_kernel_gradients[j];
            divergence +=
              other_volume * other_density * (delta_vel.dot(grad_w));
        }
        divergences[particle_index] = divergence;
    });
}

void compute_source_terms(size_t const particle_count,
                          float const dt,
                          float* const source_terms,
                          float const* const density_times_divergences,
                          float const* const artificial_densities) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const density = artificial_densities[particle_index];
        source_terms[particle_index] =
          ((1.0f - density) / dt) + density_times_divergences[particle_index];
    });
}

void compute_diagonal_pressure_gradients(
  size_t const particle_count,
  V2f* const diagonal_pressure_gradients,
  float const* const artificial_volumes,
  float const* const artificial_densities,
  uint8_t const* const rigid_other_neighbor_counts,
  Neighbor_values<size_t> const* const rigid_other_neighbor_indices,
  Neighbor_values<V2f> const* const rigid_other_neighbor_kernel_gradients) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const nbhd_count = rigid_other_neighbor_counts[particle_index];
        if (!nbhd_count) {
            diagonal_pressure_gradients[particle_index] = V2f{0.0f, 0.0f};
            return;
        }

        auto const volume = artificial_volumes[particle_index];
        auto const density = artificial_densities[particle_index];
        auto const& nbhd_indices = rigid_other_neighbor_indices[particle_index];
        auto const& nbhd_kernel_gradients =
          rigid_other_neighbor_kernel_gradients[particle_index];
        V2f pressure_grad_sum{0.0f, 0.0f};
        for (uint32_t j = 0; j < nbhd_count; ++j) {
            auto const other_particle_index = nbhd_indices[j];
            auto const other_density =
              artificial_densities[other_particle_index];
            auto const other_volume = artificial_volumes[other_particle_index];
            auto const grad_w = nbhd_kernel_gradients[j];
            pressure_grad_sum += other_volume * other_density * grad_w;
        }
        diagonal_pressure_gradients[particle_index] =
          pressure_grad_sum / sqr(density);
    });
}

void compute_diagonals(
  size_t const particle_count,
  float const dt,
  float* const diagonals,

  float const* const body_masses,
  M33f const* const body_inverse_inertial_moments,

  size_t const* const particle_body_indices,
  V2f const* const particle_centroid_offsets,

  V2f const* const diagonal_pressure_gradients,
  float const* const artificial_volumes,
  float const* const artificial_densities,

  uint8_t const* const rigid_other_neighbor_counts,
  Neighbor_values<size_t> const* const rigid_other_neighbor_indices,
  Neighbor_values<V2f> const* const rigid_other_neighbor_kernel_gradients) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const nbhd_count = rigid_other_neighbor_counts[particle_index];
        if (!nbhd_count) { diagonals[particle_index] = 0.0f; }

        auto const volume = artificial_volumes[particle_index];
        auto const density = artificial_densities[particle_index];
        auto const body_index = particle_body_indices[particle_index];

        // Compute the force at just this particle
        auto const particle_force =
          -volume * diagonal_pressure_gradients[particle_index];

        // Compute the linear velocity change for the body
        auto const body_delta_linear_velocity =
          dt * particle_force / body_masses[body_index];

        // Compute the angular velocity change for the body
        auto const r2 = particle_centroid_offsets[particle_index];
        V3f const r3{r2[0], r2[1], 0.0f};
        V3f const f3{particle_force[0], particle_force[1], 0.0f};

        // multiplication transposed because imath
        auto const body_delta_angular_velocity3 =
          dt * (r3.cross(f3) * body_inverse_inertial_moments[body_index]);

        // Particle velocity change for the body
        auto const rotation_part3 = body_delta_angular_velocity3.cross(r3);

        auto const particle_delta_velocity =
          body_delta_linear_velocity +
          V2f{rotation_part3[0], rotation_part3[1]};

        float diagonal = 0.0f;
        auto const& nbhd_indices = rigid_other_neighbor_indices[particle_index];
        auto const& nbhd_kernel_gradients =
          rigid_other_neighbor_kernel_gradients[particle_index];
        for (uint8_t j = 0; j < nbhd_count; ++j) {
            auto const other_particle_index = nbhd_indices[j];
            auto const other_volume = artificial_volumes[other_particle_index];
            auto const other_density =
              artificial_densities[other_particle_index];
            auto const grad_w = nbhd_kernel_gradients[j];

            // It's this, but we can just cancel out the negatives.
            // diagonal += -other_volume * other_density *
            // (-particle_delta_velocity.dot(grad_w));
            diagonal += other_volume * other_density *
                        (particle_delta_velocity.dot(grad_w));
        }

        diagonals[particle_index] = diagonal;
    });
}

void compute_relaxation_over_diagonals(
  size_t const particle_count,
  float* const relaxation_over_diagonals,
  uint8_t const* const rigid_other_neighbor_counts,
  size_t const* const particle_body_indices,
  size_t const* const body_num_contacts,
  float const* const diagonals) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        if (!rigid_other_neighbor_counts[particle_index]) {
            relaxation_over_diagonals[particle_index] = 0.0f;
        }

        auto const body_index = particle_body_indices[particle_index];
        auto const num_contacts = body_num_contacts[body_index];
        if (!num_contacts) {
            relaxation_over_diagonals[particle_index] = 0.0f;
            return;
        }

        auto const numer = 0.5f / static_cast<float>(num_contacts);
        auto const denom = diagonals[particle_index];

        if (is_safe_divide(numer, denom)) {
            relaxation_over_diagonals[particle_index] = numer / denom;
        } else {
            relaxation_over_diagonals[particle_index] = 0.0f;
        }
    });
}

void accumulate_pressure_gradients(
  size_t const particle_count,
  float const dt,
  V2f* const pressure_gradients,
  float const* const pressures,
  float const* const artificial_volumes,
  float const* const artificial_densities,
  uint8_t const* const neighbor_counts,
  Neighbor_values<size_t> const* const neighbor_indices,
  Neighbor_values<V2f> const* const neighbor_kernel_gradients) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const nbhd_count = neighbor_counts[particle_index];
        if (!nbhd_count) { return; }

        auto const pressure = pressures[particle_index];
        auto const volume = artificial_volumes[particle_index];
        auto const density = artificial_densities[particle_index];

        auto const pr_over_sqr_rho_r = pressure / sqr(density);

        V2f gradient_sum{0.0f, 0.0f};
        auto const& nbhd_indices = neighbor_indices[particle_index];
        auto const& nbhd_kernel_gradients =
          neighor_kernel_gradients[particle_index];
        for (uint8_t j = 0; j < nbhd_count; ++j) {
            auto const other_particle_index = nbhd_indices[j];
            auto const other_pressure = pressures[other_particle_index];
            auto const other_volume = artificial_volumes[other_particle_index];
            auto const other_density =
              artificial_densities[other_particle_index];
            auto const other_pr_over_sqr_rho_r =
              other_pressure / sqr(other_density);

            auto const grad_w = nbhd_kernel_gradients[j];

            gradient_sum += (other_volume * other_density *
                             (pr_over_sqr_rho_r + other_pr_over_sqr_rho_r)) *
                            grad_w;
        }

        pressure_gradients[particle_index] += density * gradient_sum;
    });
}

void compute_pressure_gradients(
  size_t const particle_count,
  float const dt,
  V2f* const pressure_gradients,
  float const* const artificial_volumes,
  float const* const artificial_densities,
  uint8_t const* const rigid_self_neighbor_counts,
  Neighbor_values<size_t> const* const rigid_self_neighbor_indices,
  Neighbor_values<V2f> const* const rigid_self_neighbor_kernel_gradients,
  uint8_t const* const rigid_other_neighbor_counts,
  Neighbor_values<size_t> const* const rigid_other_neighbor_indices,
  Neighbor_values<V2f> const* const rigid_other_neighbor_kernel_gradients) {
    fill_array(particle_count, pressure_gradients, V2f{0.0f, 0.0f});
    accumulate_pressure_gradients(particle_count,
                                  dt,
                                  pressure_gradients,
                                  artificial_volumes,
                                  artificial_densities,
                                  rigid_self_neighbor_counts,
                                  rigid_self_neighbor_indices,
                                  rigid_self_neighbor_kernel_gradients);
    accumulate_pressure_gradients(particle_count,
                                  dt,
                                  pressure_gradients,
                                  artificial_volumes,
                                  artificial_densities,
                                  rigid_other_neighbor_counts,
                                  rigid_other_neighbor_indices,
                                  rigid_other_neighbor_kernel_gradients);
}

void compute_body_linear_delta_velocities(
  size_t const body_count,
  float const dt,
  V2f* const body_delta_linear_velocities,
  size_t const* const body_num_contacts,
  float const* const body_masses,
  std::pair<size_t, size_t> const* const body_particle_index_ranges,
  float const* const artificial_volumes,
  V2f const* const pressure_gradients) {
    for_each_iota(body_count, [=](auto const body_index) {
        if (!body_num_contacts[body_index]) {
            body_delta_linear_velocities[body_index] = 0.0f;
            return;
        }

        auto const mass = body_masses[body_index];
        auto const body_particle_index_range =
          body_particle_index_ranges[body_index];
        V2f delta_velocity_sum{0.0f, 0.0f};
        for (size_t particle_index = body_particle_index_range.first;
             particle_index != body_particle_index_range.second;
             ++particle_index) {
            delta_velocity_sum += artificial_volumes[particle_index] *
                                  pressure_gradients[particle_index];
        }

        body_delta_linear_velocities[body_index] =
          (-dt / mass) * delta_velocity_sum;
    });
}

void compute_body_delta_angular_velocities(
  size_t const body_count,
  float const dt,
  float* const body_delta_angular_delta_velocities,
  size_t const* const body_num_contacts,
  M33f const* const body_inverse_inertial_moments,
  std::pair<size_t, size_t> const* const body_particle_index_ranges,
  V2f const* const particle_centroid_offsets,
  float const* const artificial_volumes,
  V2f const* const pressure_gradients) {
    for_each_iota(body_count, [=](auto const body_index) {
        if (!body_num_contacts[body_index]) {
            body_delta_angular_velocities[body_index] = 0.0f;
            return;
        }

        auto const I_inv = body_inverse_inertial_moments[body_index];
        auto const body_particle_index_range =
          body_particle_index_ranges[body_index];
        float delta_angular_velocity_sum = 0.0f;
        for (size_t particle_index = body_particle_index_range.first;
             particle_index != body_particle_index_range.second;
             ++particle_index) {
            auto const r2 = particle_centroid_offsets[particle_index];
            auto const volume = artificial_volumes[particle_index];
            auto const pgrad2 = pressure_gradients[particle_index];

            delta_angular_velocity_sum +=
              volume * (V3f{r2[0], r2[1], 0.0f}.cross(
                         V3f{pgrad2[0], pgrad2[1], 0.0f})[2]);
        }

        // Multiplication is transposed because Imath
        auto const I_inv_times_sum_z =
          (V3f{0.0f, 0.0f, delta_angular_velocity_sum} * I_inv)[2];
        body_delta_angular_velocities[body_index] = -dt * I_inv_times_sum_z;
    });
}

// Pressure iteration.
void compute_next_pressures(...) {

    forward_euler_integrate_body_linear_velocities(...);
    forward_euler_integrate_body_angular_velocities(...);

    compute_source_terms(...);




    compute_density_times_divergences(...);



void time_step_sketch() {
    // what to do every time

    // we have bodies.
    // they have positions and orientations.
    // they have velocities.

    // we have particles. they belong to the bodies.

    // i'm too tired






}
