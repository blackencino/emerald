#pragma once

#include <emerald/shallow_weaver/foundation.h>

#include <emerald/util/assert.h>
#include <emerald/util/format.h>

#include <algorithm>
#include <array>
#include <vector>

namespace emerald::shallow_weaver {

template <typename T>
class Slab {
public:
    Slab() noexcept = default;
    ~Slab() noexcept = default;
    explicit Slab(int2 const size)
      : m_size(size) {
        EMLD_ASSERT(m_size[0] >= 0 && m_size[1] >= 0,
                    fmt::format("invalid size: {}", m_size));
        m_data.resize(m_size[0] * m_size[1]);
    }
    explicit Slab(Slab const&) = default;
    Slab(Slab&& other) noexcept
      : m_data(std::move(other.m_data))
      , m_size(other.m_size) {
        other.m_size = int2{0, 0};
    }

    Slab& operator=(Slab const&) = default;
    Slab& operator=(Slab&& other) noexcept {
        m_data = std::move(other.m_data);
        m_size = other.m_size;
        other.m_size = int2{0, 0};
        return *this;
    }

    int2 size() const noexcept {
        return m_size;
    }

    int index(int const i, int const j) const {
        return i + j * m_size[0];
    }

    int index(int2 const ij) const {
        return ij[0] + ij[1] * m_size[0];
    }

    // Value accessors don't check the index for range validity.
    T& value(int const i, int const j) {
        return m_data[index(i, j)];
    }
    T const& value(int const i, int const j) const {
        return m_data[index(i, j)];
    }

    T& value(int2 const ij) {
        return m_data[index(ij)];
    }
    T const& value(int2 const ij) const {
        return m_data[index(ij)];
    }

    // At accessors check the range for range validity.
    T& at(int const i, int const j) {
        return m_data.at(index(i, j));
    }
    T const& at(int const i, int const j) const {
        return m_data.at(index(i, j));
    }

    T& at(int2 const ij) {
        return m_data.at(index(ij));
    }
    T const& at(int2 const ij) const {
        return m_data.at(index(ij));
    }

    T& operator[](int const i) {
        return m_data[i];
    }

    T const& operator[](int const i) const {
        return m_data[i];
    }

    // iterate over all data
    span<T> as_span() {
        return gsl::make_span(m_data);
    }
    span<const T> as_span() const {
        return gsl::make_span(m_data);
    }

    // return pointer
    T* data() {
        return m_data.data();
    }

    T const* data() const {
        return m_data.data();
    }

    void fill(T const fill_value) {
        std::fill(m_data.begin(), m_data.end(), fill_value);
    }

    void swap(Slab<T>& other) noexcept {
        m_data.swap(other.m_data);
        auto tmp = m_size;
        m_size = other.m_size;
        other.m_size = tmp;
    }

private:
    std::vector<T> m_data;
    int2 m_size = {0, 0};
};

using Float_slab = Slab<float>;

template <typename T>
void swap(Slab<T>& a, Slab<T>& b) noexcept {
    a.swap(b);
}

}  // namespace emerald::shallow_weaver
