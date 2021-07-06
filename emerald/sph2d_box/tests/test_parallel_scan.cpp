#pragma warning(push)
#pragma warning(disable : 4244)
#include <tbb/blocked_range.h>
#include <tbb/parallel_scan.h>
#pragma warning(pop)

#include <gtest/gtest.h>

#include <cstdint>
#include <numeric>
#include <vector>

namespace emerald::sph2d_box {

uint64_t do_parallel_scan(size_t const size,
                          uint64_t* const result,
                          uint32_t const* const input) {
    return tbb::parallel_scan(
      tbb::blocked_range<size_t>{size_t{0}, size},
      0,
      [result, input](tbb::blocked_range<size_t> const& range,
                      uint64_t const sum,
                      bool const is_final_scan) {
          uint64_t temp = sum;
          for (auto i = range.begin(); i != range.end(); ++i) {
              temp += input[i];
              if (is_final_scan) { result[i] = temp; }
          }
          return temp;
      },
      [](uint64_t const left, uint64_t const right) { return left + right; });
}

TEST(Sph2d_box_test, test_parallel_scan) {
    std::vector<uint32_t> values;
    std::vector<uint64_t> expected_results;
    std::vector<uint64_t> results;

    size_t const count = 54201;
    values.resize(count);
    expected_results.resize(count);
    results.resize(count);

    std::iota(values.begin(), values.end(), 1);
    EXPECT_EQ(1, values.front());
    EXPECT_EQ(count, values.back());

    size_t const expected_sum = (count * (1 + count)) / 2;

    size_t sum = 0;
    for (size_t i = 0; i < count; ++i) {
        sum += values[i];
        expected_results[i] = sum;
    }

    EXPECT_EQ(expected_sum, sum);

    auto const parallel_sum =
      do_parallel_scan(count, results.data(), values.data());
    EXPECT_EQ(expected_sum, parallel_sum);

    EXPECT_EQ(expected_results, results);
}

}  // namespace emerald::sph2d_box