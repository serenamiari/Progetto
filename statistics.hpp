#ifndef SAMPLE_HPP
#define SAMPLE_HPP

#include <algorithm>
#include <cassert>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace tb {

/// @brief Summary statistics of a numerical sample.
/// Includes mean, standard dev, skewness, and kurtosis.
struct Statistics {
  double mean;
  double sigma;
  double skewness;
  double kurtosis;
};

/// @brief Represents numerical sample and provides stats
class Sample {
  std::vector<double> values_{};

 public:
  const auto& values() const { return values_; }

  auto& values() { return values_; }

  size_t size() const;

  void add(double x);

  bool remove_all();

  using value_type = double;
  void push_back(double x) { add(x); }

  Statistics statistics() const;
};

}  // namespace tb

#endif