#include "statistics.hpp"

namespace tb {
size_t Sample::size() const { return values_.size(); }

void Sample::add(double x) { values_.push_back(x); }

bool Sample::remove_all() {
  values_.clear();
  return true;
}

Statistics Sample::statistics() const {
  const size_t N = values_.size();
  if (N < 4) throw std::runtime_error("Not enough points");

  struct Sums {
    double x = 0.0;
    double x2 = 0.0;
  };
  Sums sums = std::accumulate(values_.begin(), values_.end(), Sums{},
                              [](Sums s, double x) {
                                s.x += x;
                                s.x2 += x * x;
                                return s;
                              });

  auto NN = static_cast<double>(N);
  double mean = sums.x / NN;

  if ((sums.x2 - NN * mean * mean) <= 0) {
    return {mean, 0.0, 0.0, 0.0};
  }

  assert((sums.x2 - NN * mean * mean) / (NN - 1) > 0);
  double sigma = std::sqrt((sums.x2 - NN * mean * mean) / (NN - 1));

  /// @brief The Zahan summation algorithm is implemented to accurately
  /// calculate skewness and kurtosis for samples of various sizes.
  double z2_sum = 0.0, c2 = 0.0;
  double z3_sum = 0.0, c3 = 0.0;
  double z4_sum = 0.0, c4 = 0.0;

  auto kahan_add = [](double& sum, double& c, double v) {
    auto y = v - c;
    auto t = sum + y;
    c = (t - sum) - y;
    sum = t;
  };

  assert(sigma != 0.);
  for (double x : values_) {
    double z = (x - mean) / sigma;
    double z2 = z * z;
    kahan_add(z2_sum, c2, z2);
    kahan_add(z3_sum, c3, z2 * z);
    kahan_add(z4_sum, c4, z2 * z2);
  }

  assert(NN >= 4);
  double skewness = (NN / ((NN - 1.0) * (NN - 2.0))) * z3_sum;
  double kurtosis =
      (NN * (NN + 1.0)) / ((NN - 1.0) * (NN - 2.0) * (NN - 3.0)) * z4_sum -
      (3.0 * (NN - 1.0) * (NN - 1.0)) / ((NN - 2.0) * (NN - 3.0));

  return {mean, sigma, skewness, kurtosis};
}

}  // namespace tb