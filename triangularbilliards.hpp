#ifndef TRIANGULAR_BILLIARDS_HPP
#define TRIANGULAR_BILLIARDS_HPP

#include <memory>

#include "statistics.hpp"

namespace tb {

struct Particle {
  double x;
  double y;
  double theta;
};

enum class BorderHit { Top, Bottom, None };

struct Border {
 private:
  double r1_;
  double r2_;
  double l_;

 public:
  virtual ~Border() = default;

  explicit Border(double r1, double r2, double l) : r1_{r1}, r2_{r2}, l_{l} {}
  virtual BorderHit checkCollision(const Particle& p) const = 0;
  double getSlope() const { return (r2_ - r1_) / l_; }
  double r1() const { return r1_; }
  double r2() const { return r2_; }
  double xEnd() const { return l_; }
};

struct StraightBorder : Border {
  explicit StraightBorder(double r1, double r2, double l);
  BorderHit checkCollision(const Particle& p) const override;
};

struct OpenedBorder : Border {
  explicit OpenedBorder(double r1, double r2, double l);
  BorderHit checkCollision(const Particle& p) const override;
};

struct ClosedBorder : Border {
  explicit ClosedBorder(double r1, double r2, double l);
  BorderHit checkCollision(const Particle& p) const override;
};

std::unique_ptr<Border> createBorder(double r1, double r2, double l);

struct SingleResult {
  double x;
  double y;
  double theta;
  bool valid;
};

struct MultipleResult {
  Sample finalY;
  Sample finalTheta;
  int accepted;
  int rejected;
};

void reduceAngle(double& p);

int sign(const BorderHit dir);

void computeNextCollision(Particle& p, const Border* b, const double& sigma);

void computeFinalPosition(Particle& p, const Border* b);

class Trajectory {
  std::vector<Particle> positions_{};

 public:
  explicit Trajectory(Particle p) { positions_.push_back(p); }

  const std::vector<Particle>& positions() const { return positions_; }
  size_t size() const;

  void simulateCollisions(const Border* b);

  const Particle& getFinalPosition() const;
};

Trajectory computeSingleTrajectory(Particle& p, const Border* b);

SingleResult simulateFinalState(Particle& p, const Border* b);

SingleResult simulateFinalState(Particle& p, const Border& b) {
  return simulateFinalState(p, &b);
}

MultipleResult runMultipleSimulations(int N, double Y0_mean, double& Y0_err,
                                      double Theta0_mean, double& Theta0_err,
                                      const Border* b);

}  // namespace tb

#endif