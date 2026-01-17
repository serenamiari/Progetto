#include "triangularbilliards.hpp"

#include <random>

namespace tb {
void reduceAngle(double& p) {
  const auto two_pi = 2 * M_PI;
  p = std::fmod(p, two_pi);
}

StraightBorder::StraightBorder(double r1, double r2, double l)
    : Border{r1, r2, l} {}

BorderHit StraightBorder::checkCollision(const Particle& p) const {
  if (p.theta == 0) return BorderHit::None;
  return p.theta > 0 ? BorderHit::Top : BorderHit::Bottom;
}

OpenedBorder::OpenedBorder(double r1, double r2, double l)
    : Border{r1, r2, l}, sigma_{std::atan2(r2 - r1, l)} {}

BorderHit OpenedBorder::checkCollision(const tb::Particle& p) const {
  if (p.theta > sigma_) return BorderHit::Top;
  if (p.theta < -sigma_) return BorderHit::Bottom;
  return BorderHit::None;
}

ClosedBorder::ClosedBorder(double r1, double r2, double l)
    : Border{r1, r2, l} {}

BorderHit ClosedBorder::checkCollision(const tb::Particle& p) const {
  if (p.theta > 0 || (p.theta == 0 && p.y > 0)) return BorderHit::Top;
  if (p.theta < 0 || (p.theta == 0 && p.y < 0)) return BorderHit::Bottom;
  return BorderHit::None;
}

std::unique_ptr<tb::Border> createBorder(double r1, double r2, double l) {
  if (r1 == r2) {
    return std::make_unique<StraightBorder>(r1, r2, l);
  } else if (r2 > r1) {
    return std::make_unique<OpenedBorder>(r1, r2, l);
  } else {
    return std::make_unique<ClosedBorder>(r1, r2, l);
  }
}

int sign(const BorderHit dir) {
  switch (dir) {
    case BorderHit::Top:
      return 1;
    case BorderHit::Bottom:
      return -1;
    case BorderHit::None:
      return 0;
    default:
      return 0;
  }
}

/// @brief Computes the collision point between the particle and the line
/// passing through the border; updates its angle theta, calculated wrt
/// the x-axis. The computation works for collisions both against the top and
/// bottom border. The variable s handles the top-bottom symmetry.
/// @param p The particle to be updated.
/// @param border Pointer to the top border.
/// @param sigma The angle of the border calculated wrt to the x-axis.
void computeNextCollision(Particle& p, const Border* border,
                          const double& sigma) {
  if (std::abs(p.theta) >= M_PI / 2) {
    throw std::runtime_error("Particle moves backwards");
  }
  assert(border->checkCollision(p) != BorderHit::None);
  BorderHit hit = border->checkCollision(p);
  auto const s = sign(hit);
  auto const tan_theta = std::tan(p.theta);
  auto t = p.x;
  auto den = (tan_theta - s * border->getSlope());
  assert(den != 0);
  p.x = (s * border->r1() - p.y + tan_theta * p.x) / den;
  p.y = tan_theta * (p.x - t) + p.y;
  p.theta = (s * 2 * sigma - p.theta);
}

/// @brief Computes the coordinates of the last position of the particle, i.e.
/// the collision point between the particle and the line passing through x = L.
/// The angle theta remains the same.
void computeFinalPosition(Particle& p, const Border* border) {
  assert(std::abs(p.theta) <= M_PI / 2);
  assert(p.x <= border->xEnd());
  auto t = p.x;
  p.x = border->xEnd();
  p.y = std::tan(p.theta) * (p.x - t) + p.y;
}

size_t Trajectory::size() const { return positions_.size(); }

void Trajectory::simulateCollisions(const Border* border) {
  assert(!positions_.empty());
  auto const sigma = std::atan2(border->r2() - border->r1(), border->xEnd());
  // size_t i = 0;
  while (positions_.back().x < border->xEnd() && border->checkCollision(positions_.back()) != BorderHit::None) {
    auto p = positions_.back();

    computeNextCollision(p, border, sigma);
    assert(p.x >= positions_.back().x);

    positions_.push_back(p);
  }

  if (positions_.back().x > border->xEnd()) {
    positions_.pop_back();
  }

  auto p = positions_.back();
  computeFinalPosition(p, border);
  assert(p.y <= border->r2() || p.y >= -border->r2());

  positions_.push_back(p);
}

const Particle& Trajectory::getFinalPosition() const {
  return positions_.back();
}

Trajectory computeSingleTrajectory(Particle& p, const Border* border) {
  reduceAngle(p.theta);

  if (std::abs(p.theta) == M_PI / 2 && border->r1() == border->r2()) {
    throw std::runtime_error ("Invalid conditions");
  }

  assert(p.y <= border->r1() && p.y >= -border->r1());
  assert(border->r1() >= 0 && border->r2() >= 0 && border->xEnd() >= 0);

  Trajectory traj(p);
  traj.simulateCollisions(border);

  assert(traj.size() > 1);

  return traj;
}

SingleResult simulateFinalState(Particle& p, const Border* border) {
  auto final = computeSingleTrajectory(p, border).getFinalPosition();
  return {final.x, final.y, final.theta, true};
}

MultipleResult runMultipleSimulations(int N, double Y0_mean, double& Y0_err,
                                      double Theta0_mean, double& Theta0_err,
                                      const Border* border) {
  assert(N > 0);

  if (Y0_err < 0) {
    Y0_err = - Y0_err;
  }
  assert(Y0_err >= 0);

  if (Theta0_err < 0) {
    Theta0_err = -Theta0_err;
  }
  assert(Theta0_err >= 0);

  tb::Sample finalPosY;
  tb::Sample finalPosTheta;

  std::random_device r;
  std::default_random_engine eng{r()};
  std::normal_distribution<double> dist_y{Y0_mean, Y0_err};
  std::normal_distribution<double> dist_theta{Theta0_mean, Theta0_err};

  auto accepted = 0;
  auto rejected = 0;

  for (auto i = 0; i != N; ++i) {
    tb::Particle pos{0., dist_y(eng), dist_theta(eng)};

    if (pos.y > border->r1() || pos.y < -border->r1()) {
      ++rejected;
      continue;
    }

    SingleResult final;
    try {
      final = simulateFinalState(pos, border);
    } catch (const std::exception& e) {
      final = {0., 0., 0., false};
    }

    if (!final.valid) {
      ++rejected;
      continue;
    }

    finalPosY.push_back(final.y);
    finalPosTheta.push_back(final.theta);
    ++accepted;
  }

  return {finalPosY, finalPosTheta, accepted, rejected};
}

}  // namespace tb