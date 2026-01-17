#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "triangularbilliards.hpp"

TEST_CASE("Testing border hit") {
  SUBCASE("Border closed") {
    std::unique_ptr<tb::Border> border;
    border = std::make_unique<tb::ClosedBorder>(20., 15., 50.);

    CHECK(border->r1() == doctest::Approx(20));
    CHECK(border->r2() == doctest::Approx(15));
    CHECK(border->xEnd() == doctest::Approx(50));
    CHECK(border->getSlope() == doctest::Approx(-.1));

    SUBCASE("Particle moving upwards - Y0 > 0") {
      tb::Particle p = {0., 5.0, .785};
      CHECK(border->checkCollision(p) == tb::BorderHit::Top);
    }

    SUBCASE("Particle moving upwards - Y0 = 0") {
      tb::Particle p = {0., 0., .785};
      CHECK(border->checkCollision(p) == tb::BorderHit::Top);
    }

    SUBCASE("Particle moving upwards - Y0 < 0") {
      tb::Particle p = {0., -5.0, .785};
      CHECK(border->checkCollision(p) == tb::BorderHit::Top);
    }

    SUBCASE("Particle moving on a straight line - Y0 > 0") {
      tb::Particle p = {0., 5.0, .0};
      CHECK(border->checkCollision(p) == tb::BorderHit::Top);
    }

    SUBCASE("Particle moving on a straight line - Y0 = 0") {
      tb::Particle p = {0., 0., .0};
      CHECK(border->checkCollision(p) == tb::BorderHit::None);
    }

    SUBCASE("Particle moving on a straight line - Y0 < 0") {
      tb::Particle p = {0., -5.0, .0};
      CHECK(border->checkCollision(p) == tb::BorderHit::Bottom);
    }

    SUBCASE("Particle moving downwards - Y0 > 0") {
      tb::Particle p = {0., 5.0, -.785};
      CHECK(border->checkCollision(p) == tb::BorderHit::Bottom);
    }

    SUBCASE("Particle moving downwards - Y0 = 0") {
      tb::Particle p = {0., 0., -.785};
      CHECK(border->checkCollision(p) == tb::BorderHit::Bottom);
    }

    SUBCASE("Particle moving downwards - Y0 < 0") {
      tb::Particle p = {0., -5.0, -.785};
      CHECK(border->checkCollision(p) == tb::BorderHit::Bottom);
    }
  }

  SUBCASE("Border straight") {
    std::unique_ptr<tb::Border> border;
    border = std::make_unique<tb::StraightBorder>(20., 20., 50.);

    CHECK(border->r1() == doctest::Approx(20));
    CHECK(border->r2() == doctest::Approx(20));
    CHECK(border->xEnd() == doctest::Approx(50));
    CHECK(border->getSlope() == doctest::Approx(0));

    SUBCASE("Particle moving upwards - Y0 > 0") {
      tb::Particle p = {0., 5.0, .785};
      CHECK(border->checkCollision(p) == tb::BorderHit::Top);
    }

    SUBCASE("Particle moving upwards - Y0 = 0") {
      tb::Particle p = {0., 0., .785};
      CHECK(border->checkCollision(p) == tb::BorderHit::Top);
    }

    SUBCASE("Particle moving upwards - Y0 < 0") {
      tb::Particle p = {0., -5.0, .785};
      CHECK(border->checkCollision(p) == tb::BorderHit::Top);
    }

    SUBCASE("Particle moving on a straight line - Y0 > 0") {
      tb::Particle p = {0., 5.0, .0};
      CHECK(border->checkCollision(p) == tb::BorderHit::None);
    }

    SUBCASE("Particle moving on a straight line - Y0 = 0") {
      tb::Particle p = {0., 0., .0};
      CHECK(border->checkCollision(p) == tb::BorderHit::None);
    }

    SUBCASE("Particle moving on a straight line - Y0 < 0") {
      tb::Particle p = {0., -5.0, .0};
      CHECK(border->checkCollision(p) == tb::BorderHit::None);
    }

    SUBCASE("Particle moving downwards - Y0 > 0") {
      tb::Particle p = {0., 5.0, -.785};
      CHECK(border->checkCollision(p) == tb::BorderHit::Bottom);
    }

    SUBCASE("Particle moving downwards - Y0 = 0") {
      tb::Particle p = {0., 0., -.785};
      CHECK(border->checkCollision(p) == tb::BorderHit::Bottom);
    }

    SUBCASE("Particle moving downwards - Y0 < 0") {
      tb::Particle p = {0., -5.0, -.785};
      CHECK(border->checkCollision(p) == tb::BorderHit::Bottom);
    }
  }

  SUBCASE("Border opened") {
    std::unique_ptr<tb::Border> border;
    border = std::make_unique<tb::OpenedBorder>(15., 20., 50.);

    CHECK(border->r1() == doctest::Approx(15));
    CHECK(border->r2() == doctest::Approx(20));
    CHECK(border->xEnd() == doctest::Approx(50));
    CHECK(border->getSlope() == doctest::Approx(.1));

    SUBCASE("Particle moving upwards - theta > sigma") {
      tb::Particle p = {0., 5.0, .785};
      CHECK(border->checkCollision(p) == tb::BorderHit::Top);
    }

    SUBCASE("Particle moving upwards - theta = sigma") {
      tb::Particle p = {
          0., 0., std::atan2(border->r2() - border->r1(), border->xEnd())};
      CHECK(border->checkCollision(p) == tb::BorderHit::None);
    }

    SUBCASE("Particle moving upwards - theta < sigma") {
      tb::Particle p = {0., -5.0, .005};
      CHECK(border->checkCollision(p) == tb::BorderHit::None);
    }

    SUBCASE("Particle moving on a straight line - Y0 > 0") {
      tb::Particle p = {0., 5.0, .0};
      CHECK(border->checkCollision(p) == tb::BorderHit::None);
    }

    SUBCASE("Particle moving on a straight line - Y0 = 0") {
      tb::Particle p = {0., 0., .0};
      CHECK(border->checkCollision(p) == tb::BorderHit::None);
    }

    SUBCASE("Particle moving on a straight line - Y0 < 0") {
      tb::Particle p = {0., -5.0, .0};
      CHECK(border->checkCollision(p) == tb::BorderHit::None);
    }

    SUBCASE("Particle moving downwards - theta > sigma") {
      tb::Particle p = {0., 5.0, -.785};
      CHECK(border->checkCollision(p) == tb::BorderHit::Bottom);
    }

    SUBCASE("Particle moving downwards - theta = sigma") {
      tb::Particle p = {
          0., 0., std::atan2(border->r2() - border->r1(), border->xEnd())};
      CHECK(border->checkCollision(p) == tb::BorderHit::None);
    }

    SUBCASE("Particle moving downwards - theta < sigma") {
      tb::Particle p = {0., -5.0, -.0785};
      CHECK(border->checkCollision(p) == tb::BorderHit::None);
    }
  }
}

TEST_CASE("Testing reduceAngle() function") {
  SUBCASE("Angle equal to 0 - to be reduced") {
    double x = 8 * M_PI;
    tb::reduceAngle(x);
    CHECK(x == doctest::Approx(0));
  }
  SUBCASE("Angle equal to 0 - negative - to be reduced") {
    double x = -8 * M_PI;
    tb::reduceAngle(x);
    CHECK(x == doctest::Approx(0));
  }
  SUBCASE("Angle in first quadrant - to be reduced") {
    double x = 13 * M_PI / 3;
    tb::reduceAngle(x);
    CHECK(x == doctest::Approx(M_PI / 3));
  }
  SUBCASE("Angle in second quadrant - to be reduced") {
    double x = 17 * M_PI / 6;
    tb::reduceAngle(x);
    CHECK(x == doctest::Approx(5 * M_PI / 6));
  }
  SUBCASE("Angle in third quadrant - to be reduced") {
    double x = 31 * M_PI / 6;
    tb::reduceAngle(x);
    CHECK(x == doctest::Approx(7 * M_PI / 6));
  }
  SUBCASE("Angle in fourth quadrant - to be reduced") {
    double x = 11 * M_PI / 3;
    tb::reduceAngle(x);
    CHECK(x == doctest::Approx(5 * M_PI / 3));
  }
  SUBCASE("Angle in fourth quadrant - negative - to be reduced") {
    double x = -9 * M_PI / 4;
    tb::reduceAngle(x);
    CHECK(x == doctest::Approx(-M_PI / 4));
  }
  SUBCASE("Does not need to be reduced") {
    double x = M_PI;
    tb::reduceAngle(x);
    CHECK(x == doctest::Approx(M_PI));
  }
  SUBCASE("Large angle - even") {
    double x = 1e6 * M_PI;
    tb::reduceAngle(x);
    CHECK(x == doctest::Approx(2 * M_PI));
  }
  SUBCASE("Large angle - odd") {
    double x = (1e6 + 1) * M_PI;
    tb::reduceAngle(x);
    CHECK(x == doctest::Approx(M_PI));
  }
}

TEST_CASE("Testing computeNextCollision() function") {
  SUBCASE("Border closed") {
    std::unique_ptr<tb::Border> border;
    border = std::make_unique<tb::ClosedBorder>(20., 15., 50.);

    SUBCASE("Particle moving upwards") {
      tb::Particle p = {0., 3.0, .32};
      tb::computeNextCollision(p, border.get(), std::atan(border->getSlope()));
      tb::Particle result = {39.4076, 16.0592, -0.5193};
      CHECK(p.x == doctest::Approx(result.x).epsilon(0.001));
      CHECK(p.y == doctest::Approx(result.y).epsilon(0.001));
      CHECK(p.theta == doctest::Approx(result.theta).epsilon(0.001));
    }

    SUBCASE("Particle moving on a straight line") {
      tb::Particle p = {0., 18.0, .0};
      tb::computeNextCollision(p, border.get(), std::atan(border->getSlope()));
      tb::Particle result = {20., 18., -0.1992};
      CHECK(p.x == doctest::Approx(result.x).epsilon(0.001));
      CHECK(p.y == doctest::Approx(result.y).epsilon(0.001));
      CHECK(p.theta == doctest::Approx(result.theta).epsilon(0.001));
    }

    SUBCASE("Particle moving downwards") {
      tb::Particle p = {0., 18.0, -.785};
      tb::computeNextCollision(p, border.get(), std::atan(border->getSlope()));
      tb::Particle result = {34.5704, -16.5429, 0.9843};
      CHECK(p.x == doctest::Approx(result.x).epsilon(0.001));
      CHECK(p.y == doctest::Approx(result.y).epsilon(0.001));
      CHECK(p.theta == doctest::Approx(result.theta).epsilon(0.001));
    }
  }

  SUBCASE("Border straight") {
    std::unique_ptr<tb::Border> border;
    border = std::make_unique<tb::StraightBorder>(20., 20., 50.);

    SUBCASE("Particle moving upwards") {
      tb::Particle p = {0., 5.0, .85};
      tb::computeNextCollision(p, border.get(), std::atan(border->getSlope()));
      tb::Particle result = {13.1771, 20, -.85};
      CHECK(p.x == doctest::Approx(result.x).epsilon(0.001));
      CHECK(p.y == doctest::Approx(result.y).epsilon(0.001));
      CHECK(p.theta == doctest::Approx(result.theta).epsilon(0.001));
    }

    SUBCASE("Particle moving downwards") {
      tb::Particle p = {0., 18.0, -.785};
      tb::computeNextCollision(p, border.get(), std::atan(border->getSlope()));
      tb::Particle result = {38.0302, -20, 0.785};
      CHECK(p.x == doctest::Approx(result.x).epsilon(0.001));
      CHECK(p.y == doctest::Approx(result.y).epsilon(0.001));
      CHECK(p.theta == doctest::Approx(result.theta).epsilon(0.001));
    }
  }

  SUBCASE("Border opened") {
    std::unique_ptr<tb::Border> border;
    border = std::make_unique<tb::OpenedBorder>(15., 20., 50.);

    SUBCASE("Particle moving upwards") {
      tb::Particle p = {0., 5.0, .85};
      tb::computeNextCollision(p, border.get(), std::atan(border->getSlope()));
      tb::Particle result = {9.6308, 15.9631, -.6508};
      CHECK(p.x == doctest::Approx(result.x).epsilon(0.001));
      CHECK(p.y == doctest::Approx(result.y).epsilon(0.001));
      CHECK(p.theta == doctest::Approx(result.theta).epsilon(0.001));
    }

    SUBCASE("Particle moving downwards") {
      tb::Particle p = {5.0, 5.0, -.64};
      tb::computeNextCollision(p, border.get(), std::atan(border->getSlope()));
      tb::Particle result = {36.8054, -18.6805, 0.4408};
      CHECK(p.x == doctest::Approx(result.x).epsilon(0.001));
      CHECK(p.y == doctest::Approx(result.y).epsilon(0.001));
      CHECK(p.theta == doctest::Approx(result.theta).epsilon(0.001));
    }
  }
}

TEST_CASE("Testing computeFinalPosition() function") {
  SUBCASE("Border closed") {
    std::unique_ptr<tb::Border> border;
    border = std::make_unique<tb::ClosedBorder>(20., 15., 50.);

    SUBCASE("Particle moving upwards") {
      tb::Particle p = {45., -5.0, .964};
      tb::computeFinalPosition(p, border.get());
      tb::Particle result = {50, 2.2029, .964};
      CHECK(p.x == doctest::Approx(result.x).epsilon(0.001));
      CHECK(p.y == doctest::Approx(result.y).epsilon(0.001));
      CHECK(p.theta == doctest::Approx(result.theta).epsilon(0.001));
    }

    SUBCASE("Particle moving on a straight line") {
      tb::Particle p = {45., 3.0, .0};
      tb::computeFinalPosition(p, border.get());
      tb::Particle result = {50., 3., 0.};
      CHECK(p.x == doctest::Approx(result.x).epsilon(0.001));
      CHECK(p.y == doctest::Approx(result.y).epsilon(0.001));
      CHECK(p.theta == doctest::Approx(result.theta).epsilon(0.001));
    }

    SUBCASE("Particle moving downwards") {
      tb::Particle p = {45., -5.0, -.21};
      tb::computeFinalPosition(p, border.get());
      tb::Particle result = {50, -6.0657, -.21};
      CHECK(p.x == doctest::Approx(result.x).epsilon(0.001));
      CHECK(p.y == doctest::Approx(result.y).epsilon(0.001));
      CHECK(p.theta == doctest::Approx(result.theta).epsilon(0.001));
    }
  }

  SUBCASE("Border straight") {
    std::unique_ptr<tb::Border> border;
    border = std::make_unique<tb::StraightBorder>(20., 20., 50.);

    SUBCASE("Particle moving upwards") {
      tb::Particle p = {45., -5.0, .964};
      tb::computeFinalPosition(p, border.get());
      tb::Particle result = {50, 2.2029, .964};
      CHECK(p.x == doctest::Approx(result.x).epsilon(0.001));
      CHECK(p.y == doctest::Approx(result.y).epsilon(0.001));
      CHECK(p.theta == doctest::Approx(result.theta).epsilon(0.001));
    }

    SUBCASE("Particle moving on a straight line") {
      tb::Particle p = {20.0, 5.0, 0.0};
      tb::computeFinalPosition(p, border.get());
      tb::Particle result = {50., 5.0, 0.0};
      CHECK(p.x == doctest::Approx(result.x).epsilon(0.001));
      CHECK(p.y == doctest::Approx(result.y).epsilon(0.001));
      CHECK(p.theta == doctest::Approx(result.theta).epsilon(0.001));
    }

    SUBCASE("Particle moving downwards") {
      tb::Particle p = {46., -12.0, -.8392};
      tb::computeFinalPosition(p, border.get());
      tb::Particle result = {50, -16.4554, -.8392};
      CHECK(p.x == doctest::Approx(result.x).epsilon(0.001));
      CHECK(p.y == doctest::Approx(result.y).epsilon(0.001));
      CHECK(p.theta == doctest::Approx(result.theta).epsilon(0.001));
    }
  }

  SUBCASE("Border opened") {
    std::unique_ptr<tb::Border> border;
    border = std::make_unique<tb::OpenedBorder>(15., 20., 50.);

    SUBCASE("Particle moving upwards") {
      tb::Particle p = {34., -16.0, .8392};
      tb::computeFinalPosition(p, border.get());
      tb::Particle result = {50., 1.8214, .8392};
      CHECK(p.x == doctest::Approx(result.x).epsilon(0.001));
      CHECK(p.y == doctest::Approx(result.y).epsilon(0.001));
      CHECK(p.theta == doctest::Approx(result.theta).epsilon(0.001));
    }

    SUBCASE("Particle moving on a straight line") {
      tb::Particle p = {20.0, 2.0, 0.0};
      tb::computeFinalPosition(p, border.get());
      tb::Particle result = {50., 2.0, 0.0};
      CHECK(p.x == doctest::Approx(result.x).epsilon(0.001));
      CHECK(p.y == doctest::Approx(result.y).epsilon(0.001));
      CHECK(p.theta == doctest::Approx(result.theta).epsilon(0.001));
    }

    SUBCASE("Particle moving downwards") {
      tb::Particle p = {46., -12.0, -.8392};
      tb::computeFinalPosition(p, border.get());
      tb::Particle result = {50., -16.4554, -.8392};
      CHECK(p.x == doctest::Approx(result.x).epsilon(0.001));
      CHECK(p.y == doctest::Approx(result.y).epsilon(0.001));
      CHECK(p.theta == doctest::Approx(result.theta).epsilon(0.001));
    }
  }
}

TEST_CASE("Testing trajectory class") {
  SUBCASE(
      "Trajectory of a particle that collides and exits the system - testing "
      "multiple functionalities") {
    std::unique_ptr<tb::Border> border;
    border = std::make_unique<tb::ClosedBorder>(20., 15., 50.);
    tb::Particle p = {0., 5., .7853982};
    tb::Trajectory traj(p);
    CHECK(traj.size() == 1);
    traj.simulateCollisions(border.get());
    CHECK(traj.size() == 5);
    auto finalpos = traj.getFinalPosition();
    CHECK(finalpos.x == doctest::Approx(50));
    CHECK(traj.positions()[1].x == doctest::Approx(13.6364));
    CHECK(traj.positions()[1].y == doctest::Approx(18.6364));
  }

  SUBCASE("Testing 'particle moves backwards' exception") {
    std::unique_ptr<tb::Border> border;
    border = std::make_unique<tb::ClosedBorder>(20., 15., 50.);
    tb::Particle p = {0., 5., 3.7853982};
    tb::Trajectory traj(p);
    CHECK_THROWS(traj.simulateCollisions(border.get()));
  }

  SUBCASE("Testing 'particle moves backwards' exception") {
    std::unique_ptr<tb::Border> border;
    border = std::make_unique<tb::ClosedBorder>(20., 2., 50.);
    tb::Particle p = {0., 18., 0.0};
    tb::Trajectory traj(p);
    CHECK_THROWS(traj.simulateCollisions(border.get()));
  }

  SUBCASE(
      "Testing invalid conditions - straight border and theta equal to PI/2") {
    std::unique_ptr<tb::Border> border;
    border = std::make_unique<tb::StraightBorder>(20., 20., 50.);
    tb::Particle p = {0., 0., M_PI / 2};
    tb::Trajectory traj(p);
    CHECK_THROWS(traj.simulateCollisions(border.get()));
  }

  SUBCASE(
      "Testing invalid conditions - straight border and theta equal to -PI/2") {
    std::unique_ptr<tb::Border> border;
    border = std::make_unique<tb::StraightBorder>(20., 20., 50.);
    tb::Particle p = {0., 0., -M_PI / 2};
    tb::Trajectory traj(p);
    CHECK_THROWS(traj.simulateCollisions(border.get()));
  }

  SUBCASE(
      "Testing invalid conditions - straight border and theta equal to -PI/2") {
    std::unique_ptr<tb::Border> border;
    border = std::make_unique<tb::StraightBorder>(20., 20., 50.);
    tb::Particle p = {0., 0., -9 * M_PI / 2};
    tb::Trajectory traj(p);
    CHECK_THROWS(traj.simulateCollisions(border.get()));
  }

  SUBCASE("Testing trajectory of a particle with near-zero theta") {
    std::unique_ptr<tb::Border> border =
        std::make_unique<tb::StraightBorder>(20., 20., 50.);
    tb::Particle p = {0., 5., 1e-8};
    tb::Trajectory traj(p);
    traj.simulateCollisions(border.get());
    CHECK(traj.getFinalPosition().x == doctest::Approx(50.));
    CHECK(traj.getFinalPosition().y == doctest::Approx(5.));
    CHECK(traj.getFinalPosition().theta == doctest::Approx(0.));
  }

  SUBCASE("Testing trajectory of a particle starting at x = l") {
    std::unique_ptr<tb::Border> border =
        std::make_unique<tb::StraightBorder>(20., 20., 50.);
    tb::Particle p = {50., 5., 0.1};
    tb::Trajectory traj(p);
    traj.simulateCollisions(border.get());
    CHECK(traj.size() == 2);
    CHECK(traj.getFinalPosition().x == doctest::Approx(50.));
    CHECK(traj.getFinalPosition().y == doctest::Approx(5.));
    CHECK(traj.getFinalPosition().theta == doctest::Approx(0.1));
  }
}

TEST_CASE("Testing runMultipleSimulations() function") {
  SUBCASE("Testing N - valid initial conditions to simulate trajectory") {
    std::unique_ptr<tb::Border> border =
        std::make_unique<tb::StraightBorder>(20., 15., 50.);
    auto N = 10000;
    auto Y0_err = 0.01;
    auto Theta0_err = 0.001;
    tb::MultipleResult result =
        tb::runMultipleSimulations(N, 5., Y0_err, 0.785, Theta0_err, border.get());
    CHECK(result.accepted + result.rejected == N);
    CHECK(result.accepted > 0);
    CHECK(result.accepted == result.finalY.size());
    CHECK(result.accepted == result.finalTheta.size());
  }

  SUBCASE(
      "Testing N - initial mean conditions generate particle that moves "
      "backwards") {
    std::unique_ptr<tb::Border> border =
        std::make_unique<tb::StraightBorder>(20., 2., 20.);
    auto N = 10000;
    auto Y0_err = 0.01;
    auto Theta0_err = 0.001;
    tb::MultipleResult result = tb::runMultipleSimulations(
        N, 18., Y0_err, 0.0, Theta0_err, border.get());
    CHECK(result.accepted + result.rejected == N);
    CHECK(result.rejected >= 0);
  }

  SUBCASE("Testing negative errors") {
    std::unique_ptr<tb::Border> border =
        std::make_unique<tb::StraightBorder>(20., 2., 20.);
    auto N = 10000;
    auto Y0_err = -0.01;
    auto Theta0_err = -0.001;
    tb::MultipleResult result =
        tb::runMultipleSimulations(N, 18., Y0_err, 0.0, Theta0_err, border.get());
    CHECK(result.accepted + result.rejected == N);
    CHECK(result.rejected >= 0);
  }
}