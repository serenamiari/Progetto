#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "statistics.hpp"

#include "doctest.h"

TEST_CASE("Testing the class handling a floating point data sample") {
  tb::Sample sample;

  REQUIRE(sample.size() == 0);

  SUBCASE("Calling size() with four points") {
    sample.add(1.0);
    sample.add(1.5);
    sample.add(2.0);
    sample.add(2.5);
    CHECK(sample.size() == 4);
  }

  SUBCASE("Calling statistics() with no points throws") {
    CHECK_THROWS(sample.statistics());
  }

  SUBCASE("Calling statistics() with one point throws") {
    sample.add(4.0);
    CHECK_THROWS(sample.statistics());
  }

  SUBCASE("Calling statistics() with two points") {
    sample.add(1.);
    sample.add(2.);
    CHECK_THROWS(sample.statistics());
  }

  SUBCASE("Calling statistics() with five points - symmetric") {
    sample.add(1.5);
    sample.add(2.0);
    sample.add(2.5);
    sample.add(3.0);
    sample.add(3.5);
    auto result = sample.statistics();
    CHECK(result.mean == doctest::Approx(2.5));
    CHECK(result.sigma == doctest::Approx(0.79056942));
    CHECK(result.skewness == 0.0);
    CHECK(result.kurtosis == doctest::Approx(-1.2));
  }

  SUBCASE("Calling statistics() with five points - asymmetric") {
    sample.add(4.0);
    sample.add(7.0);
    sample.add(12.0);
    sample.add(12.5);
    sample.add(13.0);
    CHECK(sample.size() == 5);
    auto result = sample.statistics();
    CHECK(result.mean == doctest::Approx(9.7));
    CHECK(result.sigma == doctest::Approx(3.9937));
    CHECK(result.skewness == doctest::Approx(-0.8819).epsilon(.01));
    CHECK(result.kurtosis == doctest::Approx(-1.53));
  }

  SUBCASE("Calling statistics() with six points") {
    sample.add(0.0);
    sample.add(-17.0);
    sample.add(2.0);
    sample.add(150.0);
    sample.add(13.0);
    sample.add(10.0);
    CHECK(sample.size() == 6);
    auto result = sample.statistics();
    CHECK(result.mean == doctest::Approx(26.3333));
    CHECK(result.sigma == doctest::Approx(61.4839));
    CHECK(result.skewness == doctest::Approx(2.2853).epsilon(.01));
    CHECK(result.kurtosis == doctest::Approx(5.4253));
  }

  SUBCASE("Calling statistics() with twelve points") {
    sample.add(0.2);
    sample.add(-0.5);
    sample.add(0.9);
    sample.add(-0.1);
    sample.add(1.0);
    sample.add(-0.75);
    sample.add(0.64);
    sample.add(0.24);
    sample.add(-0.37);
    sample.add(0.00);
    sample.add(0.10);
    sample.add(-0.16);
    CHECK(sample.size() == 12);
    auto result = sample.statistics();
    CHECK(result.mean == doctest::Approx(0.1000));
    CHECK(result.sigma == doctest::Approx(0.5387));
    CHECK(result.skewness == doctest::Approx(0.3082).epsilon(.01));
    CHECK(result.kurtosis == doctest::Approx(-0.5571).epsilon(.0001));
  }

  SUBCASE("Calling statistics() with large, positive values") {
    sample.add(1E6);
    sample.add(1E6 + 1);
    sample.add(1E6 + 2);
    sample.add(1E6 + 3);
    CHECK(sample.size() == 4);
    auto result = sample.statistics();
    CHECK(result.mean == doctest::Approx(1E6));
    CHECK(result.sigma == doctest::Approx(1.2910));
    CHECK(result.skewness == doctest::Approx(0.000));
    CHECK(result.kurtosis == doctest::Approx(-1.200));
  }

  SUBCASE("Calling statistics() with small, negative values") {
    sample.add(-0.0002);
    sample.add(-0.000016);
    sample.add(-0.00005);
    sample.add(-0.00003);
    sample.add(-0.000064);
    sample.add(-0.0000104);
    CHECK(sample.size() == 6);
    auto result = sample.statistics();
    CHECK(result.mean == doctest::Approx(-6.1733e-5));
    CHECK(result.sigma == doctest::Approx(7.0702e-5));
    CHECK(result.skewness == doctest::Approx(-2.0192).epsilon(0.0001));
    CHECK(result.kurtosis == doctest::Approx(4.3344));
  }

  SUBCASE("Calling statistics() with five equal values") {
    sample.add(4.004);
    sample.add(4.004);
    sample.add(4.004);
    sample.add(4.004);
    sample.add(4.004);
    CHECK(sample.size() == 5);
    auto result = sample.statistics();
    CHECK(result.mean == doctest::Approx(4.004));
    CHECK(result.sigma == doctest::Approx(0));
    CHECK(result.skewness == doctest::Approx(0));
    CHECK(result.kurtosis == doctest::Approx(0));
  }

  SUBCASE("Calling statistics() with values of different precision") {
    sample.add(1);
    sample.add(2.2);
    sample.add(3.33);
    sample.add(4.444);
    sample.add(5.5555);
    sample.add(7.00000007);
    CHECK(sample.size() == 6);
    auto result = sample.statistics();
    CHECK(result.mean == doctest::Approx(3.9216));
    CHECK(result.sigma == doctest::Approx(2.2031));
    CHECK(result.skewness == doctest::Approx(0.0962).epsilon(0.0001));
    CHECK(result.kurtosis == doctest::Approx(-0.9382).epsilon(0.0001));
  }

  SUBCASE("Removing all points") {
    sample.add(0.0);
    sample.add(-17.0);
    sample.add(2.0);
    sample.add(150.0);
    sample.add(13.0);
    sample.add(10.0);
    CHECK(sample.size() == 6);
    sample.remove_all();
    CHECK(sample.size() == 0);
    CHECK_THROWS(sample.statistics());
  }
}