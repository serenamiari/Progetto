#include <fstream>
#include <iostream>

#include "simulation.hpp"
#include "statistics.hpp"
#include "triangularbilliards.hpp"

int main() {
  try {
    std::cout << "Valid commands: \n"
              << "- add borders [b R1 R2 L]\n"
              << "- calculate final conditions [f Y0 Theta0]\n"
              << "- run simulation of the trajectory [v (Y0) (Theta0)]\n"
              << "- generate data [g N Y0_mean Y0_err Theta0_mean Theta0_err]\n"
              << "- erase all values [e]\n"
              << "- print data [o]\n"
              << "- quit [q]\n";
    char cmd{};

    std::unique_ptr<tb::Border> border;
    bool bordersSet = false;
    tb::Particle pos{0., 0., 0.};
    bool particleSet = false;

    auto isValidInput = [](const tb::Particle& p, const tb::Border* b) {
      const auto two_pi = 2 * M_PI;
      return std::fmod(std::abs(p.theta), two_pi) <= M_PI / 2 &&
             p.y <= b->r1() && p.y >= -b->r1();
    };

    auto printStats = [](const tb::Statistics stats, const std::string& var) {
      std::cout << "Final " << var << " :\n - Mean : " << stats.mean
                << "\n - Sigma : " << stats.sigma
                << "\n - Skewness : " << stats.skewness
                << "\n - Kurtosis : " << stats.kurtosis << '\n';
    };

    int N;
    double Y0_mean;
    double Y0_err;
    double Theta0_mean;
    double Theta0_err;
    tb::MultipleResult resultMultiple;

    while (std::cin >> cmd) {
      if (cmd == 'b') {
        double r1;
        double r2;
        double l;
        std::cin >> r1 >> r2 >> l;

        if (r1 < 0.0 || r2 < 0.0 || l < 0.0) {
          throw std::runtime_error("Invalid border value(s)");
        }
        border = tb::createBorder(r1, r2, l);
        bordersSet = true;

      } else if (cmd == 'f' && std::cin >> pos.y && std::cin >> pos.theta) {
        if (!bordersSet) {
          throw std::runtime_error("Set borders before running command f");
        }
        if (!isValidInput(pos, border.get())) {
          throw std::runtime_error("Invalid initial conditions");
        }
        
        particleSet = true;
        auto finalPos = tb::simulateFinalState(pos, border.get());

        std::cout << "- Final X: " << finalPos.x
                  << "\n- Final Y: " << finalPos.y
                  << "\n- Final Theta: " << finalPos.theta << '\n';

      } else if (cmd == 'v') {
        if (!bordersSet) {
          throw std::runtime_error("Set borders before running command v");
        }
        if (!particleSet) {
          std::cin >> pos.y >> pos.theta;
        }

        auto const traj = tb::computeSingleTrajectory(pos, border.get());
        tb::runSimulation(traj.positions(), border.get());

      } else if (cmd == 'g' && std::cin >> N && std::cin >> Y0_mean &&
                 std::cin >> Y0_err && std::cin >> Theta0_mean &&
                 std::cin >> Theta0_err) {
        if (!bordersSet) {
          throw std::runtime_error("Set borders before running command g");
        }
        if (!isValidInput(pos, border.get())) {
          throw std::runtime_error("Invalid initial conditions");
        }

        resultMultiple = tb::runMultipleSimulations(
            N, Y0_mean, Y0_err, Theta0_mean, Theta0_err, border.get());
        std::cout << "Accepted: " << resultMultiple.accepted
                  << "\nRejected: " << resultMultiple.rejected << '\n';

        if (resultMultiple.accepted < 4) {
          throw std::runtime_error(
              "Not enough particles reach final conditions to run statistics");
        }
        const auto statsY = resultMultiple.finalY.statistics();
        const auto statsTheta = resultMultiple.finalTheta.statistics();

        printStats(statsY, "Y");
        printStats(statsTheta, "Theta");

      } else if (cmd == 'e') {
        resultMultiple.finalY.remove_all();
        resultMultiple.finalTheta.remove_all();

      } else if (cmd == 'o') {
        std::ofstream outfile{"results.txt"};

        if (!outfile) {
          throw std::runtime_error{"Impossible to open file!"};
        }

        for (size_t i = 0; i != resultMultiple.finalY.size(); ++i) {
          outfile << resultMultiple.finalY.values()[i] << " "
                  << resultMultiple.finalTheta.values()[i] << '\n';
        }

        std::cout << "Output file written successfully. " << '\n';

      } else if (cmd == 'q') {
        return EXIT_SUCCESS;

      } else {
        std::cout << "Command not found, insert new command:\n";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      }
    }
  } catch (std::exception const& e) {
    std::cerr << "Caught exception: '" << e.what() << "'\n";
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "Caught unknown exception\n";
    return EXIT_FAILURE;
  }
}