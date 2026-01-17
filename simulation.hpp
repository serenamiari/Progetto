#ifndef TB_VISUAL_HPP
#define TB_VISUAL_HPP

#include <SFML/Graphics.hpp>
#include <string>
#include <iomanip>
#include <sstream>
#include <iostream>
#include "triangularbilliards.hpp"

namespace tb {

sf::Vector2f normalize(const sf::Vector2f& v);
float distance(const sf::Vector2f& a, const sf::Vector2f& b);
std::string formatCoords(const tb::Particle& pos);

void runSimulation(const std::vector<tb::Particle>& collisions,
                   const tb::Border* borders);

}  // namespace tb

#endif  // TB_VISUAL_HPP
