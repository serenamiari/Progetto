#include "simulation.hpp"

namespace tb {

sf::Vector2f normalize(const sf::Vector2f& v) {
  float length = std::sqrt(v.x * v.x + v.y * v.y);
  if (length != 0.f) return v / length;
  return {0.f, 0.f};
}

float distance(const sf::Vector2f& a, const sf::Vector2f& b) {
  return std::hypot(b.x - a.x, b.y - a.y);
}

std::string formatCoords(const tb::Particle& pos) {
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(2);
  ss << "X: " << pos.x << "  Y: " << pos.y << "  Angle: " << pos.theta;
  return ss.str();
}

void runSimulation(const std::vector<tb::Particle>& collisions,
                   const tb::Border* borders) {
  const float windowWidth = 1000.f;
  const float windowHeight = 600.f;
  const float margin = 40.f;

  sf::RenderWindow window(
      sf::VideoMode(static_cast<unsigned int>(windowWidth),
                    static_cast<unsigned int>(windowHeight)),
      "Triangular Billiards");

  std::vector<sf::Vector2f> path;
  for (const auto& p : collisions) path.emplace_back(p.x, p.y);

  auto computeBounds =
      [](const std::vector<sf::Vector2f>& pts) -> sf::FloatRect {
    if (pts.empty()) return {};
    float minX = pts[0].x, maxX = pts[0].x;
    float minY = pts[0].y, maxY = pts[0].y;
    for (const auto& pt : pts) {
      minX = std::min(minX, pt.x);
      maxX = std::max(maxX, pt.x);
      minY = std::min(minY, pt.y);
      maxY = std::max(maxY, pt.y);
    }
    return {minX, minY, maxX - minX, maxY - minY};
  };

  sf::FloatRect bounds = computeBounds(path);

  float r1 = static_cast<float>(borders->r1());
  float r2 = static_cast<float>(borders->r2());
  float L = static_cast<float>(borders->xEnd());

  bounds.left = std::min(bounds.left, 0.f);
  bounds.width = std::max(bounds.width, L);
  bounds.top = std::min(bounds.top, -r1);
  bounds.height = std::max(bounds.height, std::max(r1, r2) * 2.f);

  float scaleX = (windowWidth - 2 * margin) / bounds.width;
  float scaleY = (windowHeight - 2 * margin) / bounds.height;
  float scale = std::min(scaleX, scaleY);
  float originY = windowHeight / 2.f;

  auto toWindowCoords = [scale,
                         originY](const sf::Vector2f& pos) -> sf::Vector2f {
    return {pos.x * scale, originY - pos.y * scale};
  };
  auto fromWindowCoords =
      [scale, originY](const sf::Vector2f& winPos) -> sf::Vector2f {
    return {winPos.x / scale, (originY - winPos.y) / scale};
  };

  sf::CircleShape particle(2.f);
  particle.setOrigin(2.f, 2.f);
  particle.setFillColor(sf::Color::Red);
  particle.setPosition(toWindowCoords(path[0]));

  sf::Vertex topBorder[] = {
      sf::Vertex(toWindowCoords({0.f, r1}), sf::Color::Yellow),
      sf::Vertex(toWindowCoords({L, r2}), sf::Color::Yellow)};
  sf::Vertex bottomBorder[] = {
      sf::Vertex(toWindowCoords({0.f, -r1}), sf::Color::Red),
      sf::Vertex(toWindowCoords({L, -r2}), sf::Color::Red)};
  sf::Vertex centerLine[] = {
      sf::Vertex({0.f, originY}, sf::Color(55, 55, 55, 120)),
      sf::Vertex({windowWidth, originY}, sf::Color(55, 55, 55, 120))};
  sf::Vertex l_axis[] = {
      sf::Vertex(toWindowCoords({L, bounds.top - 1}),
                 sf::Color(55, 55, 55, 120)),
      sf::Vertex(toWindowCoords({L, bounds.top + bounds.height + 1}),
                 sf::Color(55, 55, 55, 120))};

  std::vector<sf::CircleShape> trailDots;
  std::vector<sf::Text> waypointTexts;
  std::vector<std::string> passedCoords;

  size_t currentTarget = 1;
  float trailTimer = 0.f;
  float lastY = path[0].y;
  bool isStopped = false;
  bool pathCompleted = false;

  sf::Clock moveClock;

  sf::Font font;
  if (!font.loadFromFile("arial.ttf")) {
    std::cerr << "Failed to load font!" << std::endl;
    return;
  }

  sf::Text coordText;
  coordText.setFont(font);
  coordText.setCharacterSize(14);
  coordText.setFillColor(sf::Color::White);

  sf::Text restartHint("Press Space to restart", font, 14);
  restartHint.setFillColor(sf::Color(180, 180, 180));
  restartHint.setPosition(750.f, 550.f);

  sf::Text pauseHint("Press Space to pause", font, 14);
  pauseHint.setFillColor(sf::Color(180, 180, 180));
  pauseHint.setPosition(750.f, 550.f);

  float speed = 150.f;

  auto restartSimulation = [&]() {
    particle.setPosition(toWindowCoords(path[0]));
    particle.setFillColor(sf::Color::Red);
    trailDots.clear();
    waypointTexts.clear();
    passedCoords.clear();
    currentTarget = 1;
    pathCompleted = false;
    isStopped = false;
    moveClock.restart();
  };

  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) window.close();
      if (event.type == sf::Event::KeyPressed &&
          event.key.code == sf::Keyboard::Space) {
        if (pathCompleted)
          restartSimulation();
        else
          isStopped = !isStopped;
      }
      if (event.type == sf::Event::KeyPressed) {
        if (event.key.code == sf::Keyboard::Up) speed += 20.f;
        if (event.key.code == sf::Keyboard::Down)
          speed = std::max(20.f, speed - 20.f);
      }
    }

    float dt = moveClock.restart().asSeconds();

    if (!isStopped && !pathCompleted && currentTarget < path.size()) {
      sf::Vector2f currentPos = particle.getPosition();
      sf::Vector2f targetPos = toWindowCoords(path[currentTarget]);
      sf::Vector2f direction = targetPos - currentPos;
      float len = std::hypot(direction.x, direction.y);
      if (len != 0.f) direction /= len;

      sf::Vector2f moveVec = direction * speed * dt;

      if (len > speed * dt) {
        particle.move(moveVec);
      } else {
        particle.setPosition(targetPos);

        std::string label = formatCoords(collisions[currentTarget]);
        passedCoords.push_back(label);

        sf::Text waypointLabel;
        waypointLabel.setFont(font);
        waypointLabel.setCharacterSize(14);
        waypointLabel.setFillColor(sf::Color::White);
        waypointLabel.setString(label);
        waypointLabel.setPosition(
            750.f, 30.f + static_cast<float>(waypointTexts.size()) * 20.f);
        waypointTexts.push_back(waypointLabel);

        currentTarget++;
        if (currentTarget >= path.size()) pathCompleted = true;
      }
    }

    trailTimer += dt;
    if (trailTimer >= 0.02f && !pathCompleted) {
      trailTimer = 0.f;
      sf::CircleShape dot(1.f);
      dot.setFillColor(sf::Color(0, 55, 155, 120));
      dot.setOrigin(1.f, 1.f);
      dot.setPosition(particle.getPosition());
      trailDots.push_back(dot);
    }

    sf::Vector2f currentWorldPos = fromWindowCoords(particle.getPosition());
    float theta = 0.f;
    if (currentTarget > 0 && currentTarget < path.size()) {
      sf::Vector2f prev = path[currentTarget - 1];
      sf::Vector2f next = path[currentTarget];
      sf::Vector2f dir = normalize(next - prev);
      theta = std::atan2(dir.y, dir.x);
    }

    tb::Particle currentLive{currentWorldPos.x, currentWorldPos.y, theta};
    coordText.setString(formatCoords(currentLive));
    coordText.setPosition(particle.getPosition().x + 10.f,
                          particle.getPosition().y - 20.f);

    if (currentWorldPos.y < lastY) {
      coordText.setFillColor(sf::Color::Red);
      particle.setFillColor(sf::Color::Red);
    } else if (currentWorldPos.y > lastY) {
      coordText.setFillColor(sf::Color::Yellow);
      particle.setFillColor(sf::Color::Yellow);
    }
    lastY = currentWorldPos.y;

    window.clear(sf::Color::Black);
    for (const auto& dot : trailDots) window.draw(dot);
    window.draw(particle);
    window.draw(coordText);
    window.draw(topBorder, 2, sf::Lines);
    window.draw(bottomBorder, 2, sf::Lines);
    window.draw(centerLine, 2, sf::Lines);
    window.draw(l_axis, 2, sf::Lines);
    for (const auto& text : waypointTexts) window.draw(text);
    if (pathCompleted)
      window.draw(restartHint);
    else
      window.draw(pauseHint);
    window.display();
  }
}

}  // namespace tb
