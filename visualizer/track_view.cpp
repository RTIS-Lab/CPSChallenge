/*
 * Track View — SFML visualizer for LateralMotionControl FMU output
 * Supports multi-vehicle traces and detailed diagnostics.
 */

#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <map>

// ─── Configuration ─────────────────────────────────────────────────

static constexpr float INITIAL_VIEW_W = 120.f;
static constexpr float VEHICLE_RADIUS = 2.0f;
static constexpr float REF_RADIUS = 1.0f;
static constexpr float TRACK_HW = 0.7f;
static constexpr float TRAIL_HW = 0.55f;
static constexpr float ERR_LINE_HW = 0.3f;
static constexpr size_t TRAIL_MAX = 1200;
static constexpr size_t TRACK_FULL_MAX = 5000;
static constexpr size_t TRACK_LOCAL_MAX = 2000;

// ─── Data Structures ───────────────────────────────────────────────

struct Frame {
  double time = 0;
  int vehicle_id = 0;
  double phi_dot = 0;
  double beta = 0;
  double delta = 0;
  double delta_dot = 0;
  double e_y = 0;
  double e_y_dot = 0;
  double act_out = 0;
  double velocity = 0;
  double x_track = 0;
  double y_track = 0;
  double xveh = 0;
  double yveh = 0;
  double rolling_perf = 0;
  double avg_perf = 0;
  int thresh_errors = 0;
  bool critical = false;
  bool violated = false;
  double ff_ref_0 = 0;
  double ff_ref_1 = 0;
  bool trig_sens = false;
  bool trig_net_sc = false;
  bool trig_est = false;
  bool trig_ctrl = false;
  bool trig_ff = false;
  bool trig_merger = false;
  bool trig_net_ca = false;
  bool trig_act = false;
};

// ─── Helpers ────────────────────────────────────────────────────────

static std::vector<double> loadCSVColumn(const std::string &path) {
  std::vector<double> data;
  std::ifstream file(path);
  if (!file.is_open()) return data;
  double val;
  while (file >> val) data.push_back(val);
  return data;
}

static bool loadCSVFrames(const std::string &path, std::map<int, std::vector<Frame>> &vehicleFrames) {
  std::ifstream file(path);
  if (!file.is_open()) return false;
  std::string line;
  if (!std::getline(file, line)) return false;

  size_t count = 0;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] < '0' || line[0] > '9') continue;
    std::istringstream ss(line);
    std::string field;
    Frame f;
    int col = 0;
    while (std::getline(ss, field, ',')) {
      double val = 0;
      try { val = std::stod(field); } catch (...) { col++; continue; }
      switch (col) {
      case 0: f.time = val; break;
      case 1: f.vehicle_id = (int)val; break;
      case 2: f.phi_dot = val; break;
      case 3: f.beta = val; break;
      case 4: f.delta = val; break;
      case 5: f.delta_dot = val; break;
      case 6: f.e_y = val; break;
      case 7: f.e_y_dot = val; break;
      case 8: f.act_out = val; break;
      case 9: f.velocity = val; break;
      case 10: f.x_track = val; break;
      case 11: f.y_track = val; break;
      case 12: f.xveh = val; break;
      case 13: f.yveh = val; break;
      case 14: f.rolling_perf = val; break;
      case 15: f.avg_perf = val; break;
      case 16: f.thresh_errors = (int)val; break;
      case 17: f.critical = (val != 0); break;
      case 18: f.violated = (val != 0); break;
      case 19: f.ff_ref_0 = val; break;
      case 20: f.ff_ref_1 = val; break;
      case 21: f.trig_sens = (val != 0); break;
      case 22: f.trig_net_sc = (val != 0); break;
      case 23: f.trig_est = (val != 0); break;
      case 24: f.trig_ctrl = (val != 0); break;
      case 25: f.trig_ff = (val != 0); break;
      case 26: f.trig_merger = (val != 0); break;
      case 27: f.trig_net_ca = (val != 0); break;
      case 28: f.trig_act = (val != 0); break;
      }
      col++;
    }
    if (col >= 14) { vehicleFrames[f.vehicle_id].push_back(f); count++; }
  }
  std::cerr << "Loaded " << count << " frames for " << vehicleFrames.size() << " vehicles.\n";
  return !vehicleFrames.empty();
}

static sf::Color e_yToColor(double e_y, double threshold = 0.2) {
  float ratio = static_cast<float>(std::min(std::abs(e_y) / threshold, 1.0));
  if (ratio < 0.25f) return sf::Color(30, 220, 30);
  else if (ratio < 0.50f) return sf::Color((uint8_t)(30 + 225 * (ratio-0.25f)/0.25f), 220, 30);
  else if (ratio < 0.75f) return sf::Color(255, (uint8_t)(220 * (1.f - (ratio-0.5f)/0.25f)), 30);
  else return sf::Color(255, (uint8_t)(40 * (1.f - ratio)), 30);
}

static inline sf::Vector2f toDisp(double wx, double wy) {
  return sf::Vector2f(static_cast<float>(wx), static_cast<float>(wy));
}

static void buildThickLine(const std::vector<sf::Vector2f> &pts, float halfW, sf::Color color, sf::VertexArray &out) {
  if (pts.size() < 2) return;
  out.setPrimitiveType(sf::PrimitiveType::TriangleStrip);
  for (size_t i = 0; i < pts.size(); i++) {
    sf::Vector2f normal;
    if (i == 0) {
      sf::Vector2f d = pts[1] - pts[0];
      float len = std::sqrt(d.x * d.x + d.y * d.y);
      normal = (len < 1e-6f) ? sf::Vector2f(0, 1) : sf::Vector2f(-d.y / len, d.x / len);
    } else if (i == pts.size() - 1) {
      sf::Vector2f d = pts[i] - pts[i - 1];
      float len = std::sqrt(d.x * d.x + d.y * d.y);
      normal = (len < 1e-6f) ? sf::Vector2f(0, 1) : sf::Vector2f(-d.y / len, d.x / len);
    } else {
      sf::Vector2f d1 = pts[i] - pts[i - 1], d2 = pts[i + 1] - pts[i];
      float l1 = std::sqrt(d1.x*d1.x+d1.y*d1.y), l2 = std::sqrt(d2.x*d2.x+d2.y*d2.y);
      if (l1 > 1e-6f) d1 /= l1; else d1 = {0,0};
      if (l2 > 1e-6f) d2 /= l2; else d2 = d1;
      sf::Vector2f avg = d1 + d2;
      float al = std::sqrt(avg.x*avg.x+avg.y*avg.y);
      if (al < 1e-6f) normal = sf::Vector2f(-d1.y, d1.x);
      else {
          normal = sf::Vector2f(-avg.y / al, avg.x / al);
          if (normal.x * (-d1.y) + normal.y * d1.x < 0) normal = -normal;
      }
    }
    out.append(sf::Vertex(pts[i] + normal * halfW, color));
    out.append(sf::Vertex(pts[i] - normal * halfW, color));
  }
}

static std::vector<sf::Vector2f> subsampleLocal(const std::vector<sf::Vector2f> &full, float xCenter, float xHalf, size_t maxPts) {
  float xMin = xCenter - xHalf * 1.5f, xMax = xCenter + xHalf * 1.5f;
  std::vector<sf::Vector2f> local;
  for (auto const& p : full) if (p.x >= xMin && p.x <= xMax) local.push_back(p);
  if (local.size() > maxPts) {
      std::vector<sf::Vector2f> sub;
      size_t skip = local.size() / maxPts;
      for (size_t i = 0; i < local.size(); i += skip) sub.push_back(local[i]);
      return sub;
  }
  return local;
}

static std::string fmt(double v, int p = 4) {
  std::ostringstream s; s << std::fixed << std::setprecision(p) << v; return s.str();
}

int main(int argc, char *argv[]) {
  if (argc < 3) { std::cerr << "Usage: track_view <csv_file> <example_dir>\n"; return 1; }

  std::string csvFile = argv[1], exampleDir = argv[2];
  std::vector<double> tx = loadCSVColumn(exampleDir + "/x_position_track.csv");
  std::vector<double> ty = loadCSVColumn(exampleDir + "/y_position_track.csv");
  if (tx.empty()) return 1;

  std::vector<sf::Vector2f> trackFull;
  for (size_t i = 0; i < tx.size(); i++) trackFull.push_back(toDisp(tx[i], ty[i]));

  std::map<int, std::vector<Frame>> vehicleFrames;
  if (!loadCSVFrames(csvFile, vehicleFrames)) return 1;
  double maxSimTime = 0;
  for (auto const& [id, frames] : vehicleFrames) if (!frames.empty()) maxSimTime = std::max(maxSimTime, frames.back().time);

  sf::RenderWindow window(sf::VideoMode({1440, 900}), "Track View — Bosch CPS Challenge");
  window.setFramerateLimit(60);
  sf::Font font; font.openFromFile("/usr/share/fonts/liberation/LiberationMono-Regular.ttf");

  int focusedVehicle = vehicleFrames.begin()->first;
  float playSpeed = 1.0f;
  bool paused = false, followCar = true;
  sf::Clock clock;
  double simTime = 0;
  float viewW = INITIAL_VIEW_W;
  sf::Vector2f panOff(0,0);
  bool panning = false;
  sf::Vector2f panStart;

  while (window.isOpen()) {
    float dt = clock.restart().asSeconds();
    if (dt > 0.1f) dt = 0.1f;

    while (auto opt = window.pollEvent()) {
      const sf::Event &event = *opt;
      if (event.is<sf::Event::Closed>()) window.close();
      else if (auto *kp = event.getIf<sf::Event::KeyPressed>()) {
        if (kp->code == sf::Keyboard::Key::Escape || kp->code == sf::Keyboard::Key::Q) window.close();
        else if (kp->code == sf::Keyboard::Key::Space) paused = !paused;
        else if (kp->code == sf::Keyboard::Key::R) { simTime = 0; }
        else if (kp->code == sf::Keyboard::Key::F) { followCar = !followCar; panOff = {0,0}; }
        else if (kp->code == sf::Keyboard::Key::Up) playSpeed *= 2.f;
        else if (kp->code == sf::Keyboard::Key::Down) playSpeed /= 2.f;
        else if (kp->code >= sf::Keyboard::Key::Num0 && kp->code <= sf::Keyboard::Key::Num9) {
            int id = static_cast<int>(kp->code) - static_cast<int>(sf::Keyboard::Key::Num0);
            if (vehicleFrames.count(id)) focusedVehicle = id;
        }
      } else if (auto *mw = event.getIf<sf::Event::MouseWheelScrolled>()) {
          viewW *= (mw->delta > 0 ? 0.85f : 1.18f);
      } else if (auto *mb = event.getIf<sf::Event::MouseButtonPressed>()) {
          if (mb->button == sf::Mouse::Button::Right) { panning = true; panStart = {(float)mb->position.x, (float)mb->position.y}; }
      } else if (auto *mb = event.getIf<sf::Event::MouseButtonReleased>()) {
          if (mb->button == sf::Mouse::Button::Right) panning = false;
      } else if (auto *mm = event.getIf<sf::Event::MouseMoved>()) {
          if (panning && !followCar) {
              float dx = (float)mm->position.x - panStart.x, dy = (float)mm->position.y - panStart.y;
              panOff += sf::Vector2f(-dx * viewW / 1440.f, dy * viewW / 1440.f);
              panStart = {(float)mm->position.x, (float)mm->position.y};
          }
      }
    }

    if (!paused) { simTime += dt * playSpeed; if (simTime > maxSimTime) simTime = maxSimTime; }
    
    std::map<int, size_t> curIdx;
    for (auto const& [id, frames] : vehicleFrames) {
        size_t i = 0;
        while (i < frames.size() - 1 && frames[i+1].time <= simTime) i++;
        curIdx[id] = i;
    }
    const Frame &fFoc = vehicleFrames[focusedVehicle][curIdx[focusedVehicle]];

    sf::Vector2f camCenter = toDisp(fFoc.xveh, fFoc.yveh);
    float viewH = viewW * 900.f / 1440.f;
    sf::View worldView(camCenter + (followCar ? sf::Vector2f(0,0) : panOff), sf::Vector2f(viewW, -viewH));

    window.clear(sf::Color(20, 22, 30));
    window.setView(worldView);

    // Track
    std::vector<sf::Vector2f> trackLocal = subsampleLocal(trackFull, camCenter.x+(followCar?0:panOff.x), viewW, TRACK_LOCAL_MAX);
    if (trackLocal.size() >= 2) { sf::VertexArray va; buildThickLine(trackLocal, TRACK_HW, sf::Color(70,75,95), va); window.draw(va); }

    // Vehicles
    for (auto const& [id, frames] : vehicleFrames) {
        const Frame &f = frames[curIdx[id]];
        bool foc = (id == focusedVehicle);
        if (foc) {
            size_t start = (curIdx[id] >= TRAIL_MAX) ? curIdx[id] - TRAIL_MAX : 0;
            std::vector<sf::Vector2f> trail;
            for (size_t i = start; i <= curIdx[id]; i += 5) trail.push_back(toDisp(frames[i].xveh, frames[i].yveh));
            trail.push_back(toDisp(f.xveh, f.yveh));
            sf::VertexArray tva; buildThickLine(trail, TRAIL_HW, sf::Color(55,55,70), tva); window.draw(tva);
        }
        sf::CircleShape v(VEHICLE_RADIUS, 16);
        sf::Color c = e_yToColor(f.e_y); if (!foc) c.a = 80;
        v.setFillColor(c); v.setOutlineColor(foc ? sf::Color::White : sf::Color(150,150,150,80));
        v.setOutlineThickness(0.3f); v.setPosition(toDisp(f.xveh, f.yveh) - sf::Vector2f(VEHICLE_RADIUS, VEHICLE_RADIUS));
        window.draw(v);
        
        if (foc) {
            sf::CircleShape r(REF_RADIUS, 10); r.setFillColor(sf::Color(70,150,255));
            r.setPosition(toDisp(f.x_track, f.y_track) - sf::Vector2f(REF_RADIUS, REF_RADIUS)); window.draw(r);
            std::vector<sf::Vector2f> seg = {toDisp(f.x_track, f.y_track), toDisp(f.xveh, f.yveh)};
            sf::VertexArray eva; buildThickLine(seg, ERR_LINE_HW, e_yToColor(f.e_y), eva); window.draw(eva);
        }
    }

    // HUD
    window.setView(window.getDefaultView());
    auto drawText = [&](std::string t, float x, float y, sf::Color c=sf::Color::White, int s=16) {
        sf::Text txt(font, t, s); txt.setFillColor(c); txt.setPosition({x,y}); window.draw(txt);
    };
    
    // Dashboard Panel
    sf::RectangleShape dashBg({350, 240}); dashBg.setPosition({1070, 20}); dashBg.setFillColor(sf::Color(0,0,0,180));
    dashBg.setOutlineColor(sf::Color(100,100,150)); dashBg.setOutlineThickness(1); window.draw(dashBg);
    
    float dx = 1080, dy = 30, lh = 22;
    drawText("Veh #" + std::to_string(focusedVehicle) + " [" + (paused?"PAUSED":"x"+fmt(playSpeed,1)) + "]", dx, dy, sf::Color(180,180,255), 18); dy += lh+5;
    drawText("Time:      " + fmt(simTime, 3) + " s", dx, dy); dy += lh;
    drawText("Velocity:  " + fmt(fFoc.velocity, 2) + " m/s", dx, dy); dy += lh;
    drawText("e_y:       " + fmt(fFoc.e_y, 5) + " m", dx, dy, e_yToColor(fFoc.e_y)); dy += lh;
    drawText("Steering:  " + fmt(fFoc.act_out, 5) + " rad", dx, dy); dy += lh;
    drawText("Roll Perf: " + fmt(fFoc.rolling_perf, 4), dx, dy); dy += lh;
    drawText("Violations:" + std::to_string(fFoc.thresh_errors), dx, dy, fFoc.violated?sf::Color::Red:sf::Color::White); dy += lh;
    drawText("Section:   " + std::string(fFoc.critical?"CURVE":"STRAIGHT"), dx, dy, fFoc.critical?sf::Color::Yellow:sf::Color::Green);

    // Task Chain Panel
    sf::RectangleShape tcBg({350, 200}); tcBg.setPosition({1070, 270}); tcBg.setFillColor(sf::Color(0,0,0,180));
    tcBg.setOutlineColor(sf::Color(100,100,150)); tcBg.setOutlineThickness(1); window.draw(tcBg);
    drawText("Task Chain", 1080, 280, sf::Color(180,180,255), 16);
    
    auto drawBox = [&](const char* n, bool a, float x, float y) {
        sf::RectangleShape b({75, 30}); b.setPosition({x,y}); b.setFillColor(a?sf::Color(0,100,0,200):sf::Color(50,50,60,150));
        b.setOutlineColor(a?sf::Color::Green:sf::Color(100,100,100)); b.setOutlineThickness(a?2:1); window.draw(b);
        drawText(n, x+5, y+5, a?sf::Color::White:sf::Color(150,150,150), 12);
    };
    float bx = 1080, by = 310;
    drawBox("Sens", fFoc.trig_sens, bx, by); drawBox("Net-SC", fFoc.trig_net_sc, bx+85, by); 
    drawBox("Estim", fFoc.trig_est, bx+170, by); drawBox("Ctrl", fFoc.trig_ctrl, bx+255, by);
    by += 45;
    drawBox("FF", fFoc.trig_ff, bx, by); drawBox("Merg", fFoc.trig_merger, bx+85, by);
    drawBox("Net-CA", fFoc.trig_net_ca, bx+170, by); drawBox("Act", fFoc.trig_act, bx+255, by);
    
    drawText("Keys: 0-9 focus, F follow, R reset, Up/Dn speed", 20, 870, sf::Color(150,150,150), 14);
    window.display();
  }
  return 0;
}
