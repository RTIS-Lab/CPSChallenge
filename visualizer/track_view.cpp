/*
 * Track View — SFML visualizer for LateralMotionControl FMU output
 *
 * Reads a CSV file produced by fmu_harness plus track geometry from the
 * example directory, then animates the vehicle driving along the track.
 *
 * Build: built by top-level CMakeLists.txt (requires libsfml-dev)
 * Run:   ./track_view <csv_file> <example_dir>
 *
 * Controls:
 *   Space         Pause / resume playback
 *   Left / Right  Step one frame backward / forward (paused)
 *   Up / Down     Speed up / slow down
 *   R             Restart from beginning
 *   F             Toggle follow-car / free camera
 *   Scroll wheel  Zoom in / out
 *   Right-drag    Pan camera (free mode)
 *   Esc / Q       Quit
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

// ─── Configuration ─────────────────────────────────────────────────

static constexpr float INITIAL_VIEW_W =
    120.f; // meters of x-axis visible at start
static constexpr float VEHICLE_RADIUS = 2.0f;
static constexpr float REF_RADIUS = 1.0f;
static constexpr float TRACK_HW = 0.7f;  // half-width of track line (display)
static constexpr float TRAIL_HW = 0.55f; // half-width of trail line
static constexpr float ERR_LINE_HW = 0.3f;
static constexpr size_t TRAIL_MAX = 1200;
static constexpr size_t TRACK_FULL_MAX = 5000;
static constexpr size_t TRACK_LOCAL_MAX = 2000;

// ─── Data Structures ───────────────────────────────────────────────

struct Frame {
  double time = 0;
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
  // Extended fields for task-chain visualization
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
  if (!file.is_open()) {
    std::cerr << "WARNING: Cannot open '" << path << "'\n";
    return data;
  }
  double val;
  while (file >> val)
    data.push_back(val);
  return data;
}

static bool loadCSVFrames(const std::string &path, std::vector<Frame> &frames) {
  std::ifstream file(path);
  if (!file.is_open()) {
    std::cerr << "ERROR: Cannot open CSV '" << path << "'\n";
    return false;
  }
  std::string line;
  if (!std::getline(file, line)) {
    std::cerr << "ERROR: Empty CSV\n";
    return false;
  }

  while (std::getline(file, line)) {
    if (line.empty() || line[0] < '0' || line[0] > '9')
      continue;
    std::istringstream ss(line);
    std::string field;
    Frame f;
    int col = 0;
    while (std::getline(ss, field, ',')) {
      double val = 0;
      try {
        val = std::stod(field);
      } catch (...) {
        continue;
      }
      switch (col) {
      case 0:
        f.time = val;
        break;
      case 1:
        f.phi_dot = val;
        break;
      case 2:
        f.beta = val;
        break;
      case 3:
        f.delta = val;
        break;
      case 4:
        f.delta_dot = val;
        break;
      case 5:
        f.e_y = val;
        break;
      case 6:
        f.e_y_dot = val;
        break;
      case 7:
        f.act_out = val;
        break;
      case 8:
        f.velocity = val;
        break;
      case 9:
        f.x_track = val;
        break;
      case 10:
        f.y_track = val;
        break;
      case 11:
        f.xveh = val;
        break;
      case 12:
        f.yveh = val;
        break;
      case 13:
        f.rolling_perf = val;
        break;
      case 14:
        f.avg_perf = val;
        break;
      case 15:
        f.thresh_errors = static_cast<int>(val);
        break;
      case 16:
        f.critical = (val != 0);
        break;
      case 17:
        f.violated = (val != 0);
        break;
      case 18:
        f.ff_ref_0 = val;
        break;
      case 19:
        f.ff_ref_1 = val;
        break;
      case 20:
        f.trig_sens = (val != 0);
        break;
      case 21:
        f.trig_net_sc = (val != 0);
        break;
      case 22:
        f.trig_est = (val != 0);
        break;
      case 23:
        f.trig_ctrl = (val != 0);
        break;
      case 24:
        f.trig_ff = (val != 0);
        break;
      case 25:
        f.trig_merger = (val != 0);
        break;
      case 26:
        f.trig_net_ca = (val != 0);
        break;
      case 27:
        f.trig_act = (val != 0);
        break;
      }
      col++;
    }
    if (col >= 13)
      frames.push_back(f);
  }
  std::cerr << "Loaded " << frames.size() << " frames from " << path << "\n";
  return !frames.empty();
}

static sf::Color e_yToColor(double e_y, double threshold = 0.2) {
  float ratio = static_cast<float>(std::min(std::abs(e_y) / threshold, 1.0));
  if (ratio < 0.25f)
    return sf::Color(30, 220, 30);
  else if (ratio < 0.50f)
    return sf::Color(static_cast<uint8_t>(30 + 225 * (ratio - 0.25f) / 0.25f),
                     220, 30);
  else if (ratio < 0.75f)
    return sf::Color(
        255, static_cast<uint8_t>(220 * (1.f - (ratio - 0.5f) / 0.25f)), 30);
  else
    return sf::Color(255, static_cast<uint8_t>(40 * (1.f - ratio)), 30);
}

// Convert world (x,y) to display coordinates
static inline sf::Vector2f toDisp(double wx, double wy) {
  return sf::Vector2f(static_cast<float>(wx), static_cast<float>(wy));
}

// Build a thick polyline as a triangle strip.
static void buildThickLine(const std::vector<sf::Vector2f> &pts, float halfW,
                           sf::Color color, sf::VertexArray &out) {
  if (pts.size() < 2)
    return;
  out.setPrimitiveType(sf::PrimitiveType::TriangleStrip);
  for (size_t i = 0; i < pts.size(); i++) {
    sf::Vector2f normal;
    if (i == 0) {
      sf::Vector2f d = pts[1] - pts[0];
      float len = std::sqrt(d.x * d.x + d.y * d.y);
      normal = (len < 1e-6f) ? sf::Vector2f(0, 1)
                             : sf::Vector2f(-d.y / len, d.x / len);
    } else if (i == pts.size() - 1) {
      sf::Vector2f d = pts[i] - pts[i - 1];
      float len = std::sqrt(d.x * d.x + d.y * d.y);
      normal = (len < 1e-6f) ? sf::Vector2f(0, 1)
                             : sf::Vector2f(-d.y / len, d.x / len);
    } else {
      sf::Vector2f d1 = pts[i] - pts[i - 1];
      sf::Vector2f d2 = pts[i + 1] - pts[i];
      float l1 = std::sqrt(d1.x * d1.x + d1.y * d1.y);
      float l2 = std::sqrt(d2.x * d2.x + d2.y * d2.y);
      if (l1 < 1e-6f)
        d1 = d2;
      else
        d1 /= l1;
      if (l2 < 1e-6f)
        d2 = d1;
      else
        d2 /= l2;
      sf::Vector2f avg = d1 + d2;
      float al = std::sqrt(avg.x * avg.x + avg.y * avg.y);
      if (al < 1e-6f) {
        normal = sf::Vector2f(-d1.y, d1.x);
      } else {
        normal = sf::Vector2f(-avg.y / al, avg.x / al);
        if (normal.x * (-d1.y) + normal.y * d1.x < 0)
          normal = -normal;
      }
    }
    sf::Vector2f p = pts[i];
    out.append(sf::Vertex(p + normal * halfW, color));
    out.append(sf::Vertex(p - normal * halfW, color));
  }
}

// Subsample a polyline around a given x-range in display coords
static std::vector<sf::Vector2f>
subsampleLocal(const std::vector<sf::Vector2f> &full, float xCenter,
               float xHalf, size_t maxPts) {
  float xMin = xCenter - xHalf * 1.5f;
  float xMax = xCenter + xHalf * 1.5f;
  size_t count = 0;
  for (auto &p : full)
    if (p.x >= xMin && p.x <= xMax)
      count++;
  size_t skip = (count > maxPts) ? count / maxPts : 1;
  std::vector<sf::Vector2f> local;
  bool started = false;
  size_t idx = 0;
  for (auto &p : full) {
    if (p.x < xMin || p.x > xMax) {
      started = false;
      continue;
    }
    if (!started) {
      local.push_back(p);
      started = true;
      idx = 0;
      continue;
    }
    idx++;
    if (idx % skip == 0)
      local.push_back(p);
  }
  return local;
}

static std::string fmt(double v, int p = 4) {
  std::ostringstream s;
  s << std::fixed << std::setprecision(p) << v;
  return s.str();
}

// ─── Main ────────────────────────────────────────────────────────────

int main(int argc, char *argv[]) {
  if (argc < 3) {
    std::cerr << "Usage: track_view <csv_file> <example_dir>\n"
              << "  csv_file:    output of fmu_harness (CSV)\n"
              << "  example_dir: e.g. ../examples/example_v_10\n";
    return 1;
  }

  std::string csvFile = argv[1];
  std::string exampleDir = argv[2];

  // ── Load track data ──
  std::vector<double> track_x =
      loadCSVColumn(exampleDir + "/x_position_track.csv");
  std::vector<double> track_y =
      loadCSVColumn(exampleDir + "/y_position_track.csv");
  if (track_x.empty() || track_y.empty()) {
    std::cerr << "ERROR: Could not load track data from '" << exampleDir
              << "'\n";
    return 1;
  }
  std::cerr << "Track: " << track_x.size() << " points\n";

  // Build full track in display coordinates, subsampled
  size_t fullSkip =
      (track_x.size() > TRACK_FULL_MAX) ? track_x.size() / TRACK_FULL_MAX : 1;
  std::vector<sf::Vector2f> trackFull;
  trackFull.reserve(track_x.size() / fullSkip + 2);
  for (size_t i = 0; i < track_x.size(); i += fullSkip)
    trackFull.push_back(toDisp(track_x[i], track_y[i]));
  if (trackFull.back() != toDisp(track_x.back(), track_y.back()))
    trackFull.push_back(toDisp(track_x.back(), track_y.back()));

  // Track bounds (display coords)
  float dMinX = 1e9f, dMaxX = -1e9f, dMinY = 1e9f, dMaxY = -1e9f;
  for (auto &p : trackFull) {
    dMinX = std::min(dMinX, p.x);
    dMaxX = std::max(dMaxX, p.x);
    dMinY = std::min(dMinY, p.y);
    dMaxY = std::max(dMaxY, p.y);
  }

  // ── Load simulation data ──
  std::vector<Frame> frames;
  if (!loadCSVFrames(csvFile, frames))
    return 1;

  // ── Window ──
  unsigned winW = 1440, winH = 900;
  sf::RenderWindow window(sf::VideoMode({winW, winH}),
                          "Track View — LateralMotionControl FMU");
  window.setFramerateLimit(60);

  // ── Font ──
  std::vector<std::string> fontPaths = {
      "/usr/share/fonts/TTF/HackNerdFontMono-Regular.ttf",
      "/usr/share/fonts/liberation/LiberationMono-Regular.ttf",
      "/usr/share/fonts/Adwaita/AdwaitaMono-Regular.ttf",
      "/usr/share/fonts/noto/NotoSansMono-Regular.ttf",
      "/usr/share/fonts/TTF/DejaVuSansMono.ttf",
      "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
      "/usr/share/fonts/truetype/liberation/LiberationMono.ttf",
      "/usr/share/fonts/noto/NotoSansMono-Light.ttf",
      "/usr/share/fonts/truetype/ubuntu/UbuntuMono-R.ttf",
  };
  sf::Font font;
  bool hasFont = false;
  for (auto &p : fontPaths)
    if (font.openFromFile(p)) {
      hasFont = true;
      break;
    }
  if (!hasFont)
    std::cerr << "WARNING: No monospace font found.\n";

  // ── Playback state ──
  size_t curFrame = 0;
  float playSpeed = 1.0f;
  bool paused = false;
  bool followCar = true;
  sf::Clock clock;
  double simTime = 0;

  // ── Camera ──
  float viewW = INITIAL_VIEW_W;
  sf::Vector2f camCenter = toDisp(frames[0].xveh, frames[0].yveh);
  sf::Vector2f panOff(0, 0);
  bool panning = false;
  sf::Vector2f panStart;

  // ── Main loop ──
  while (window.isOpen()) {
    float dt = clock.restart().asSeconds();
    if (dt > 0.1f)
      dt = 0.1f;

    // SFML 3 event loop
    while (auto opt = window.pollEvent()) {
      const sf::Event &event = *opt;
      if (event.is<sf::Event::Closed>()) {
        window.close();
      } else if (auto *kp = event.getIf<sf::Event::KeyPressed>()) {
        switch (kp->code) {
        case sf::Keyboard::Key::Escape:
        case sf::Keyboard::Key::Q:
          window.close();
          break;
        case sf::Keyboard::Key::Space:
          paused = !paused;
          break;
        case sf::Keyboard::Key::R:
          curFrame = 0;
          simTime = 0;
          break;
        case sf::Keyboard::Key::F:
          followCar = !followCar;
          panOff = sf::Vector2f(0, 0);
          break;
        case sf::Keyboard::Key::Up:
          playSpeed = std::min(playSpeed * 2.f, 64.f);
          break;
        case sf::Keyboard::Key::Down:
          playSpeed = std::max(playSpeed / 2.f, 0.0625f);
          break;
        case sf::Keyboard::Key::Right:
          if (paused && curFrame < frames.size() - 1) {
            curFrame++;
            simTime = frames[curFrame].time;
          }
          break;
        case sf::Keyboard::Key::Left:
          if (paused && curFrame > 0) {
            curFrame--;
            simTime = frames[curFrame].time;
          }
          break;
        default:
          break;
        }
      } else if (auto *mw = event.getIf<sf::Event::MouseWheelScrolled>()) {
        float factor = (mw->delta > 0) ? 0.85f : 1.18f;
        viewW = std::clamp(viewW * factor, 2.f, 5000.f);
      } else if (auto *mb = event.getIf<sf::Event::MouseButtonPressed>()) {
        if (mb->button == sf::Mouse::Button::Right) {
          panning = true;
          panStart = sf::Vector2f(static_cast<float>(mb->position.x),
                                  static_cast<float>(mb->position.y));
        }
      } else if (auto *mb = event.getIf<sf::Event::MouseButtonReleased>()) {
        if (mb->button == sf::Mouse::Button::Right)
          panning = false;
      } else if (auto *mm = event.getIf<sf::Event::MouseMoved>()) {
        if (panning && !followCar) {
          float dx = static_cast<float>(mm->position.x) - panStart.x;
          float dy = static_cast<float>(mm->position.y) - panStart.y;
          panOff += sf::Vector2f(-dx * viewW / winW, dy * viewW / winW);
          panStart = sf::Vector2f(static_cast<float>(mm->position.x),
                                  static_cast<float>(mm->position.y));
        }
      } else if (auto *rs = event.getIf<sf::Event::Resized>()) {
        winW = rs->size.x;
        winH = rs->size.y;
      }
    }

    // ── Advance playback ──
    if (!paused) {
      simTime += dt * playSpeed;
      while (curFrame < frames.size() - 1 &&
             frames[curFrame + 1].time <= simTime)
        curFrame++;
      if (simTime > frames.back().time)
        curFrame = frames.size() - 1;
    }
    if (curFrame >= frames.size())
      curFrame = frames.size() - 1;
    const Frame &f = frames[curFrame];

    // ── Update camera ──
    sf::Vector2f carDisp = toDisp(f.xveh, f.yveh);
    if (followCar) {
      camCenter = carDisp;
      panOff = sf::Vector2f(0, 0);
    }

    float viewH = viewW * static_cast<float>(winH) / static_cast<float>(winW);
    // Negative height flips Y so y-up in our display coords
    sf::View worldView(camCenter + panOff, sf::Vector2f(viewW, -viewH));

    // ── Build local track subset ──
    std::vector<sf::Vector2f> trackLocal = subsampleLocal(
        trackFull, camCenter.x + panOff.x, viewW, TRACK_LOCAL_MAX);

    // ── Build trail ──
    size_t trailStart = (curFrame >= TRAIL_MAX) ? curFrame - TRAIL_MAX + 1 : 0;

    // ── Render ──
    window.clear(sf::Color(20, 22, 30));

    // ── World view ──
    window.setView(worldView);

    // Track reference path (thick gray)
    if (trackLocal.size() >= 2) {
      sf::VertexArray trackLine;
      buildThickLine(trackLocal, TRACK_HW, sf::Color(70, 75, 95), trackLine);
      window.draw(trackLine);
    }

    // Vehicle trail
    size_t tEnd = curFrame + 1;
    if (tEnd - trailStart >= 2) {
      std::vector<sf::Vector2f> trailPts;
      size_t skip = (tEnd - trailStart > 2000) ? (tEnd - trailStart) / 2000 : 1;
      trailPts.reserve((tEnd - trailStart) / skip + 2);
      for (size_t i = trailStart; i < tEnd; i += skip)
        trailPts.push_back(toDisp(frames[i].xveh, frames[i].yveh));
      if (trailPts.back() != carDisp)
        trailPts.push_back(carDisp);

      sf::VertexArray trailLine;
      buildThickLine(trailPts, TRAIL_HW, sf::Color(55, 55, 70), trailLine);
      window.draw(trailLine);

      // Colored dots for recent portion
      size_t colorStart = (tEnd > 500 + trailStart) ? tEnd - 500 : trailStart;
      for (size_t i = colorStart; i < tEnd; i++) {
        sf::Vector2f pt = toDisp(frames[i].xveh, frames[i].yveh);
        float vx = camCenter.x + panOff.x;
        float vy = camCenter.y + panOff.y;
        if (std::abs(pt.x - vx) > viewW || std::abs(pt.y - vy) > viewH * 2)
          continue;
        float alpha = 0.15f + 0.85f * static_cast<float>(i - colorStart) /
                                  static_cast<float>(tEnd - colorStart);
        sf::Color c = e_yToColor(frames[i].e_y);
        sf::CircleShape dot(TRAIL_HW * 1.2f, 6);
        dot.setFillColor(
            sf::Color(c.r, c.g, c.b, static_cast<uint8_t>(alpha * 220)));
        dot.setPosition({pt.x - TRAIL_HW * 1.2f, pt.y - TRAIL_HW * 1.2f});
        window.draw(dot);
      }
    }

    // Reference point
    {
      sf::Vector2f refPt = toDisp(f.x_track, f.y_track);
      sf::CircleShape refDot(REF_RADIUS, 10);
      refDot.setFillColor(sf::Color(70, 150, 255, 210));
      refDot.setOutlineColor(sf::Color(180, 210, 255));
      refDot.setOutlineThickness(0.2f);
      refDot.setPosition({refPt.x - REF_RADIUS, refPt.y - REF_RADIUS});
      window.draw(refDot);
    }

    // Lateral error line
    {
      std::vector<sf::Vector2f> seg = {toDisp(f.x_track, f.y_track),
                                       toDisp(f.xveh, f.yveh)};
      sf::VertexArray errLine;
      buildThickLine(seg, ERR_LINE_HW, e_yToColor(f.e_y), errLine);
      window.draw(errLine);
    }

    // Vehicle
    {
      sf::Vector2f vehPt = toDisp(f.xveh, f.yveh);
      sf::CircleShape vehicle(VEHICLE_RADIUS, 16);
      vehicle.setFillColor(e_yToColor(f.e_y));
      vehicle.setOutlineColor(sf::Color::White);
      vehicle.setOutlineThickness(0.2f);
      vehicle.setPosition({vehPt.x - VEHICLE_RADIUS, vehPt.y - VEHICLE_RADIUS});
      window.draw(vehicle);
    }

    // Scale bars
    {
      float sbX = camCenter.x + panOff.x - viewW * 0.42f;
      float sbY = camCenter.y + panOff.y + viewH * 0.40f;

      // Horizontal: 50 real meters
      float hBarLen = 50.f;
      sf::RectangleShape hBar(sf::Vector2f(hBarLen, 0.4f));
      hBar.setPosition({sbX, sbY});
      hBar.setFillColor(sf::Color(200, 200, 200));
      window.draw(hBar);
      sf::RectangleShape hCap1(sf::Vector2f(0.4f, 1.5f));
      hCap1.setPosition({sbX - 0.2f, sbY - 0.55f});
      hCap1.setFillColor(sf::Color(200, 200, 200));
      window.draw(hCap1);
      sf::RectangleShape hCap2(sf::Vector2f(0.4f, 1.5f));
      hCap2.setPosition({sbX + hBarLen - 0.2f, sbY - 0.55f});
      hCap2.setFillColor(sf::Color(200, 200, 200));
      window.draw(hCap2);
    }

    // ── Minimap (screen space) ──
    window.setView(window.getDefaultView());

    const float MM_W = 260.f, MM_H = 110.f, MM_PAD = 12.f;
    float mmOX = MM_PAD, mmOY = static_cast<float>(winH) - MM_H - MM_PAD;

    sf::RectangleShape mmBg(sf::Vector2f(MM_W, MM_H));
    mmBg.setPosition({mmOX, mmOY});
    mmBg.setFillColor(sf::Color(10, 10, 20, 210));
    mmBg.setOutlineColor(sf::Color(70, 70, 100));
    mmBg.setOutlineThickness(1.f);
    window.draw(mmBg);

    float padX = (dMaxX - dMinX) * 0.05f + 1.f;
    float padY = (dMaxY - dMinY) * 0.05f + 1.f;
    float bxMin = dMinX - padX, bxMax = dMaxX + padX;
    float byMin = dMinY - padY, byMaxY = dMaxY + padY;
    float bxW = bxMax - bxMin, bxH = byMaxY - byMin;

    auto mapToMM = [&](sf::Vector2f p) -> sf::Vector2f {
      float nx = (p.x - bxMin) / bxW;
      float ny = (p.y - byMin) / bxH;
      return sf::Vector2f(mmOX + 4.f + nx * (MM_W - 8.f),
                          mmOY + MM_H - 4.f - ny * (MM_H - 8.f));
    };

    // Track on minimap
    size_t mmSkip = (trackFull.size() > 1000) ? trackFull.size() / 1000 : 1;
    for (size_t i = mmSkip; i < trackFull.size(); i += mmSkip) {
      sf::Vertex line[] = {
          sf::Vertex(mapToMM(trackFull[i - mmSkip]), sf::Color(90, 90, 110)),
          sf::Vertex(mapToMM(trackFull[i]), sf::Color(90, 90, 110))};
      window.draw(line, 2, sf::PrimitiveType::Lines);
    }

    // Vehicle on minimap
    {
      sf::Vector2f vMM = mapToMM(toDisp(f.xveh, f.yveh));
      sf::CircleShape dot(4.f, 8);
      dot.setFillColor(e_yToColor(f.e_y));
      dot.setOutlineColor(sf::Color::White);
      dot.setOutlineThickness(1.f);
      dot.setPosition({vMM.x - 4.f, vMM.y - 4.f});
      window.draw(dot);
    }

    // View rectangle on minimap
    {
      sf::Vector2f vc1 =
          mapToMM(sf::Vector2f(camCenter.x + panOff.x - viewW / 2.f,
                               camCenter.y + panOff.y - viewH / 2.f));
      sf::Vector2f vc2 =
          mapToMM(sf::Vector2f(camCenter.x + panOff.x + viewW / 2.f,
                               camCenter.y + panOff.y + viewH / 2.f));
      sf::RectangleShape vRect(
          sf::Vector2f(std::abs(vc2.x - vc1.x), std::abs(vc2.y - vc1.y)));
      vRect.setPosition({std::min(vc1.x, vc2.x), std::min(vc1.y, vc2.y)});
      vRect.setFillColor(sf::Color(255, 255, 255, 30));
      vRect.setOutlineColor(sf::Color(200, 200, 255, 100));
      vRect.setOutlineThickness(1.f);
      window.draw(vRect);
    }

    // Scale label on minimap
    if (hasFont) {
      sf::Text mmLabel(font, "true scale", 11);
      mmLabel.setFillColor(sf::Color(160, 160, 180));
      mmLabel.setPosition({mmOX + 4, mmOY + 2});
      window.draw(mmLabel);
    }

    // ── Dashboard ──
    auto drawText = [&](const std::string &text, float x, float y,
                        sf::Color color = sf::Color::White, int size = 16) {
      if (!hasFont)
        return;
      sf::Text txt(font, text, size);
      txt.setFillColor(color);
      txt.setPosition({x, y});
      window.draw(txt);
    };

    // Lambda to draw a task-chain box with optional period label
    auto drawTaskBox = [&](const char *name, bool active, float periodMs,
                           sf::Color activeColor, float bx, float by, float bw,
                           float bh) {
      sf::Color bgCol = active ? sf::Color(activeColor.r / 2, activeColor.g / 2,
                                           activeColor.b / 2, 220)
                               : sf::Color(30, 30, 40, 180);
      sf::RectangleShape box(sf::Vector2f(bw, bh));
      box.setPosition({bx, by});
      box.setFillColor(bgCol);
      box.setOutlineColor(active ? activeColor : sf::Color(80, 80, 100));
      box.setOutlineThickness(active ? 2.f : 1.f);
      window.draw(box);

      drawText(name, bx + 4.f, by + 3.f,
               active ? sf::Color::White : sf::Color(120, 120, 140), 12);
      if (periodMs > 0) {
        char perBuf[16];
        snprintf(perBuf, sizeof(perBuf), "%.0fms", periodMs);
        drawText(perBuf, bx + 4.f, by + 17.f,
                 active ? activeColor : sf::Color(90, 90, 110), 10);
      }
    };

    // Lambda to draw a horizontal arrow
    auto drawHArrow = [&](float x1, float x2, float y,
                          sf::Color col = sf::Color(120, 120, 150)) {
      sf::Vertex line[] = {
          sf::Vertex(sf::Vector2f(x1, y), col),
          sf::Vertex(sf::Vector2f(x2, y), col),
      };
      window.draw(line, 2, sf::PrimitiveType::Lines);
      sf::Vertex head[] = {
          sf::Vertex(sf::Vector2f(x2 - 4.f, y - 3.f), col),
          sf::Vertex(sf::Vector2f(x2, y), col),
          sf::Vertex(sf::Vector2f(x2 - 4.f, y + 3.f), col),
      };
      window.draw(head, 3, sf::PrimitiveType::Lines);
    };

    // Lambda to draw a vertical arrow
    auto drawVArrow = [&](float x, float y1, float y2,
                          sf::Color col = sf::Color(120, 120, 150)) {
      sf::Vertex line[] = {
          sf::Vertex(sf::Vector2f(x, y1), col),
          sf::Vertex(sf::Vector2f(x, y2), col),
      };
      window.draw(line, 2, sf::PrimitiveType::Lines);
      sf::Vertex head[] = {
          sf::Vertex(sf::Vector2f(x - 3.f, y2 - 4.f), col),
          sf::Vertex(sf::Vector2f(x, y2), col),
          sf::Vertex(sf::Vector2f(x + 3.f, y2 - 4.f), col),
      };
      window.draw(head, 3, sf::PrimitiveType::Lines);
    };

    // ── Dashboard panel (right side) ──
    float dashX = static_cast<float>(winW) - 370.f;
    float dashY = static_cast<float>(winH) - 460.f;
    sf::RectangleShape dashBg(sf::Vector2f(360.f, 215.f));
    dashBg.setPosition({dashX, dashY});
    dashBg.setFillColor(sf::Color(0, 0, 0, 185));
    dashBg.setOutlineColor(sf::Color(70, 70, 100));
    dashBg.setOutlineThickness(1.f);
    window.draw(dashBg);

    float tx = dashX + 12.f, ty = dashY + 8.f;
    float lh = 19.f;

    std::string speedLabel = paused ? "PAUSED" : ("x" + fmt(playSpeed, 2));
    drawText("Track View  [" + speedLabel + "]", tx, ty,
             sf::Color(180, 180, 255), 17);
    ty += lh + 4;

    drawText(fmt(f.time, 3) + " s", tx, ty, sf::Color(200, 200, 200), 22);
    ty += lh + 2;

    drawText("Velocity:  " + fmt(f.velocity, 2) + " m/s", tx, ty);
    ty += lh;
    drawText("e_y:       " + fmt(f.e_y, 5) + " m", tx, ty, e_yToColor(f.e_y));
    ty += lh;
    drawText("e_y_dot:   " + fmt(f.e_y_dot, 5) + " m/s", tx, ty);
    ty += lh;
    drawText("Steering:  " + fmt(f.act_out, 6) + " rad", tx, ty);
    ty += lh;
    drawText("Roll perf: " + fmt(f.rolling_perf, 4), tx, ty);
    ty += lh;
    drawText("Avg perf:  " + fmt(f.avg_perf, 6), tx, ty);
    ty += lh;
    drawText("Violations: " + std::to_string(f.thresh_errors), tx, ty,
             f.violated ? sf::Color(255, 80, 80) : sf::Color::White);
    ty += lh;

    sf::Color critC =
        f.critical ? sf::Color(255, 200, 50) : sf::Color(80, 200, 80);
    drawText("Section:   " + std::string(f.critical ? "CURVE" : "STRAIGHT"), tx,
             ty, critC);
    ty += lh;
    bool ffActive =
        (std::fabs(f.ff_ref_0) > 1e-6 || std::fabs(f.ff_ref_1) > 1e-6);
    drawText(
        "ff_ref:    [" + fmt(f.ff_ref_0, 5) + ", " + fmt(f.ff_ref_1, 5) + "]",
        tx, ty, ffActive ? sf::Color(255, 220, 100) : sf::Color(100, 100, 120));

    // ── Task Chain panel (below dashboard) ──
    float tcX = dashX;
    float tcY = dashY + 225.f;
    float tcW = 360.f;
    float tcH = 230.f;

    sf::RectangleShape tcBg(sf::Vector2f(tcW, tcH));
    tcBg.setPosition({tcX, tcY});
    tcBg.setFillColor(sf::Color(0, 0, 0, 185));
    tcBg.setOutlineColor(sf::Color(70, 70, 100));
    tcBg.setOutlineThickness(1.f);
    window.draw(tcBg);

    drawText("Task Chain", tcX + 12.f, tcY + 6.f, sf::Color(180, 180, 255), 15);

    // Task definitions: name, trigger active flag, period (0 = network), colour
    struct TaskDef {
      const char *name;
      bool active;
      float periodMs;
      sf::Color color;
    };
    TaskDef tasks[] = {
        {"Sensor", f.trig_sens, 5, sf::Color(100, 200, 100)},
        {"Net→SC", f.trig_net_sc, 0, sf::Color(130, 180, 255)},
        {"Estimator", f.trig_est, 10, sf::Color(255, 200, 80)},
        {"Ctrl", f.trig_ctrl, 20, sf::Color(255, 140, 80)},
        {"Feedfwd", f.trig_ff, 20, sf::Color(255, 220, 100)},
        {"Merger", f.trig_merger, 20, sf::Color(180, 130, 255)},
        {"Net→CA", f.trig_net_ca, 0, sf::Color(100, 170, 255)},
        {"Actuator", f.trig_act, 30, sf::Color(100, 220, 220)},
    };

    // Layout — two rows, arranged to match the data-flow graph:
    //   Row 1: Sensor ──→ Net→SC ──→ Estimator ──→ Ctrl ──┐
    //                                                        ↓
    //   Row 2:            Feedfwd ──────→ Merger ──→ Net→CA ──→ Actuator
    //                                       ↑
    //                             (controller output also merges here)
    float bw = 68.f, bh = 30.f, gap = 18.f;
    float stride = bw + gap;
    float r1Y = tcY + 30.f;
    float r2Y = r1Y + bh + 28.f;

    // Row 1 positions (4 boxes, same grid for row 2)
    float colX[] = {
        tcX + 10.f,              // col 0
        tcX + 10.f + 1 * stride, // col 1
        tcX + 10.f + 2 * stride, // col 2
        tcX + 10.f + 3 * stride, // col 3
    };

    // Row 1: Sensor → Net→SC → Estimator → Controller
    for (int i = 0; i < 4; i++) {
      drawTaskBox(tasks[i].name, tasks[i].active, tasks[i].periodMs,
                  tasks[i].color, colX[i], r1Y, bw, bh);
      if (i < 3)
        drawHArrow(colX[i] + bw + 2.f, colX[i + 1] - 2.f, r1Y + bh / 2.f);
    }

    // Row 2: Feedfwd → Merger → Net→CA → Actuator
    // Col 0=Feedfwd, Col 1=Merger, Col 2=Net→CA, Col 3=Actuator
    drawTaskBox(tasks[4].name, tasks[4].active, tasks[4].periodMs,
                tasks[4].color, colX[0], r2Y, bw, bh);
    drawTaskBox(tasks[5].name, tasks[5].active, tasks[5].periodMs,
                tasks[5].color, colX[1], r2Y, bw, bh);
    drawTaskBox(tasks[6].name, tasks[6].active, tasks[6].periodMs,
                tasks[6].color, colX[2], r2Y, bw, bh);
    drawTaskBox(tasks[7].name, tasks[7].active, tasks[7].periodMs,
                tasks[7].color, colX[3], r2Y, bw, bh);

    // Row-2 horizontal arrows: Feedfwd → Merger → Net→CA → Actuator
    drawHArrow(colX[0] + bw + 2.f, colX[1] - 2.f, r2Y + bh / 2.f);
    drawHArrow(colX[1] + bw + 2.f, colX[2] - 2.f, r2Y + bh / 2.f);
    drawHArrow(colX[2] + bw + 2.f, colX[3] - 2.f, r2Y + bh / 2.f);

    // L-shaped arrow: Controller (row1,col3) output drops down and
    // routes left into Merger (row2,col1)
    {
      float ctrlCx = colX[3] + bw / 2.f;   // center-x of Controller
      float mergCx = colX[1] + bw / 2.f;   // center-x of Merger
      float yBottom = r1Y + bh + 2.f;      // just below row1
      float yTop = r2Y - 2.f;              // just above row2
      float yMid = (yBottom + yTop) / 2.f; // midpoint for the L-bend
      // Vertical down from Controller
      sf::Vertex vDown[] = {
          sf::Vertex(sf::Vector2f(ctrlCx, yBottom), sf::Color(255, 140, 80)),
          sf::Vertex(sf::Vector2f(ctrlCx, yMid), sf::Color(255, 140, 80)),
      };
      window.draw(vDown, 2, sf::PrimitiveType::Lines);
      // Horizontal left to Merger column
      sf::Vertex hLeft[] = {
          sf::Vertex(sf::Vector2f(ctrlCx, yMid), sf::Color(255, 140, 80)),
          sf::Vertex(sf::Vector2f(mergCx, yMid), sf::Color(255, 140, 80)),
      };
      window.draw(hLeft, 2, sf::PrimitiveType::Lines);
      // Vertical down into Merger
      drawVArrow(mergCx, yMid, yTop, sf::Color(255, 140, 80));
    }

    // Legend row at the bottom of the panel
    float legY = tcY + tcH - 22.f;
    drawText("Bright border = active this step", tcX + 12.f, legY,
             sf::Color(110, 110, 130), 11);

    // ── Help bar ──
    drawText("Space=Pause  R=Restart  F=Follow  Up/Down=Speed  Scroll=Zoom  "
             "RightDrag=Pan  Esc=Quit",
             8.f, 4.f, sf::Color(110, 110, 140), 13);

    // ── Scale labels (screen space) ──
    {
      float scaleX = 14.f;
      float scaleY = static_cast<float>(winH) - 65.f;
      drawText("50 m (horizontal)", scaleX, scaleY - 16.f,
               sf::Color(200, 200, 200), 12);
    }

    window.display();
  }

  return 0;
}
