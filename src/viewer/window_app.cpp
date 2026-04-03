#define SDL_MAIN_HANDLED
#include <SDL2/SDL.h>

#include "rex/viewer/window_app.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rex/platform/profile.hpp"
#include "rex/viewer/controller.hpp"

namespace rex::viewer {

namespace {

constexpr int kDefaultWidth = 1280;
constexpr int kDefaultHeight = 800;
constexpr int kSelectionOutlineThickness = 3;
constexpr double kAverageAlpha = 0.15;
constexpr double kOrbitRadiansPerPixel = 0.012;
constexpr double kPanWorldUnitsPerPixel = 0.0022;
constexpr double kClickThresholdPixels = 6.0;
constexpr double kTrailFrameWindow = 48.0;
constexpr double kPi = 3.14159265358979323846;
constexpr int kSphereLatitudeSegments = 12;
constexpr int kSphereLongitudeSegments = 18;

struct ViewerFrameProfile {
  double event_ms{0.0};
  double live_pump_ms{0.0};
  double cache_build_ms{0.0};
  double draw_ms{0.0};
  double frame_ms{0.0};
};

struct ViewerFrameAverages {
  double event_ms{0.0};
  double live_pump_ms{0.0};
  double cache_build_ms{0.0};
  double draw_ms{0.0};
  double frame_ms{0.0};
};

struct MeshTriangle {
  rex::math::Vec3 a{};
  rex::math::Vec3 b{};
  rex::math::Vec3 c{};
};

struct MeshData {
  std::vector<MeshTriangle> triangles{};
};

struct RenderTriangle {
  std::array<SDL_Vertex, 3> vertices{};
  double depth{0.0};
};

struct RenderLine {
  ScreenPoint start{};
  ScreenPoint end{};
  SDL_Color color{};
  double depth{0.0};
};

struct RenderScene {
  std::vector<RenderTriangle> triangles{};
  std::vector<RenderLine> lines{};
};

struct MeshLibrary {
  MeshData unit_box{};
  MeshData unit_sphere{};
  std::filesystem::path mesh_dir{};
  std::unordered_map<std::uint32_t, std::optional<MeshData>> imported{};
};

enum class DragMode {
  kNone,
  kTimeline,
  kOrbit,
  kPan,
};

struct PointerDragState {
  DragMode mode{DragMode::kNone};
  ScreenPoint anchor{};
  ScreenPoint last{};
  bool click_candidate{false};
};

[[nodiscard]] auto make_vertex(double x, double y, const SDL_Color& color) -> SDL_Vertex {
  return {
    .position = {static_cast<float>(x), static_cast<float>(y)},
    .color = color,
    .tex_coord = {0.0f, 0.0f},
  };
}

auto set_color(SDL_Renderer* renderer, const SDL_Color& color) -> void {
  SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
}

auto color_for_body(rex::platform::EntityId id) -> SDL_Color {
  const std::uint32_t seed = id.index * 2654435761u;
  return {
    static_cast<std::uint8_t>(90 + (seed % 120)),
    static_cast<std::uint8_t>(100 + ((seed >> 8) % 100)),
    static_cast<std::uint8_t>(120 + ((seed >> 16) % 100)),
    255,
  };
}

[[nodiscard]] auto modulate_color(const SDL_Color& color, double factor, std::uint8_t alpha = 255) -> SDL_Color {
  factor = std::clamp(factor, 0.0, 1.4);
  return {
    static_cast<std::uint8_t>(std::clamp(std::lround(static_cast<double>(color.r) * factor), 0l, 255l)),
    static_cast<std::uint8_t>(std::clamp(std::lround(static_cast<double>(color.g) * factor), 0l, 255l)),
    static_cast<std::uint8_t>(std::clamp(std::lround(static_cast<double>(color.b) * factor), 0l, 255l)),
    alpha,
  };
}

[[nodiscard]] auto distance_squared(const ScreenPoint& lhs, const ScreenPoint& rhs) -> double {
  const double dx = lhs.x - rhs.x;
  const double dy = lhs.y - rhs.y;
  return (dx * dx) + (dy * dy);
}

[[nodiscard]] auto make_title(
  const FrameSnapshot& frame,
  const ViewerState& state,
  const ReplayLog& replay,
  const ViewerFrameAverages& averages,
  bool live_mode) -> std::string {
  std::ostringstream title{};
  title << "rex viewer | frame " << (state.current_frame + 1) << '/' << replay.size()
        << " | " << (state.playback == PlaybackMode::kPlaying ? "playing" : "paused")
        << " | contacts " << frame.trace.solver.contact_count
        << " | sim " << frame.trace.profile.total_ms << "ms"
        << " | draw " << averages.draw_ms << "ms"
        << " | cache " << averages.cache_build_ms << "ms"
        << " | frame " << averages.frame_ms << "ms";

  if (live_mode) {
    title << " | live";
  }

  if (state.selection.body_index.has_value() && *state.selection.body_index < frame.bodies.size()) {
    const SnapshotBody& body = frame.bodies[*state.selection.body_index];
    title << " | body " << body.id.index
          << " pos=(" << body.translation.x << "," << body.translation.y << "," << body.translation.z << ")";
  } else if (state.selection.contact_index.has_value() && *state.selection.contact_index < frame.contacts.size()) {
    const SnapshotContact& contact = frame.contacts[*state.selection.contact_index];
    title << " | contact "
          << contact.body_a.index << "-" << contact.body_b.index
          << " pen=" << contact.penetration;
  }

  title << " | mouse: drag orbit/right-pan/wheel zoom | keys: space arrows wasd +/- c n g t r";
  return title.str();
}

auto draw_filled_circle(SDL_Renderer* renderer, int cx, int cy, int radius) -> void {
  for (int dy = -radius; dy <= radius; ++dy) {
    const int dx = static_cast<int>(std::sqrt(std::max((radius * radius) - (dy * dy), 0)));
    SDL_RenderDrawLine(renderer, cx - dx, cy + dy, cx + dx, cy + dy);
  }
}

auto create_renderer(SDL_Window* window) -> SDL_Renderer* {
  constexpr std::array<Uint32, 3> kRendererFlags = {
    SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC,
    SDL_RENDERER_ACCELERATED,
    SDL_RENDERER_SOFTWARE,
  };

  for (Uint32 flags : kRendererFlags) {
    if (SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, flags)) {
      return renderer;
    }
  }

  return nullptr;
}

auto draw_rect_outline(SDL_Renderer* renderer, SDL_Rect rect, int thickness) -> void {
  for (int offset = 0; offset < thickness; ++offset) {
    SDL_RenderDrawRect(renderer, &rect);
    --rect.x;
    --rect.y;
    rect.w += 2;
    rect.h += 2;
  }
}

auto draw_polygon_outline(SDL_Renderer* renderer, const std::vector<ScreenPoint>& polygon, int thickness = 1) -> void {
  if (polygon.size() < 2) {
    return;
  }

  for (int offset = 0; offset < thickness; ++offset) {
    for (std::size_t index = 0; index < polygon.size(); ++index) {
      const ScreenPoint& start = polygon[index];
      const ScreenPoint& end = polygon[(index + 1) % polygon.size()];
      SDL_RenderDrawLine(
        renderer,
        static_cast<int>(std::round(start.x)),
        static_cast<int>(std::round(start.y)) + offset,
        static_cast<int>(std::round(end.x)),
        static_cast<int>(std::round(end.y)) + offset);
    }
  }
}

auto viewport_for_window(SDL_Window* window) -> FrameViewport {
  int width = kDefaultWidth;
  int height = kDefaultHeight;
  SDL_GetWindowSize(window, &width, &height);
  return {
    .width = static_cast<double>(width),
    .height = static_cast<double>(height),
    .margin = 60.0,
  };
}

void save_screenshot(SDL_Renderer* renderer, const FrameViewport& viewport, const std::filesystem::path& path) {
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
  SDL_Surface* surface = SDL_CreateRGBSurfaceWithFormat(
    0,
    static_cast<int>(std::round(viewport.width)),
    static_cast<int>(std::round(viewport.height)),
    32,
    SDL_PIXELFORMAT_ARGB8888);
  if (surface == nullptr) {
    throw std::runtime_error(std::string{"SDL_CreateRGBSurfaceWithFormat failed: "} + SDL_GetError());
  }

  const int read_result = SDL_RenderReadPixels(renderer, nullptr, SDL_PIXELFORMAT_ARGB8888, surface->pixels, surface->pitch);
  if (read_result != 0) {
    SDL_FreeSurface(surface);
    throw std::runtime_error(std::string{"SDL_RenderReadPixels failed: "} + SDL_GetError());
  }

  if (SDL_SaveBMP(surface, path.string().c_str()) != 0) {
    SDL_FreeSurface(surface);
    throw std::runtime_error(std::string{"SDL_SaveBMP failed: "} + SDL_GetError());
  }

  SDL_FreeSurface(surface);
}

void update_average(double sample, double& average) {
  average = average == 0.0 ? sample : (average + ((sample - average) * kAverageAlpha));
}

void update_averages(const ViewerFrameProfile& sample, ViewerFrameAverages& averages) {
  update_average(sample.event_ms, averages.event_ms);
  update_average(sample.live_pump_ms, averages.live_pump_ms);
  update_average(sample.cache_build_ms, averages.cache_build_ms);
  update_average(sample.draw_ms, averages.draw_ms);
  update_average(sample.frame_ms, averages.frame_ms);
}

[[nodiscard]] auto make_unit_box_mesh() -> MeshData {
  MeshData mesh{};
  const std::array<rex::math::Vec3, 8> corners = {{
    {-1.0, -1.0, -1.0},
    {1.0, -1.0, -1.0},
    {1.0, 1.0, -1.0},
    {-1.0, 1.0, -1.0},
    {-1.0, -1.0, 1.0},
    {1.0, -1.0, 1.0},
    {1.0, 1.0, 1.0},
    {-1.0, 1.0, 1.0},
  }};
  const auto add_face = [&](int a, int b, int c, int d) {
    mesh.triangles.push_back({corners[a], corners[b], corners[c]});
    mesh.triangles.push_back({corners[a], corners[c], corners[d]});
  };

  add_face(0, 1, 2, 3);
  add_face(4, 7, 6, 5);
  add_face(0, 4, 5, 1);
  add_face(1, 5, 6, 2);
  add_face(2, 6, 7, 3);
  add_face(3, 7, 4, 0);
  return mesh;
}

[[nodiscard]] auto spherical_point(double theta, double phi) -> rex::math::Vec3 {
  return {
    std::sin(phi) * std::cos(theta),
    std::sin(phi) * std::sin(theta),
    std::cos(phi),
  };
}

[[nodiscard]] auto make_unit_sphere_mesh() -> MeshData {
  MeshData mesh{};
  for (int lat = 0; lat < kSphereLatitudeSegments; ++lat) {
    const double phi0 = kPi * static_cast<double>(lat) / static_cast<double>(kSphereLatitudeSegments);
    const double phi1 = kPi * static_cast<double>(lat + 1) / static_cast<double>(kSphereLatitudeSegments);
    for (int lon = 0; lon < kSphereLongitudeSegments; ++lon) {
      const double theta0 = (2.0 * kPi * static_cast<double>(lon)) / static_cast<double>(kSphereLongitudeSegments);
      const double theta1 = (2.0 * kPi * static_cast<double>(lon + 1)) / static_cast<double>(kSphereLongitudeSegments);

      const rex::math::Vec3 p00 = spherical_point(theta0, phi0);
      const rex::math::Vec3 p01 = spherical_point(theta1, phi0);
      const rex::math::Vec3 p10 = spherical_point(theta0, phi1);
      const rex::math::Vec3 p11 = spherical_point(theta1, phi1);

      if (lat > 0) {
        mesh.triangles.push_back({p00, p10, p11});
      }
      if (lat + 1 < kSphereLatitudeSegments) {
        mesh.triangles.push_back({p00, p11, p01});
      }
    }
  }

  return mesh;
}

[[nodiscard]] auto try_parse_index(std::string_view token) -> int {
  if (token.empty()) {
    return 0;
  }

  return std::stoi(std::string{token});
}

[[nodiscard]] auto normalize_mesh(MeshData mesh) -> MeshData {
  rex::math::Vec3 min{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
  rex::math::Vec3 max{-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};

  for (const MeshTriangle& triangle : mesh.triangles) {
    for (const rex::math::Vec3 vertex : {triangle.a, triangle.b, triangle.c}) {
      min.x = std::min(min.x, vertex.x);
      min.y = std::min(min.y, vertex.y);
      min.z = std::min(min.z, vertex.z);
      max.x = std::max(max.x, vertex.x);
      max.y = std::max(max.y, vertex.y);
      max.z = std::max(max.z, vertex.z);
    }
  }

  const rex::math::Vec3 center = (min + max) * 0.5;
  const rex::math::Vec3 half_extents{
    std::max((max.x - min.x) * 0.5, 1.0e-6),
    std::max((max.y - min.y) * 0.5, 1.0e-6),
    std::max((max.z - min.z) * 0.5, 1.0e-6),
  };

  for (MeshTriangle& triangle : mesh.triangles) {
    triangle.a = {
      (triangle.a.x - center.x) / half_extents.x,
      (triangle.a.y - center.y) / half_extents.y,
      (triangle.a.z - center.z) / half_extents.z,
    };
    triangle.b = {
      (triangle.b.x - center.x) / half_extents.x,
      (triangle.b.y - center.y) / half_extents.y,
      (triangle.b.z - center.z) / half_extents.z,
    };
    triangle.c = {
      (triangle.c.x - center.x) / half_extents.x,
      (triangle.c.y - center.y) / half_extents.y,
      (triangle.c.z - center.z) / half_extents.z,
    };
  }

  return mesh;
}

[[nodiscard]] auto load_obj_mesh(const std::filesystem::path& path) -> std::optional<MeshData> {
  std::ifstream input{path};
  if (!input.is_open()) {
    return std::nullopt;
  }

  std::vector<rex::math::Vec3> positions{};
  MeshData mesh{};
  std::string line{};
  while (std::getline(input, line)) {
    std::istringstream stream{line};
    std::string kind{};
    stream >> kind;
    if (kind == "v") {
      rex::math::Vec3 position{};
      stream >> position.x >> position.y >> position.z;
      positions.push_back(position);
      continue;
    }

    if (kind != "f") {
      continue;
    }

    std::vector<int> face_indices{};
    std::string token{};
    while (stream >> token) {
      const std::size_t slash = token.find('/');
      const std::string_view index_token = slash == std::string::npos
        ? std::string_view{token}
        : std::string_view{token}.substr(0, slash);
      const int index = try_parse_index(index_token);
      if (index == 0) {
        continue;
      }
      face_indices.push_back(index > 0 ? index - 1 : static_cast<int>(positions.size()) + index);
    }

    if (face_indices.size() < 3) {
      continue;
    }

    for (std::size_t face_index = 1; face_index + 1 < face_indices.size(); ++face_index) {
      const int a = face_indices[0];
      const int b = face_indices[face_index];
      const int c = face_indices[face_index + 1];
      if (a < 0 || b < 0 || c < 0 ||
          a >= static_cast<int>(positions.size()) ||
          b >= static_cast<int>(positions.size()) ||
          c >= static_cast<int>(positions.size())) {
        continue;
      }
      mesh.triangles.push_back({positions[a], positions[b], positions[c]});
    }
  }

  if (mesh.triangles.empty()) {
    return std::nullopt;
  }

  return normalize_mesh(std::move(mesh));
}

[[nodiscard]] auto make_mesh_library() -> MeshLibrary {
  MeshLibrary library{};
  library.unit_box = make_unit_box_mesh();
  library.unit_sphere = make_unit_sphere_mesh();
  if (const char* mesh_dir = std::getenv("REX_VIEWER_MESH_DIR")) {
    library.mesh_dir = mesh_dir;
  }
  return library;
}

auto mesh_for_body(MeshLibrary& library, const SnapshotBody& body) -> const MeshData& {
  if (!library.mesh_dir.empty()) {
    const auto cached = library.imported.find(body.id.index);
    if (cached == library.imported.end()) {
      const std::filesystem::path mesh_path = library.mesh_dir / (std::to_string(body.id.index) + ".obj");
      library.imported.emplace(body.id.index, load_obj_mesh(mesh_path));
    }

    const auto it = library.imported.find(body.id.index);
    if (it != library.imported.end() && it->second.has_value()) {
      return *it->second;
    }
  }

  return body.shape == SnapshotShapeKind::kBox ? library.unit_box : library.unit_sphere;
}

[[nodiscard]] auto body_scale(const SnapshotBody& body) -> rex::math::Vec3 {
  if (body.shape == SnapshotShapeKind::kSphere) {
    return {body.dimensions.x, body.dimensions.x, body.dimensions.x};
  }

  return body.dimensions;
}

[[nodiscard]] auto transform_mesh_vertex(
  const SnapshotBody& body,
  const rex::math::Vec3& local_vertex) -> rex::math::Vec3 {
  const rex::math::Vec3 scale = body_scale(body);
  return rex::math::transform_point(
    {.rotation = body.rotation, .translation = body.translation},
    {
      local_vertex.x * scale.x,
      local_vertex.y * scale.y,
      local_vertex.z * scale.z,
    });
}

[[nodiscard]] auto triangle_depth(
  const Camera3D& camera,
  const rex::math::Vec3& a,
  const rex::math::Vec3& b,
  const rex::math::Vec3& c) -> double {
  const rex::math::Vec3 camera_pos = camera_position(camera);
  const rex::math::Vec3 forward = camera_forward(camera);
  return (
    rex::math::dot(a - camera_pos, forward) +
    rex::math::dot(b - camera_pos, forward) +
    rex::math::dot(c - camera_pos, forward)) / 3.0;
}

void append_body_mesh(
  RenderScene& scene,
  MeshLibrary& library,
  const Camera3D& camera,
  const FrameViewport& viewport,
  const SnapshotBody& body,
  bool selected) {
  const MeshData& mesh = mesh_for_body(library, body);
  const SDL_Color base_color = color_for_body(body.id);
  const rex::math::Vec3 camera_pos = camera_position(camera);
  const rex::math::Vec3 light_dir = rex::math::normalized_or(rex::math::Vec3{0.45, -0.35, 0.8});

  for (const MeshTriangle& triangle : mesh.triangles) {
    const rex::math::Vec3 a = transform_mesh_vertex(body, triangle.a);
    const rex::math::Vec3 b = transform_mesh_vertex(body, triangle.b);
    const rex::math::Vec3 c = transform_mesh_vertex(body, triangle.c);
    const rex::math::Vec3 normal = rex::math::normalized_or(rex::math::cross(b - a, c - a), {0.0, 0.0, 1.0});
    const rex::math::Vec3 centroid = (a + b + c) / 3.0;
    const rex::math::Vec3 to_camera = rex::math::normalized_or(camera_pos - centroid, {0.0, 1.0, 0.0});
    if (rex::math::dot(normal, to_camera) <= 0.0) {
      continue;
    }

    const ProjectedPoint pa = project_to_screen(camera, viewport, a);
    const ProjectedPoint pb = project_to_screen(camera, viewport, b);
    const ProjectedPoint pc = project_to_screen(camera, viewport, c);
    if (!pa.visible || !pb.visible || !pc.visible) {
      continue;
    }

    const double diffuse = std::max(rex::math::dot(normal, light_dir), 0.0);
    const double rim = std::pow(1.0 - std::max(rex::math::dot(normal, to_camera), 0.0), 2.0);
    const double intensity = 0.24 + (diffuse * 0.72) + (rim * 0.14) + (selected ? 0.12 : 0.0);
    const SDL_Color color = modulate_color(base_color, intensity);

    scene.triangles.push_back({
      .vertices = {{
        make_vertex(pa.point.x, pa.point.y, color),
        make_vertex(pb.point.x, pb.point.y, color),
        make_vertex(pc.point.x, pc.point.y, color),
      }},
      .depth = triangle_depth(camera, a, b, c),
    });
  }
}

void append_grid(RenderScene& scene, const Camera3D& camera, const FrameViewport& viewport) {
  const double extent = std::max(6.0, std::ceil(camera.distance * 1.5));
  const double target_x = std::round(camera.target.x);
  const double target_y = std::round(camera.target.y);

  for (int offset = -static_cast<int>(extent); offset <= static_cast<int>(extent); ++offset) {
    const double x = target_x + static_cast<double>(offset);
    const double y = target_y + static_cast<double>(offset);
    const SDL_Color x_color = offset == 0 ? SDL_Color{194, 104, 89, 255} : SDL_Color{205, 200, 188, 160};
    const SDL_Color y_color = offset == 0 ? SDL_Color{91, 151, 96, 255} : SDL_Color{205, 200, 188, 160};

    const rex::math::Vec3 x0{x, target_y - extent, 0.0};
    const rex::math::Vec3 x1{x, target_y + extent, 0.0};
    const rex::math::Vec3 y0{target_x - extent, y, 0.0};
    const rex::math::Vec3 y1{target_x + extent, y, 0.0};

    const ProjectedPoint px0 = project_to_screen(camera, viewport, x0);
    const ProjectedPoint px1 = project_to_screen(camera, viewport, x1);
    const ProjectedPoint py0 = project_to_screen(camera, viewport, y0);
    const ProjectedPoint py1 = project_to_screen(camera, viewport, y1);

    if (px0.visible && px1.visible) {
      scene.lines.push_back({.start = px0.point, .end = px1.point, .color = x_color, .depth = std::min(px0.depth, px1.depth)});
    }
    if (py0.visible && py1.visible) {
      scene.lines.push_back({.start = py0.point, .end = py1.point, .color = y_color, .depth = std::min(py0.depth, py1.depth)});
    }
  }

  const ProjectedPoint z0 = project_to_screen(camera, viewport, {target_x, target_y, 0.0});
  const ProjectedPoint z1 = project_to_screen(camera, viewport, {target_x, target_y, extent * 0.75});
  if (z0.visible && z1.visible) {
    scene.lines.push_back({.start = z0.point, .end = z1.point, .color = {76, 118, 201, 255}, .depth = std::min(z0.depth, z1.depth)});
  }
}

void append_trails(
  RenderScene& scene,
  const ReplayLog& replay,
  const ViewerState& state,
  const FrameViewport& viewport) {
  if (replay.empty()) {
    return;
  }

  const std::size_t first_frame = state.current_frame > static_cast<std::size_t>(kTrailFrameWindow)
    ? state.current_frame - static_cast<std::size_t>(kTrailFrameWindow)
    : 0;

  for (std::size_t body_index = 0; body_index < replay.frames()[state.current_frame].bodies.size(); ++body_index) {
    const SnapshotBody& current_body = replay.frames()[state.current_frame].bodies[body_index];
    ScreenPoint previous_point{};
    double previous_depth = 0.0;
    bool has_previous = false;
    for (std::size_t frame_index = first_frame; frame_index <= state.current_frame; ++frame_index) {
      const FrameSnapshot& frame = replay.frames()[frame_index];
      if (body_index >= frame.bodies.size()) {
        continue;
      }

      const ProjectedPoint projected = project_to_screen(state.camera, viewport, frame.bodies[body_index].translation);
      if (!projected.visible) {
        has_previous = false;
        continue;
      }

      if (has_previous) {
        const double alpha = static_cast<double>(frame_index - first_frame + 1) /
          static_cast<double>(state.current_frame - first_frame + 1);
        scene.lines.push_back({
          .start = previous_point,
          .end = projected.point,
          .color = modulate_color(color_for_body(current_body.id), 0.85, static_cast<std::uint8_t>(50 + std::lround(alpha * 110.0))),
          .depth = std::min(previous_depth, projected.depth),
        });
      }

      previous_point = projected.point;
      previous_depth = projected.depth;
      has_previous = true;
    }
  }
}

void append_scene_geometry(
  RenderScene& scene,
  MeshLibrary& library,
  const ReplayLog& replay,
  const FrameSnapshot& frame,
  const ViewerState& state,
  const FrameViewport& viewport) {
  if (state.overlay.show_grid) {
    append_grid(scene, state.camera, viewport);
  }
  if (state.overlay.show_trails) {
    append_trails(scene, replay, state, viewport);
  }

  for (std::size_t body_index = 0; body_index < frame.bodies.size(); ++body_index) {
    append_body_mesh(
      scene,
      library,
      state.camera,
      viewport,
      frame.bodies[body_index],
      state.selection.body_index == std::optional<std::size_t>{body_index});
  }

  std::sort(
    scene.triangles.begin(),
    scene.triangles.end(),
    [](const RenderTriangle& lhs, const RenderTriangle& rhs) {
      return lhs.depth > rhs.depth;
    });
  std::sort(
    scene.lines.begin(),
    scene.lines.end(),
    [](const RenderLine& lhs, const RenderLine& rhs) {
      return lhs.depth > rhs.depth;
    });
}

void draw_timeline(SDL_Renderer* renderer, const ReplayLog& replay, const ViewerState& state, const FrameViewport& viewport) {
  const TimelineRect timeline = timeline_rect(viewport);
  SDL_Rect background{
    .x = static_cast<int>(std::round(timeline.left)),
    .y = static_cast<int>(std::round(timeline.top)),
    .w = static_cast<int>(std::round(timeline.width)),
    .h = static_cast<int>(std::round(timeline.height)),
  };
  set_color(renderer, {228, 222, 203, 255});
  SDL_RenderFillRect(renderer, &background);
  set_color(renderer, {90, 87, 80, 255});
  SDL_RenderDrawRect(renderer, &background);

  if (replay.size() > 1) {
    const double alpha = static_cast<double>(state.current_frame) / static_cast<double>(replay.size() - 1);
    const int thumb_x = static_cast<int>(std::round(timeline.left + (alpha * timeline.width)));
    set_color(renderer, {57, 108, 176, 255});
    SDL_RenderDrawLine(
      renderer,
      thumb_x,
      static_cast<int>(std::round(timeline.top)) - 3,
      thumb_x,
      static_cast<int>(std::round(timeline.top + timeline.height)) + 3);

    const std::size_t tick_count = std::min<std::size_t>(replay.size(), 20);
    for (std::size_t tick_index = 0; tick_index < tick_count; ++tick_index) {
      const double tick_alpha = tick_count == 1 ? 0.0 : static_cast<double>(tick_index) / static_cast<double>(tick_count - 1);
      const int tick_x = static_cast<int>(std::round(timeline.left + (tick_alpha * timeline.width)));
      set_color(renderer, {150, 145, 134, 255});
      SDL_RenderDrawLine(
        renderer,
        tick_x,
        static_cast<int>(std::round(timeline.top)),
        tick_x,
        static_cast<int>(std::round(timeline.top + timeline.height)));
    }
  }
}

void draw_frame(
  SDL_Renderer* renderer,
  MeshLibrary& library,
  const ReplayLog& replay,
  const FrameSnapshot& frame,
  const ViewerState& state,
  const FrameViewport& viewport,
  const FrameProjectionCache& cache) {
  REX_PROFILE_SCOPE("viewer::draw_frame");
  set_color(renderer, {244, 241, 232, 255});
  SDL_RenderClear(renderer);
  SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

  RenderScene scene{};
  append_scene_geometry(scene, library, replay, frame, state, viewport);
  for (const RenderTriangle& triangle : scene.triangles) {
    SDL_RenderGeometry(renderer, nullptr, triangle.vertices.data(), static_cast<int>(triangle.vertices.size()), nullptr, 0);
  }
  for (const RenderLine& line : scene.lines) {
    set_color(renderer, line.color);
    SDL_RenderDrawLine(
      renderer,
      static_cast<int>(std::round(line.start.x)),
      static_cast<int>(std::round(line.start.y)),
      static_cast<int>(std::round(line.end.x)),
      static_cast<int>(std::round(line.end.y)));
  }

  if (state.overlay.show_contacts) {
    for (std::size_t contact_index = 0; contact_index < frame.contacts.size(); ++contact_index) {
      const ProjectedContact& projected = cache.contacts[contact_index];
      if (!projected.visible) {
        continue;
      }

      set_color(renderer, {200, 69, 69, 220});
      draw_filled_circle(
        renderer,
        static_cast<int>(std::round(projected.position.x)),
        static_cast<int>(std::round(projected.position.y)),
        4);

      if (state.overlay.show_normals) {
        SDL_RenderDrawLine(
          renderer,
          static_cast<int>(std::round(projected.position.x)),
          static_cast<int>(std::round(projected.position.y)),
          static_cast<int>(std::round(projected.normal_end.x)),
          static_cast<int>(std::round(projected.normal_end.y)));
      }
    }
  }

  if (state.selection.body_index.has_value() && *state.selection.body_index < frame.bodies.size()) {
    const SnapshotBody& body = frame.bodies[*state.selection.body_index];
    const ProjectedBody& projected = cache.bodies[*state.selection.body_index];
    set_color(renderer, {33, 90, 166, 255});
    if (body.shape == SnapshotShapeKind::kBox && !projected.outline.empty()) {
      draw_polygon_outline(renderer, projected.outline, kSelectionOutlineThickness);
    } else {
      const int radius = std::max(static_cast<int>(std::round(projected.radius_pixels)), 2);
      SDL_Rect highlight{
        .x = static_cast<int>(std::round(projected.center.x)) - radius,
        .y = static_cast<int>(std::round(projected.center.y)) - radius,
        .w = radius * 2,
        .h = radius * 2,
      };
      draw_rect_outline(renderer, highlight, kSelectionOutlineThickness);
    }
  }

  if (state.selection.contact_index.has_value() && *state.selection.contact_index < frame.contacts.size()) {
    const ProjectedContact& contact = cache.contacts[*state.selection.contact_index];
    set_color(renderer, {219, 122, 28, 255});
    SDL_RenderDrawLine(
      renderer,
      static_cast<int>(std::round(contact.position.x)) - 8,
      static_cast<int>(std::round(contact.position.y)),
      static_cast<int>(std::round(contact.position.x)) + 8,
      static_cast<int>(std::round(contact.position.y)));
    SDL_RenderDrawLine(
      renderer,
      static_cast<int>(std::round(contact.position.x)),
      static_cast<int>(std::round(contact.position.y)) - 8,
      static_cast<int>(std::round(contact.position.x)),
      static_cast<int>(std::round(contact.position.y)) + 8);
  }

  draw_timeline(renderer, replay, state, viewport);
  SDL_RenderPresent(renderer);
}

auto handle_key(
  ViewerState& state,
  const ReplayLog& replay,
  const SDL_KeyboardEvent& key,
  const FrameViewport& viewport,
  bool& should_quit) -> void {
  if (key.repeat != 0) {
    return;
  }

  switch (key.keysym.sym) {
    case SDLK_ESCAPE:
      should_quit = true;
      return;
    case SDLK_SPACE:
      apply_command(state, replay, ViewerCommand::kTogglePlayPause, viewport);
      return;
    case SDLK_RIGHT:
      apply_command(state, replay, ViewerCommand::kStepForward, viewport);
      return;
    case SDLK_LEFT:
      apply_command(state, replay, ViewerCommand::kStepBackward, viewport);
      return;
    case SDLK_c:
      apply_command(state, replay, ViewerCommand::kToggleContacts, viewport);
      return;
    case SDLK_n:
      apply_command(state, replay, ViewerCommand::kToggleNormals, viewport);
      return;
    case SDLK_g:
      apply_command(state, replay, ViewerCommand::kToggleGrid, viewport);
      return;
    case SDLK_t:
      apply_command(state, replay, ViewerCommand::kToggleTrails, viewport);
      return;
    case SDLK_r:
      apply_command(state, replay, ViewerCommand::kResetCamera, viewport);
      return;
    case SDLK_EQUALS:
    case SDLK_PLUS:
    case SDLK_KP_PLUS:
      apply_command(state, replay, ViewerCommand::kZoomIn, viewport);
      return;
    case SDLK_MINUS:
    case SDLK_KP_MINUS:
      apply_command(state, replay, ViewerCommand::kZoomOut, viewport);
      return;
    case SDLK_a:
      apply_command(state, replay, ViewerCommand::kPanLeft, viewport);
      return;
    case SDLK_d:
      apply_command(state, replay, ViewerCommand::kPanRight, viewport);
      return;
    case SDLK_w:
      apply_command(state, replay, ViewerCommand::kPanUp, viewport);
      return;
    case SDLK_s:
      apply_command(state, replay, ViewerCommand::kPanDown, viewport);
      return;
    default:
      return;
  }
}

}  // namespace

auto run_windowed_viewer_impl(ReplayLog replay, LiveFramePump frame_pump, WindowRunOptions options) -> int {
  if (frame_pump && replay.empty()) {
    if (!frame_pump(replay) || replay.empty()) {
      throw std::runtime_error("live viewer could not produce an initial frame");
    }
  }

  if (replay.empty()) {
    throw std::runtime_error("cannot open a window for an empty replay");
  }

  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
    throw std::runtime_error(std::string{"SDL_Init failed: "} + SDL_GetError());
  }

  SDL_SetHint(SDL_HINT_RENDER_BATCHING, "1");
  SDL_SetHint(SDL_HINT_RENDER_VSYNC, "1");
  SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "best");

  SDL_Window* window = SDL_CreateWindow(
    "rex viewer",
    SDL_WINDOWPOS_CENTERED,
    SDL_WINDOWPOS_CENTERED,
    kDefaultWidth,
    kDefaultHeight,
    SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
  if (window == nullptr) {
    SDL_Quit();
    throw std::runtime_error(std::string{"SDL_CreateWindow failed: "} + SDL_GetError());
  }

  SDL_Renderer* renderer = create_renderer(window);
  if (renderer == nullptr) {
    SDL_DestroyWindow(window);
    SDL_Quit();
    throw std::runtime_error(std::string{"SDL_CreateRenderer failed: "} + SDL_GetError());
  }

  MeshLibrary mesh_library = make_mesh_library();
  ViewerState state = make_viewer_state(replay);
  if (frame_pump) {
    state.playback = PlaybackMode::kPlaying;
  }
  FrameViewport viewport = viewport_for_window(window);
  fit_camera_to_frame(state, replay.frames()[state.current_frame], viewport);

  bool should_quit = false;
  PointerDragState drag{};
  bool screenshot_captured = false;
  std::size_t rendered_frames = 0;
  std::uint32_t last_ticks = SDL_GetTicks();
  double live_accumulated_time = 0.0;
  std::string current_title{};
  ViewerFrameAverages averages{};

  while (!should_quit) {
    REX_PROFILE_FRAME_MARK();
    ViewerFrameProfile frame_profile{};
    rex::platform::ScopedMilliseconds frame_timer(frame_profile.frame_ms);
    SDL_Event event{};
    const bool should_block_for_events =
      drag.mode == DragMode::kNone &&
      state.playback != PlaybackMode::kPlaying;
    {
      REX_PROFILE_SCOPE("viewer::events");
      rex::platform::ScopedMilliseconds event_timer(frame_profile.event_ms);
      bool has_event = should_block_for_events
        ? SDL_WaitEventTimeout(&event, 16) == 1
        : SDL_PollEvent(&event) != 0;
      while (has_event) {
        switch (event.type) {
          case SDL_QUIT:
            should_quit = true;
            break;

          case SDL_KEYDOWN:
            handle_key(state, replay, event.key, viewport, should_quit);
            break;

          case SDL_MOUSEWHEEL: {
            const double factor = std::pow(1.12, static_cast<double>(event.wheel.y));
            dolly_camera(state.camera, factor);
            break;
          }

          case SDL_MOUSEBUTTONDOWN: {
            const ScreenPoint point{
              .x = static_cast<double>(event.button.x),
              .y = static_cast<double>(event.button.y),
            };
            drag.anchor = point;
            drag.last = point;
            if (event.button.button == SDL_BUTTON_LEFT) {
              if (point_in_timeline(timeline_rect(viewport), point)) {
                drag.mode = DragMode::kTimeline;
                scrub_to_timeline(state, replay, viewport, point.x);
              } else {
                drag.mode = DragMode::kOrbit;
                drag.click_candidate = true;
              }
            } else if (event.button.button == SDL_BUTTON_RIGHT || event.button.button == SDL_BUTTON_MIDDLE) {
              drag.mode = DragMode::kPan;
            }
            break;
          }

          case SDL_MOUSEBUTTONUP: {
            const ScreenPoint point{
              .x = static_cast<double>(event.button.x),
              .y = static_cast<double>(event.button.y),
            };
            if (event.button.button == SDL_BUTTON_LEFT &&
                drag.mode == DragMode::kOrbit &&
                drag.click_candidate &&
                distance_squared(drag.anchor, point) <= (kClickThresholdPixels * kClickThresholdPixels)) {
              select_at_point(state, replay.frames()[state.current_frame], viewport, point);
            }
            drag = {};
            break;
          }

          case SDL_MOUSEMOTION: {
            const ScreenPoint point{
              .x = static_cast<double>(event.motion.x),
              .y = static_cast<double>(event.motion.y),
            };
            const double dx = point.x - drag.last.x;
            const double dy = point.y - drag.last.y;
            drag.last = point;
            if (drag.mode == DragMode::kTimeline) {
              scrub_to_timeline(state, replay, viewport, point.x);
            } else if (drag.mode == DragMode::kOrbit) {
              if (distance_squared(drag.anchor, point) > (kClickThresholdPixels * kClickThresholdPixels)) {
                drag.click_candidate = false;
              }
              orbit_camera(state.camera, -dx * kOrbitRadiansPerPixel, dy * kOrbitRadiansPerPixel);
            } else if (drag.mode == DragMode::kPan) {
              const double pan_scale = std::max(state.camera.distance * kPanWorldUnitsPerPixel, 0.002);
              pan_camera(state.camera, -dx * pan_scale, dy * pan_scale);
            }
            break;
          }

          case SDL_WINDOWEVENT:
            if (event.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {
              viewport.width = static_cast<double>(event.window.data1);
              viewport.height = static_cast<double>(event.window.data2);
            }
            break;

          default:
            break;
        }

        has_event = SDL_PollEvent(&event) != 0;
      }
    }

    const std::uint32_t ticks = SDL_GetTicks();
    const double delta_seconds = static_cast<double>(ticks - last_ticks) / 1000.0;
    last_ticks = ticks;

    if (frame_pump && state.playback == PlaybackMode::kPlaying && state.playback_fps > 0.0) {
      REX_PROFILE_SCOPE("viewer::live_pump");
      rex::platform::ScopedMilliseconds live_pump_timer(frame_profile.live_pump_ms);
      live_accumulated_time += delta_seconds;
      const double frame_period = 1.0 / state.playback_fps;
      while (live_accumulated_time >= frame_period) {
        live_accumulated_time -= frame_period;
        if (!frame_pump(replay)) {
          break;
        }
      }
    }

    update_playback(state, replay, delta_seconds);
    const FrameSnapshot& frame = replay.frames()[state.current_frame];
    FrameProjectionCache cache{};
    {
      REX_PROFILE_SCOPE("viewer::cache");
      rex::platform::ScopedMilliseconds cache_timer(frame_profile.cache_build_ms);
      cache = build_frame_projection_cache(frame, state.camera, viewport);
    }
    {
      REX_PROFILE_SCOPE("viewer::draw");
      rex::platform::ScopedMilliseconds draw_timer(frame_profile.draw_ms);
      draw_frame(renderer, mesh_library, replay, frame, state, viewport, cache);
    }
    if (!screenshot_captured && !options.screenshot_path.empty()) {
      save_screenshot(renderer, viewport, options.screenshot_path);
      screenshot_captured = true;
      if (options.max_frames == 0) {
        should_quit = true;
      }
    }
    update_averages(frame_profile, averages);
    const std::string next_title = make_title(frame, state, replay, averages, static_cast<bool>(frame_pump));
    if (next_title != current_title) {
      SDL_SetWindowTitle(window, next_title.c_str());
      current_title = next_title;
    }

    ++rendered_frames;
    if (options.max_frames > 0 && rendered_frames >= options.max_frames) {
      should_quit = true;
    }
  }

  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}

auto run_windowed_viewer(const ReplayLog& replay, WindowRunOptions options) -> int {
  return run_windowed_viewer_impl(replay, {}, options);
}

auto run_live_windowed_viewer(ReplayLog replay, LiveFramePump frame_pump, WindowRunOptions options) -> int {
  if (!frame_pump) {
    throw std::runtime_error("live viewer requires a valid frame pump");
  }

  return run_windowed_viewer_impl(std::move(replay), std::move(frame_pump), options);
}

}  // namespace rex::viewer
