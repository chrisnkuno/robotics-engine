#include "rex/viewer/svg_renderer.hpp"

#include <algorithm>
#include <array>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "rex/viewer/controller.hpp"

namespace rex::viewer {

namespace {

constexpr double kPolygonTolerance = 1.0e-6;

struct Bounds {
  double min_x{0.0};
  double max_x{0.0};
  double min_z{0.0};
  double max_z{0.0};
};

[[nodiscard]] auto cross_z(const rex::viewer::ScreenPoint& origin, const rex::viewer::ScreenPoint& lhs, const rex::viewer::ScreenPoint& rhs)
  -> double {
  const double ax = lhs.x - origin.x;
  const double ay = lhs.y - origin.y;
  const double bx = rhs.x - origin.x;
  const double by = rhs.y - origin.y;
  return (ax * by) - (ay * bx);
}

[[nodiscard]] auto screen_point_less(const rex::viewer::ScreenPoint& lhs, const rex::viewer::ScreenPoint& rhs) -> bool {
  if (lhs.x < rhs.x) {
    return true;
  }

  if (rhs.x < lhs.x) {
    return false;
  }

  return lhs.y < rhs.y;
}

[[nodiscard]] auto body_box_corners(const SnapshotBody& body) -> std::vector<rex::math::Vec3> {
  const std::array<double, 2> signs = {-1.0, 1.0};
  std::vector<rex::math::Vec3> corners{};
  corners.reserve(8);
  const rex::math::Transform transform{
    .rotation = body.rotation,
    .translation = body.translation,
  };

  for (double sx : signs) {
    for (double sy : signs) {
      for (double sz : signs) {
        corners.push_back(rex::math::transform_point(
          transform,
          {
            sx * body.dimensions.x,
            sy * body.dimensions.y,
            sz * body.dimensions.z,
          }));
      }
    }
  }

  return corners;
}

[[nodiscard]] auto frame_bounds(const FrameSnapshot& frame) -> Bounds {
  Bounds bounds{};
  bool initialized = false;

  auto extend = [&](double x, double z) {
    if (!initialized) {
      bounds.min_x = bounds.max_x = x;
      bounds.min_z = bounds.max_z = z;
      initialized = true;
      return;
    }

    bounds.min_x = std::min(bounds.min_x, x);
    bounds.max_x = std::max(bounds.max_x, x);
    bounds.min_z = std::min(bounds.min_z, z);
    bounds.max_z = std::max(bounds.max_z, z);
  };

  for (const SnapshotBody& body : frame.bodies) {
    if (body.shape == SnapshotShapeKind::kSphere) {
      extend(body.translation.x - body.dimensions.x, body.translation.z - body.dimensions.x);
      extend(body.translation.x + body.dimensions.x, body.translation.z + body.dimensions.x);
    } else {
      for (const rex::math::Vec3& corner : body_box_corners(body)) {
        extend(corner.x, corner.z);
      }
    }
  }

  for (const SnapshotContact& contact : frame.contacts) {
    extend(contact.position.x, contact.position.z);
  }

  if (!initialized) {
    extend(-1.0, -1.0);
    extend(1.0, 1.0);
  }

  return bounds;
}

[[nodiscard]] auto project_x(double world_x, const Bounds& bounds, const SvgRenderConfig& config) -> double {
  const double range = std::max(bounds.max_x - bounds.min_x, 1.0);
  const double usable_width = static_cast<double>(config.width) - (config.margin * 2.0);
  return config.margin + ((world_x - bounds.min_x) / range) * usable_width;
}

[[nodiscard]] auto project_y(double world_z, const Bounds& bounds, const SvgRenderConfig& config) -> double {
  const double range = std::max(bounds.max_z - bounds.min_z, 1.0);
  const double usable_height = static_cast<double>(config.height) - (config.margin * 2.0);
  const double normalized = (world_z - bounds.min_z) / range;
  return static_cast<double>(config.height) - config.margin - (normalized * usable_height);
}

[[nodiscard]] auto project_scale(const Bounds& bounds, const SvgRenderConfig& config) -> double {
  const double range_x = std::max(bounds.max_x - bounds.min_x, 1.0);
  const double range_z = std::max(bounds.max_z - bounds.min_z, 1.0);
  const double usable_width = static_cast<double>(config.width) - (config.margin * 2.0);
  const double usable_height = static_cast<double>(config.height) - (config.margin * 2.0);
  return std::min(usable_width / range_x, usable_height / range_z);
}

[[nodiscard]] auto convex_hull(std::vector<rex::viewer::ScreenPoint> points)
  -> std::vector<rex::viewer::ScreenPoint> {
  if (points.size() <= 1) {
    return points;
  }

  std::sort(points.begin(), points.end(), screen_point_less);
  points.erase(
    std::unique(
      points.begin(),
      points.end(),
      [](const rex::viewer::ScreenPoint& lhs, const rex::viewer::ScreenPoint& rhs) {
        return std::abs(lhs.x - rhs.x) <= kPolygonTolerance &&
               std::abs(lhs.y - rhs.y) <= kPolygonTolerance;
      }),
    points.end());

  if (points.size() <= 2) {
    return points;
  }

  std::vector<rex::viewer::ScreenPoint> lower{};
  for (const auto& point : points) {
    while (lower.size() >= 2 &&
           cross_z(lower[lower.size() - 2], lower.back(), point) <= kPolygonTolerance) {
      lower.pop_back();
    }
    lower.push_back(point);
  }

  std::vector<rex::viewer::ScreenPoint> upper{};
  for (auto it = points.rbegin(); it != points.rend(); ++it) {
    while (upper.size() >= 2 &&
           cross_z(upper[upper.size() - 2], upper.back(), *it) <= kPolygonTolerance) {
      upper.pop_back();
    }
    upper.push_back(*it);
  }

  lower.pop_back();
  upper.pop_back();
  lower.insert(lower.end(), upper.begin(), upper.end());
  return lower;
}

[[nodiscard]] auto svg_polygon_points(const std::vector<rex::viewer::ScreenPoint>& points) -> std::string {
  std::ostringstream points_stream{};
  for (std::size_t index = 0; index < points.size(); ++index) {
    if (index > 0) {
      points_stream << ' ';
    }
    points_stream << points[index].x << ',' << points[index].y;
  }
  return points_stream.str();
}

}  // namespace

auto render_frame_svg(const FrameSnapshot& frame, const SvgRenderConfig& config) -> std::string {
  const Bounds bounds = frame_bounds(frame);
  const double scale = project_scale(bounds, config);

  std::ostringstream svg{};
  svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << config.width
      << "\" height=\"" << config.height
      << "\" viewBox=\"0 0 " << config.width << ' ' << config.height << "\">\n";
  svg << "<rect width=\"100%\" height=\"100%\" fill=\"#f7f4ea\" />\n";
  svg << "<text x=\"24\" y=\"32\" font-family=\"monospace\" font-size=\"18\" fill=\"#2a2622\">"
      << "frame=" << frame.frame_index
      << " time=" << frame.sim_time
      << " bodies=" << frame.trace.body_count
      << " contacts=" << frame.trace.solver.contact_count
      << "</text>\n";

  for (const SnapshotBody& body : frame.bodies) {
    const double cx = project_x(body.translation.x, bounds, config);
    const double cy = project_y(body.translation.z, bounds, config);

    if (body.shape == SnapshotShapeKind::kSphere) {
      svg << "<circle cx=\"" << cx
          << "\" cy=\"" << cy
          << "\" r=\"" << (body.dimensions.x * scale)
          << "\" fill=\"#8db3c7\" stroke=\"#193549\" stroke-width=\"2\" />\n";
    } else {
      std::vector<rex::viewer::ScreenPoint> projected{};
      projected.reserve(8);
      for (const rex::math::Vec3& corner : body_box_corners(body)) {
        projected.push_back({
          .x = project_x(corner.x, bounds, config),
          .y = project_y(corner.z, bounds, config),
        });
      }

      svg << "<polygon points=\"" << svg_polygon_points(convex_hull(std::move(projected)))
          << "\" fill=\"#d9a441\" stroke=\"#5b3200\" stroke-width=\"2\" />\n";
    }

    svg << "<text x=\"" << (cx + 8.0)
        << "\" y=\"" << (cy - 8.0)
        << "\" font-family=\"monospace\" font-size=\"14\" fill=\"#2a2622\">"
        << body.id.index
        << "</text>\n";
  }

  if (config.show_contacts) {
    for (const SnapshotContact& contact : frame.contacts) {
      const double px = project_x(contact.position.x, bounds, config);
      const double py = project_y(contact.position.z, bounds, config);
      svg << "<circle cx=\"" << px
          << "\" cy=\"" << py
          << "\" r=\"4\" fill=\"#c84545\" />\n";

      if (config.show_normals) {
        const double end_x = project_x(contact.position.x + (contact.normal.x * 0.4), bounds, config);
        const double end_y = project_y(contact.position.z + (contact.normal.z * 0.4), bounds, config);
        svg << "<line x1=\"" << px
            << "\" y1=\"" << py
            << "\" x2=\"" << end_x
            << "\" y2=\"" << end_y
            << "\" stroke=\"#c84545\" stroke-width=\"2\" />\n";
      }
    }
  }

  svg << "</svg>\n";
  return svg.str();
}

void write_frame_svg(
  const std::filesystem::path& path,
  const FrameSnapshot& frame,
  const SvgRenderConfig& config) {
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("failed to open svg file for writing: " + path.string());
  }

  output << render_frame_svg(frame, config);
}

}  // namespace rex::viewer
