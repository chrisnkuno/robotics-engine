#include "rex/viewer/svg_renderer.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace rex::viewer {

namespace {

struct Bounds {
  double min_x{0.0};
  double max_x{0.0};
  double min_z{0.0};
  double max_z{0.0};
};

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
      extend(body.translation.x - body.dimensions.x, body.translation.z - body.dimensions.z);
      extend(body.translation.x + body.dimensions.x, body.translation.z + body.dimensions.z);
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
      const double width = body.dimensions.x * 2.0 * scale;
      const double height = body.dimensions.z * 2.0 * scale;
      svg << "<rect x=\"" << (cx - (width * 0.5))
          << "\" y=\"" << (cy - (height * 0.5))
          << "\" width=\"" << width
          << "\" height=\"" << height
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
