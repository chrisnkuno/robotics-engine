#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "rex/math/types.hpp"
#include "rex/platform/handles.hpp"
#include "rex/sim/engine.hpp"

namespace rex::viewer {

enum class SnapshotShapeKind {
  kSphere,
  kBox,
};

struct SnapshotBody {
  rex::platform::EntityId id{};
  SnapshotShapeKind shape{SnapshotShapeKind::kSphere};
  rex::math::Vec3 translation{};
  rex::math::Vec3 dimensions{};
};

struct SnapshotContact {
  rex::platform::EntityId body_a{};
  rex::platform::EntityId body_b{};
  rex::math::Vec3 position{};
  rex::math::Vec3 normal{};
  double penetration{0.0};
};

struct FrameSnapshot {
  std::uint64_t frame_index{0};
  double sim_time{0.0};
  rex::sim::StepTrace trace{};
  std::vector<SnapshotBody> bodies{};
  std::vector<SnapshotContact> contacts{};
};

class ReplayLog {
 public:
  void add_frame(FrameSnapshot frame);

  [[nodiscard]] auto frames() const noexcept -> const std::vector<FrameSnapshot>&;
  [[nodiscard]] auto empty() const noexcept -> bool;
  [[nodiscard]] auto size() const noexcept -> std::size_t;

  void save(const std::filesystem::path& path) const;
  [[nodiscard]] static auto load(const std::filesystem::path& path) -> ReplayLog;

 private:
  std::vector<FrameSnapshot> frames_{};
};

[[nodiscard]] auto capture_frame(
  const rex::dynamics::WorldState& world,
  const rex::sim::StepTrace& trace,
  std::uint64_t frame_index,
  double sim_time) -> FrameSnapshot;

}  // namespace rex::viewer

