#pragma once

#include <compare>
#include <cstdint>
#include <limits>

namespace rex::platform {

using EntityIndex = std::uint32_t;
using Generation = std::uint32_t;

struct EntityId {
  static constexpr EntityIndex kInvalidIndex = std::numeric_limits<EntityIndex>::max();

  EntityIndex index{kInvalidIndex};
  Generation generation{0};

  [[nodiscard]] constexpr bool valid() const noexcept {
    return index != kInvalidIndex;
  }

  [[nodiscard]] static constexpr auto invalid() noexcept -> EntityId {
    return {};
  }

  auto operator<=>(const EntityId&) const = default;
};

}  // namespace rex::platform

