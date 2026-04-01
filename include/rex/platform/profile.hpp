#pragma once

#include <chrono>

#if defined(REX_ENABLE_TRACY) && __has_include(<tracy/Tracy.hpp>)
#include <tracy/Tracy.hpp>
#define REX_PROFILE_SCOPE(name_literal) ZoneScopedN(name_literal)
#define REX_PROFILE_FRAME_MARK() FrameMark
#else
#define REX_PROFILE_SCOPE(name_literal) ((void)0)
#define REX_PROFILE_FRAME_MARK() ((void)0)
#endif

namespace rex::platform {

using ProfileClock = std::chrono::steady_clock;

class ScopedMilliseconds {
 public:
  explicit ScopedMilliseconds(double& target) : target_(&target), start_(ProfileClock::now()) {}

  ScopedMilliseconds(const ScopedMilliseconds&) = delete;
  auto operator=(const ScopedMilliseconds&) -> ScopedMilliseconds& = delete;

  ScopedMilliseconds(ScopedMilliseconds&&) = delete;
  auto operator=(ScopedMilliseconds&&) -> ScopedMilliseconds& = delete;

  ~ScopedMilliseconds() {
    *target_ +=
      std::chrono::duration<double, std::milli>(ProfileClock::now() - start_).count();
  }

 private:
  double* target_{nullptr};
  ProfileClock::time_point start_{};
};

}  // namespace rex::platform
