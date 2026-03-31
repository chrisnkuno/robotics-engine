#include "rex/gpu/backend.hpp"

namespace rex::gpu {

auto backend_name() -> std::string {
  return "cuda-placeholder";
}

}  // namespace rex::gpu

