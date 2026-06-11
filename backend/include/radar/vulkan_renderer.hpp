#pragma once

#include "radar/render.hpp"

#include <memory>
#include <string>

namespace radar {

class VulkanRenderer {
public:
  VulkanRenderer(std::uint32_t width = 800, std::uint32_t height = 480);
  ~VulkanRenderer();

  bool available() const;
  std::string status() const;
  RgbaFrame render(const RenderState &state);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace radar
