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

  // CPU-renders the state into an RgbaFrame (used for PBM export). Window
  // events are pumped separately via window_should_close().
  RgbaFrame render(const RenderState &state);

  // Presents an already-rendered frame to the Vulkan swapchain window. No-op
  // when the Vulkan present path is unavailable (headless / setup failure).
  void present(const RgbaFrame &frame);

  // Pumps window events and reports whether the user asked to close the window.
  // Returns false when there is no live window.
  bool window_should_close();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace radar
