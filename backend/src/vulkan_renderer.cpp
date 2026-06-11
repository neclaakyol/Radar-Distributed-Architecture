#include "radar/vulkan_renderer.hpp"

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <stdexcept>
#include <vector>

namespace radar {

struct VulkanRenderer::Impl {
  explicit Impl(std::uint32_t width, std::uint32_t height)
      : software(width, height) {
    if (glfwInit() != GLFW_TRUE) {
      status_text = "GLFW initialization failed";
      return;
    }

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    window = glfwCreateWindow(static_cast<int>(width), static_cast<int>(height),
                              "Radar PPI", nullptr, nullptr);
    if (window == nullptr) {
      status_text = "GLFW window creation failed";
      glfwTerminate();
      return;
    }

    if (glfwVulkanSupported() != GLFW_TRUE) {
      status_text = "Vulkan is not supported by GLFW on this system";
      return;
    }

    std::uint32_t extension_count = 0;
    const char **extensions = glfwGetRequiredInstanceExtensions(&extension_count);
    if (extensions == nullptr || extension_count == 0) {
      status_text = "GLFW did not provide Vulkan instance extensions";
      return;
    }

    VkApplicationInfo app_info{};
    app_info.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    app_info.pApplicationName = "Radar PPI";
    app_info.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
    app_info.pEngineName = "RadarBackend";
    app_info.engineVersion = VK_MAKE_VERSION(1, 0, 0);
    app_info.apiVersion = VK_API_VERSION_1_0;

    VkInstanceCreateInfo create_info{};
    create_info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    create_info.pApplicationInfo = &app_info;
    create_info.enabledExtensionCount = extension_count;
    create_info.ppEnabledExtensionNames = extensions;

    if (vkCreateInstance(&create_info, nullptr, &instance) != VK_SUCCESS) {
      status_text = "Vulkan instance creation failed";
      return;
    }

    available_flag = true;
    status_text = "Vulkan window initialized; software framebuffer readback active";
  }

  ~Impl() {
    if (instance != VK_NULL_HANDLE) {
      vkDestroyInstance(instance, nullptr);
      instance = VK_NULL_HANDLE;
    }
    if (window != nullptr) {
      glfwDestroyWindow(window);
      window = nullptr;
    }
    glfwTerminate();
  }

  SoftwareRenderer software;
  GLFWwindow *window = nullptr;
  VkInstance instance = VK_NULL_HANDLE;
  bool available_flag = false;
  std::string status_text = "not initialized";
};

VulkanRenderer::VulkanRenderer(std::uint32_t width, std::uint32_t height)
    : impl_(std::make_unique<Impl>(width, height)) {}

VulkanRenderer::~VulkanRenderer() = default;

bool VulkanRenderer::available() const { return impl_ && impl_->available_flag; }

std::string VulkanRenderer::status() const {
  return impl_ ? impl_->status_text : "not initialized";
}

RgbaFrame VulkanRenderer::render(const RenderState &state) {
  if (impl_ && impl_->window != nullptr) {
    glfwPollEvents();
  }
  return impl_->software.render(state);
}

} // namespace radar
