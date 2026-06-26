#include "radar/vulkan_renderer.hpp"

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <limits>
#include <vector>

namespace radar {
namespace {

// Picks a swapchain surface format, preferring a 8-bit BGRA/RGBA UNORM so the
// host-side staging copy is a plain channel reorder with no conversion.
VkSurfaceFormatKHR choose_surface_format(const std::vector<VkSurfaceFormatKHR> &formats) {
  for (const VkSurfaceFormatKHR &f : formats) {
    if ((f.format == VK_FORMAT_B8G8R8A8_UNORM || f.format == VK_FORMAT_R8G8B8A8_UNORM) &&
        f.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
      return f;
    }
  }
  return formats.front();
}

} // namespace

struct VulkanRenderer::Impl {
  explicit Impl(std::uint32_t width, std::uint32_t height)
      : software(width, height), frame_width(width), frame_height(height) {
    if (glfwInit() != GLFW_TRUE) {
      status_text = "GLFW initialization failed";
      return;
    }
    glfw_ready = true;

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
    window = glfwCreateWindow(static_cast<int>(width), static_cast<int>(height),
                              "Radar PPI", nullptr, nullptr);
    if (window == nullptr) {
      status_text = "GLFW window creation failed (headless?)";
      return;
    }

    if (glfwVulkanSupported() != GLFW_TRUE) {
      status_text = "Vulkan is not supported by GLFW on this system";
      return;
    }

    if (!create_instance() || !create_surface() || !pick_device() ||
        !create_swapchain() || !create_commands() || !create_sync()) {
      // status_text already set by the failing step; software fallback remains.
      return;
    }

    available_flag = true;
    status_text = "Vulkan swapchain live present active";
  }

  ~Impl() {
    if (device != VK_NULL_HANDLE) {
      vkDeviceWaitIdle(device);
    }
    destroy_sync();
    destroy_commands();
    destroy_swapchain();
    if (device != VK_NULL_HANDLE) {
      vkDestroyDevice(device, nullptr);
      device = VK_NULL_HANDLE;
    }
    if (surface != VK_NULL_HANDLE) {
      vkDestroySurfaceKHR(instance, surface, nullptr);
      surface = VK_NULL_HANDLE;
    }
    if (instance != VK_NULL_HANDLE) {
      vkDestroyInstance(instance, nullptr);
      instance = VK_NULL_HANDLE;
    }
    if (window != nullptr) {
      glfwDestroyWindow(window);
      window = nullptr;
    }
    if (glfw_ready) {
      glfwTerminate();
    }
  }

  bool fail(const char *message) {
    status_text = message;
    return false;
  }

  bool create_instance() {
    std::uint32_t extension_count = 0;
    const char **extensions = glfwGetRequiredInstanceExtensions(&extension_count);
    if (extensions == nullptr || extension_count == 0) {
      return fail("GLFW did not provide Vulkan instance extensions");
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
      return fail("Vulkan instance creation failed");
    }
    return true;
  }

  bool create_surface() {
    if (glfwCreateWindowSurface(instance, window, nullptr, &surface) != VK_SUCCESS) {
      return fail("window surface creation failed");
    }
    return true;
  }

  bool device_has_swapchain_ext(VkPhysicalDevice dev) const {
    std::uint32_t count = 0;
    vkEnumerateDeviceExtensionProperties(dev, nullptr, &count, nullptr);
    std::vector<VkExtensionProperties> props(count);
    vkEnumerateDeviceExtensionProperties(dev, nullptr, &count, props.data());
    for (const VkExtensionProperties &p : props) {
      if (std::strcmp(p.extensionName, VK_KHR_SWAPCHAIN_EXTENSION_NAME) == 0) {
        return true;
      }
    }
    return false;
  }

  // Selects a physical device + a single queue family that supports both
  // graphics and presentation to our surface (the common case on one GPU).
  bool pick_device() {
    std::uint32_t device_count = 0;
    vkEnumeratePhysicalDevices(instance, &device_count, nullptr);
    if (device_count == 0) {
      return fail("no Vulkan physical devices found");
    }
    std::vector<VkPhysicalDevice> devices(device_count);
    vkEnumeratePhysicalDevices(instance, &device_count, devices.data());

    for (VkPhysicalDevice dev : devices) {
      if (!device_has_swapchain_ext(dev)) {
        continue;
      }
      std::uint32_t family_count = 0;
      vkGetPhysicalDeviceQueueFamilyProperties(dev, &family_count, nullptr);
      std::vector<VkQueueFamilyProperties> families(family_count);
      vkGetPhysicalDeviceQueueFamilyProperties(dev, &family_count, families.data());

      for (std::uint32_t i = 0; i < family_count; ++i) {
        const bool graphics = (families[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) != 0;
        VkBool32 present = VK_FALSE;
        vkGetPhysicalDeviceSurfaceSupportKHR(dev, i, surface, &present);
        if (graphics && present == VK_TRUE) {
          physical_device = dev;
          queue_family = i;
          break;
        }
      }
      if (physical_device != VK_NULL_HANDLE) {
        break;
      }
    }

    if (physical_device == VK_NULL_HANDLE) {
      return fail("no graphics+present capable Vulkan device");
    }

    vkGetPhysicalDeviceMemoryProperties(physical_device, &memory_properties);

    const float priority = 1.0f;
    VkDeviceQueueCreateInfo queue_info{};
    queue_info.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
    queue_info.queueFamilyIndex = queue_family;
    queue_info.queueCount = 1;
    queue_info.pQueuePriorities = &priority;

    const char *device_extensions[] = {VK_KHR_SWAPCHAIN_EXTENSION_NAME};
    VkDeviceCreateInfo device_info{};
    device_info.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    device_info.queueCreateInfoCount = 1;
    device_info.pQueueCreateInfos = &queue_info;
    device_info.enabledExtensionCount = 1;
    device_info.ppEnabledExtensionNames = device_extensions;

    if (vkCreateDevice(physical_device, &device_info, nullptr, &device) != VK_SUCCESS) {
      return fail("logical device creation failed");
    }
    vkGetDeviceQueue(device, queue_family, 0, &queue);
    return true;
  }

  std::uint32_t find_memory_type(std::uint32_t type_filter, VkMemoryPropertyFlags props) const {
    for (std::uint32_t i = 0; i < memory_properties.memoryTypeCount; ++i) {
      if ((type_filter & (1U << i)) != 0 &&
          (memory_properties.memoryTypes[i].propertyFlags & props) == props) {
        return i;
      }
    }
    return std::numeric_limits<std::uint32_t>::max();
  }

  bool create_swapchain() {
    VkSurfaceCapabilitiesKHR caps{};
    vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physical_device, surface, &caps);

    if ((caps.supportedUsageFlags & VK_IMAGE_USAGE_TRANSFER_DST_BIT) == 0) {
      return fail("swapchain images do not support TRANSFER_DST");
    }

    std::uint32_t format_count = 0;
    vkGetPhysicalDeviceSurfaceFormatsKHR(physical_device, surface, &format_count, nullptr);
    if (format_count == 0) {
      return fail("no surface formats");
    }
    std::vector<VkSurfaceFormatKHR> formats(format_count);
    vkGetPhysicalDeviceSurfaceFormatsKHR(physical_device, surface, &format_count, formats.data());
    const VkSurfaceFormatKHR surface_format = choose_surface_format(formats);
    swapchain_format = surface_format.format;
    swapchain_is_bgra = (surface_format.format == VK_FORMAT_B8G8R8A8_UNORM);

    // Resolve the extent. currentExtent == 0xFFFFFFFF means "pick your own".
    if (caps.currentExtent.width != std::numeric_limits<std::uint32_t>::max()) {
      swapchain_extent = caps.currentExtent;
    } else {
      int fb_w = 0;
      int fb_h = 0;
      glfwGetFramebufferSize(window, &fb_w, &fb_h);
      swapchain_extent.width = std::clamp(static_cast<std::uint32_t>(fb_w),
                                          caps.minImageExtent.width, caps.maxImageExtent.width);
      swapchain_extent.height = std::clamp(static_cast<std::uint32_t>(fb_h),
                                           caps.minImageExtent.height, caps.maxImageExtent.height);
    }
    if (swapchain_extent.width == 0 || swapchain_extent.height == 0) {
      return fail("zero swapchain extent (window minimized)");
    }

    std::uint32_t image_count = caps.minImageCount + 1;
    if (caps.maxImageCount > 0 && image_count > caps.maxImageCount) {
      image_count = caps.maxImageCount;
    }

    VkSwapchainCreateInfoKHR info{};
    info.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
    info.surface = surface;
    info.minImageCount = image_count;
    info.imageFormat = swapchain_format;
    info.imageColorSpace = surface_format.colorSpace;
    info.imageExtent = swapchain_extent;
    info.imageArrayLayers = 1;
    info.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
    info.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    info.preTransform = caps.currentTransform;
    info.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    info.presentMode = VK_PRESENT_MODE_FIFO_KHR; // always supported, vsync
    info.clipped = VK_TRUE;
    info.oldSwapchain = VK_NULL_HANDLE;

    if (vkCreateSwapchainKHR(device, &info, nullptr, &swapchain) != VK_SUCCESS) {
      return fail("swapchain creation failed");
    }

    std::uint32_t actual = 0;
    vkGetSwapchainImagesKHR(device, swapchain, &actual, nullptr);
    swapchain_images.resize(actual);
    vkGetSwapchainImagesKHR(device, swapchain, &actual, swapchain_images.data());

    return create_staging();
  }

  bool create_staging() {
    const VkDeviceSize size =
        static_cast<VkDeviceSize>(swapchain_extent.width) * swapchain_extent.height * 4U;

    VkBufferCreateInfo buffer_info{};
    buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    buffer_info.size = size;
    buffer_info.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
    buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    if (vkCreateBuffer(device, &buffer_info, nullptr, &staging_buffer) != VK_SUCCESS) {
      return fail("staging buffer creation failed");
    }

    VkMemoryRequirements reqs{};
    vkGetBufferMemoryRequirements(device, staging_buffer, &reqs);
    const std::uint32_t type = find_memory_type(
        reqs.memoryTypeBits,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    if (type == std::numeric_limits<std::uint32_t>::max()) {
      return fail("no host-visible memory type for staging buffer");
    }

    VkMemoryAllocateInfo alloc{};
    alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    alloc.allocationSize = reqs.size;
    alloc.memoryTypeIndex = type;
    if (vkAllocateMemory(device, &alloc, nullptr, &staging_memory) != VK_SUCCESS) {
      return fail("staging memory allocation failed");
    }
    vkBindBufferMemory(device, staging_buffer, staging_memory, 0);
    if (vkMapMemory(device, staging_memory, 0, size, 0, &staging_mapped) != VK_SUCCESS) {
      return fail("staging memory map failed");
    }
    staging_size = size;
    return true;
  }

  bool create_commands() {
    VkCommandPoolCreateInfo pool_info{};
    pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    pool_info.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    pool_info.queueFamilyIndex = queue_family;
    if (vkCreateCommandPool(device, &pool_info, nullptr, &command_pool) != VK_SUCCESS) {
      return fail("command pool creation failed");
    }

    VkCommandBufferAllocateInfo alloc{};
    alloc.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    alloc.commandPool = command_pool;
    alloc.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    alloc.commandBufferCount = 1;
    if (vkAllocateCommandBuffers(device, &alloc, &command_buffer) != VK_SUCCESS) {
      return fail("command buffer allocation failed");
    }
    return true;
  }

  bool create_sync() {
    VkSemaphoreCreateInfo sem_info{};
    sem_info.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
    VkFenceCreateInfo fence_info{};
    fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    fence_info.flags = VK_FENCE_CREATE_SIGNALED_BIT;

    if (vkCreateSemaphore(device, &sem_info, nullptr, &image_available) != VK_SUCCESS ||
        vkCreateSemaphore(device, &sem_info, nullptr, &render_finished) != VK_SUCCESS ||
        vkCreateFence(device, &fence_info, nullptr, &in_flight) != VK_SUCCESS) {
      return fail("sync object creation failed");
    }
    return true;
  }

  void destroy_staging() {
    if (staging_memory != VK_NULL_HANDLE) {
      vkUnmapMemory(device, staging_memory);
    }
    if (staging_buffer != VK_NULL_HANDLE) {
      vkDestroyBuffer(device, staging_buffer, nullptr);
      staging_buffer = VK_NULL_HANDLE;
    }
    if (staging_memory != VK_NULL_HANDLE) {
      vkFreeMemory(device, staging_memory, nullptr);
      staging_memory = VK_NULL_HANDLE;
    }
    staging_mapped = nullptr;
    staging_size = 0;
  }

  void destroy_swapchain() {
    destroy_staging();
    swapchain_images.clear();
    if (swapchain != VK_NULL_HANDLE) {
      vkDestroySwapchainKHR(device, swapchain, nullptr);
      swapchain = VK_NULL_HANDLE;
    }
  }

  void destroy_commands() {
    if (command_pool != VK_NULL_HANDLE) {
      vkDestroyCommandPool(device, command_pool, nullptr);
      command_pool = VK_NULL_HANDLE;
    }
  }

  void destroy_sync() {
    if (image_available != VK_NULL_HANDLE) {
      vkDestroySemaphore(device, image_available, nullptr);
      image_available = VK_NULL_HANDLE;
    }
    if (render_finished != VK_NULL_HANDLE) {
      vkDestroySemaphore(device, render_finished, nullptr);
      render_finished = VK_NULL_HANDLE;
    }
    if (in_flight != VK_NULL_HANDLE) {
      vkDestroyFence(device, in_flight, nullptr);
      in_flight = VK_NULL_HANDLE;
    }
  }

  bool recreate_swapchain() {
    vkDeviceWaitIdle(device);
    destroy_swapchain();
    if (!create_swapchain()) {
      available_flag = false; // e.g. minimized; present() will keep no-op'ing
      return false;
    }
    available_flag = true;
    return true;
  }

  // Nearest-samples the source RgbaFrame into the mapped staging buffer in the
  // swapchain's channel order. Source and swapchain extents are usually equal
  // (fixed 800x480 window); sampling makes it robust if they ever differ.
  void fill_staging(const RgbaFrame &frame) {
    auto *dst = static_cast<std::uint8_t *>(staging_mapped);
    const std::uint32_t dw = swapchain_extent.width;
    const std::uint32_t dh = swapchain_extent.height;
    const std::uint32_t sw = frame.width;
    const std::uint32_t sh = frame.height;
    const bool have_src = sw > 0 && sh > 0 && frame.rgba.size() >= static_cast<std::size_t>(sw) * sh * 4U;

    for (std::uint32_t y = 0; y < dh; ++y) {
      const std::uint32_t sy = (have_src && dh > 0) ? (y * sh / dh) : 0;
      for (std::uint32_t x = 0; x < dw; ++x) {
        std::uint8_t r = 0, g = 0, b = 0, a = 255;
        if (have_src) {
          const std::uint32_t sx = (dw > 0) ? (x * sw / dw) : 0;
          const std::size_t s = (static_cast<std::size_t>(sy) * sw + sx) * 4U;
          r = frame.rgba[s + 0];
          g = frame.rgba[s + 1];
          b = frame.rgba[s + 2];
          a = frame.rgba[s + 3];
        }
        const std::size_t d = (static_cast<std::size_t>(y) * dw + x) * 4U;
        if (swapchain_is_bgra) {
          dst[d + 0] = b;
          dst[d + 1] = g;
          dst[d + 2] = r;
          dst[d + 3] = a;
        } else {
          dst[d + 0] = r;
          dst[d + 1] = g;
          dst[d + 2] = b;
          dst[d + 3] = a;
        }
      }
    }
  }

  void record_copy(VkImage target) {
    vkResetCommandBuffer(command_buffer, 0);

    VkCommandBufferBeginInfo begin{};
    begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(command_buffer, &begin);

    VkImageMemoryBarrier to_dst{};
    to_dst.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    to_dst.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    to_dst.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    to_dst.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    to_dst.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    to_dst.image = target;
    to_dst.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
    to_dst.srcAccessMask = 0;
    to_dst.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    vkCmdPipelineBarrier(command_buffer, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                         VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, 1, &to_dst);

    VkBufferImageCopy region{};
    region.bufferOffset = 0;
    region.bufferRowLength = 0;
    region.bufferImageHeight = 0;
    region.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
    region.imageOffset = {0, 0, 0};
    region.imageExtent = {swapchain_extent.width, swapchain_extent.height, 1};
    vkCmdCopyBufferToImage(command_buffer, staging_buffer, target,
                           VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);

    VkImageMemoryBarrier to_present{};
    to_present.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    to_present.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    to_present.newLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
    to_present.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    to_present.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    to_present.image = target;
    to_present.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
    to_present.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    to_present.dstAccessMask = 0;
    vkCmdPipelineBarrier(command_buffer, VK_PIPELINE_STAGE_TRANSFER_BIT,
                         VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, 0, 0, nullptr, 0, nullptr, 1,
                         &to_present);

    vkEndCommandBuffer(command_buffer);
  }

  void present(const RgbaFrame &frame) {
    if (!available_flag) {
      return;
    }

    vkWaitForFences(device, 1, &in_flight, VK_TRUE, std::numeric_limits<std::uint64_t>::max());

    std::uint32_t image_index = 0;
    VkResult acquire = vkAcquireNextImageKHR(device, swapchain,
                                             std::numeric_limits<std::uint64_t>::max(),
                                             image_available, VK_NULL_HANDLE, &image_index);
    if (acquire == VK_ERROR_OUT_OF_DATE_KHR) {
      recreate_swapchain();
      return;
    }
    if (acquire != VK_SUCCESS && acquire != VK_SUBOPTIMAL_KHR) {
      return;
    }

    vkResetFences(device, 1, &in_flight);

    fill_staging(frame);
    record_copy(swapchain_images[image_index]);

    const VkPipelineStageFlags wait_stage = VK_PIPELINE_STAGE_TRANSFER_BIT;
    VkSubmitInfo submit{};
    submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submit.waitSemaphoreCount = 1;
    submit.pWaitSemaphores = &image_available;
    submit.pWaitDstStageMask = &wait_stage;
    submit.commandBufferCount = 1;
    submit.pCommandBuffers = &command_buffer;
    submit.signalSemaphoreCount = 1;
    submit.pSignalSemaphores = &render_finished;
    vkQueueSubmit(queue, 1, &submit, in_flight);

    VkPresentInfoKHR present_info{};
    present_info.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
    present_info.waitSemaphoreCount = 1;
    present_info.pWaitSemaphores = &render_finished;
    present_info.swapchainCount = 1;
    present_info.pSwapchains = &swapchain;
    present_info.pImageIndices = &image_index;
    const VkResult present_result = vkQueuePresentKHR(queue, &present_info);
    if (present_result == VK_ERROR_OUT_OF_DATE_KHR || present_result == VK_SUBOPTIMAL_KHR) {
      recreate_swapchain();
    }
  }

  SoftwareRenderer software;
  std::uint32_t frame_width;
  std::uint32_t frame_height;

  bool glfw_ready = false;
  GLFWwindow *window = nullptr;
  VkInstance instance = VK_NULL_HANDLE;
  VkSurfaceKHR surface = VK_NULL_HANDLE;
  VkPhysicalDevice physical_device = VK_NULL_HANDLE;
  VkPhysicalDeviceMemoryProperties memory_properties{};
  std::uint32_t queue_family = 0;
  VkDevice device = VK_NULL_HANDLE;
  VkQueue queue = VK_NULL_HANDLE;

  VkSwapchainKHR swapchain = VK_NULL_HANDLE;
  std::vector<VkImage> swapchain_images;
  VkFormat swapchain_format = VK_FORMAT_B8G8R8A8_UNORM;
  bool swapchain_is_bgra = true;
  VkExtent2D swapchain_extent{0, 0};

  VkBuffer staging_buffer = VK_NULL_HANDLE;
  VkDeviceMemory staging_memory = VK_NULL_HANDLE;
  void *staging_mapped = nullptr;
  VkDeviceSize staging_size = 0;

  VkCommandPool command_pool = VK_NULL_HANDLE;
  VkCommandBuffer command_buffer = VK_NULL_HANDLE;
  VkSemaphore image_available = VK_NULL_HANDLE;
  VkSemaphore render_finished = VK_NULL_HANDLE;
  VkFence in_flight = VK_NULL_HANDLE;

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
  return impl_->software.render(state);
}

void VulkanRenderer::present(const RgbaFrame &frame) {
  if (impl_) {
    impl_->present(frame);
  }
}

bool VulkanRenderer::window_should_close() {
  if (!impl_ || impl_->window == nullptr) {
    return false;
  }
  glfwPollEvents();
  return glfwWindowShouldClose(impl_->window) == GLFW_TRUE;
}

} // namespace radar
