#include "radar/io.hpp"
#include "radar/mti.hpp"
#include "radar/protocol.hpp"
#include "radar/raster.hpp"
#include "radar/render.hpp"
#include "radar/sweep.hpp"

#ifndef RADAR_HAS_VULKAN
#define RADAR_HAS_VULKAN 0
#endif

#if RADAR_HAS_VULKAN
#include "radar/vulkan_renderer.hpp"
#endif

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdlib>
#include <csignal>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

std::atomic_bool g_running{true};

void handle_signal(int) { g_running = false; }

struct Config {
  std::optional<std::string> serial_port;
  std::optional<std::filesystem::path> replay_file;
  int baud_rate = 115200;
  double tau_mm = 80.0;
  std::filesystem::path log_dir = "logs";
  std::uint8_t threshold = 128;
  bool no_render = false;
};

void print_usage(const char *argv0) {
  std::cerr << "Usage:\n"
            << "  " << argv0 << " --serial COM3 --baud 115200\n"
            << "  " << argv0 << " --serial /dev/ttyACM0 --baud 115200\n"
            << "  " << argv0 << " --replay frames.bin [--no-render]\n\n"
            << "Options:\n"
            << "  --tau-mm N       MTI match threshold, default 80\n"
            << "  --log-dir PATH   PBM output directory, default logs\n"
            << "  --threshold N    PBM luminance threshold, default 128\n"
            << "  --no-render      Disable framebuffer/PBM generation\n";
}

Config parse_args(int argc, char **argv) {
  Config config;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto need_value = [&](const char *name) -> std::string {
      if (i + 1 >= argc) {
        throw std::invalid_argument(std::string("missing value for ") + name);
      }
      return argv[++i];
    };

    if (arg == "--serial") {
      config.serial_port = need_value("--serial");
    } else if (arg == "--replay") {
      config.replay_file = need_value("--replay");
    } else if (arg == "--baud") {
      config.baud_rate = std::stoi(need_value("--baud"));
    } else if (arg == "--tau-mm") {
      config.tau_mm = std::stod(need_value("--tau-mm"));
    } else if (arg == "--log-dir") {
      config.log_dir = need_value("--log-dir");
    } else if (arg == "--threshold") {
      const int threshold = std::stoi(need_value("--threshold"));
      if (threshold < 0 || threshold > 255) {
        throw std::invalid_argument("--threshold must be in 0..255");
      }
      config.threshold = static_cast<std::uint8_t>(threshold);
    } else if (arg == "--no-render") {
      config.no_render = true;
    } else if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]);
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown argument: " + arg);
    }
  }

  if (config.serial_port.has_value() == config.replay_file.has_value()) {
    throw std::invalid_argument("choose exactly one input: --serial or --replay");
  }
  return config;
}

struct SharedIngestState {
  std::mutex mutex;
  std::condition_variable cv;
  std::deque<radar::TelemetryPoint> points;
  radar::ProtocolCounters counters;
  bool done = false;
  std::string error;
};

struct JoinThreadOnExit {
  std::thread &thread;

  ~JoinThreadOnExit() {
    g_running = false;
    if (thread.joinable()) {
      thread.join();
    }
  }
};

std::thread start_ingest_thread(radar::ByteSource &source,
                                SharedIngestState &shared) {
  return std::thread([&source, &shared]() {
    radar::FrameParser parser;
    try {
      while (g_running) {
        std::optional<std::uint8_t> byte = source.read_byte();
        if (!byte) {
          if (source.eof()) {
            break;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
          continue;
        }

        auto points = parser.ingest(*byte);
        {
          std::lock_guard<std::mutex> lock(shared.mutex);
          shared.counters = parser.counters();
          for (const auto &p : points) {
            shared.points.push_back(p);
          }
        }
        shared.cv.notify_one();
      }
    } catch (const std::exception &ex) {
      std::lock_guard<std::mutex> lock(shared.mutex);
      shared.error = ex.what();
    }

    {
      std::lock_guard<std::mutex> lock(shared.mutex);
      shared.counters = parser.counters();
      shared.done = true;
    }
    shared.cv.notify_one();
  });
}

std::filesystem::path pbm_path(const std::filesystem::path &log_dir,
                               std::uint64_t cycle_index) {
  std::ostringstream name;
  name << "radar_" << std::setw(6) << std::setfill('0') << cycle_index
       << ".pbm";
  return log_dir / name.str();
}

std::vector<radar::RadarPoint>
combined_history(const std::optional<radar::Sweep> &forward,
                 const std::optional<radar::Sweep> &ret) {
  std::vector<radar::RadarPoint> history;
  if (forward) {
    history.insert(history.end(), forward->points.begin(), forward->points.end());
  }
  if (ret) {
    history.insert(history.end(), ret->points.begin(), ret->points.end());
  }
  constexpr std::size_t max_points = 720;
  if (history.size() > max_points) {
    history.erase(history.begin(),
                  history.begin() + static_cast<std::ptrdiff_t>(history.size() - max_points));
  }
  return history;
}

} // namespace

int main(int argc, char **argv) {
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  Config config;
  try {
    config = parse_args(argc, argv);
  } catch (const std::exception &ex) {
    std::cerr << "Argument error: " << ex.what() << "\n\n";
    print_usage(argv[0]);
    return 2;
  }

  try {
    std::unique_ptr<radar::ByteSource> source;
    if (config.replay_file) {
      source = std::make_unique<radar::ReplayByteSource>(*config.replay_file);
    } else {
      source =
          std::make_unique<radar::SerialByteSource>(*config.serial_port,
                                                    config.baud_rate);
    }

    const std::string source_name = source->description();
    std::cout << "Input: " << source_name << "\n"
              << "Baud: " << config.baud_rate << "\n"
              << "MTI tau: " << config.tau_mm << " mm\n";

    radar::SoftwareRenderer software_renderer(800, 480);

#if RADAR_HAS_VULKAN
    std::unique_ptr<radar::VulkanRenderer> vulkan_renderer;
    if (!config.no_render) {
      vulkan_renderer = std::make_unique<radar::VulkanRenderer>(800, 480);
      std::cout << "Renderer: " << vulkan_renderer->status() << "\n";
    }
#else
    if (!config.no_render) {
      std::cout << "Renderer: software framebuffer (Vulkan dependencies not found at build time)\n";
    }
#endif

    SharedIngestState shared;
    std::thread ingest_thread = start_ingest_thread(*source, shared);
    JoinThreadOnExit join_ingest{ingest_thread};

    radar::SweepBuilder sweeps;
    radar::GpuMti mti;
    radar::RenderState render_state;
    render_state.source_name = source_name;

    std::uint64_t completed_sweeps = 0;
    std::uint64_t completed_cycles = 0;
    std::vector<radar::MotionVector> latest_vectors;

    while (g_running) {
      std::deque<radar::TelemetryPoint> batch;
      radar::ProtocolCounters protocol_counters;
      bool done = false;
      std::string error;

      {
        std::unique_lock<std::mutex> lock(shared.mutex);
        shared.cv.wait_for(lock, std::chrono::milliseconds(16), [&] {
          return !shared.points.empty() || shared.done || !g_running.load();
        });
        batch.swap(shared.points);
        protocol_counters = shared.counters;
        done = shared.done;
        error = shared.error;
      }

      if (!error.empty()) {
        throw std::runtime_error(error);
      }

      for (const radar::TelemetryPoint &telemetry : batch) {
        render_state.stats.protocol = protocol_counters;
        render_state.sweep_angle_deg = telemetry.angle_deg;

        if (telemetry.distance_mm == 0) {
          continue;
        }

        std::optional<radar::CompletedSweepEvent> event =
            sweeps.ingest(telemetry);
        if (const auto &current = sweeps.current_sweep()) {
          render_state.current_points = current->points;
        }
        render_state.history_points =
            combined_history(sweeps.last_forward_sweep(),
                             sweeps.last_return_sweep());

        if (!event) {
          continue;
        }

        ++completed_sweeps;
        render_state.stats.completed_sweeps = completed_sweeps;

        if (event->previous_opposite) {
          latest_vectors =
              mti.compute(*event->previous_opposite, event->completed,
                          config.tau_mm);
          render_state.vectors = latest_vectors;
          render_state.stats.vector_count = latest_vectors.size();
        }

        if (!event->completed_bidirectional_cycle) {
          std::cout << "Sweep " << event->completed.id << " complete ("
                    << radar::to_string(event->completed.direction) << ", "
                    << event->completed.points.size() << " points)\n";
          continue;
        }

        completed_cycles = event->cycle_index;
        render_state.stats.completed_cycles = completed_cycles;
        render_state.current_points = event->completed.points;
        render_state.history_points =
            combined_history(sweeps.last_forward_sweep(),
                             sweeps.last_return_sweep());

        std::cout << "Cycle " << completed_cycles << " complete: "
                  << latest_vectors.size() << " MTI vectors, "
                  << protocol_counters.valid_frames << " valid frames\n";

        if (!config.no_render) {
#if RADAR_HAS_VULKAN
          radar::RgbaFrame frame =
              vulkan_renderer && vulkan_renderer->available()
                  ? vulkan_renderer->render(render_state)
                  : software_renderer.render(render_state);
#else
          radar::RgbaFrame frame = software_renderer.render(render_state);
#endif
          const std::filesystem::path output =
              pbm_path(config.log_dir, completed_cycles);
          radar::write_pbm(output, frame, config.threshold);
          std::cout << "Wrote " << output.string() << "\n";
        }
      }

      render_state.stats.protocol = protocol_counters;

      if (done && batch.empty()) {
        break;
      }
    }

    g_running = false;

    std::cout << "Done. Valid frames: "
              << render_state.stats.protocol.valid_frames
              << ", CRC drops: " << render_state.stats.protocol.crc_drops
              << ", sync drops: " << render_state.stats.protocol.sync_drops
              << ", timeouts: "
              << render_state.stats.protocol.timeout_readings << "\n";
    return 0;
  } catch (const std::exception &ex) {
    g_running = false;
    std::cerr << "Fatal error: " << ex.what() << "\n";
    return 1;
  }
}
