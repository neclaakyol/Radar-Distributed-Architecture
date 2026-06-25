#include "radar/corroboration.hpp"
#include "radar/geometry.hpp"
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

#include <array>
#include <atomic>
#include <chrono>
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

  // Sensor mount geometry and corroboration tuning. Defaults are
  // placeholders (see geometry.hpp) pending physical measurement of the
  // 3-sensor rig; override via CLI once the rig is built and measured.
  radar::SensorGeometry geometry;
  radar::CorroborationConfig corroboration;
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
            << "  --no-render      Disable framebuffer/PBM generation\n\n"
            << "Sensor mount geometry (placeholders -- see geometry.hpp):\n"
            << "  --left-offset-x-mm N, --left-offset-y-mm N, --left-heading-deg N\n"
            << "  --right-offset-x-mm N, --right-offset-y-mm N, --right-heading-deg N\n\n"
            << "Cross-sensor corroboration tuning:\n"
            << "  --corroboration-band-main-deg N   default 15\n"
            << "  --corroboration-band-flank-deg N  default 30\n"
            << "  --corroboration-tolerance-mm N    default 100\n"
            << "  --corroboration-window-ms N       default 500\n"
            << "  --max-range-mm N                  sanity bound, default 4500\n";
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
    } else if (arg == "--left-offset-x-mm") {
      config.geometry.mounts[1].offset_x_mm = std::stod(need_value("--left-offset-x-mm"));
    } else if (arg == "--left-offset-y-mm") {
      config.geometry.mounts[1].offset_y_mm = std::stod(need_value("--left-offset-y-mm"));
    } else if (arg == "--left-heading-deg") {
      config.geometry.mounts[1].heading_offset_deg = std::stod(need_value("--left-heading-deg"));
    } else if (arg == "--right-offset-x-mm") {
      config.geometry.mounts[2].offset_x_mm = std::stod(need_value("--right-offset-x-mm"));
    } else if (arg == "--right-offset-y-mm") {
      config.geometry.mounts[2].offset_y_mm = std::stod(need_value("--right-offset-y-mm"));
    } else if (arg == "--right-heading-deg") {
      config.geometry.mounts[2].heading_offset_deg = std::stod(need_value("--right-heading-deg"));
    } else if (arg == "--corroboration-band-main-deg") {
      config.corroboration.main_boundary_band_deg =
          std::stod(need_value("--corroboration-band-main-deg"));
    } else if (arg == "--corroboration-band-flank-deg") {
      config.corroboration.flank_inward_band_deg =
          std::stod(need_value("--corroboration-band-flank-deg"));
    } else if (arg == "--corroboration-tolerance-mm") {
      config.corroboration.match_distance_tolerance_mm =
          std::stod(need_value("--corroboration-tolerance-mm"));
    } else if (arg == "--corroboration-window-ms") {
      config.corroboration.match_time_window =
          std::chrono::milliseconds(std::stoi(need_value("--corroboration-window-ms")));
    } else if (arg == "--max-range-mm") {
      config.corroboration.max_valid_distance_mm = std::stod(need_value("--max-range-mm"));
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

    std::array<radar::SweepBuilder, radar::kNumSensors> sweep_builders;
    radar::Corroborator corroborator(config.geometry, config.corroboration);
    radar::GpuMti mti;
    radar::RenderState render_state;
    render_state.source_name = source_name;
    render_state.geometry = config.geometry;

    std::uint64_t completed_sweeps = 0;
    std::uint64_t completed_cycles = 0;
    std::array<std::vector<radar::MotionVector>, radar::kNumSensors> latest_vectors_by_sensor;

    auto rebuild_combined_render_points = [&]() {
      render_state.current_points.clear();
      render_state.history_points.clear();
      for (const radar::SweepBuilder &builder : sweep_builders) {
        if (const auto &current = builder.current_sweep()) {
          render_state.current_points.insert(render_state.current_points.end(),
                                             current->points.begin(),
                                             current->points.end());
        }
        std::vector<radar::RadarPoint> hist =
            combined_history(builder.last_forward_sweep(), builder.last_return_sweep());
        render_state.history_points.insert(render_state.history_points.end(),
                                           hist.begin(), hist.end());
      }
    };

    auto rebuild_vectors = [&]() {
      render_state.vectors.clear();
      for (const std::vector<radar::MotionVector> &vectors : latest_vectors_by_sensor) {
        render_state.vectors.insert(render_state.vectors.end(), vectors.begin(),
                                    vectors.end());
      }
      render_state.stats.vector_count = render_state.vectors.size();
    };

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

        if (telemetry.sensor_id >= radar::kNumSensors || telemetry.distance_mm == 0) {
          continue;
        }

        render_state.sweep_angle_by_sensor[telemetry.sensor_id] = telemetry.angle_deg;

        radar::RadarPoint built_point;
        built_point.angle_deg = telemetry.angle_deg;
        built_point.distance_mm = telemetry.distance_mm;
        built_point.timestamp = radar::RadarClock::now();
        built_point.sequence = telemetry.sequence;
        built_point.sensor_id = telemetry.sensor_id;

        // Cross-sensor corroboration ("double check") runs before the point
        // enters its sweep builder, so the confirmation state it sets is
        // baked into the RadarPoint that ends up stored in the Sweep.
        const radar::WorldPoint world = corroborator.update(built_point);

        radar::SweepBuilder &builder = sweep_builders[telemetry.sensor_id];
        std::optional<radar::CompletedSweepEvent> event = builder.ingest(world.source);
        rebuild_combined_render_points();

        if (!event) {
          continue;
        }

        ++completed_sweeps;
        render_state.stats.completed_sweeps = completed_sweeps;

        if (event->previous_opposite) {
          latest_vectors_by_sensor[telemetry.sensor_id] =
              mti.compute(*event->previous_opposite, event->completed, config.tau_mm);
          rebuild_vectors();
        }

        if (!event->completed_bidirectional_cycle) {
          std::cout << "Sensor " << static_cast<int>(telemetry.sensor_id) << " sweep "
                    << event->completed.id << " complete ("
                    << radar::to_string(event->completed.direction) << ", "
                    << event->completed.points.size() << " points)\n";
          continue;
        }

        // A combined frame renders whenever ANY one sensor completes a
        // bidirectional cycle (their cadences are independent -- main's
        // 0-120 deg sweep and the flanks' 0-180 deg sweeps don't finish in
        // lockstep), using the freshest state from all 3 builders.
        ++completed_cycles;
        render_state.stats.completed_cycles = completed_cycles;
        rebuild_combined_render_points();

        std::cout << "Cycle " << completed_cycles << " complete (sensor "
                  << static_cast<int>(telemetry.sensor_id) << "): "
                  << render_state.vectors.size() << " MTI vectors, "
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
