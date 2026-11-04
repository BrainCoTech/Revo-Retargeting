// Copyright (c) 2025 BrainCo
// Setup tool: Revo3 Modbus port scanner (slave_id 126=left, 127=right).
// Built by revo3_driver CMake; used only by setup/bootstrap_revo3.sh --auto.

#include <stark-sdk.h>

#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <regex>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{

constexpr uint8_t kLeftSlaveId = 126;
constexpr uint8_t kRightSlaveId = 127;

// ── scan_serial_ports ──────────────────────────────────────────────────────────
// Discover all hardware serial ports by scanning /dev/ for character devices
// whose name matches tty[A-Z]* (ttyUSB*, ttyACM*, ttyCH343USB*, ttyS*, etc.).
// Excludes virtual consoles (tty0-tty63) and pseudo-terminals (pty*).
std::vector<std::string> scan_serial_ports()
{
  std::vector<std::string> ports;
  const std::regex serial_re(R"(^ttyCH343)");  // Revo3 uses CH343 USB-serial chips; skip other tty devices to avoid blocking
  std::error_code ec;

  for (const auto & entry : std::filesystem::directory_iterator("/dev", ec)) {
    if (ec) { break; }
    if (!entry.is_character_file(ec)) { continue; }
    const std::string name = entry.path().filename().string();
    if (std::regex_search(name, serial_re)) {
      ports.push_back(entry.path().string());
    }
  }

  // Sort for deterministic order
  std::sort(ports.begin(), ports.end());
  return ports;
}

bool probe_port(const std::string & port, uint8_t & slave_id_out)
{
  // Revo3 uses auto_detect_modbus_revo3 (Stark SDK V3 protocol).
  // Note: revo3 auto-detect does not support a 'quick' flag
  // (passes only port — matches modbus_session.cpp).
  CDeviceConfig * cfg = auto_detect_modbus_revo3(port.c_str());
  if (!cfg || !cfg->port_name) {
    if (cfg) {
      free_device_config(cfg);
    }
    return false;
  }

  const std::string detected_port = cfg->port_name;
  slave_id_out = cfg->slave_id;
  const uint32_t baud = cfg->baudrate;
  free_device_config(cfg);

  if (detected_port != port) {
    return false;
  }

  DeviceHandler * handle = modbus_open(port.c_str(), baud);
  if (!handle) {
    return false;
  }
  modbus_close(handle);
  return true;
}

}  // namespace

int main()
{
  init_logging(LOG_LEVEL_ERROR);
  stark_set_v3_protocol(true);  // Revo3 uses V3 New protocol

  std::unordered_map<uint8_t, std::string> by_slave;
  const auto ports = scan_serial_ports();

  if (ports.empty()) {
    std::fprintf(stderr, "[ERROR] no serial ports found in /dev/ (ttyUSB*, ttyACM*, ttyCH343USB*, etc.)\n");
    return 1;
  }

  std::printf("[INFO] scanning %zu serial port(s) for Revo3 Modbus...\n", ports.size());

  for (const auto & port : ports) {
    uint8_t slave_id = 0;
    if (!probe_port(port, slave_id)) {
      continue;
    }
    std::printf("[OK] port=%s slave_id=%u\n", port.c_str(), slave_id);

    const auto existing = by_slave.find(slave_id);
    if (existing != by_slave.end() && existing->second != port) {
      std::fprintf(
        stderr, "[WARN] duplicate slave_id %u on %s and %s\n", slave_id, existing->second.c_str(),
        port.c_str());
    }
    by_slave[slave_id] = port;
  }

  const auto left_it = by_slave.find(kLeftSlaveId);
  const auto right_it = by_slave.find(kRightSlaveId);

  if (left_it != by_slave.end()) {
    std::printf("REVO3_LEFT_PORT=%s\n", left_it->second.c_str());
    std::printf("REVO3_LEFT_SLAVE=%u\n", kLeftSlaveId);
  }
  if (right_it != by_slave.end()) {
    std::printf("REVO3_RIGHT_PORT=%s\n", right_it->second.c_str());
    std::printf("REVO3_RIGHT_SLAVE=%u\n", kRightSlaveId);
  }

  if (left_it == by_slave.end() || right_it == by_slave.end()) {
    std::fprintf(
      stderr,
      "[ERROR] need both slave_id %u (left) and %u (right). Found %zu device(s).\n",
      kLeftSlaveId, kRightSlaveId, by_slave.size());
    for (const auto & entry : by_slave) {
      std::fprintf(stderr, "        slave_id=%u port=%s\n", entry.first, entry.second.c_str());
    }
    return 1;
  }

  return 0;
}
