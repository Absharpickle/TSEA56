// robot_server.cpp
// TCP command/telemetry server for the Raspberry Pi communication module.
// Camera stream is handled separately via mjpg-streamer.
//
// Build:
//   sudo apt install nlohmann-json3-dev
//   g++ -std=c++17 -o robot_server robot_server.cpp -lpthread
//
// Run alongside your main communication module logic.

#include <iostream>
#include <string>
#include <thread>
#include <atomic>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

// ── Configuration ───────────────────────────────────────────────
constexpr int     TCP_PORT    = 5000;
constexpr char    I2C_BUS[]   = "/dev/i2c-1";
constexpr uint8_t DRIVE_ADDR  = 0x10;   // ATmega1284 (styrmodul)
constexpr uint8_t SENSOR_ADDR = 0x11;   // ATmega16   (sensormodul)

// ── Drive command bytes (must match styrmodul switch cases) ─────
enum DriveCmd : uint8_t {
    CMD_STOP      = 0x00,
    CMD_FWD       = 0x01,
    CMD_BWD       = 0x02,
    CMD_FWD_LEFT  = 0x03,
    CMD_FWD_RIGHT = 0x04,
    CMD_BWD_LEFT  = 0x05,
    CMD_BWD_RIGHT = 0x06,
    CMD_LEFT      = 0x07,
    CMD_RIGHT     = 0x08,
};

// ── Arm command bytes ────────────────────────────────────────────
enum ArmCmd : uint8_t {
    ARM_X_PLUS    = 0x10, ARM_X_MINUS  = 0x11,
    ARM_Y_PLUS    = 0x12, ARM_Y_MINUS  = 0x13,
    ARM_Z_PLUS    = 0x14, ARM_Z_MINUS  = 0x15,
    ARM_BASE_CW   = 0x16, ARM_BASE_CCW = 0x17,
    ARM_CLAW_CW   = 0x18, ARM_CLAW_CCW = 0x19,
    ARM_TILT_UP   = 0x1A, ARM_TILT_DOWN= 0x1B,
    ARM_OPEN      = 0x1C, ARM_CLOSE    = 0x1D,
    ARM_NONE      = 0xFF,
};

// ── Globals ──────────────────────────────────────────────────────
std::atomic<bool> manual_mode{false};
int i2c_fd = -1;

// ── I2C helpers ──────────────────────────────────────────────────
bool i2c_init() {
    i2c_fd = open(I2C_BUS, O_RDWR);
    if (i2c_fd < 0) {
        std::cerr << "[I2C] Failed to open " << I2C_BUS << "\n";
        return false;
    }
    return true;
}

void i2c_send(uint8_t addr, uint8_t cmd) {
    if (i2c_fd < 0) return;
    ioctl(i2c_fd, I2C_SLAVE, addr);
    write(i2c_fd, &cmd, 1);
}

// ── Telemetry: read state from sensor module ─────────────────────
// Extend this to include your path-planner's current route/goods.
json build_telemetry() {
    uint8_t buf[4] = {0};
    if (i2c_fd >= 0) {
        ioctl(i2c_fd, I2C_SLAVE, SENSOR_ADDR);
        read(i2c_fd, buf, sizeof(buf));
    }
    json t;
    t["type"]     = "telemetry";
    t["position"] = buf[0];          // current node (0 = unknown)
    t["route"]    = json::array();   // populate from your path planner
    t["goods"]    = json::array();   // populate from your mission state
    return t;
}

// ── Message dispatcher ────────────────────────────────────────────
void handle_message(const json& msg) {
    const std::string type = msg.value("type", "");

    if (type == "mode") {
        const std::string cmd = msg.value("cmd", "");
        if (cmd == "manual") {
            manual_mode = true;
            i2c_send(DRIVE_ADDR, CMD_STOP);
            std::cout << "[Mode] MANUAL\n";
        } else if (cmd == "autonomous") {
            manual_mode = false;
            std::cout << "[Mode] AUTONOMOUS\n";
        }
        return;
    }

    if (type == "drive" && manual_mode) {
        const std::string cmd = msg.value("cmd", "");
        DriveCmd byte = CMD_STOP;
        if      (cmd == "fwd")       byte = CMD_FWD;
        else if (cmd == "bwd")       byte = CMD_BWD;
        else if (cmd == "fwd_left")  byte = CMD_FWD_LEFT;
        else if (cmd == "fwd_right") byte = CMD_FWD_RIGHT;
        else if (cmd == "bwd_left")  byte = CMD_BWD_LEFT;
        else if (cmd == "bwd_right") byte = CMD_BWD_RIGHT;
        else if (cmd == "left")      byte = CMD_LEFT;
        else if (cmd == "right")     byte = CMD_RIGHT;
        i2c_send(DRIVE_ADDR, byte);
        return;
    }

    if (type == "arm" && manual_mode) {
        const std::string cmd = msg.value("cmd", "");
        ArmCmd byte = ARM_NONE;
        if      (cmd == "x+")         byte = ARM_X_PLUS;
        else if (cmd == "x-")         byte = ARM_X_MINUS;
        else if (cmd == "y+")         byte = ARM_Y_PLUS;
        else if (cmd == "y-")         byte = ARM_Y_MINUS;
        else if (cmd == "z+")         byte = ARM_Z_PLUS;
        else if (cmd == "z-")         byte = ARM_Z_MINUS;
        else if (cmd == "base_cw")    byte = ARM_BASE_CW;
        else if (cmd == "base_ccw")   byte = ARM_BASE_CCW;
        else if (cmd == "claw_cw")    byte = ARM_CLAW_CW;
        else if (cmd == "claw_ccw")   byte = ARM_CLAW_CCW;
        else if (cmd == "tilt_up")    byte = ARM_TILT_UP;
        else if (cmd == "tilt_down")  byte = ARM_TILT_DOWN;
        else if (cmd == "claw_open")  byte = ARM_OPEN;
        else if (cmd == "claw_close") byte = ARM_CLOSE;
        if (byte != ARM_NONE) i2c_send(DRIVE_ADDR, byte);
        return;
    }

    if (type == "mission") {
        int target = msg.value("target", -1);
        if (target > 0) {
            std::cout << "[Mission] Target node: " << target << "\n";
            // TODO: call your planner here, e.g.:
            // path_planner.set_target(target);
        }
    }
}

// ── Per-client thread ─────────────────────────────────────────────
void client_thread(int fd) {
    std::cout << "[Server] Client connected\n";
    char        buf[1024];
    std::string line_buf;

    while (true) {
        int n = recv(fd, buf, sizeof(buf) - 1, 0);
        if (n <= 0) break;
        buf[n] = '\0';
        line_buf += buf;

        size_t pos;
        while ((pos = line_buf.find('\n')) != std::string::npos) {
            std::string line = line_buf.substr(0, pos);
            line_buf.erase(0, pos + 1);
            if (line.empty()) continue;
            try {
                handle_message(json::parse(line));
            } catch (const std::exception& e) {
                std::cerr << "[Parse error] " << e.what() << "\n";
            }
        }

        // Send telemetry back after every received message
        std::string telem = build_telemetry().dump() + "\n";
        send(fd, telem.c_str(), telem.size(), 0);
    }

    close(fd);
    std::cout << "[Server] Client disconnected\n";
}

// ── main ──────────────────────────────────────────────────────────
int main() {
    i2c_init();   // non-fatal if hardware not connected yet

    int srv = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(TCP_PORT);

    if (bind(srv, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "[Server] bind() failed\n";
        return 1;
    }
    listen(srv, 5);
    std::cout << "[Server] Listening on port " << TCP_PORT << "\n";

    while (true) {
        int client = accept(srv, nullptr, nullptr);
        if (client < 0) continue;
        std::thread(client_thread, client).detach();
    }

    close(srv);
    return 0;
}
