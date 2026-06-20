/*
 * Copyright (C) 2018 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cmath>
#include <set>

#include <android-base/logging.h>
#include <hidl/HidlTransportSupport.h>
#include <sys/socket.h>
#include <sys/sysinfo.h>
#include <linux/vm_sockets.h>
#include <regex>
#include <dirent.h>
#include <cstring>
#include <cstdio>
#include <fstream>
#include <map>
#include <system_error>
#include <charconv>
#include <algorithm>
#include <mutex>
#include <unistd.h>
#include <vector>
#include <string>

#include "Thermal.h"

#define CPU_NUM_MAX                 Thermal::getNumCpu()
#define CPU_USAGE_PARAS_NUM         5
#define CPU_USAGE_FILE              "/proc/stat"
#define CPU_ONLINE_FILE             "/sys/devices/system/cpu/online"
#define TEMP_UNIT                   1000.0
#define THERMAL_PORT                14096
#define MAX_ZONES                   40

namespace android {
namespace hardware {
namespace thermal {
namespace V2_0 {
namespace implementation {

using ::android::sp;
using ::android::hardware::interfacesEqual;
using ::android::hardware::thermal::V1_0::ThermalStatus;
using ::android::hardware::thermal::V1_0::ThermalStatusCode;

// =============================================================================
// DRIVER REGISTRY
// =============================================================================
//
// This table documents all hwmon drivers and thermal zone types currently
// supported by this HAL. To add a new driver, search for "EXTENSION POINT"
// markers in this file (numbered #1 through #7) and see DRIVER_INTEGRATION.md.
//
// ┌───────────────────┬─────────────────┬──────────────────────────────────────┐
// │ hwmon name        │ Sensor Type     │ Labels                               │
// ├───────────────────┼─────────────────┼──────────────────────────────────────┤
// │ coretemp          │ CPU (Intel)     │ Core N, Package id N, Physical id N  │
// │ k10temp           │ CPU (AMD)       │ Tdie, Tctl, TccdN                    │
// │ zenpower          │ CPU (AMD)       │ [cpuN] Tdie, [cpuN] Tctl, TccdN      │
// ├───────────────────┼─────────────────┼──────────────────────────────────────┤
// │ (new driver)      │ CPU/GPU/Battery │ (see DRIVER_INTEGRATION.md)          │
// └───────────────────┴─────────────────┴──────────────────────────────────────┘
//
// Thermal zone types (for fallback when hwmon is unavailable):
//   x86_pkg_temp (Intel), k10temp (AMD), acpitz (generic ACPI)
//
// Examples of drivers that could be integrated:
//   thinkpad_acpi    - ThinkPad thermal sensors, fan control
//   steamdeck-hwmon  - Steam Deck GPU/APU temperature
//   oxp-sensors      - OneXPlayer/AOKZOE device sensors
//   amdgpu           - AMD GPU temperature (edge/junction/mem)
//   nouveau / i915   - NVIDIA/Intel GPU temperature
//   power_supply     - Battery temperature (via /sys/class/power_supply/)
//

// =============================================================================
// Dynamic CPU Labels (replaces fixed 16-element CPU_LABEL array)
// =============================================================================
static std::vector<std::string> S_CPU_LABELS;

static void init_cpu_labels(int num_cpus) {
    S_CPU_LABELS.clear();
    S_CPU_LABELS.reserve(num_cpus);
    for (int i = 0; i < num_cpus; i++) {
        S_CPU_LABELS.push_back("CPU" + std::to_string(i));
    }
    ALOGI("Initialized %d dynamic CPU labels (up from fixed 16)", num_cpus);
}

// =============================================================================
// CPU Vendor Detection
// =============================================================================
enum CpuVendor { CPU_VENDOR_UNKNOWN, CPU_VENDOR_INTEL, CPU_VENDOR_AMD };
static CpuVendor S_CPU_VENDOR = CPU_VENDOR_UNKNOWN;

static CpuVendor detect_cpu_vendor() {
    std::ifstream cpuinfo("/proc/cpuinfo");
    if (!cpuinfo.is_open()) {
        ALOGE("Failed to open /proc/cpuinfo for vendor detection");
        return CPU_VENDOR_UNKNOWN;
    }
    std::string line;
    while (std::getline(cpuinfo, line)) {
        if (line.find("vendor_id") != std::string::npos) {
            if (line.find("GenuineIntel") != std::string::npos) {
                ALOGI("Detected CPU vendor: Intel");
                return CPU_VENDOR_INTEL;
            }
            if (line.find("AuthenticAMD") != std::string::npos) {
                ALOGI("Detected CPU vendor: AMD");
                return CPU_VENDOR_AMD;
            }
        }
    }
    ALOGW("Unknown CPU vendor");
    return CPU_VENDOR_UNKNOWN;
}

// =============================================================================
// Structs (unchanged)
// =============================================================================
struct zone_info {
	uint32_t temperature;
	uint32_t trip_0;
	uint32_t trip_1;
	uint32_t trip_2;
	uint16_t number;
	int16_t type;
};

struct header {
	uint8_t intelipcid[9];
	uint16_t notifyid;
	uint16_t length;
};

static const char *THROTTLING_SEVERITY_LABEL[] = {
                                                  "NONE",
                                                  "LIGHT",
                                                  "MODERATE",
                                                  "SEVERE",
                                                  "CRITICAL",
                                                  "EMERGENCY",
                                                  "SHUTDOWN"};

// =============================================================================
// Static Temperature / Threshold / Cooling Variables
// =============================================================================
static const Temperature_1_0 kTemp_1_0 = {
        .type = static_cast<::android::hardware::thermal::V1_0::TemperatureType>(
                TemperatureType::CPU),
        .name = "TCPU",
        .currentValue = 25,
        .throttlingThreshold = 108,
        .shutdownThreshold = 109,
        .vrThrottlingThreshold = NAN,
};

// =============================================================================
// Temperature & Threshold Variables
//
// Each temperature source requires:
//   1. A Temperature_2_0 variable (holds current value + throttle status)
//   2. A TemperatureThreshold variable (holds hot/cold threshold arrays)
//   3. Registration in getCurrentTemperatures() callback
//   4. Registration in getTemperatureThresholds() callback
//
// Currently supported:
//   - CPU:     kTemp_2_0      + kTempThreshold    (real data from hwmon/thermal zone)
//   - BATTERY: kTemp_2_0_1    + kTempThreshold_1  (real data from VSOCK or dummy)
//   - GPU:     kDummyTemp     + kDummyTempThreshold (dummy data for VTS compliance)
//
// HOW TO ADD A NEW REAL TEMPERATURE SOURCE (e.g., GPU from amdgpu hwmon):
//   1. Define a Temperature_2_0 variable:
//        static Temperature_2_0 kTemp_GPU = {
//            .type = TemperatureType::GPU,
//            .name = "TGPU",
//            .value = 25,
//            .throttlingStatus = ThrottlingSeverity::NONE,
//        };
//   2. Define a TemperatureThreshold:
//        static TemperatureThreshold kTempThreshold_GPU = {
//            .type = TemperatureType::GPU,
//            .name = "TGPU",
//            .hotThrottlingThresholds = {{NAN, NAN, NAN, NAN, NAN, 90, 100}},
//            .coldThrottlingThresholds = {{NAN, NAN, NAN, NAN, NAN, NAN, NAN}},
//            .vrThrottlingThreshold = NAN,
//        };
//   3. Add reading logic in the CheckThermalServerity thread (see extension
//      points in that function)
//   4. Add the variable to getCurrentTemperatures() and
//      getTemperatureThresholds() callbacks
//   5. Protect updates with s_temp_data_mutex
// =============================================================================
static Temperature_2_0 kTemp_2_0 = {
        .type = TemperatureType::CPU,
        .name = "TCPU",
        .value = 25,
        .throttlingStatus = ThrottlingSeverity::NONE,
};

// Workaround for VTS. Dummy entry for GPU temperature.
static Temperature_2_0 kDummyTemp = {
	.type = TemperatureType::GPU,
	.name = "test sensor",
	.value = 25,
	.throttlingStatus = ThrottlingSeverity::NONE,
};

static Temperature_2_0 kTemp_2_0_1 = {
        .type = TemperatureType::BATTERY,
        .name = "TBATTERY",
        .value = 25,
        .throttlingStatus = ThrottlingSeverity::NONE,
};

static TemperatureThreshold kTempThreshold = {
        .type = TemperatureType::CPU,
        .name = "TCPU",
        .hotThrottlingThresholds = {{NAN, NAN, NAN, NAN, NAN, 99, 108}},
        .coldThrottlingThresholds = {{NAN, NAN, NAN, NAN, NAN, NAN, NAN}},
        .vrThrottlingThreshold = NAN,
};

static TemperatureThreshold kTempThreshold_1 = {
        .type = TemperatureType::BATTERY,
        .name = "TBATTERY",
        .hotThrottlingThresholds = {{NAN, NAN, NAN, NAN, NAN, 65, 68}},
        .coldThrottlingThresholds = {{NAN, NAN, NAN, NAN, NAN, NAN, NAN}},
        .vrThrottlingThreshold = NAN,
};

// ┌──────────────────────────────────────────────────────────────────────────┐
// │ EXTENSION POINT #1: Add Temperature/Threshold globals for new types     │
// │ See DRIVER_INTEGRATION.md Step 1                                        │
// │                                                                         │
// │ For each new sensor type (GPU, Battery, etc.), add:                     │
// │   - A Temperature_2_0 global to hold the current reading                │
// │   - A TemperatureThreshold global to hold throttling thresholds         │
// │                                                                         │
// │ Example for a real GPU sensor (replacing the dummy):                    │
// │   static Temperature_2_0 kGpuTemp = {                                  │
// │       .type = TemperatureType::GPU,                                     │
// │       .name = "TGPU",                                                   │
// │       .value = 25,                                                      │
// │       .throttlingStatus = ThrottlingSeverity::NONE,                     │
// │   };                                                                    │
// │   static TemperatureThreshold kGpuTempThreshold = {                    │
// │       .type = TemperatureType::GPU,                                     │
// │       .name = "TGPU",                                                   │
// │       .hotThrottlingThresholds = {{NAN, NAN, NAN, NAN, NAN, 90, 100}}, │
// │       .coldThrottlingThresholds = {{NAN, NAN, NAN, NAN, NAN, NAN, NAN}},│
// │       .vrThrottlingThreshold = NAN,                                     │
// │   };                                                                    │
// └──────────────────────────────────────────────────────────────────────────┘

// Workaround for VTS. Dummy entry for GPU threshold.
static TemperatureThreshold kDummyTempThreshold = {
        .type = TemperatureType::GPU,
        .name = "test sensor",
        .hotThrottlingThresholds = {{NAN, NAN, NAN, NAN, NAN, 90, 100}},
        .coldThrottlingThresholds = {{NAN, NAN, NAN, NAN, NAN, NAN, NAN}},
        .vrThrottlingThreshold = NAN,
};

static const CoolingDevice_1_0 kCooling_1_0 = {
        .type = ::android::hardware::thermal::V1_0::CoolingType::FAN_RPM,
        .name = "test cooling device",
        .currentValue = 100.0,
};

static const CoolingDevice_2_0 kCooling_2_0 = {
        .type = CoolingType::FAN,
        .name = "test cooling device",
        .value = 100,
};

static const CpuUsage kCpuUsage = {
        .name = "cpu_name",
        .active = 0,
        .total = 0,
        .isOnline = true,
};

static bool is_vsock_present;

struct temp_info {
    int16_t type;
    uint32_t temp;
};

// =============================================================================
// Thread-Safety: Mutex for temperature/threshold globals
// =============================================================================
// Protects: kTemp_2_0, kTemp_2_0_1, kTempThreshold, kTempThreshold_1
// The existing member thermal_temp_mutex was declared but never used.
// We use a file-scope static mutex so static functions can also lock it.
static std::mutex s_temp_data_mutex;

// =============================================================================
// Smart Thermal Zone Discovery (replaces hardcoded thermal_zone0)
// =============================================================================
static std::string discover_cpu_thermal_zone(bool enable_logging) {
    const std::string thermal_base = "/sys/class/thermal/";

    // Priority-ordered list of thermal zone types that reliably indicate CPU temperature.
    // Lower index = higher priority.
    //
    // HOW TO ADD A NEW CPU THERMAL ZONE TYPE:
    //   1. Run on target hardware: cat /sys/class/thermal/thermal_zone*/type
    //   2. Identify which zone type corresponds to CPU temperature
    //   3. Add it to this list with an appropriate priority number
    //      (1 = most specific/accurate, 10+ = generic fallback)
    //
    // EXTENSION POINT: Add new entries for other platform-specific CPU
    // thermal zones. Examples:
    //   {"thinkpad",     5},    // ThinkPad ACPI thermal zone
    //   {"cpu-thermal",  3},    // Device-tree based platforms
    //   {"soc-thermal",  4},    // SoC-level thermal zone
    struct ZoneTypePriority {
        std::string type_name;
        int priority;
    };
    static const std::vector<ZoneTypePriority> known_cpu_zones = {
        {"x86_pkg_temp", 1},    // Intel package temperature (most specific, most accurate)
        {"k10temp",      2},    // AMD k10temp (if exposed as thermal zone)
        {"acpitz",       10},   // ACPI thermal zone (generic, often inaccurate but better than nothing)
        // EXTENSION POINT #2: Add new thermal zone types here (see DRIVER_INTEGRATION.md Step 2a)
        // Example: {"thinkpad", 3},  // ThinkPad ACPI thermal zone
    };

    std::string best_path;
    std::string best_type;
    int best_priority = INT_MAX;

    for (int i = 0; i < 100; i++) {
        std::string zone_dir = thermal_base + "thermal_zone" + std::to_string(i);
        std::string type_path = zone_dir + "/type";

        std::ifstream type_file(type_path);
        if (!type_file.is_open()) break;  // No more zones

        std::string zone_type;
        std::getline(type_file, zone_type);
        type_file.close();

        for (const auto& known : known_cpu_zones) {
            if (zone_type == known.type_name && known.priority < best_priority) {
                std::string temp_path = zone_dir + "/temp";
                if (access(temp_path.c_str(), R_OK) == 0) {
                    best_priority = known.priority;
                    best_path = temp_path;
                    best_type = zone_type;
                    if (enable_logging) {
                        ALOGI("Found CPU thermal zone candidate: %s (type=%s, priority=%d)",
                              zone_dir.c_str(), zone_type.c_str(), known.priority);
                    }
                }
            }
        }
    }

    if (!best_path.empty()) {
        if (enable_logging) {
            ALOGI("Selected CPU thermal zone: %s (type=%s)", best_path.c_str(), best_type.c_str());
        }
    } else {
        if (enable_logging) {
            ALOGW("No known CPU thermal zone type found via discovery.");
        }
    }

    return best_path;
}

// =============================================================================
// Dynamic Threshold Detection (replaces hardcoded Intel-specific values)
// =============================================================================

// Scan thermal zones for critical trip points to determine hardware limits.
static float read_critical_trip_from_zones() {
    const std::string thermal_base = "/sys/class/thermal/";
    float min_critical = NAN;

    for (int z = 0; z < 100; z++) {
        std::string zone_dir = thermal_base + "thermal_zone" + std::to_string(z);
        std::string type_path = zone_dir + "/type";

        std::ifstream type_file(type_path);
        if (!type_file.is_open()) break;

        std::string zone_type;
        std::getline(type_file, zone_type);
        type_file.close();

        // Only examine CPU-related zones for critical trip points.
        // EXTENSION POINT: Add thermal zone type strings for new CPU drivers.
        if (zone_type != "x86_pkg_temp" && zone_type != "acpitz" &&
            zone_type != "k10temp" && zone_type != "coretemp" &&
            /* EXTENSION POINT: Add new CPU zone types here.
             * Examples:
             *   zone_type != "thinkpad" &&
             *   zone_type != "cpu-thermal" &&
             *   zone_type != "soc-thermal" &&
             */
            zone_type.find("cpu") == std::string::npos &&
            zone_type.find("CPU") == std::string::npos) {
            continue;
        }

        for (int t = 0; t < 20; t++) {
            std::string trip_type_path = zone_dir + "/trip_point_" + std::to_string(t) + "_type";
            std::string trip_temp_path = zone_dir + "/trip_point_" + std::to_string(t) + "_temp";

            std::ifstream trip_type_file(trip_type_path);
            if (!trip_type_file.is_open()) break;

            std::string trip_type;
            std::getline(trip_type_file, trip_type);
            trip_type_file.close();

            if (trip_type == "critical") {
                std::ifstream trip_temp_file(trip_temp_path);
                float raw_temp;
                if (trip_temp_file >> raw_temp) {
                    float temp_c = raw_temp / 1000.0f;
                    // Sanity check: must be a reasonable CPU critical temp
                    if (temp_c > 50.0f && temp_c < 200.0f) {
                        if (isnan(min_critical) || temp_c < min_critical) {
                            min_critical = temp_c;
                        }
                        ALOGI("Found critical trip point: %.1f°C in zone type=%s",
                              temp_c, zone_type.c_str());
                    }
                }
            }
        }
    }

    return min_critical;
}

// Configure CPU thermal thresholds based on hardware detection.
// Called once at startup from the check thread.
static void initialize_thermal_config() {
    // 1. Try to read actual hardware critical temperature from sysfs trip points
    float critical_temp = read_critical_trip_from_zones();

    if (!isnan(critical_temp)) {
        // Use hardware-reported critical temp: EMERGENCY at critical-10, SHUTDOWN at critical
        float emergency = critical_temp - 10.0f;
        float shutdown = critical_temp;

        std::lock_guard<std::mutex> _lock(s_temp_data_mutex);
        kTempThreshold.hotThrottlingThresholds[5] = emergency;
        kTempThreshold.hotThrottlingThresholds[6] = shutdown;
        ALOGI("CPU thresholds from hardware trip points: EMERGENCY=%.1f, SHUTDOWN=%.1f",
              emergency, shutdown);
        return;
    }

    // 2. Fallback: vendor-specific defaults
    std::lock_guard<std::mutex> _lock(s_temp_data_mutex);
    switch (S_CPU_VENDOR) {
        case CPU_VENDOR_INTEL:
            // Intel defaults (original values): TjMax typically 100-105°C
            kTempThreshold.hotThrottlingThresholds[5] = 99;
            kTempThreshold.hotThrottlingThresholds[6] = 108;
            ALOGI("Using Intel default thresholds: EMERGENCY=99, SHUTDOWN=108");
            break;

        case CPU_VENDOR_AMD:
            // AMD defaults: TjMax typically 95°C (desktop) or 100°C (mobile)
            kTempThreshold.hotThrottlingThresholds[5] = 85;
            kTempThreshold.hotThrottlingThresholds[6] = 95;
            ALOGI("Using AMD default thresholds: EMERGENCY=85, SHUTDOWN=95");
            break;

        default:
            // Conservative defaults for unknown CPUs
            kTempThreshold.hotThrottlingThresholds[5] = 85;
            kTempThreshold.hotThrottlingThresholds[6] = 95;
            ALOGI("Using conservative default thresholds: EMERGENCY=85, SHUTDOWN=95");
            break;
    }
}

// =============================================================================
// Temperature Reading: Thermal Zone (parameterized path)
// =============================================================================
static int get_soc_pkg_temperature(float* temp, const std::string& sysfs_path)
{
    float fTemp = 0;
    int len = 0;
    FILE *file = NULL;

    file = fopen(sysfs_path.c_str(), "r");

    if (file == NULL) {
        ALOGE("%s: failed to open file %s: %s", __func__, sysfs_path.c_str(), strerror(errno));
        return -errno;
    }

    len = fscanf(file, "%f", &fTemp);
    if (len < 0) {
        ALOGE("%s: failed to read file %s: %s", __func__, sysfs_path.c_str(), strerror(errno));
        fclose(file);
        return -errno;
    }

    fclose(file);
    *temp = fTemp / TEMP_UNIT;

    return 0;
}

// =============================================================================
// Temperature Reading: hwmon (coretemp + k10temp + zenpower)
// =============================================================================

// --- Constants ---
const float TEMP_UNIT_DIVISOR = 1000.0f;

// --- Structures: CoreTemperature and AllCpuTemperatures ---
struct CoreTemperature {
    int id = -1;
    std::string label;
    float temperature = NAN;
    CoreTemperature(int i, const std::string& l, float t) : id(i), label(l), temperature(t) {}
};

struct AllCpuTemperatures {
    float package_temperature = NAN;
    std::string package_label;
    std::vector<CoreTemperature> core_temps;
    int detected_core_count = 0;
    float max_core_temp = NAN;
    float max_overall_temp = NAN;
    void update_max_temps() {
        max_core_temp = NAN;
        max_overall_temp = NAN;
        if (!isnan(package_temperature)) {
            max_overall_temp = package_temperature;
        }
        for (const auto& core_temp : core_temps) {
            if (!isnan(core_temp.temperature)) {
                if (isnan(max_core_temp) || core_temp.temperature > max_core_temp) {
                    max_core_temp = core_temp.temperature;
                }
                if (isnan(max_overall_temp) || core_temp.temperature > max_overall_temp) {
                    max_overall_temp = core_temp.temperature;
                }
            }
        }
        detected_core_count = core_temps.size();
    }
};

// --- Cached Sensor Information ---
struct DiscoveredSensor {
    int id = -1;
    std::string label;
    std::string input_file_path;
    bool is_package_sensor = false;
};

static std::vector<DiscoveredSensor> S_CACHED_SENSORS;
static std::vector<std::string> S_CACHED_HWMON_DIRS;
static bool S_CACHE_INITIALIZED = false;

// --- Function: find_hwmon_dirs_for_device  ---
std::vector<std::string> find_hwmon_dirs_for_device(const std::string& device_name, bool enable_logging) {
    std::vector<std::string> hwmon_paths;
    const std::string hwmon_base_dir = "/sys/class/hwmon/";
    DIR* dir = opendir(hwmon_base_dir.c_str());
    if (!dir) {
        if (enable_logging) ALOGE("Failed to open directory: %s - %s", hwmon_base_dir.c_str(), strerror(errno));
        return hwmon_paths;
    }
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        if (entry->d_type == DT_DIR || entry->d_type == DT_LNK) {
            std::string dirname = entry->d_name;
            if (dirname == "." || dirname == "..") {
                continue;
            }
            std::string name_file_path = hwmon_base_dir + dirname + "/name";
            std::ifstream name_file(name_file_path);
            if (name_file.is_open()) {
                std::string current_device_name;
                if (std::getline(name_file, current_device_name) && current_device_name == device_name) {
                    hwmon_paths.push_back(hwmon_base_dir + dirname);
                }
                name_file.close();
            }
        }
    }
    closedir(dir);
    return hwmon_paths;
}

// --- Function: clear_cache ---
void clear_cache() {
    S_CACHED_SENSORS.clear();
    S_CACHED_HWMON_DIRS.clear();
    S_CACHE_INITIALIZED = false;
    ALOGW("Temperature sensor cache has been cleared.");
}

AllCpuTemperatures get_cpu_temperatures(bool enable_logging = true) {
    AllCpuTemperatures all_temps;
    bool read_from_cache_successful = true;

    if (S_CACHE_INITIALIZED) {
        for (const auto& sensor_info : S_CACHED_SENSORS) {
            std::ifstream file(sensor_info.input_file_path);
            float temp_raw;
            if (file.is_open() && (file >> temp_raw)) {
                float temp_celsius = temp_raw / TEMP_UNIT_DIVISOR;
                if (sensor_info.is_package_sensor) {
                    if (isnan(all_temps.package_temperature) || temp_celsius > all_temps.package_temperature) {
                         all_temps.package_temperature = temp_celsius;
                         all_temps.package_label = sensor_info.label;
                    }
                } else {
                    all_temps.core_temps.emplace_back(sensor_info.id, sensor_info.label, temp_celsius);
                }
            } else {
                if (enable_logging) ALOGE("Failed to read from cached path: %s. Invalidating cache.", sensor_info.input_file_path.c_str());
                clear_cache();
                read_from_cache_successful = false;
                break;
            }
            file.close();
        }
        if (read_from_cache_successful) {
             std::sort(all_temps.core_temps.begin(), all_temps.core_temps.end(),
              [](const CoreTemperature& a, const CoreTemperature& b) {
                  return a.id < b.id;
              });
            all_temps.update_max_temps();
            return all_temps;
        }
    }

    if (enable_logging && !read_from_cache_successful) {
        ALOGI("Cache miss or invalidation. Performing full sensor discovery...");
    } else if (enable_logging && !S_CACHE_INITIALIZED) {
        ALOGI("Cache not initialized. Performing initial sensor discovery...");
    }

    S_CACHED_SENSORS.clear();
    S_CACHED_HWMON_DIRS.clear();

    // =========================================================================
    // hwmon Driver Discovery Chain
    //
    // This chain determines which hwmon driver to use for CPU temperature.
    // Only one driver will be active (first match wins).
    //
    // Current chain: coretemp (Intel) → k10temp (AMD) → zenpower (AMD OOT)
    //
    // HOW TO ADD A NEW CPU hwmon DRIVER:
    //   1. Find the driver's hwmon name on target hardware:
    //        cat /sys/class/hwmon/hwmon*/name
    //   2. Check what temp labels it exposes:
    //        cat /sys/class/hwmon/hwmon*/temp*_label
    //   3. Add a new find_hwmon_dirs_for_device() call in this chain
    //   4. If the driver uses non-standard labels (not "Core N" or
    //      "Tdie"/"Tctl"/"TccdN"), add new regex patterns below
    //      (see "Label Pattern Extension" section)
    //
    // FUTURE: To add GPU or Battery hwmon support, create separate
    // discovery functions (e.g., get_gpu_temperatures()) following
    // this same pattern. Candidate drivers:
    //   GPU:     "amdgpu", "nouveau", "nvidia", "i915"
    //   Battery: "bq24190_charger", "max17042_battery"
    //   Other:   "thinkpad" (thinkpad_acpi), "steamdeck_hwmon",
    //            "oxpec" (oxp-sensors)
    // =========================================================================
    bool is_amd_sensor = false;
    S_CACHED_HWMON_DIRS = find_hwmon_dirs_for_device("coretemp", enable_logging);
    if (S_CACHED_HWMON_DIRS.empty()) {
        S_CACHED_HWMON_DIRS = find_hwmon_dirs_for_device("k10temp", enable_logging);
        if (S_CACHED_HWMON_DIRS.empty()) {
            // zenpower / zenpower5: out-of-tree AMD driver that replaces k10temp
            // with additional sensors (SVI2 voltage/current, CCD temps, etc.)
            // hwmon device name is "zenpower" for all zenpower versions.
            S_CACHED_HWMON_DIRS = find_hwmon_dirs_for_device("zenpower", enable_logging);
            if (!S_CACHED_HWMON_DIRS.empty()) {
                is_amd_sensor = true;
                if (enable_logging) ALOGI("Using AMD zenpower hwmon driver for temperature sensing");
            }
            // EXTENSION POINT #3: Add more CPU hwmon driver names here
            // See DRIVER_INTEGRATION.md Step 2b
            // Example:
            // if (S_CACHED_HWMON_DIRS.empty()) {
            //     S_CACHED_HWMON_DIRS = find_hwmon_dirs_for_device("thinkpad", enable_logging);
            // }
        } else {
            is_amd_sensor = true;
            if (enable_logging) ALOGI("Using AMD k10temp hwmon driver for temperature sensing");
        }
    } else {
        if (enable_logging) ALOGI("Using Intel coretemp hwmon driver for temperature sensing");
    }

    if (S_CACHED_HWMON_DIRS.empty()){
        if (enable_logging) ALOGE("No coretemp, k10temp, or zenpower hwmon directories found.");
        S_CACHE_INITIALIZED = false;
        return all_temps;
   }

    std::regex temp_file_regex("^temp([0-9]+)_(input|label)$");

    // =========================================================================
    // Label Pattern Extension
    //
    // Each hwmon driver uses different label strings. The regex patterns below
    // determine how sensor labels are classified (core vs. package).
    //
    // HOW TO ADD LABEL PATTERNS FOR A NEW DRIVER:
    //   1. Read all labels: cat /sys/class/hwmon/hwmonN/temp*_label
    //   2. Identify which labels are per-core and which are package-level
    //   3. Add new regex patterns below
    //   4. Add matching else-if blocks in the label classification loop
    //      (around line 664 onwards)
    //
    // Example: thinkpad_acpi might expose:
    //   temp1_label = "CPU"         → package sensor
    //   temp2_label = "GPU"         → GPU sensor (would need new handler)
    //   temp3_label = "Battery"     → battery sensor
    //
    // Example: steamdeck_hwmon might expose:
    //   temp1_label = "SoC"         → package sensor (APU temperature)
    //   temp2_label = "Battery"     → battery sensor
    //
    // EXTENSION POINT: Add new regex patterns for new drivers here.
    // static std::regex thinkpad_cpu_regex("^CPU$");
    // static std::regex thinkpad_gpu_regex("^GPU$");
    // static std::regex steamdeck_soc_regex("^SoC$");
    // =========================================================================

    // Intel label patterns
    std::regex core_label_regex("^Core\\s+([0-9]+)$");
    std::regex package_label_regex("^(Package id [0-9]+|Physical id [0-9]+)$");

    // AMD label patterns (k10temp / zenpower)
    // Tctl = control temp (may include offset on Threadripper/EPYC)
    // Tdie = actual die temp (preferred over Tctl when available)
    // TccdN = per-CCD temperature (Zen 2+, treated like per-core temps)
    //
    // zenpower5 on multi-CPU systems (Threadripper/EPYC) prefixes labels
    // with "cpuN " (e.g., "cpu0 Tdie", "cpu1 Tccd1"). The optional
    // (?:cpu[0-9]+\s+)? group handles both formats transparently.
    std::regex amd_ccd_label_regex("^(?:cpu[0-9]+\\s+)?Tccd([0-9]+)$");
    std::regex amd_die_label_regex("^(?:cpu[0-9]+\\s+)?(Tdie|Tctl)$");

    // ┌──────────────────────────────────────────────────────────────────────┐
    // │ EXTENSION POINT #4: Add label regex patterns for new drivers         │
    // │ See DRIVER_INTEGRATION.md Step 3                                     │
    // │                                                                      │
    // │ To find your labels: cat /sys/class/hwmon/hwmon*/temp*_label          │
    // │                                                                      │
    // │ Examples:                                                            │
    // │   // thinkpad_acpi: std::regex thinkpad_cpu_regex("^CPU$");          │
    // │   // amdgpu: std::regex amdgpu_edge_regex("^edge$");                │
    // │   // steamdeck-hwmon: std::regex steamdeck_regex("^(CPU|APU)$");     │
    // └──────────────────────────────────────────────────────────────────────┘

    std::map<int, std::string> discovered_labels;
    std::map<int, std::string> discovered_input_paths;

    for (const std::string& hwmon_dir : S_CACHED_HWMON_DIRS) {
        DIR* dir = opendir(hwmon_dir.c_str());
        if (!dir) {
            if (enable_logging) ALOGE("Failed to open hwmon directory: %s - %s", hwmon_dir.c_str(), strerror(errno));
            continue;
        }
        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            if (entry->d_type == DT_REG) {
                std::string filename = entry->d_name;
                std::smatch match;
                if (std::regex_match(filename, match, temp_file_regex)) {
                    std::string index_str = match[1].str();
                    int parsed_index;
                    auto fc_res_idx = std::from_chars(index_str.data(), index_str.data() + index_str.size(), parsed_index);

                    if (fc_res_idx.ec == std::errc() && fc_res_idx.ptr == index_str.data() + index_str.size()) {
                        std::string type = match[2].str();
                        std::string full_path = hwmon_dir + "/" + filename;
                        if (type == "label") {
                            std::ifstream file(full_path);
                            if (file.is_open()) {
                                std::string label_content;
                                if (std::getline(file, label_content)) {
                                    discovered_labels[parsed_index] = label_content;
                                }
                                file.close();
                            }
                        } else if (type == "input") {
                            discovered_input_paths[parsed_index] = full_path;
                        }
                    } else {
                        if (enable_logging) ALOGW("Failed to parse index from filename fragment '%s' in %s. Error: %d",
                                                  index_str.c_str(), filename.c_str(), static_cast<int>(fc_res_idx.ec));
                    }
                }
            }
        }
        closedir(dir);
    }

    for (auto const& [index_val, label_content] : discovered_labels) {
        if (discovered_input_paths.count(index_val)) {
            std::string input_path = discovered_input_paths[index_val];
            std::ifstream temp_file(input_path);
            float temp_raw;
            if (temp_file.is_open() && (temp_file >> temp_raw)) {
                float temp_celsius = temp_raw / TEMP_UNIT_DIVISOR;
                DiscoveredSensor current_sensor_info;
                current_sensor_info.label = label_content;
                current_sensor_info.input_file_path = input_path;

                std::smatch core_match;

                // --- Intel: Core N ---
                if (std::regex_match(label_content, core_match, core_label_regex)) {
                    std::string core_id_str = core_match[1].str();
                    int parsed_core_id;
                    auto fc_res_core_id = std::from_chars(core_id_str.data(), core_id_str.data() + core_id_str.size(), parsed_core_id);

                    if (fc_res_core_id.ec == std::errc() && fc_res_core_id.ptr == core_id_str.data() + core_id_str.size()) {
                        current_sensor_info.id = parsed_core_id;
                        current_sensor_info.is_package_sensor = false;
                        all_temps.core_temps.emplace_back(current_sensor_info.id, label_content, temp_celsius);
                        S_CACHED_SENSORS.push_back(current_sensor_info);
                    } else {
                         if (enable_logging) ALOGW("Failed to parse core ID from label '%s' (value '%s'). Error: %d",
                                                  label_content.c_str(), core_id_str.c_str(), static_cast<int>(fc_res_core_id.ec));
                    }

                // --- AMD: TccdN (per-CCD temperature, treated like per-core) ---
                } else if (std::regex_match(label_content, core_match, amd_ccd_label_regex)) {
                    std::string ccd_id_str = core_match[1].str();
                    int parsed_ccd_id;
                    auto fc_res_ccd = std::from_chars(ccd_id_str.data(), ccd_id_str.data() + ccd_id_str.size(), parsed_ccd_id);

                    if (fc_res_ccd.ec == std::errc() && fc_res_ccd.ptr == ccd_id_str.data() + ccd_id_str.size()) {
                        current_sensor_info.id = parsed_ccd_id;
                        current_sensor_info.is_package_sensor = false;
                        all_temps.core_temps.emplace_back(current_sensor_info.id, label_content, temp_celsius);
                        S_CACHED_SENSORS.push_back(current_sensor_info);
                        if (enable_logging) ALOGI("Discovered AMD CCD sensor: %s = %.1f°C", label_content.c_str(), temp_celsius);
                    } else {
                        if (enable_logging) ALOGW("Failed to parse CCD ID from label '%s'", label_content.c_str());
                    }

                // --- Intel: Package id N / Physical id N ---
                } else if (std::regex_match(label_content, package_label_regex)) {
                    current_sensor_info.is_package_sensor = true;
                    if (isnan(all_temps.package_temperature) || temp_celsius > all_temps.package_temperature) {
                        all_temps.package_temperature = temp_celsius;
                        all_temps.package_label = label_content;
                    }
                    S_CACHED_SENSORS.push_back(current_sensor_info);

                // --- AMD: Tdie / Tctl (package-level temperature) ---
                } else if (std::regex_match(label_content, amd_die_label_regex)) {
                    current_sensor_info.is_package_sensor = true;
                    // Prefer Tdie over Tctl: Tdie is the actual die temperature,
                    // while Tctl may include an artificial offset (Threadripper/EPYC).
                    bool should_update = false;
                    if (label_content == "Tdie" || label_content.find("Tdie") != std::string::npos) {
                        // Tdie always takes priority
                        should_update = true;
                    } else if (all_temps.package_label.find("Tdie") != std::string::npos) {
                        // Don't overwrite Tdie with Tctl
                        should_update = false;
                    } else {
                        // No Tdie seen yet; use Tctl
                        should_update = isnan(all_temps.package_temperature) || temp_celsius > all_temps.package_temperature;
                    }

                    if (should_update) {
                        all_temps.package_temperature = temp_celsius;
                        all_temps.package_label = label_content;
                    }
                    S_CACHED_SENSORS.push_back(current_sensor_info);
                    if (enable_logging) ALOGI("Discovered AMD die sensor: %s = %.1f°C%s",
                                              label_content.c_str(), temp_celsius,
                                              should_update ? " (active)" : " (shadowed by Tdie)");
                }
                // ┌──────────────────────────────────────────────────────────┐
                // │ EXTENSION POINT #5: Match new driver labels here         │
                // │ See DRIVER_INTEGRATION.md Step 4                         │
                // │                                                          │
                // │ Add else-if branches for your driver's label patterns.   │
                // │ For GPU/Battery, update separate globals under mutex.    │
                // │                                                          │
                // │ Example:                                                 │
                // │ } else if (std::regex_match(label, gpu_regex)) {         │
                // │     std::lock_guard<std::mutex> _l(s_temp_data_mutex);    │
                // │     kGpuTemp.value = temp_celsius;                        │
                // │ }                                                         │
                // └──────────────────────────────────────────────────────────┘
            } else {
                 if (enable_logging) ALOGW("Failed to read temp from discovered path: %s", input_path.c_str());
            }
            temp_file.close();
        }
    }

    if (!S_CACHED_SENSORS.empty()) {
        S_CACHE_INITIALIZED = true;
        if (enable_logging) ALOGI("Sensor cache populated with %zu entries (%s mode).",
                                  S_CACHED_SENSORS.size(), is_amd_sensor ? "AMD" : "Intel");
    } else {
        S_CACHE_INITIALIZED = false;
        if (enable_logging) ALOGW("No sensors were successfully discovered to populate cache.");
    }

    std::sort(all_temps.core_temps.begin(), all_temps.core_temps.end(),
              [](const CoreTemperature& a, const CoreTemperature& b) {
                  return a.id < b.id;
              });
    all_temps.update_max_temps();
    return all_temps;
}

// =============================================================================
// EXTENSION POINT #6: Non-CPU Sensor Discovery Functions
// =============================================================================
//
// To add GPU, Battery, or other non-CPU hwmon sensor support, create
// dedicated discovery functions following the get_cpu_temperatures() pattern.
//
// Each function should:
//   1. Use find_hwmon_dirs_for_device("driver_name") to locate hwmon dirs
//   2. Read temp*_input files for temperature values
//   3. Update the corresponding global (e.g., kGpuTemp) under s_temp_data_mutex
//   4. Be called from CheckThermalServerity() in the monitoring loop
//
// Example skeleton for GPU temperature:
//
// static float get_gpu_temperature(bool enable_logging = true) {
//     static std::vector<std::string> gpu_hwmon_dirs;
//     static bool gpu_discovered = false;
//
//     if (!gpu_discovered) {
//         // Try AMD GPU first, then Intel, then NVIDIA
//         gpu_hwmon_dirs = find_hwmon_dirs_for_device("amdgpu", enable_logging);
//         if (gpu_hwmon_dirs.empty())
//             gpu_hwmon_dirs = find_hwmon_dirs_for_device("i915", enable_logging);
//         if (gpu_hwmon_dirs.empty())
//             gpu_hwmon_dirs = find_hwmon_dirs_for_device("nouveau", enable_logging);
//         gpu_discovered = true;
//     }
//
//     if (gpu_hwmon_dirs.empty()) return NAN;
//
//     // Read temp1_input (most GPU drivers use this for edge/junction temp)
//     std::string path = gpu_hwmon_dirs[0] + "/temp1_input";
//     std::ifstream file(path);
//     float raw;
//     if (file.is_open() && (file >> raw)) {
//         return raw / 1000.0f;
//     }
//     return NAN;
// }
//
// Example skeleton for Battery temperature:
//
// static float get_battery_temperature(bool enable_logging = true) {
//     // Battery temp is typically at /sys/class/power_supply/BAT0/temp
//     const char* paths[] = {
//         "/sys/class/power_supply/BAT0/temp",
//         "/sys/class/power_supply/BAT1/temp",
//         "/sys/class/power_supply/battery/temp",
//     };
//     for (const char* path : paths) {
//         std::ifstream file(path);
//         float raw;
//         if (file.is_open() && (file >> raw)) {
//             return raw / 10.0f;  // power_supply uses 0.1°C units
//         }
//     }
//     return NAN;
// }

// =============================================================================
// CPU Usage (dynamic labels)
// =============================================================================
int Thermal::thermal_get_cpu_usages(CpuUsage *list)
{
    int vals, cpu_num, i, j;
    bool online;
    ssize_t read;
    unsigned long long user, nice, system, idle, active, total;
    char *line = NULL;
    size_t len = 0;
    size_t size = 0;
    FILE *file;
    FILE *cpu_file;

    if (list == NULL) {
        return CPU_NUM_MAX;
    }

    // Ensure CPU labels are initialized (safety check)
    if (S_CPU_LABELS.empty()) {
        init_cpu_labels(get_nprocs());
    }

    file = fopen(CPU_USAGE_FILE, "r");
    if (file == NULL) {
        ALOGE("%s: failed to open: %s", __func__, strerror(errno));
        return -errno;
    }

    // Read online CPU information.
    cpu_file = fopen(CPU_ONLINE_FILE, "r");
    if (cpu_file == NULL) {
        ALOGE("%s: failed to open file: %s (%s)", __func__, CPU_ONLINE_FILE, strerror(errno));
        fclose(file);
        return -errno;
    }

    if (2 != fscanf(cpu_file, "%d-%d", &i, &j)) {
        ALOGE("%s: failed to read CPU online information from file: %s (%s)", __func__,
                CPU_ONLINE_FILE, strerror(errno));
        fclose(cpu_file);
        fclose(file);
        return errno ? -errno : -EIO;
    }

    while ((read = getline(&line, &len, file)) != -1) {
        // Skip non "cpu[0-9]" lines.
        if (strnlen(line, read) < 4 || strncmp(line, "cpu", 3) != 0 || !isdigit(line[3])) {
            continue;
        }

        vals = sscanf(line, "cpu%d %llu %llu %llu %llu", &cpu_num, &user,
                &nice, &system, &idle);

        if (vals != CPU_USAGE_PARAS_NUM || size == (size_t)CPU_NUM_MAX) {
            if (vals != CPU_USAGE_PARAS_NUM) {
                ALOGE("%s: failed to read CPU information from file: %s", __func__,
                        strerror(errno));
            } else {
                ALOGE("/proc/stat file has incorrect format.");
            }
            free(line);
            fclose(cpu_file);
            fclose(file);
            return errno ? -errno : -EIO;
        }

        active = user + nice + system;
        total = active + idle;

        online = 0;
        if (size >= (size_t)i && size <= (size_t)j) {
            online = 1;
        }

        // FIX: Use dynamic CPU labels instead of fixed 16-element array.
        // S_CPU_LABELS is sized to get_nprocs() and supports any core count.
        if (size < S_CPU_LABELS.size()) {
            list[size] = (CpuUsage) {
                .name = S_CPU_LABELS[size],
                .active = active,
                .total = total,
                .isOnline = online
            };
            size++;
        } else {
            ALOGE("%s: CPU core count (%zu) exceeds label count (%zu)",
                  __func__, size, S_CPU_LABELS.size());
            free(line);
            fclose(cpu_file);
            fclose(file);
            return errno ? -errno : -EIO;
        }

    }

    free(line);
    fclose(cpu_file);
    fclose(file);

    if (size > (size_t)CPU_NUM_MAX) {
        ALOGE("/proc/stat file has incorrect format.");
        return -EIO;
    }
    return (int)size;
}

// =============================================================================
// VSOCK Communication (unchanged per user request)
// =============================================================================
static int connect_vsock(int *vsock_fd)
{
     struct sockaddr_vm sa = {
        .svm_family = AF_VSOCK,
        .svm_cid = VMADDR_CID_HOST,
        .svm_port = THERMAL_PORT,
    };
    *vsock_fd = socket(AF_VSOCK, SOCK_STREAM, 0);
    if (*vsock_fd < 0) {
            ALOGI("Thermal HAL socket init failed\n");
            return -1;
    }

    if (connect(*vsock_fd, (struct sockaddr*)&sa, sizeof(sa)) != 0) {
            ALOGI("Thermal HAL connect failed\n");
            close(*vsock_fd);
            return -1;
    }
    return 0;
}

static void parse_zone_info(struct zone_info *zone)
{
        switch (zone->type) {
             case 0:
                 kTempThreshold.hotThrottlingThresholds[5] = zone->trip_0 / TEMP_UNIT;
                 kTempThreshold.hotThrottlingThresholds[6] = zone->trip_1 / TEMP_UNIT;
                 kTemp_2_0.value = zone->temperature / TEMP_UNIT;
                 break;
             case 2:
                 kTemp_2_0_1.value = zone->temperature / TEMP_UNIT;
                 break;
             default:
                 break;
        }
}

static void parse_temp_info(struct temp_info *t)
{
    switch(t->type) {
        case 0:
            kTemp_2_0.value = t->temp / TEMP_UNIT;
            break;
        case 2:
            kTemp_2_0_1.value = t->temp / TEMP_UNIT;
            break;
        default:
            break;
    }
}

static int recv_vsock(int *vsock_fd)
{
    char msgbuf[1024];
    int ret;
    struct header *head = (struct header *)malloc(sizeof(struct header));
    if (!head)
        return -ENOMEM;
    memset(msgbuf, 0, sizeof(msgbuf));
    ret = recv(*vsock_fd, msgbuf, sizeof(msgbuf), MSG_DONTWAIT);
    if (ret < 0 && errno == EBADF) {
        if (connect_vsock(vsock_fd) == 0)
            ret = recv(*vsock_fd, msgbuf, sizeof(msgbuf), MSG_DONTWAIT);
    }
    if (ret > 0) {
        memcpy(head, msgbuf, sizeof(struct header));
        int num_zones = 0;
        int ptr = 0;
        int i;
        if (head->notifyid == 1) {
            num_zones = head->length / sizeof(struct zone_info);
            num_zones = num_zones < MAX_ZONES ? num_zones : MAX_ZONES;
            ptr += sizeof(struct header);
            struct zone_info *zinfo = (struct zone_info *)malloc(sizeof(struct zone_info));
            if (!zinfo) {
                free(head);
                return -ENOMEM;
            }
            for (i = 0; i < num_zones; i++) {
                memcpy(zinfo, msgbuf + ptr, sizeof(struct zone_info));
                parse_zone_info(zinfo);
                ptr = ptr + sizeof(struct zone_info);
            }
            free(zinfo);
        } else if (head->notifyid == 2) {
            ptr = sizeof(struct header); //offset of num_zones field
            struct temp_info *tinfo = (struct temp_info *)malloc(sizeof(struct temp_info));
            if (!tinfo) {
                free(head);
                return -ENOMEM;
            }
            memcpy(&num_zones, msgbuf + ptr, sizeof(num_zones));
            num_zones = num_zones < MAX_ZONES ? num_zones : MAX_ZONES;
            ptr += 4; //offset of first zone type info
            for (i = 0; i < num_zones; i++) {
                memcpy(&tinfo->type, msgbuf + ptr, sizeof(tinfo->type));
                memcpy(&tinfo->temp, msgbuf + ptr + sizeof(tinfo->type), sizeof(tinfo->temp));
                parse_temp_info(tinfo);
                ptr += sizeof(tinfo->type) + sizeof(tinfo->temp);
            }
            free(tinfo);
        }
    }
    free(head);
    return 0;
}

// =============================================================================
// Thermal Constructor
// =============================================================================
Thermal::Thermal() {
    mNumCpu = get_nprocs();
    init_cpu_labels(mNumCpu);
    mCheckThread = std::thread(&Thermal::CheckThermalServerity, this);
    mCheckThread.detach();
}

// =============================================================================
// Temperature Monitoring Thread (major rework)
// =============================================================================
void Thermal::CheckThermalServerity() {
    float temp = NAN;
    int res = -1;
    int vsock_fd;
    bool first_iteration_logging_enabled = true;

    ALOGI("Start check temp thread.\n");

    // --- One-time initialization ---
    S_CPU_VENDOR = detect_cpu_vendor();
    initialize_thermal_config();

    // Try VSOCK connection (Celadon VM path)
    if (!connect_vsock(&vsock_fd))
        is_vsock_present = true;

    // Discover best CPU thermal zone as fallback for hwmon
    std::string cpu_zone_path = discover_cpu_thermal_zone(true);
    if (cpu_zone_path.empty()) {
        // Absolute last resort: try thermal_zone0 (may not be CPU!)
        const std::string zone0_path = "/sys/class/thermal/thermal_zone0/temp";
        if (access(zone0_path.c_str(), F_OK) == 0) {
            cpu_zone_path = zone0_path;
            ALOGW("Using thermal_zone0 as last-resort fallback (zone type unknown, may not be CPU)");
        }
    }

    // --- Main monitoring loop ---
    while (1) {
        temp = NAN;
        res = -1;

        if (is_vsock_present) {
            // VSOCK path: temperature data pushed from host VM daemon.
            // parse_zone_info/parse_temp_info update globals directly.
            res = 0;
            recv_vsock(&vsock_fd);
        } else {
            // Bare-metal path: try hwmon first, then thermal zone fallback.

            // Primary: hwmon (coretemp on Intel, k10temp on AMD)
            AllCpuTemperatures temps = get_cpu_temperatures(first_iteration_logging_enabled);
            if (first_iteration_logging_enabled) {
                first_iteration_logging_enabled = false;
            }

            if (!isnan(temps.max_overall_temp)) {
                temp = temps.max_overall_temp;
                res = 0;
            } else if (!cpu_zone_path.empty()) {
                // Fallback: discovered thermal zone
                res = get_soc_pkg_temperature(&temp, cpu_zone_path);
            }

            if (res == 0 && !isnan(temp)) {
                std::lock_guard<std::mutex> _lock(s_temp_data_mutex);
                kTemp_2_0.value = temp;
            }
        }

        if (res) {
            ALOGE("Can not get temperature of type %d", kTemp_1_0.type);
        } else {
            // --- Check CPU throttling severity ---
            // FIX: Added break after first match so we get the HIGHEST severity,
            // not the lowest. Original code kept iterating and overwrote with
            // progressively lower severities.
            // FIX: Reset throttlingStatus each cycle so it clears when temp drops.
            bool cpu_throttled = false;
            {
                std::lock_guard<std::mutex> _lock(s_temp_data_mutex);
                kTemp_2_0.throttlingStatus = ThrottlingSeverity::NONE;
                for (size_t i = kTempThreshold.hotThrottlingThresholds.size() - 1; i > 0; i--) {
                    if (kTemp_2_0.value >= kTempThreshold.hotThrottlingThresholds[i]) {
                        ALOGI("CheckThermalServerity: CPU hit ThrottlingSeverity %s, temperature is %f",
                              THROTTLING_SEVERITY_LABEL[i], kTemp_2_0.value);
                        kTemp_2_0.throttlingStatus = (ThrottlingSeverity)i;
                        cpu_throttled = true;
                        break;
                    }
                }
            }
            if (cpu_throttled) {
                std::lock_guard<std::mutex> _lock(thermal_callback_mutex_);
                for (auto cb : callbacks_) {
                    cb.callback->notifyThrottling(kTemp_2_0);
                }
            }

            // --- Check Battery throttling severity ---
            bool battery_throttled = false;
            {
                std::lock_guard<std::mutex> _lock(s_temp_data_mutex);
                kTemp_2_0_1.throttlingStatus = ThrottlingSeverity::NONE;
                for (size_t i = kTempThreshold_1.hotThrottlingThresholds.size() - 1; i > 0; i--) {
                    if (kTemp_2_0_1.value >= kTempThreshold_1.hotThrottlingThresholds[i]) {
                        ALOGI("CheckThermalServerity: Battery hit ThrottlingSeverity %s, temperature is %f",
                              THROTTLING_SEVERITY_LABEL[i], kTemp_2_0_1.value);
                        kTemp_2_0_1.throttlingStatus = (ThrottlingSeverity)i;
                        battery_throttled = true;
                        break;
                    }
                }
            }
            if (battery_throttled) {
                std::lock_guard<std::mutex> _lock(thermal_callback_mutex_);
                for (auto cb : callbacks_) {
                    cb.callback->notifyThrottling(kTemp_2_0_1);
                }
            }
        }
        sleep(1);
    }
}

// =============================================================================
// V1.0 IThermal Methods
// =============================================================================
Return<void> Thermal::getTemperatures(getTemperatures_cb _hidl_cb) {
    ThermalStatus status;
    std::vector<Temperature_1_0> temperatures = {kTemp_1_0};

    status.code = ThermalStatusCode::SUCCESS;
    if (!is_vsock_present) {
        // FIX: Use the value maintained by the check thread instead of
        // independently re-reading sysfs. This ensures consistency and
        // works with all temperature sources (hwmon, thermal zone, etc.)
        std::lock_guard<std::mutex> _lock(s_temp_data_mutex);
        temperatures[0].currentValue = kTemp_2_0.value;
    }
    _hidl_cb(status, temperatures);
    return Void();
}

Return<void> Thermal::getCpuUsages(getCpuUsages_cb _hidl_cb) {
    ThermalStatus status;
    hidl_vec<CpuUsage> cpuUsages;
    int res = 0;

    status.code = ThermalStatusCode::SUCCESS;
    cpuUsages.resize(CPU_NUM_MAX);
    res = thermal_get_cpu_usages(cpuUsages.data());
    if (res <= 0) {
        status.code = ThermalStatusCode::FAILURE;
        status.debugMessage = strerror(-res);
    }
    _hidl_cb(status, cpuUsages);
    return Void();
}

Return<void> Thermal::getCoolingDevices(getCoolingDevices_cb _hidl_cb) {
    ThermalStatus status;
    status.code = ThermalStatusCode::SUCCESS;
    std::vector<CoolingDevice_1_0> cooling_devices = {kCooling_1_0};
    _hidl_cb(status, cooling_devices);
    return Void();
}

// =============================================================================
// V2.0 IThermal Methods
// =============================================================================
Return<void> Thermal::getCurrentTemperatures(bool filterType, TemperatureType type,
                                             getCurrentTemperatures_cb _hidl_cb) {
    ThermalStatus status;
    status.code = ThermalStatusCode::SUCCESS;
    std::vector<Temperature_2_0> temperatures;

    if (!is_vsock_present) {
        // FIX: Use the value maintained by the check thread instead of
        // independently re-reading sysfs. This works with all backends
        // (coretemp, k10temp, thermal zones) and is thread-safe.
        if (filterType && type != kTemp_2_0.type) {
            // Not CPU type — fall through to VTS workaround below
            // EXTENSION POINT #7a: Add filter matches for new sensor types
            // Example:
            // if (type == kGpuTemp.type) {
            //     std::lock_guard<std::mutex> _lock(s_temp_data_mutex);
            //     temperatures = {kGpuTemp};
            // }
        } else {
            std::lock_guard<std::mutex> _lock(s_temp_data_mutex);
            temperatures = {kTemp_2_0};
        }
    } else {
        std::lock_guard<std::mutex> _lock(s_temp_data_mutex);
        if (!filterType) {
            // No filter type. Send all temperatures.
            // Workaround for VTS which expects temperatures in order:
            // GPU temp is not exposed in Intel Android platforms.
            // So add the dummy entry for GPU temp.
            temperatures = {kTemp_2_0, kDummyTemp, kTemp_2_0_1};
            // EXTENSION POINT #7b: When adding real GPU/Battery sensors,
            // replace kDummyTemp with the real global (e.g., kGpuTemp)
            // and add any additional sensor types to this vector:
            //   temperatures = {kTemp_2_0, kGpuTemp, kTemp_2_0_1};
            for (size_t i = kTempThreshold.hotThrottlingThresholds.size() - 1; i > 0; i--) {
                if (temperatures[0].value >= kTempThreshold.hotThrottlingThresholds[i]) {
                    temperatures[0].throttlingStatus = (ThrottlingSeverity)i;
                    break;
                }
            }
            // FIX: Check temperatures[2] (battery), not temperatures[1] (GPU dummy).
            // Original code checked the GPU dummy against battery thresholds.
            for (size_t i = kTempThreshold_1.hotThrottlingThresholds.size() - 1; i > 0; i--) {
                if (temperatures[2].value >= kTempThreshold_1.hotThrottlingThresholds[i]) {
                    temperatures[2].throttlingStatus = (ThrottlingSeverity)i;
                    break;
                }
            }
        } else if (type == kTemp_2_0.type) {
            temperatures = {kTemp_2_0};
            for (size_t i = kTempThreshold.hotThrottlingThresholds.size() - 1; i > 0; i--) {
                if (temperatures[0].value >= kTempThreshold.hotThrottlingThresholds[i]) {
                    temperatures[0].throttlingStatus = (ThrottlingSeverity)i;
                    break;
                }
            }
        } else if (type == kTemp_2_0_1.type) {
            temperatures = {kTemp_2_0_1};
            for (size_t i = kTempThreshold_1.hotThrottlingThresholds.size() - 1; i > 0; i--) {
                if (temperatures[0].value >= kTempThreshold_1.hotThrottlingThresholds[i]) {
                    temperatures[0].throttlingStatus = (ThrottlingSeverity)i;
                    break;
                }
            }
        }
        // EXTENSION POINT #7c: Add else-if for new sensor types in VSOCK path
        // Example:
        // else if (type == kGpuTemp.type) {
        //     temperatures = {kGpuTemp};
        // }
    }

    // Workaround for VTS Test
    if (filterType && temperatures.size() == 0) {
        temperatures = {(Temperature_2_0) {
                            .type = type,
                            .name = "FAKE_DATA",
                            .value = 0,
                            .throttlingStatus = ThrottlingSeverity::NONE}};
    }
    _hidl_cb(status, temperatures);
    return Void();
}

Return<void> Thermal::getTemperatureThresholds(bool filterType, TemperatureType type,
                                               getTemperatureThresholds_cb _hidl_cb) {
    ThermalStatus status;
    status.code = ThermalStatusCode::SUCCESS;
    std::vector<TemperatureThreshold> temperature_thresholds;

    // FIX: Protect threshold reads with mutex (thresholds may be updated
    // by initialize_thermal_config or parse_zone_info from check thread).
    std::lock_guard<std::mutex> _lock(s_temp_data_mutex);

    if (!is_vsock_present) {
        if (filterType && type != kTempThreshold.type) {
            // Workaround for VTS test
            // EXTENSION POINT #7d: Add threshold filter matches for new types
            // Example:
            // if (type == kGpuTempThreshold.type)
            //     temperature_thresholds = {kGpuTempThreshold};
        } else {
            temperature_thresholds = {kTempThreshold};
        }
    } else if (filterType) {
        if (type == kTempThreshold.type)
            temperature_thresholds = {kTempThreshold};
        else if (type == kTempThreshold_1.type)
            temperature_thresholds = {kTempThreshold_1};
        // EXTENSION POINT #7e: Add threshold filter for new sensor types
        // Example:
        // else if (type == kGpuTempThreshold.type)
        //     temperature_thresholds = {kGpuTempThreshold};
    } else {
        temperature_thresholds = {kTempThreshold, kDummyTempThreshold, kTempThreshold_1};
        // EXTENSION POINT #7f: Replace kDummyTempThreshold with real GPU thresholds
        // and add any new sensor type thresholds to this vector.
    }

    //Workaround for VTS.
    if (filterType && temperature_thresholds.size() == 0) {
        temperature_thresholds = {(TemperatureThreshold) {
            .type = type,
            .name = "FAKE DATA",
            .hotThrottlingThresholds = {{NAN, NAN, NAN, NAN, NAN, 90, 100}},
            .coldThrottlingThresholds = {{NAN, NAN, NAN, NAN, NAN, NAN, NAN}},
            .vrThrottlingThreshold = NAN,}};
    }
    _hidl_cb(status, temperature_thresholds);
    return Void();
}

Return<void> Thermal::getCurrentCoolingDevices(bool filterType, CoolingType type,
                                               getCurrentCoolingDevices_cb _hidl_cb) {
    ThermalStatus status;
    status.code = ThermalStatusCode::SUCCESS;
    std::vector<CoolingDevice_2_0> cooling_devices;

    if (filterType && type != kCooling_2_0.type) {
        // Workaround for VTS Test
        //status.code = ThermalStatusCode::FAILURE;
        //status.debugMessage = "Failed to read data";
    } else {
        cooling_devices = {kCooling_2_0};
    }
    // Workaround for VTS Test
    if (filterType && cooling_devices.size() == 0) {
        cooling_devices = {(CoolingDevice_2_0) {
                            .type = type,
                            .name = "FAKE_DATA",
                            .value = 0}};
    }

    _hidl_cb(status, cooling_devices);
    return Void();
}

Return<void> Thermal::registerThermalChangedCallback(const sp<IThermalChangedCallback>& callback,
                                                     bool filterType, TemperatureType type,
                                                     registerThermalChangedCallback_cb _hidl_cb) {
    ThermalStatus status;
    if (callback == nullptr) {
        status.code = ThermalStatusCode::FAILURE;
        status.debugMessage = "Invalid nullptr callback";
        LOG(ERROR) << status.debugMessage;
        _hidl_cb(status);
        return Void();
    } else {
        status.code = ThermalStatusCode::SUCCESS;
    }
    std::lock_guard<std::mutex> _lock(thermal_callback_mutex_);
    if (std::any_of(callbacks_.begin(), callbacks_.end(), [&](const CallbackSetting& c) {
            return interfacesEqual(c.callback, callback);
        })) {
        status.code = ThermalStatusCode::FAILURE;
        status.debugMessage = "Same callback interface registered already";
        LOG(ERROR) << status.debugMessage;
    } else {
        callbacks_.emplace_back(callback, filterType, type);
        LOG(INFO) << "A callback has been registered to ThermalHAL, isFilter: " << filterType
                  << " Type: " << android::hardware::thermal::V2_0::toString(type);
    }
    _hidl_cb(status);
    return Void();
}

Return<void> Thermal::unregisterThermalChangedCallback(
    const sp<IThermalChangedCallback>& callback, unregisterThermalChangedCallback_cb _hidl_cb) {
    ThermalStatus status;
    if (callback == nullptr) {
        status.code = ThermalStatusCode::FAILURE;
        status.debugMessage = "Invalid nullptr callback";
        LOG(ERROR) << status.debugMessage;
        _hidl_cb(status);
        return Void();
    } else {
        status.code = ThermalStatusCode::SUCCESS;
    }
    bool removed = false;
    std::lock_guard<std::mutex> _lock(thermal_callback_mutex_);
    callbacks_.erase(
        std::remove_if(callbacks_.begin(), callbacks_.end(),
                       [&](const CallbackSetting& c) {
                           if (interfacesEqual(c.callback, callback)) {
                               LOG(INFO)
                                   << "A callback has been unregistered from ThermalHAL, isFilter: "
                                   << c.is_filter_type << " Type: "
                                   << android::hardware::thermal::V2_0::toString(c.type);
                               removed = true;
                               return true;
                           }
                           return false;
                       }),
        callbacks_.end());
    if (!removed) {
        status.code = ThermalStatusCode::FAILURE;
        status.debugMessage = "The callback was not registered before";
        LOG(ERROR) << status.debugMessage;
    }
    _hidl_cb(status);
    return Void();
}

}  // namespace implementation
}  // namespace V2_0
}  // namespace thermal
}  // namespace hardware
}  // namespace android
