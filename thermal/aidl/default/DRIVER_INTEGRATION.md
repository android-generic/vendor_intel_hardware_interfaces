# Driver Integration Guide — AIDL Thermal HAL

This guide explains how to add support for new hardware drivers (hwmon devices,
thermal zones) to the Android Thermal HAL. This applies to CPU, GPU, battery,
and any other thermal sensor type.

## Background

### How Linux exposes thermal data

Linux exposes hardware sensors via two main subsystems:

| Subsystem | Path | Best for |
|---|---|---|
| **hwmon** | `/sys/class/hwmon/hwmonN/` | Per-sensor temperature, voltage, current, fan speeds |
| **thermal_zone** | `/sys/class/thermal/thermal_zoneN/` | System-level thermal management, trip points |

Most drivers expose an hwmon device. Each hwmon device has:
- `name` — the driver name (e.g., `coretemp`, `k10temp`, `amdgpu`)
- `temp1_input`, `temp2_input`, ... — temperature readings in millidegrees C
- `temp1_label`, `temp2_label`, ... — human-readable labels for each sensor

### Finding your driver's sensors

```bash
# List all hwmon devices and their names
for d in /sys/class/hwmon/hwmon*; do
    echo "=== $d ==="
    cat "$d/name" 2>/dev/null
    cat "$d/temp"*_label 2>/dev/null
    cat "$d/temp"*_input 2>/dev/null
done

# List all thermal zones
for d in /sys/class/thermal/thermal_zone*; do
    echo "=== $d ==="
    cat "$d/type"
    cat "$d/temp" 2>/dev/null
done
```

### Known hwmon driver examples

| Driver | hwmon `name` | Sensor labels | Type |
|---|---|---|---|
| Intel CPU | `coretemp` | `Core 0`, `Core 1`, `Package id 0` | CPU |
| AMD CPU (kernel) | `k10temp` | `Tdie`, `Tctl`, `Tccd1`-`Tccd8` | CPU |
| AMD CPU (zenpower) | `zenpower` | `Tdie`/`Tctl`/`TccdN` or `cpu0 Tdie`/etc. | CPU |
| ThinkPad | `thinkpad` | `CPU`, `GPU`, numbered temps | CPU/GPU |
| AMD GPU | `amdgpu` | `edge`, `junction`, `mem` | GPU |
| Intel GPU | `i915` | `temp1` (often no label) | GPU |
| Steam Deck | `steamdeck-hwmon` | (check device) | APU/GPU |
| OneXPlayer | `oxp-sensors` | (check device) | Device-specific |

---

## Step-by-Step Integration

Search for `EXTENSION POINT` markers in `Thermal.cpp` — they are numbered
#1 through #7 to match these steps.

### Step 1: Add Temperature/Threshold globals

**Location**: `EXTENSION POINT #1` in `Thermal.cpp`

For each new sensor type, add a `Temperature` global and a `TemperatureThreshold`
global. These hold the latest reading and the throttling thresholds.

```cpp
// Example: GPU temperature
Temperature kGpuTemp;
TemperatureThreshold kGpuTempThreshold;
```

Initialize them in `CheckThermalServerity()` setup (after the CPU init block):

```cpp
kGpuTemp.type = TemperatureType::GPU;
kGpuTemp.name = "TGPU";
kGpuTemp.value = 25;
kGpuTemp.throttlingStatus = ThrottlingSeverity::NONE;

kGpuTempThreshold.type = TemperatureType::GPU;
kGpuTempThreshold.name = "TGPU";
kGpuTempThreshold.hotThrottlingThresholds = {{NAN, NAN, NAN, NAN, NAN, 90, 100}};
kGpuTempThreshold.coldThrottlingThresholds = {{NAN, NAN, NAN, NAN, NAN, NAN, NAN}};
```

Don't forget to add them to the `s_temp_data_mutex` protection scope.

### Step 2a: Add thermal zone types (optional)

**Location**: `EXTENSION POINT #2` in `discover_cpu_thermal_zone()`

If your driver also exposes a thermal zone, add its type string and a priority:

```cpp
static const std::vector<ZoneTypePriority> known_cpu_zones = {
    {"x86_pkg_temp", 1},
    {"k10temp",      2},
    {"acpitz",       10},
    {"thinkpad",     3},    // <-- NEW
};
```

### Step 2b: Add hwmon driver names

**Location**: `EXTENSION POINT #3` in `get_cpu_temperatures()`

Add your driver's hwmon name to the discovery chain:

```cpp
// After the zenpower check:
if (S_CACHED_HWMON_DIRS.empty()) {
    S_CACHED_HWMON_DIRS = find_hwmon_dirs_for_device("thinkpad", enable_logging);
    if (!S_CACHED_HWMON_DIRS.empty()) {
        if (enable_logging) ALOGI("Using thinkpad_acpi hwmon for temperature sensing");
    }
}
```

**For non-CPU sensors** (GPU, battery), don't add them to `get_cpu_temperatures()`.
Instead, create a separate discovery function (see Step 5).

### Step 3: Add label regex patterns

**Location**: `EXTENSION POINT #4` in `get_cpu_temperatures()`

Add regex patterns to match your driver's sensor labels:

```cpp
// ThinkPad example
std::regex thinkpad_cpu_regex("^CPU$");
std::regex thinkpad_gpu_regex("^GPU$");

// amdgpu example
std::regex amdgpu_edge_regex("^edge$");
std::regex amdgpu_junction_regex("^junction$");
std::regex amdgpu_mem_regex("^mem$");
```

### Step 4: Add label matching branches

**Location**: `EXTENSION POINT #5` in the label matching loop

Add `else if` branches to match your labels and route to the correct sensor type:

```cpp
// CPU sensor from new driver
} else if (std::regex_match(label_content, thinkpad_cpu_regex)) {
    current_sensor_info.is_package_sensor = true;
    all_temps.package_temperature = temp_celsius;
    all_temps.package_label = label_content;
    S_CACHED_SENSORS.push_back(current_sensor_info);

// GPU sensor (updates separate global)
} else if (std::regex_match(label_content, amdgpu_edge_regex)) {
    std::lock_guard<std::mutex> _lock(s_temp_data_mutex);
    kGpuTemp.value = temp_celsius;
}
```

### Step 5: Create non-CPU discovery functions (GPU, Battery)

**Location**: `EXTENSION POINT #6`

For GPU/Battery, create separate functions:

```cpp
static float get_gpu_temperature(bool enable_logging = true) {
    // Try AMD GPU
    auto dirs = find_hwmon_dirs_for_device("amdgpu", enable_logging);
    if (dirs.empty()) {
        // Try NVIDIA
        dirs = find_hwmon_dirs_for_device("nouveau", enable_logging);
    }
    if (dirs.empty()) return NAN;

    // Most GPU drivers use temp1_input for edge/junction temp
    std::string temp_path = dirs[0] + "/temp1_input";
    std::ifstream file(temp_path);
    float raw;
    if (file.is_open() && (file >> raw)) {
        return raw / 1000.0f;
    }
    return NAN;
}

static float get_battery_temperature(bool enable_logging = true) {
    // Battery temp is in /sys/class/power_supply/BAT*/temp
    // (in tenths of a degree: 250 = 25.0°C)
    const std::string ps_base = "/sys/class/power_supply/";
    DIR* dir = opendir(ps_base.c_str());
    if (!dir) return NAN;

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string name = entry->d_name;
        if (name.find("BAT") != 0) continue;

        std::string temp_path = ps_base + name + "/temp";
        std::ifstream file(temp_path);
        float raw;
        if (file.is_open() && (file >> raw)) {
            closedir(dir);
            return raw / 10.0f;  // tenths of degree to degrees
        }
    }
    closedir(dir);
    return NAN;
}
```

Call these from the `CheckThermalServerity()` monitoring loop:

```cpp
// In the monitoring loop, after CPU temp update:
float gpu_temp = get_gpu_temperature(first_iteration_logging_enabled);
if (!isnan(gpu_temp)) {
    std::lock_guard<std::mutex> _lock(s_temp_data_mutex);
    kGpuTemp.value = gpu_temp;
}
```

### Step 6: Return new sensors in getter methods

**Location**: `EXTENSION POINT #7a` and `#7b`

Add your new sensors to the return vectors in `fillTemperatures()` and
`getTemperaturesWithType()`:

```cpp
// In fillTemperatures():
ret.emplace_back(kGpuTemp);
ret.emplace_back(kBatteryTemp);

// In getTemperaturesWithType():
if (in_type == TemperatureType::GPU) {
    out_temperatures->emplace_back(kGpuTemp);
}
if (in_type == TemperatureType::BATTERY) {
    out_temperatures->emplace_back(kBatteryTemp);
}
```

Also update `getTemperatureThresholds()` and `getTemperatureThresholdsWithType()`:

```cpp
ret.emplace_back(kGpuTempThreshold);

if (in_type == TemperatureType::GPU) {
    out_temperatureThresholds->emplace_back(kGpuTempThreshold);
}
```

### Step 7: Add throttle checking (optional)

In `CheckThermalServerity()`, add throttle severity checking for the new sensor,
following the same pattern as CPU:

```cpp
// After CPU throttle check:
{
    std::lock_guard<std::mutex> _lock(s_temp_data_mutex);
    kGpuTemp.throttlingStatus = ThrottlingSeverity::NONE;
    for (size_t i = kGpuTempThreshold.hotThrottlingThresholds.size() - 1; i > 0; i--) {
        if (kGpuTemp.value >= kGpuTempThreshold.hotThrottlingThresholds[i]) {
            kGpuTemp.throttlingStatus = (ThrottlingSeverity)i;
            // notify callbacks for GPU throttling
            break;
        }
    }
}
```

---

## Checklist

- [ ] Identified driver's hwmon name (`cat /sys/class/hwmon/hwmon*/name`)
- [ ] Identified sensor labels (`cat /sys/class/hwmon/hwmon*/temp*_label`)
- [ ] Added Temperature + TemperatureThreshold globals (#1)
- [ ] Added thermal zone type if applicable (#2)
- [ ] Added hwmon name to discovery chain (#3)
- [ ] Added label regex patterns (#4)
- [ ] Added label matching branch (#5)
- [ ] Created discovery function for non-CPU sensors (#6)
- [ ] Added to getter return vectors (#7)
- [ ] Updated DRIVER REGISTRY comment at top of file
- [ ] Tested on target hardware

## Temperature Unit Reference

| Source | Unit | Conversion |
|---|---|---|
| hwmon `temp*_input` | millidegrees C | `/ 1000.0` |
| thermal_zone `temp` | millidegrees C | `/ 1000.0` |
| power_supply `temp` | tenths of degree C | `/ 10.0` |
| This HAL's Temperature.value | degrees C | (already converted) |
