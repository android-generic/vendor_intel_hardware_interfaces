# Driver Integration Guide — HIDL Thermal HAL (V2.0)

This guide explains how to add support for new hardware drivers (hwmon devices,
thermal zones) to the HIDL Thermal HAL (V2.0). This applies to CPU, GPU, battery,
and any other thermal sensor type.

> **Note**: This HAL is used by BlissOS 15/16 (Android 12/13). For Android 14+,
> see the AIDL version's `DRIVER_INTEGRATION.md` instead. The extension points
> are numbered identically across both versions.

## Background

### How Linux exposes thermal data

Linux exposes hardware sensors via two main subsystems:

| Subsystem | Path | Best for |
|---|---|---|
| **hwmon** | `/sys/class/hwmon/hwmonN/` | Per-sensor temperature, voltage, current |
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

For each new sensor type, add a `Temperature_2_0` global and a
`TemperatureThreshold` global:

```cpp
// Example: replacing the dummy GPU with a real one
static Temperature_2_0 kGpuTemp = {
    .type = TemperatureType::GPU,
    .name = "TGPU",
    .value = 25,
    .throttlingStatus = ThrottlingSeverity::NONE,
};
static TemperatureThreshold kGpuTempThreshold = {
    .type = TemperatureType::GPU,
    .name = "TGPU",
    .hotThrottlingThresholds = {{NAN, NAN, NAN, NAN, NAN, 90, 100}},
    .coldThrottlingThresholds = {{NAN, NAN, NAN, NAN, NAN, NAN, NAN}},
    .vrThrottlingThreshold = NAN,
};
```

These globals must be accessed under `s_temp_data_mutex`.

### Step 2a: Add thermal zone types (optional)

**Location**: `EXTENSION POINT #2` in `discover_cpu_thermal_zone()`

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

```cpp
// After the zenpower check:
if (S_CACHED_HWMON_DIRS.empty()) {
    S_CACHED_HWMON_DIRS = find_hwmon_dirs_for_device("thinkpad", enable_logging);
}
```

For non-CPU sensors (GPU, battery), create a separate function (Step 5).

### Step 3: Add label regex patterns

**Location**: `EXTENSION POINT #4` in `get_cpu_temperatures()`

```cpp
std::regex thinkpad_cpu_regex("^CPU$");
std::regex amdgpu_edge_regex("^edge$");
```

### Step 4: Add label matching branches

**Location**: `EXTENSION POINT #5` in the label matching loop

```cpp
} else if (std::regex_match(label_content, thinkpad_cpu_regex)) {
    current_sensor_info.is_package_sensor = true;
    all_temps.package_temperature = temp_celsius;
    all_temps.package_label = label_content;
    S_CACHED_SENSORS.push_back(current_sensor_info);
}
```

### Step 5: Create non-CPU discovery functions

**Location**: `EXTENSION POINT #6`

```cpp
static float get_gpu_temperature(bool enable_logging = true) {
    auto dirs = find_hwmon_dirs_for_device("amdgpu", enable_logging);
    if (dirs.empty()) dirs = find_hwmon_dirs_for_device("nouveau", enable_logging);
    if (dirs.empty()) return NAN;

    std::string temp_path = dirs[0] + "/temp1_input";
    std::ifstream file(temp_path);
    float raw;
    if (file.is_open() && (file >> raw)) {
        return raw / 1000.0f;
    }
    return NAN;
}
```

### Step 6: Return new sensors in getter methods

**Location**: `EXTENSION POINT #7`

The HIDL HAL has three temperature getters:
- `getTemperatures()` — V1.0 compatibility, returns `Temperature_1_0`
- `getCurrentTemperatures()` — V2.0, returns `Temperature_2_0`
- `getTemperatureThresholds()` — V2.0, returns thresholds

Add your sensors to the return vectors in each method.

### Step 7: Add throttle checking

In `CheckThermalServerity()`, add throttle checking after the CPU check:

```cpp
{
    std::lock_guard<std::mutex> _lock(s_temp_data_mutex);
    kGpuTemp.throttlingStatus = ThrottlingSeverity::NONE;
    for (size_t i = kGpuTempThreshold.hotThrottlingThresholds.size() - 1; i > 0; i--) {
        if (kGpuTemp.value >= kGpuTempThreshold.hotThrottlingThresholds[i]) {
            kGpuTemp.throttlingStatus = (ThrottlingSeverity)i;
            break;
        }
    }
}
```

---

## Differences from the AIDL version

| Aspect | HIDL (this file) | AIDL |
|---|---|---|
| Temperature type | `Temperature_2_0` | `Temperature` |
| Threshold type | `TemperatureThreshold` | `TemperatureThreshold` |
| V1.0 compat | Has `getTemperatures()` returning `Temperature_1_0` | No V1.0 |
| Callback notify | `cb.callback->notifyThrottling(temp)` | `cb.callback->notifyThrottling(temp)` |
| Battery temp | Has `kTemp_2_0_1` (from VSOCK) | Not present |
| GPU dummy | Has `kDummyTemp` for VTS | Not present |

---

## Checklist

- [ ] Identified driver's hwmon name
- [ ] Identified sensor labels
- [ ] Added Temperature_2_0 + TemperatureThreshold globals (#1)
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
| HAL's Temperature.value | degrees C | (already converted) |
