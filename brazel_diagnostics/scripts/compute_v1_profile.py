#!/usr/bin/env python3
"""
# compute_v1_profile.py

CARLA + ROS2 performance profiler for compute-v1 (9600X, 9070XT 16GB, 32GB DDR5)

Captures 
- GPU (MangoHud frametime/FPS/GPU% + DRM sysfs VRAM)
- CPU (overall utilization, per-core load, load averages)
- System RAM
- CARLA per-process CPU/RSS

---

## Requirements
1. MangoHud config at brazel_diagnostics/perf/mangohud_brazel_bench.conf

---

## Usage

Terminal 1 - launch CARLA with MangoHud:

```bash
MANGOHUD_CONFIGFILE=brazel_diagnostics/perf/mangohud_brazel_bench.conf \
MANGOHUD=1 MANGOHUD_LOG=1 \
./CarlaUE4.sh --ros2
```

Terminal 2 - once CARLA is steady, run the profiler

```bash
python3 compute_v1_profile.py --config empty_world --duration 60
```

### `--config` (freeform tag)

1. `empty_world`: Empty world with nothing spawned.
2. `vehicle_v1`: World with the `vehicle_v1.yaml` spawned.

---

## Outputs Per Run

```
brazel_diagnostics/perf/profiles/<connfig>_mangohud.csv
brazel_diagnostics/perf/profiles/<connfig>_summary.json
```
"""

import argparse
import csv
import json
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

# --- sysfs paths ---

DRM_BASE = "/sys/class/drm"
GPU_BUSY_PCT = "device/gpu_busy_percent"
VRAM_USED = "device/mem_info_vram_used"
VRAM_TOTAL = "device/mem_info_vram_total"

PROC_STAT = "/proc/stat"
PROC_LOADAVG = "/proc/loadavg"
PROC_MEMINFO = "/proc/meminfo"

# --- helpers ---

def find_amd_card():
    """Return the first AMD card path under /sys/class/drm."""
    for entry in sorted(Path(DRM_BASE).iterdir()):
        if not entry.name.startswith("card"):
            continue
        vendor_path = entry / "device/vendor"
        if vendor_path.exists():
            vendor = vendor_path.read_text().strip()
            if vendor == "0x1002":   # AMD
                return entry
    raise FileNotFoundError("No AMD GPU found in /sys/class/drm")

def read_sysfs(path):
    """Read int from sysfs; None on failure."""
    try:
        return int(Path(path).read_text().strip())
    except (OSError, ValueError):
        return None
    
def read_file(path):
    """Read raw text from a file; '' on failure."""
    try:
        return Path(path).read_text()
    except OSError:
        return ""
    
def find_carla_pid():
    """Return the PID of the first CARLA UE4 process, or None."""
    try:
        raw = subprocess.check_output(
            ["pgrep", "-f", "CarlaUE4"], text=True, timeout=5
        ).strip()
        if raw:
            return int(raw.splitlines()[0])
    except (subprocess.CalledProcessError, ValueError, subprocess.TimeoutExpired):
        pass
    return None

def read_proc_pid_stat(pid):
    """Return (utime, stime, rss_pages) from /proc/<pid>/stat, or (None, None, None)."""
    try:
        fields = Path(f"/proc/{pid}/stat").read_text().split()
        # fields[13]=utime, [14]=stime, [23]=rss  (1-indexed)
        utime    = int(fields[13])
        stime    = int(fields[14])
        rss      = int(fields[23])          # pages
        return utime, stime, rss
    except (OSError, IndexError, ValueError):
        return None, None, None
    
def read_cpu_times():
    """Return list of CPU lines from /proc/stat (first is aggregate, rest per-core)."""
    lines = read_file(PROC_STAT).splitlines()
    cpu_lines = [l for l in lines if l.startswith("cpu")]
    return cpu_lines

def cpu_percent(prev, cur):
    """Compute CPU utilisation % between two ticks dicts {cpuN: [idle, total]}."""
    result = {}
    for cpu in cur:
        if cpu not in prev:
            continue
        idle_delta  = cur[cpu][0] - prev[cpu][0]
        total_delta = cur[cpu][1] - prev[cpu][1]
        if total_delta > 0:
            result[cpu] = round(100.0 * (1.0 - idle_delta / total_delta), 2)
        else:
            result[cpu] = 0.0
    return result

def parse_cpu_line(line):
    """Return (cpu_name, idle_ticks, total_ticks) from a /proc/stat cpu line."""
    parts = line.split()
    name  = parts[0]                     # "cpu" or "cpu0", "cpu1" ...
    vals  = list(map(int, parts[1:]))
    # idle = idle + iowait
    idle  = vals[3] + vals[4]            # idle + iowait
    total = sum(vals)
    return name, idle, total

def read_meminfo():
    """Return dict of key→kB from /proc/meminfo."""
    mem = {}
    for line in read_file(PROC_MEMINFO).splitlines():
        if ":" in line:
            key, rest = line.split(":", 1)
            val = rest.strip().split()[0]
            try:
                mem[key.strip()] = int(val)
            except ValueError:
                pass
    return mem

def read_loadavg():
    """Return (load1, load5, load15) or (None, None, None)."""
    try:
        parts = read_file(PROC_LOADAVG).split()
        return float(parts[0]), float(parts[1]), float(parts[2])
    except (IndexError, ValueError):
        return None, None, None

def parse_mangohud_csv(path):
    """Parse MangoHud log CSV into list of dicts (skips comment lines)."""
    rows = []
    with open(path) as f:
        reader = csv.DictReader(row for row in f if not row.startswith("#"))
        for row in reader:
            rows.append(row)
    return rows

def compute_stats(values):
    """min, max, mean, median, p95, p99 for a list of numbers."""
    if not values:
        return {}
    s = sorted(values)
    n = len(s)
    return {
        "min":    round(s[0], 2),
        "max":    round(s[-1], 2),
        "mean":   round(sum(s) / n, 2),
        "median": round(s[n // 2], 2),
        "p95":    round(s[int(n * 0.95)], 2),
        "p99":    round(s[int(n * 0.99)], 2),
    }

def clamp(v, lo=0, hi=100):
    """Clamp a value to [lo, hi]; return None if input is None."""
    if v is None:
        return None
    return max(lo, min(hi, v))

# --- main ---

def main():
    parser = argparse.ArgumentParser(description="compute-v1 CARLA + ROS2 profiler")
    parser.add_argument(
        "--config", 
        required=True, 
        help="Config tag that becomes the output filename prefix",
        )
    parser.add_argument(
        "--duration", 
        type=int, 
        default=60, 
        help="Sample duration in seconds (default 60)",
        )
    parser.add_argument(
        "--output-dir", 
        default="brazel_diagnostics/perf/profiles",
        help="Output directory (default: brazel_diagnostics/perf/profiles)",
    )
    parser.add_argument(
        "--no-mangohud",
        action="store_true",
        help="Skip MangoHud CSV parsing (e.g. CARLA not running)",
    )
    args = parser.parse_args()

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # GPU Discovery
    card = find_amd_card()
    print(f"[profile] AMD GPU: {card}")

    # CARLA PID
    carla_pid = find_carla_pid()
    if carla_pid:
        print(f"[profile] CARLA PID: {carla_pid}")
    else:
        print(f"[profile] CARLA PID not found, per-process stats will be empty")

    # MangoHud CSV path
    