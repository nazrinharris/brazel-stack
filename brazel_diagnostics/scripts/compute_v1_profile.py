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

## Args:

1. `--config` CONFIG : Config tag for this run (becomes filename prefix)
2. `--duration` SECONDS : Sample duration in seconds (default: 60)
3. `--output-dir` PATH : Where profiles land (default: `src/brazel_diagnostics/perf/profiles`)
4. `--no-mangohud : Skip MangoHud CSV parsing (use when CARLA isn't running)

---

## Requirements (MUST DO!)
1. MangoHud config at `brazel_diagnostics/perf/mangohud_brazel_bench.conf`
2. Verify `mangohud_brazel_bench.conf`'s `output_folder` matches the script's `--output-dir` (default: `src/brazel_diagnostics/perf/profiles`)
    - So for example, in the `.conf` file, `output_folder=/mnt/av-storage/brazel_ws/src/brazel_diagnostics/perf/profiles`
3. For the CARLA startup, make sure the absolute path of the MANGOHUD_CONFIGGILE is correct, it's located in `~/brazel_ws/src/brazel_diagnostics/perf/mangohud_brazel_bench.conf`

---

## Usage

> !!! Aside from the CARLA launch, anything else must be run from `~/brazel_ws`!

Terminal 1 - launch CARLA with MangoHud:

Do NOT move the camera in CARLA, this is to make sure the benchmark is reproducible.

```bash
MANGOHUD_CONFIGFILE=/mnt/av-storage/brazel_ws/src/brazel_diagnostics/perf/mangohud_brazel_bench.conf \
MANGOHUD=1 MANGOHUD_LOG=1 \
./CarlaUE4.sh --ros2
```

Terminal 2 - once CARLA is steady, run the profiler

```bash
python3 src/brazel_diagnostics/scripts/compute_v1_profile.py --config empty_world --duration 60
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
        default="src/brazel_diagnostics/perf/profiles",
        help="Output directory (default: src/brazel_diagnostics/perf/profiles)",
    )
    parser.add_argument(
        "--no-mangohud",
        action="store_true",
        help="Skip MangoHud CSV parsing (e.g. CARLA not running)",
    )
    args = parser.parse_args()

     # --- Guard: Must run from brazel_ws root
    if not (Path.cwd() / "src").is_dir():
        print("ERROR: compute_v1_profile.py must be run from the brazel_ws root directory.", file=sys.stderr)
        print(f"Current directory: {Path.cwd()}", file=sys.stderr)
        sys.exit(1)

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # --- GPU Discovery ---
    card = find_amd_card()
    print(f"[profile] AMD GPU: {card}")

    # --- CARLA PID ---
    carla_pid = find_carla_pid()
    if carla_pid:
        print(f"[profile] CARLA PID: {carla_pid}")
    else:
        print(f"[profile] CARLA PID not found, per-process stats will be empty")

    # --- MangoHud CSV path ---
    mangohud_csv = out_dir / f"{args.config}_mangohud.csv"
    print(f"[profile] MangoHud log: {mangohud_csv}")

    # --- Clock Tick (for /proc/stat CPU% calc) ---
    try:
        clock_ticks = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
    except (KeyError, ValueError):
        clock_ticks = 100 # Fallback
    
    # --- Polling Loop ---
    print(f"[profile] Sampling for {args.duration}s, do NOT move/change the main CARLA window camera!")

    samples = []    # list of dicts, one per 0.5s tick

    # prime CPU baseline
    prev_cpu_raw = read_cpu_times()

    # prime CARLA CPU baseline
    carla_prev_utime = carla_prev_stime = None
    if carla_pid:
        carla_prev_utime, carla_prev_stime, _ = read_proc_pid_stat(carla_pid)
    
    start = time.time()

    try:
        while time.time() - start < args.duration:
            tick = time.time()

            # --- GPU sysfs
            gpu_busy = read_sysfs(card / GPU_BUSY_PCT)
            vram_used = read_sysfs(card / VRAM_USED)
            vram_total = read_sysfs(card / VRAM_TOTAL)

            # --- CPU Utilization
            cur_cpu_raw = read_cpu_times()

            prev_map = {}
            for line in prev_cpu_raw:
                name, idle, total = parse_cpu_line(line)
                prev_map[name] = (idle, total)
            cur_map = {}
            for line in cur_cpu_raw:
                name, idle, total = parse_cpu_line(line)
                cur_map[name] = (idle, total)

            cpu_pcts = cpu_percent(prev_map, cur_map)
            prev_cpu_raw = cur_cpu_raw

            # --- CARLA per-process CPU
            carla_cpu_pct = None
            carla_rss_mib = None
            if carla_pid:
                utime, stime, rss = read_proc_pid_stat(carla_pid)
                if None not in (utime, stime, rss, carla_prev_utime, carla_prev_stime):
                    utime_delta  = utime - carla_prev_utime
                    stime_delta  = stime - carla_prev_stime
                    # CPU% since last tick (0.5 s)
                    carla_cpu_pct = clamp(
                        round(100.0 * (utime_delta + stime_delta) / clock_ticks / 0.5, 2)
                    )
                    carla_rss_mib = round(rss * os.sysconf("SC_PAGE_SIZE") / (1024 * 1024), 1)
                carla_prev_utime = utime
                carla_prev_stime = stime
            
            # --- Load averages
            l1, l5, l15 = read_loadavg()

            # --- System RAM
            mem = read_meminfo()

            # --- Sample Parsing
            sample = {
                "ts":              round(tick - start, 1),
                "gpu_busy_pct":    gpu_busy,
                "vram_used_mib":   round(vram_used / (1024 * 1024), 1) if vram_used else None,
                "vram_total_mib":  round(vram_total / (1024 * 1024), 1) if vram_total else None,
                "cpu_pct":         cpu_pcts.get("cpu"),        # aggregate
                "cpu_per_core":    {k: v for k, v in cpu_pcts.items() if k.startswith("cpu") and k != "cpu"},
                "load_1m":         l1,
                "load_5m":         l5,
                "load_15m":        l15,
                "ram_total_mib":   round(mem.get("MemTotal", 0) / 1024, 1) if mem else None,
                "ram_avail_mib":   round(mem.get("MemAvailable", 0) / 1024, 1) if mem else None,
                "ram_used_mib":    None,
                "carla_cpu_pct":   carla_cpu_pct,
                "carla_rss_mib":   carla_rss_mib,
            }
            # derive RAM used
            if sample["ram_total_mib"] and sample["ram_avail_mib"]:
                sample["ram_used_mib"] = round(sample["ram_total_mib"] - sample["ram_avail_mib"], 1)

            samples.append(sample)
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[profile] Interrupted - computing stats from partial data")

    elapsed = time.time() - start
    print(f"[profile] {len(samples)} ticks over {elapsed:.1f}s")

    # --- Parse MangoHud CSV
    if not args.no_mangohud:
        carla_binaries = ("CarlaUE4", "CarlaUE4.sh", "CarlaUE4-Linux-Shipping")
        mangohud_files = []
        for f in out_dir.glob("*_*.csv"):
            stem = f.stem
            for prefix in carla_binaries:
                if stem.startswith(prefix + "_") and stem[len(prefix)+1:].isdigit():
                    mangohud_files.append(f)
                    break
        mangohud_files.sort(key=os.path.getmtime)
        if mangohud_files:
            actual_csv = mangohud_files[-1]
            actual_csv.rename(mangohud_csv)
            print(f"[profile] MangoHud CSV captured: {mangohud_csv}")
            mh_rows = parse_mangohud_csv(mangohud_csv)
        else:
            print("[profile] !!! No MangoHud CSV found - was MANGOHUD_LOG=1 set?")
            mh_rows = []
    else:
        mh_rows = []
    
    # --- Build Summary
    summary = {
        "config":          args.config,
        "machine":         "compute-v1",
        "timestamp":       datetime.now().isoformat(),
        "duration_s":      round(elapsed, 1),
        "sample_ticks":    len(samples),
        "mangohud_frames": len(mh_rows),
        "carla_pid":       carla_pid,
    }

    # GPU busy %
    gpu_vals = [s["gpu_busy_pct"] for s in samples if s["gpu_busy_pct"] is not None]
    if gpu_vals:
        summary["gpu_busy_pct"] = compute_stats(gpu_vals)
    
    # VRAM
    vram_vals = [s["vram_used_mib"] for s in samples if s["vram_used_mib"] is not None]
    if vram_vals:
        summary["vram_used_mib"] = compute_stats(vram_vals)
        summary["vram_used_peak_mib"] = max(vram_vals)

    # CPU aggregate %
    cpu_vals = [s["cpu_pct"] for s in samples if s["cpu_pct"] is not None]
    if cpu_vals:
        summary["cpu_pct"] = compute_stats(cpu_vals)

    # CPU per-core (average across the run)
    per_core = {}
    for s in samples:
        for core, pct in s["cpu_per_core"].items():
            per_core.setdefault(core, []).append(pct)
    if per_core:
        summary["cpu_per_core"] = {c: compute_stats(vals) for c, vals in per_core.items()}

    # Load averages
    load1 = [s["load_1m"] for s in samples if s["load_1m"] is not None]
    if load1:
        summary["load_1m"]  = compute_stats(load1)
    load5 = [s["load_5m"] for s in samples if s["load_5m"] is not None]
    if load5:
        summary["load_5m"]  = compute_stats(load5)
    load15 = [s["load_15m"] for s in samples if s["load_15m"] is not None]
    if load15:
        summary["load_15m"] = compute_stats(load15)

    # RAM
    ram_vals = [s["ram_used_mib"] for s in samples if s["ram_used_mib"] is not None]
    if ram_vals:
        summary["ram_used_mib"] = compute_stats(ram_vals)
        summary["ram_used_peak_mib"] = max(ram_vals)

    # CARLA per-process
    carla_cpu = [s["carla_cpu_pct"] for s in samples if s["carla_cpu_pct"] is not None]
    if carla_cpu:
        summary["carla_cpu_pct"] = compute_stats(carla_cpu)
    carla_rss = [s["carla_rss_mib"] for s in samples if s["carla_rss_mib"] is not None]
    if carla_rss:
        summary["carla_rss_mib"] = compute_stats(carla_rss)
        summary["carla_rss_peak_mib"] = max(carla_rss)

    # MangoHud frametime / FPS
    if mh_rows:
        for field in ("frametime", "frame_time", "frametime_ms"):
            vals = []
            for row in mh_rows:
                try:
                    vals.append(float(row.get(field, 0)))
                except (ValueError, TypeError):
                    pass
            if vals:
                summary[f"frametime_ms_{field}"] = compute_stats(vals)
                break

        for field in ("fps", "FPS"):
            vals = []
            for row in mh_rows:
                try:
                    vals.append(float(row.get(field, 0)))
                except (ValueError, TypeError):
                    pass
            if vals:
                summary[f"fps_{field}"] = compute_stats(vals)
                break

    # --- Write Summary
    summary_path = out_dir / f"{args.config}_summary.json"
    with open(summary_path, "w") as f:
        json.dump(summary, f, indent=2)
    
    print(f"[profile] Summary: {summary_path}")
    print(json.dumps(summary, indent=2))

if __name__ == "__main__":
    main()
