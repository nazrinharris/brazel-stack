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
1. MangoHud config at `~/brazel_ws/src/brazel_diagnostics/perf/mangohud_brazel_bench.conf`
2. Verify `mangohud_brazel_bench.conf`'s `output_folder` matches the script's `--output-dir` (default: `~/brazel_ws/src/brazel_diagnostics/perf/profiles`) and MAKE SURE it's an absolute path
    - So for example, in the `.conf` file, `output_folder=/mnt/av-storage/brazel_ws/src/brazel_diagnostics/perf/profiles`
3. For the CARLA startup, make sure the absolute path of the MANGOHUD_CONFIGFILE is correct, it's located in `~/brazel_ws/src/brazel_diagnostics/perf/mangohud_brazel_bench.conf`

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