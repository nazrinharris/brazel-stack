# brazel-stack

Autonomous driving stack on ROS2 + CARLA. Mostly meant to be a solid testbed for me to experiment with AV techniques hands on. The target vehicle in CARLA I'm working on right now will be an SUV, specifically the Audi E-Tron.

The current goal is to further my fundamentals on map-first localization. So things like pre-builiding a vocel map, math LiDAR scans against it, fuse with IMU, etc. So basically hardening the scan mathing, state estimation, and sensor fusion fundamentals. In the longer term would like to explore more state-of-the-art stuff and going mapless, but getting a solid footing first.

What it really means though is to implement NDT scan matching ([Biber & Strasser 2003](https://www.researchgate.net/publication/4045903_The_Normal_Distributions_Transform_A_New_Approach_to_Laser_Scan_Matching), [Takubo 2009](http://vigir.ee.missouri.edu/~gdesouza/Research/Conference_CDs/IEEE_IROS_2009/papers/0786.pdf)) fused with IMU via EKF.

---

## Status

#### Phase 0: Getting the skeleton up and walking

- [x] ROS2 Humble workspace
- [x] `brazel_utils` initial setup and basic messages
- [x] Verify CARLA 0.9.16 and native ROS2 bridge working on current hardware/software
- [x] `brazel_bringup` initial setup for vehicle spawning, sensors, etc.
- [ ] rviz2 config (sensors + vehicle transform)
- [ ] Manual keyboard control
- [ ] Baseline performance numbers measurement and definition

---

## Hardware and Software

|     |     |
| --- | --- |
| CPU | AMD Ryzen 5 9600X |
| GPU | AMD Radeon RX 9070XT (16GB) - (Mesa 25.0.7 / RADV Vulkan) |
| RAM | 32GB |
| ROS 2 | Humble |
| CARLA | 0.9.16 |
| OS | Ubuntu 22.04 (jammy) |

As of now, the simulaton and ROS stack are all on one device. So performance budgeting is absolutely important as bottlenecks will definitely prop up. 

GPU is also AMD, so no CUDA. This is gonna bite me when I go further on but this is what I have so it'll do for now. Depending on where the actual bottlenecks, likely will go for a 2-machine setup with a mid-range RTX 3060 as the ROS stack machine.

---

## Roadmap

1. **Phase 0 - Skeleton**. Just basically getting CARLA working on the system, solving driver issues, getting the sensor flowing data through the stack, manual control of the vehicle
2. **Phase 1 - Voxel Localization**. NDT scan matching on voxel maps, fused with IMU. There's a LOT of details under here that hasn't been fleshed out yet. But we'll get there.

After that is when I can continue on with more stuff, like occupancy-based planning with MPPI, BEV perception, sim-to-real evaluation, mapless perception, etc. But that's further out.

---

## Getting Started

```
# build
cd brazel_ws
colcon build
source install/setup.bash

# start CARLA
./CarlaUE4.sh --ros2

# launch the stack (currently just spawns an auto-routing vehicle)
ros2 launch brazel_bringup hero_spawner.launch.py
```