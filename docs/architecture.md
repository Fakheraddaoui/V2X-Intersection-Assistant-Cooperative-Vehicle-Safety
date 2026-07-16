# Architecture — V2X Intersection Assistant

Each vehicle:
```
 camera/LiDAR ─→ ROS 2 perception ─→ /v2x/vehicle_states (Wi-Fi/5G, all vehicles)
                                     │
                    v2x_conflict_detector (50 ms): pairwise TTC
                                     │  TTC < 2 s
                          /v2x/brake_commands ─→ micro-ROS agent
                                     │
 STM32H7 (ThreadX): 200 Hz Kalman (GPS+IMU+odom) ─→ state broadcast
                    brake_controller ─→ EtherCAT (1 ms, DC-synced) ─→ brake actuator
```

- **EtherCAT Distributed Clocks** give sub-µs common time; all V2X messages are
  stamped in this timebase so TTC math uses consistent snapshots.
- **CMock fault injection** (unit tests): simulated GPS drift, IMU noise, and
  dropped V2X frames verify the estimator gate and the stale-agent eviction.
- **OTA**: dual-bank STM32 flash; new bank must be signature-valid; app confirms
  within 3 boots or the bootloader auto-reverts.
