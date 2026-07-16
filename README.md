# 🚦 V2X-Enabled Intersection Assistant

Cooperative collision avoidance: vehicles broadcast state (pose, velocity, intention)
over ROS 2 / Wi-Fi–5G; each vehicle computes time-to-collision (TTC) for every
pair and triggers deterministic emergency braking through EtherCAT.

- **Perception**: ROS 2 + camera/LiDAR (traffic lights, pedestrians)
- **State estimation**: STM32H7, ThreadX + micro-ROS, GPS+IMU+encoder Kalman filter @ 200 Hz
- **Conflict detection**: ROS 2 node, TTC < 2 s ⇒ emergency brake via micro-ROS
- **Actuation**: EtherCAT brake/throttle, Distributed-Clock time sync for V2X timestamping
- **OTA**: STM32 dual-bank flash, 3-strike automatic rollback

## Layout
```
firmware/                 STM32H7 ThreadX app: linear Kalman filter, V2X codec, brake ctrl
  test/                   Unity + Ceedling (+ CMock configuration for GPS/IMU fault injection)
  bootloader/             Dual-bank OTA with boot-attempt rollback counter
ros2_ws/src/
  v2x_msgs/               VehicleState.msg / BrakeCommand.msg
  v2x_conflict_detector/  TTC computation + conflict arbitration (GTest)
```

## Run the tests
```bash
cd firmware && ceedling test:all
cd ros2_ws && colcon build && colcon test
```
