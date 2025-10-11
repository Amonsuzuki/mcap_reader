# 0
build autoware environment ```./docker_run.sh dev cpu```

download ```/SSH``` to the environment

# 1
copy these files to your ```/SSH``` directory
-  open_mcap.py
-  master_exec.bash
-  occupancy_grid_map.yaml (for map drawing)
-  occupancy_grid_map.pgm (for map drawing)

# 2
```chmod +x master_exec.bash```

```./master_exec.bash```

# 3
```cd /SSH/rosbag2_2025_09_30_12_52_32```
* This is the only directory which have valid trajectory data.



```python3 ../open_mcap.py rosbag2_2025_09_30_12_52_32_13.mcap --overwrite```

```python3 ../open_mcap.py rosbag2_2025_09_30_12_52_32_14.mcap --overwrite```

```python3 ../open_mcap.py rosbag2_2025_09_30_12_52_32_15.mcap --overwrite```
<img width="1351" height="1333" alt="Image" src="https://github.com/user-attachments/assets/5efb296e-b84a-4332-b38b-5d1ad85d204b" />

...
# 4
```python3 ../open_mcap2.py rosbag2_2025_09_30_12_52_32_13.mcap --overwrite```
<img width="2341" height="1288" alt="Image" src="https://github.com/user-attachments/assets/56b1d251-9c35-4ced-a541-75d37cbe3c4a" />

# appendix
You can see which mcap files have valid trajectory information by checking ```/SSH/outputs/log_rosbag2_2025_09_30_12_52_32.txt```

**Valid mcap files have 40 ~ 70 ekf_x, ekf_y range.**

for example,
- rosbag2_2025_09_30_12_52_32_11.mcap is **invalid**
- rosbag2_2025_09_30_12_52_32_12.mcap is  **valid**


<img width="1306" height="1261" alt="Image" src="https://github.com/user-attachments/assets/2e3cc3c1-da84-4edd-a711-95236a921668" />


## sensor values

| **Variable Name**             | **Meaning**                               | **Sensor / Module (Physical Source)**                   | **ROS Topic**                        | **Message Field Path**                             |
| ----------------------------- | ----------------------------------------- | ------------------------------------------------------- | ------------------------------------ | -------------------------------------------------- |
| `ekf_x`                       | Estimated X position in map               | **Localization module (EKF / State Estimator)**         | `/localization/kinematic_state`      | `msg.pose.pose.position.x`                         |
| `ekf_y`                       | Estimated Y position                      | **Localization module (EKF / State Estimator)**         | `/localization/kinematic_state`      | `msg.pose.pose.position.y`                         |
| `ekf_yaw`                     | Estimated yaw angle (heading)             | **Localization module (EKF / State Estimator)**         | `/localization/kinematic_state`      | `euler_from_quaternion(msg.pose.pose.orientation)` |
| `loc_acc_x`                   | Linear acceleration along X               | **Derived from IMU or localization filter**             | `/localization/acceleration`         | `msg.accel.accel.linear.x`                         |
| `loc_acc_y`                   | Linear acceleration along Y               | **Derived from IMU or localization filter**             | `/localization/acceleration`         | `msg.accel.accel.linear.y`                         |
| `vx`                          | Longitudinal vehicle speed                | **Wheel speed sensor or vehicle CAN bus**               | `/vehicle/status/velocity_status`    | `msg.longitudinal_velocity`                        |
| `vy`                          | Lateral vehicle speed                     | **Estimated (from vehicle dynamics model)**             | `/vehicle/status/velocity_status`    | `msg.lateral_velocity`                             |
| `steer`                       | Measured steering tire angle              | **Steering angle sensor (CAN)**                         | `/vehicle/status/steering_status`    | `msg.steering_tire_angle`                          |
| `gyro_z`                      | Angular velocity around Z-axis (yaw rate) | **IMU gyroscope**                                       | `/sensing/imu/imu_data`              | `msg.angular_velocity.z`                           |
| `gnss_x`                      | GNSS-based X coordinate                   | **GNSS receiver (GPS/RTK)**                             | `/sensing/gnss/pose_with_covariance` | `msg.pose.pose.position.x`                         |
| `gnss_y`                      | GNSS-based Y coordinate                   | **GNSS receiver (GPS/RTK)**                             | `/sensing/gnss/pose_with_covariance` | `msg.pose.pose.position.y`                         |
| `gnss_z`                      | GNSS altitude                             | **GNSS receiver (GPS/RTK)**                             | `/sensing/gnss/pose_with_covariance` | `msg.pose.pose.position.z`                         |
| `steering_tire_angle_command` | Commanded steering angle                  | **Control module (Path tracking / Lateral controller)** | `/control/command/control_cmd`       | `msg.lateral.steering_tire_angle`                  |
| `speed_command`               | Commanded longitudinal speed              | **Control module (Longitudinal controller)**            | `/control/command/control_cmd`       | `msg.longitudinal.speed`                           |
| `acceleration_command`        | Commanded longitudinal acceleration       | **Control module (Longitudinal controller)**            | `/control/command/control_cmd`       | `msg.longitudinal.acceleration`                    |
| `actuation_accel_cmd`         | Throttle actuator command                 | **Vehicle actuator interface (Drive-by-wire ECU)**      | `/control/command/actuation_cmd`     | `msg.actuation.accel_cmd`                          |
| `actuation_brake_cmd`         | Brake actuator command                    | **Vehicle actuator interface (Drive-by-wire ECU)**      | `/control/command/actuation_cmd`     | `msg.actuation.brake_cmd`                          |
| `actuation_steer_cmd`         | Steering actuator command                 | **Vehicle actuator interface (Drive-by-wire ECU)**      | `/control/command/actuation_cmd`     | `msg.actuation.steer_cmd`                          |
