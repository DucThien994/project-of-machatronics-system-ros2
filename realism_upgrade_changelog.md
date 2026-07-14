# Realism Upgrade Changelog — AMR Mecanum ROS2 Simulation

Tài liệu tham chiếu để đối chiếu (diff) khi áp dụng các thay đổi giảm tính lý tưởng hóa mô phỏng.
Trạng thái baseline: `D:\Mutual_ros2\ros2` (sau khi dọn trash code, dùng `libgazebo_ros_planar_move`).
Mỗi mục ghi: file → vị trí → trạng thái hiện tại (`-`) → đề xuất (`+`) → lý do/công thức liên quan.

---

## A. Tầng Actuation/Physics — chuyển `planar_move` → `ros2_control` (bánh thật)

### A1. Xóa plugin planar_move
**File:** `src/amr_description/urdf/amr_robot.urdf.xacro`
**Vị trí:** khối `<gazebo>` chứa plugin `planar_move` (~dòng 211-226)

```diff
- <gazebo>
-   <plugin name="planar_move" filename="libgazebo_ros_planar_move.so">
-     <ros>
-       <remapping>cmd_vel:=cmd_vel_safe</remapping>
-       <remapping>odom:=odom</remapping>
-     </ros>
-     <update_rate>50.0</update_rate>
-     <robot_base_frame>base_footprint</robot_base_frame>
-     <odometry_frame>odom</odometry_frame>
-     <publish_odom>true</publish_odom>
-     <publish_odom_tf>true</publish_odom_tf>
-     <covariance_x>0.001</covariance_x>
-     <covariance_y>0.001</covariance_y>
-     <covariance_yaw>0.001</covariance_yaw>
-   </plugin>
- </gazebo>
```
Lý do: không thể chạy song song với ros2_control — 2 nguồn set velocity thân xe sẽ xung đột vật lý.

### A2. Thêm khối ros2_control
**File:** `src/amr_description/urdf/amr_robot.urdf.xacro`
**Vị trí:** chèn mới, sau lệnh gọi macro 4 bánh (~dòng 107), trước link `lidar_link`

```diff
+ <ros2_control name="GazeboSystem" type="system">
+   <hardware>
+     <plugin>gazebo_ros2_control/GazeboSystem</plugin>
+   </hardware>
+   <joint name="front_left_wheel_joint">
+     <command_interface name="velocity"/>
+     <state_interface name="velocity"/>
+     <state_interface name="position"/>
+   </joint>
+   <joint name="front_right_wheel_joint">
+     <command_interface name="velocity"/>
+     <state_interface name="velocity"/>
+     <state_interface name="position"/>
+   </joint>
+   <joint name="rear_left_wheel_joint">
+     <command_interface name="velocity"/>
+     <state_interface name="velocity"/>
+     <state_interface name="position"/>
+   </joint>
+   <joint name="rear_right_wheel_joint">
+     <command_interface name="velocity"/>
+     <state_interface name="velocity"/>
+     <state_interface name="position"/>
+   </joint>
+ </ros2_control>
```

### A3. Thêm plugin gazebo_ros2_control
**File:** `src/amr_description/urdf/amr_robot.urdf.xacro`
**Vị trí:** cạnh khối `<gazebo><plugin name="joint_state_publisher"...>` (~dòng 228-238)

```diff
+ <gazebo>
+   <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
+     <parameters>$(find amr_bringup)/config/mecanum_controllers.yaml</parameters>
+   </plugin>
+ </gazebo>
```
Cân nhắc xóa plugin `joint_state_publisher` cũ (dòng ~229-237) vì `joint_state_broadcaster` (ros2_control) sẽ đảm nhiệm publish `/joint_states`, tránh 2 publisher trùng topic.

### A4. Ma sát bánh mecanum — mô phỏng trượt thật
**File:** `src/amr_description/urdf/amr_robot.urdf.xacro`
**Vị trí:** macro `mecanum_wheel`, khối `<gazebo reference="${prefix}_wheel_link">` (dòng 86-95)

```diff
  <gazebo reference="${prefix}_wheel_link">
-   <mu1>0.0</mu1>
-   <mu2>1.0</mu2>
+   <mu1>0.08</mu1>
+   <mu2>0.80</mu2>
+   <slip1>0.05</slip1>
+   <slip2>0.02</slip2>
    <fdir1>${fdir1}</fdir1>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
    <minDepth>0.001</minDepth>
    <maxVel>5.0</maxVel>
    <material>Gazebo/Black</material>
  </gazebo>
```
Công thức liên quan: Phần C (mô hình trượt Coulomb) trong phụ lục công thức.

### A5. Ma sát khớp bánh (damping/friction)
**File:** `src/amr_description/urdf/amr_robot.urdf.xacro`
**Vị trí:** joint mỗi bánh, thuộc tính `<dynamics>` (dòng 84)

```diff
- <dynamics damping="0.005" friction="0.05"/>
+ <dynamics damping="0.02" friction="0.15"/>
```
Lý do: giá trị cũ mô phỏng bánh quay gần như tự do (ổ trục lý tưởng) — không phản ánh lực cản hộp số/ổ bi thật.

### A6. Tạo lại file controller config
**File:** `src/amr_bringup/config/mecanum_controllers.yaml` (đã bị xóa ở bước dọn trash code — cần tạo lại)

```yaml
controller_manager:
  ros__parameters:
    update_rate: 50
    use_sim_time: true
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    mecanum_drive_controller:
      type: mecanum_drive_controller/MecanumDriveController

mecanum_drive_controller:
  ros__parameters:
    reference_timeout: 0.5
    use_stamped_vel: false      # → topic reference_unstamped (Twist thường)
    front_left_wheel_command_joint_name:  "front_left_wheel_joint"
    front_right_wheel_command_joint_name: "front_right_wheel_joint"
    rear_right_wheel_command_joint_name:  "rear_right_wheel_joint"
    rear_left_wheel_command_joint_name:   "rear_left_wheel_joint"
    kinematics:
      base_frame_offset: {x: 0.0, y: 0.0, theta: 0.0}
      wheels_radius: 0.075
      sum_of_robot_center_projection_on_X_Y_axis: 0.30
    base_frame_id: "base_footprint"
    odom_frame_id: "odom"
    enable_odom_tf: true
    # KHÔNG copy nguyên [0.001]*6 từ bản cũ — xem mục D (Phụ lục công thức)
    # để tính covariance theo độ phân giải encoder + trượt bánh thực tế.
    twist_covariance_diagonal: [0.01, 0.01, 0.01, 0.01, 0.01, 0.02]
    pose_covariance_diagonal:  [0.01, 0.01, 0.01, 0.01, 0.01, 0.02]
```
Topic thực tế xác nhận qua tài liệu chính thức [control.ros.org/humble/mecanum_drive_controller](https://control.ros.org/humble/doc/ros2_controllers/mecanum_drive_controller/doc/userdoc.html):
command = `/mecanum_drive_controller/reference_unstamped` (`geometry_msgs/Twist`), odometry = `/mecanum_drive_controller/odometry` (`nav_msgs/Odometry`).

### A7. Khôi phục dependency ros2_control
**File:** `src/amr_description/package.xml`

```diff
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>
+ <exec_depend>gazebo_ros2_control</exec_depend>
+ <exec_depend>ros2_controllers</exec_depend>
+ <exec_depend>joint_state_broadcaster</exec_depend>
+ <exec_depend>controller_manager</exec_depend>
```

### A8. Spawner controller trong gazebo.launch.py
**File:** `src/amr_simulation/launch/gazebo.launch.py`
**Vị trí:** sau `spawn_robot` TimerAction (~dòng 56-70)

```diff
+ joint_state_broadcaster_spawner = TimerAction(period=7.0, actions=[
+     Node(package='controller_manager', executable='spawner',
+          arguments=['joint_state_broadcaster'])
+ ])
+ mecanum_controller_spawner = TimerAction(period=8.5, actions=[
+     Node(package='controller_manager', executable='spawner',
+          arguments=['mecanum_drive_controller'])
+ ])

  return LaunchDescription([
      gazebo_model_path,
      gazebo_resource_path,
      x_pose_arg, y_pose_arg, z_pose_arg, yaw_arg,
      gazebo,
      rsp,
      spawn_robot,
+     joint_state_broadcaster_spawner,
+     mecanum_controller_spawner,
  ])
```

### A9. Remap output collision_warning_node
**File:** `src/amr_safety/launch/safety.launch.py`
**Vị trí:** `Node(package='amr_safety', executable='collision_warning_node', ...)` (dòng 17-26)

```diff
  Node(
      package='amr_safety',
      executable='collision_warning_node',
      name='collision_warning_node',
      output='screen',
      parameters=[{
          'publish_rate': 20.0,
          'cmd_timeout':   0.5,
      }],
+     remappings=[('/cmd_vel_safe', '/mecanum_drive_controller/reference_unstamped')],
  )
```

### A10. Đổi odom_topic trong Nav2
**File:** `src/amr_navigation/config/nav2_params.yaml`
**Vị trí:** `bt_navigator.odom_topic` (dòng 47)

```diff
  bt_navigator:
    ros__parameters:
-     odom_topic:        /odom
+     odom_topic:        /mecanum_drive_controller/odometry
```
Thay thế: thêm node `topic_tools relay /mecanum_drive_controller/odometry /odom` nếu muốn giữ nguyên các chỗ khác đang giả định topic `/odom`.

---

## B. Tầng cảm biến — nhiễu thực tế hơn

### B1. Dropout tia LiDAR
**File cần thêm mới:** `src/amr_safety/scripts/scan_dropout_node.py` (hoặc gói mới `amr_sensor_sim`)
Lý do: `libgazebo_ros_ray_sensor.so` chỉ hỗ trợ Gaussian noise liên tục (URDF dòng ~150-154, `stddev=0.008` — giữ nguyên), không hỗ trợ dropout tia rời rạc. Cần node subscribe `/scan` gốc, set ngẫu nhiên N% số tia thành `range = inf`, republish topic riêng cho SLAM/Nav2 dùng.

### B2. Bias trôi IMU (random walk)
**File cần thêm mới:** `src/amr_safety/scripts/imu_bias_node.py`
Lý do: `libgazebo_ros_imu_sensor.so` chỉ hỗ trợ white noise tĩnh (URDF dòng ~192-202), không có bias trôi theo thời gian. Cần node subscribe `/imu/data`, cộng dồn bias theo công thức Phần G (phụ lục), republish.

---

## C. Tầng vật lý solver

**File:** `src/amr_simulation/worlds/warehouse_v5.world`
**Vị trí:** `<physics><ode><solver>` (dòng 14)

```diff
  <ode>
-   <solver><type>quick</type><iters>150</iters><sor>1.4</sor></solver>
+   <solver><type>world</type><iters>150</iters><sor>1.4</sor></solver>
  </ode>
```
Nếu giữ `quick` vì lý do hiệu năng real-time, tăng `iters` lên 300-500 thay vì đổi `type`.

> Lưu ý riêng (không thuộc phạm vi realism): dòng 10 cùng file từng có bug tag `<real_time_update_ratse>` lệch tên với `</real_time_update_rate>` — xác nhận đã sửa trước khi áp dụng các thay đổi trên.

---

## D. Tầng localization — retune theo noise model mới

**File:** `src/amr_navigation/config/nav2_params.yaml`

```diff
  amcl:
    ros__parameters:
-     alpha1: 0.2
-     alpha2: 0.2
-     alpha3: 0.2
-     alpha4: 0.2
-     alpha5: 0.2
+     alpha1: <đo thực nghiệm — xem Phụ lục F>
+     alpha2: <đo thực nghiệm>
+     alpha3: <đo thực nghiệm>
+     alpha4: <đo thực nghiệm>
+     alpha5: <đo thực nghiệm>
      ...
-     update_min_a:         0.1
-     update_min_d:         0.1
+     update_min_a:         0.05
+     update_min_d:         0.05
```

Đồng thời (tồn đọng từ trash-code audit trước, chưa liên quan realism nhưng nên gộp chung đợt sửa):
```diff
  planner_server:
    GridBased:
-     use_astar:     false
+     use_astar:     true
```

---

## E. Tầng môi trường — vật cản động, tải trọng biến thiên

### E1. Vật cản động
**File:** `src/amr_simulation/worlds/warehouse_v5.world`
**Vị trí:** chèn model mới trước khối `<gui>` (~dòng 266)

```diff
+ <model name="dynamic_obstacle_1">
+   <pose>2 -2 0.4 0 0 0</pose>
+   <link name="link">
+     <collision name="col"><geometry><box><size>0.6 0.6 0.8</size></box></geometry></collision>
+     <visual name="vis"><geometry><box><size>0.6 0.6 0.8</size></box></geometry></visual>
+   </link>
+   <plugin name="waypoint_mover" filename="libgazebo_ros_planar_move.so">
+     <!-- hoặc plugin actor/waypoint tùy phiên bản Gazebo -->
+   </plugin>
+ </model>
```
Toàn bộ model hiện tại (dòng 26-264) đều `<static>true</static>` — không có vật cản di động để kiểm thử phản ứng thực của `collision_warning_node`/MPPI.

### E2. Tải trọng biến thiên
**File:** `src/amr_description/urdf/amr_robot.urdf.xacro`
**Vị trí:** dòng 10

```diff
- <xacro:property name="base_mass"    value="12.0"/>
+ <xacro:arg name="payload_mass" default="12.0"/>
+ <xacro:property name="base_mass" value="$(arg payload_mass)"/>
```
Truyền giá trị random qua `gazebo.launch.py` theo mẫu `x_pose_arg` đã có sẵn.

---

## F. Trễ động cơ (chỉ cần nếu KHÔNG dùng full ros2_control, mô phỏng lag ở tầng cmd_vel)

**File cần thêm mới:** node trung gian giữa `collision_warning_node` và tầng actuation, áp dụng công thức bậc 1 (Phụ lục E).

---

# PHỤ LỤC — Công thức nền tảng

## Công thức A — Động học nghịch Mecanum
```
ω_FL = (1/r) * ( vx - vy - (lx+ly)*wz )
ω_FR = (1/r) * ( vx + vy + (lx+ly)*wz )
ω_RL = (1/r) * ( vx + vy - (lx+ly)*wz )
ω_RR = (1/r) * ( vx - vy + (lx+ly)*wz )
```
r = 0.075 m, lx+ly = 0.30 m

## Công thức B — Động học thuận (odometry ngược)
```
[vx]     r   [ 1        1        1        1      ] [ω_FL]
[vy]  =  -- *[-1        1        1       -1      ] [ω_FR]
[wz]     4   [-1/(lx+ly) 1/(lx+ly) -1/(lx+ly) 1/(lx+ly)] [ω_RL]
                                                          [ω_RR]
```

## Công thức C — Mô hình trượt Coulomb
```
|F_t| ≤ μ * F_n
s_i = (r*ω_i - v_contact_i) / (r*ω_i)
Δx_slip = ∫[0,T] Σ J_i * s_i(τ) * r*ω_i(τ) dτ
```

## Công thức D — Covariance odometry theo encoder + quãng đường
```
σ_ω = (2π / N_ppr) / Δt
σ_v = r * σ_ω
σ_x(d) = k * sqrt(d),   k ≈ 0.01–0.03 m/√m
```

## Công thức E — Trễ động cơ bậc 1
```
τ * dv/dt + v = v_cmd
v[k+1] = v[k] + (Δt/τ) * (v_cmd[k] - v[k])
τ ≈ 0.05–0.15 s
```

## Công thức F — Mô hình nhiễu chuyển động AMCL (Thrun et al.)
```
δ̂_rot1  = δ_rot1  + N(0, α1*δ_rot1² + α2*δ_trans²)
δ̂_trans = δ_trans + N(0, α3*δ_trans² + α4*(δ_rot1²+δ_rot2²))
δ̂_rot2  = δ_rot2  + N(0, α1*δ_rot2² + α2*δ_trans²)
```
Giải ngược α_i từ σ đo thực nghiệm (quay tại chỗ vs đi thẳng) sau khi có odometry thật.

## Công thức G — Bias trôi IMU (random walk)
```
b_g(t+Δt) = b_g(t) + w(t)*sqrt(Δt),   w ~ N(0, σ_rw²)
σ_rw ≈ 0.0002–0.001 rad/s/√s (IMU MEMS tầm trung)
```

---

# Bảng theo dõi trạng thái áp dụng

| Mục | Trạng thái | Ghi chú |
|---|---|---|
| A1-A10 (ros2_control) | ☐ Chưa áp dụng | |
| B1 (LiDAR dropout) | ☐ Chưa áp dụng | Cần viết node mới |
| B2 (IMU bias) | ☐ Chưa áp dụng | Cần viết node mới |
| C (solver world/quick) | ☐ Chưa áp dụng | |
| D (AMCL alpha retune) | ☐ Chưa áp dụng | Phụ thuộc đo thực nghiệm sau A |
| D (use_astar) | ☐ Chưa áp dụng | Tồn đọng từ audit trash code |
| E1 (vật cản động) | ☐ Chưa áp dụng | |
| E2 (payload random) | ☐ Chưa áp dụng | |
| F (motor lag) | ☐ Chỉ cần nếu không dùng A | |
