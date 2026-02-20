# MIRACLE Digital Twin - ROS2 Command Guide

All commands are copy-paste ready — they run inside the Docker container automatically.

## Prerequisites

Start the bridge container (must be running before Unity can connect):

```bash
docker rm miracle_bridge 2>/dev/null; docker run --rm -it -p 10000:10000 --name miracle_bridge miracle_ros2:latest \
  bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && ros2 launch miracle_unity_bridge unity_bridge.launch.py"
```

Then hit **Play** in Unity. Console should show:
```
ROS Connection to 127.0.0.1:10000 succeeded!
```

All commands below run in a **second terminal**.

---

## Monitoring

```bash
# List all active topics
docker exec miracle_bridge bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && ros2 topic list"
```

```bash
# List all nodes
docker exec miracle_bridge bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && ros2 node list"
```

```bash
# Echo a topic (see live data, Ctrl+C to stop)
docker exec -it miracle_bridge bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && ros2 topic echo /miracle/robots/joint_states"
```

```bash
# Check publish rate
docker exec -it miracle_bridge bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && ros2 topic hz /miracle/robots/joint_states"
```

```bash
# Show message definition
docker exec miracle_bridge bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && ros2 interface show miracle_msgs/msg/RobotJointState"
```

```bash
docker exec miracle_bridge bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && ros2 interface show miracle_msgs/msg/MachineState"
```

---

## Robot Control

Robot IDs: `ned2` (Niryo Ned2), `lite6` (xArm 6 Lite)

Positions are in **radians**. Each robot has 6 joints (J1-J6, base to wrist).

### Move Niryo Ned2 to a fixed pose

```bash
docker exec -it miracle_bridge bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && ros2 topic pub /miracle/robots/joint_states miracle_msgs/msg/RobotJointState '{
  robot_id: \"ned2\",
  positions: [0.5, -0.3, 0.8, 0.0, 1.2, 0.0],
  velocities: [0,0,0,0,0,0],
  efforts: [0,0,0,0,0,0],
  gripper_state: \"open\",
  task_state: \"moving\"
}' --rate 10"
```

### Move xArm 6 Lite to a fixed pose

```bash
docker exec -it miracle_bridge bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && ros2 topic pub /miracle/robots/joint_states miracle_msgs/msg/RobotJointState '{
  robot_id: \"lite6\",
  positions: [0.3, -0.5, 1.0, 0.2, -0.8, 0.0],
  velocities: [0,0,0,0,0,0],
  efforts: [0,0,0,0,0,0],
  gripper_state: \"closed\",
  task_state: \"moving\"
}' --rate 10"
```

### Home Niryo (all joints to zero)

```bash
docker exec -it miracle_bridge bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && ros2 topic pub /miracle/robots/joint_states miracle_msgs/msg/RobotJointState '{
  robot_id: \"ned2\",
  positions: [0,0,0,0,0,0],
  velocities: [0,0,0,0,0,0],
  efforts: [0,0,0,0,0,0],
  gripper_state: \"open\",
  task_state: \"idle\"
}' --rate 10"
```

### Home xArm (all joints to zero)

```bash
docker exec -it miracle_bridge bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && ros2 topic pub /miracle/robots/joint_states miracle_msgs/msg/RobotJointState '{
  robot_id: \"lite6\",
  positions: [0,0,0,0,0,0],
  velocities: [0,0,0,0,0,0],
  efforts: [0,0,0,0,0,0],
  gripper_state: \"open\",
  task_state: \"idle\"
}' --rate 10"
```

### Niryo: Pick-and-place sequence (scripted)

```bash
# Approach position
docker exec -it miracle_bridge bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && ros2 topic pub /miracle/robots/joint_states miracle_msgs/msg/RobotJointState '{
  robot_id: \"ned2\",
  positions: [0.8, -0.4, 0.6, 0.0, 1.0, 0.3],
  velocities: [0,0,0,0,0,0],
  efforts: [0,0,0,0,0,0],
  gripper_state: \"open\",
  task_state: \"approaching\"
}' --rate 10 --times 30"
```

```bash
# Grab (close gripper, lower)
docker exec -it miracle_bridge bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && ros2 topic pub /miracle/robots/joint_states miracle_msgs/msg/RobotJointState '{
  robot_id: \"ned2\",
  positions: [0.8, -0.2, 0.9, 0.0, 1.3, 0.3],
  velocities: [0,0,0,0,0,0],
  efforts: [0,0,0,0,0,0],
  gripper_state: \"closed\",
  task_state: \"gripping\"
}' --rate 10 --times 30"
```

```bash
# Move to drop zone
docker exec -it miracle_bridge bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && ros2 topic pub /miracle/robots/joint_states miracle_msgs/msg/RobotJointState '{
  robot_id: \"ned2\",
  positions: [-0.6, -0.4, 0.6, 0.0, 1.0, -0.3],
  velocities: [0,0,0,0,0,0],
  efforts: [0,0,0,0,0,0],
  gripper_state: \"closed\",
  task_state: \"transporting\"
}' --rate 10 --times 30"
```

```bash
# Release
docker exec -it miracle_bridge bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && ros2 topic pub /miracle/robots/joint_states miracle_msgs/msg/RobotJointState '{
  robot_id: \"ned2\",
  positions: [-0.6, -0.2, 0.9, 0.0, 1.3, -0.3],
  velocities: [0,0,0,0,0,0],
  efforts: [0,0,0,0,0,0],
  gripper_state: \"open\",
  task_state: \"releasing\"
}' --rate 10 --times 30"
```

### xArm: Reach forward and back

```bash
# Extended reach
docker exec -it miracle_bridge bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && ros2 topic pub /miracle/robots/joint_states miracle_msgs/msg/RobotJointState '{
  robot_id: \"lite6\",
  positions: [0.0, -1.2, 1.5, 0.0, 0.5, 0.0],
  velocities: [0,0,0,0,0,0],
  efforts: [0,0,0,0,0,0],
  gripper_state: \"open\",
  task_state: \"reaching\"
}' --rate 10 --times 30"
```

```bash
# Retracted/tucked
docker exec -it miracle_bridge bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && ros2 topic pub /miracle/robots/joint_states miracle_msgs/msg/RobotJointState '{
  robot_id: \"lite6\",
  positions: [0.0, -0.3, 0.3, 0.0, 0.2, 0.0],
  velocities: [0,0,0,0,0,0],
  efforts: [0,0,0,0,0,0],
  gripper_state: \"closed\",
  task_state: \"retracted\"
}' --rate 10 --times 30"
```

---

## CNC Machine Control

Machine IDs: `cnc1` (Bantam Desktop Explorer), `cnc2` (CoastRunner CR-1)

MiracleBridge subscribes to `/miracle/cnc1/state` by default. Axis positions are in **mm** [X, Y, Z].

### Bantam: Move spindle to center of work envelope

```bash
docker exec -it miracle_bridge bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && ros2 topic pub /miracle/cnc1/state miracle_msgs/msg/MachineState '{
  machine_id: \"cnc1\",
  status: \"RUNNING\",
  spindle_speed: 12000.0,
  feed_rate: 800.0,
  axis_positions: [76.0, 50.0, 35.0],
  axis_velocities: [0.0, 0.0, 0.0],
  spindle_load: 40.0,
  coolant_level: 90.0,
  current_program: \"manual_jog.nc\",
  current_line: 1,
  cycle_time_elapsed: 0.0,
  cycle_time_remaining: 0.0
}' --rate 20"
```

### Bantam: Simulate a square pocket toolpath

```bash
# Corner 1
docker exec -it miracle_bridge bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && ros2 topic pub /miracle/cnc1/state miracle_msgs/msg/MachineState '{
  machine_id: \"cnc1\", status: \"RUNNING\", spindle_speed: 12000.0, feed_rate: 800.0,
  axis_positions: [30.0, 25.0, 15.0], axis_velocities: [800.0, 0.0, 0.0],
  spindle_load: 45.0, coolant_level: 90.0, current_program: \"square_pocket.nc\",
  current_line: 10, cycle_time_elapsed: 5.0, cycle_time_remaining: 55.0
}' --rate 20 --times 40"
```

```bash
# Corner 2
docker exec -it miracle_bridge bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && ros2 topic pub /miracle/cnc1/state miracle_msgs/msg/MachineState '{
  machine_id: \"cnc1\", status: \"RUNNING\", spindle_speed: 12000.0, feed_rate: 800.0,
  axis_positions: [120.0, 25.0, 15.0], axis_velocities: [0.0, 800.0, 0.0],
  spindle_load: 42.0, coolant_level: 90.0, current_program: \"square_pocket.nc\",
  current_line: 20, cycle_time_elapsed: 10.0, cycle_time_remaining: 50.0
}' --rate 20 --times 40"
```

```bash
# Corner 3
docker exec -it miracle_bridge bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && ros2 topic pub /miracle/cnc1/state miracle_msgs/msg/MachineState '{
  machine_id: \"cnc1\", status: \"RUNNING\", spindle_speed: 12000.0, feed_rate: 800.0,
  axis_positions: [120.0, 80.0, 15.0], axis_velocities: [-800.0, 0.0, 0.0],
  spindle_load: 38.0, coolant_level: 90.0, current_program: \"square_pocket.nc\",
  current_line: 30, cycle_time_elapsed: 15.0, cycle_time_remaining: 45.0
}' --rate 20 --times 40"
```

```bash
# Corner 4
docker exec -it miracle_bridge bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && ros2 topic pub /miracle/cnc1/state miracle_msgs/msg/MachineState '{
  machine_id: \"cnc1\", status: \"RUNNING\", spindle_speed: 12000.0, feed_rate: 800.0,
  axis_positions: [30.0, 80.0, 15.0], axis_velocities: [0.0, -800.0, 0.0],
  spindle_load: 41.0, coolant_level: 90.0, current_program: \"square_pocket.nc\",
  current_line: 40, cycle_time_elapsed: 20.0, cycle_time_remaining: 40.0
}' --rate 20 --times 40"
```

### Bantam: Test X axis only (gantry left-right)

```bash
docker exec -it miracle_bridge bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && ros2 topic pub /miracle/cnc1/state miracle_msgs/msg/MachineState '{
  machine_id: \"cnc1\", status: \"RUNNING\", spindle_speed: 12000.0, feed_rate: 800.0,
  axis_positions: [140.0, 0.0, 0.0], axis_velocities: [0.0, 0.0, 0.0],
  spindle_load: 0.0, coolant_level: 90.0, current_program: \"test_x.nc\",
  current_line: 1, cycle_time_elapsed: 0.0, cycle_time_remaining: 0.0
}' --rate 20 --times 60"
```

### Bantam: Test Y axis only (table front-back)

```bash
docker exec -it miracle_bridge bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && ros2 topic pub /miracle/cnc1/state miracle_msgs/msg/MachineState '{
  machine_id: \"cnc1\", status: \"RUNNING\", spindle_speed: 12000.0, feed_rate: 800.0,
  axis_positions: [0.0, 90.0, 0.0], axis_velocities: [0.0, 0.0, 0.0],
  spindle_load: 0.0, coolant_level: 90.0, current_program: \"test_y.nc\",
  current_line: 1, cycle_time_elapsed: 0.0, cycle_time_remaining: 0.0
}' --rate 20 --times 60"
```

### Bantam: Test Z axis only (head down into cut)

```bash
docker exec -it miracle_bridge bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && ros2 topic pub /miracle/cnc1/state miracle_msgs/msg/MachineState '{
  machine_id: \"cnc1\", status: \"RUNNING\", spindle_speed: 12000.0, feed_rate: 200.0,
  axis_positions: [0.0, 0.0, 50.0], axis_velocities: [0.0, 0.0, 0.0],
  spindle_load: 0.0, coolant_level: 90.0, current_program: \"test_z.nc\",
  current_line: 1, cycle_time_elapsed: 0.0, cycle_time_remaining: 0.0
}' --rate 20 --times 60"
```

### Bantam: Home (return all axes to zero)

```bash
docker exec -it miracle_bridge bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && ros2 topic pub /miracle/cnc1/state miracle_msgs/msg/MachineState '{
  machine_id: \"cnc1\", status: \"IDLE\", spindle_speed: 0.0, feed_rate: 0.0,
  axis_positions: [0.0, 0.0, 0.0], axis_velocities: [0.0, 0.0, 0.0],
  spindle_load: 0.0, coolant_level: 90.0, current_program: \"\",
  current_line: 0, cycle_time_elapsed: 0.0, cycle_time_remaining: 0.0
}' --rate 20 --times 40"
```

### CoastRunner: Move to cutting position

```bash
docker exec -it miracle_bridge bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && ros2 topic pub /miracle/cnc2/state miracle_msgs/msg/MachineState '{
  machine_id: \"cnc2\",
  status: \"RUNNING\",
  spindle_speed: 10000.0,
  feed_rate: 600.0,
  axis_positions: [44.0, 120.0, 40.0],
  axis_velocities: [0.0, 0.0, 0.0],
  spindle_load: 35.0,
  coolant_level: 85.0,
  current_program: \"facing_op.nc\",
  current_line: 1,
  cycle_time_elapsed: 0.0,
  cycle_time_remaining: 0.0
}' --rate 20"
```

### Stop a CNC (set status to IDLE)

```bash
docker exec -it miracle_bridge bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && ros2 topic pub /miracle/cnc1/state miracle_msgs/msg/MachineState '{
  machine_id: \"cnc1\",
  status: \"IDLE\",
  spindle_speed: 0.0,
  feed_rate: 0.0,
  axis_positions: [0.0, 0.0, 0.0],
  axis_velocities: [0.0, 0.0, 0.0],
  spindle_load: 0.0,
  coolant_level: 90.0,
  current_program: \"\",
  current_line: 0,
  cycle_time_elapsed: 0.0,
  cycle_time_remaining: 0.0
}' --rate 20 --times 20"
```

---

## Dashboard & KPIs

### Override system KPIs

```bash
docker exec -it miracle_bridge bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && ros2 topic pub /miracle/system_kpis miracle_msgs/msg/SystemKPIs '{
  oee: 92.5,
  availability: 98.0,
  performance: 95.5,
  quality: 99.2,
  cpk: 1.67,
  mtbf: 480.0,
  mttr: 12.0,
  energy_efficiency: 0.91,
  schedule_adherence: 97.0,
  tool_life_utilization: 78.0,
  jobs_completed_today: 14,
  jobs_in_progress: 2,
  jobs_queued: 5
}' --rate 1"
```

---

## Alerts & Events

### Trigger anomaly alert

```bash
docker exec -it miracle_bridge bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && ros2 topic pub --once /miracle/cnc1/anomaly miracle_msgs/msg/AnomalyAlert '{
  machine_id: \"cnc1\",
  anomaly_type: \"vibration_spike\",
  confidence: 0.92,
  severity: 0.75,
  contributing_factors: [\"bearing_wear\", \"unbalanced_tool\"],
  feature_contributions: [0.6, 0.4],
  recommended_action: \"Inspect spindle bearings and tool holder\",
  requires_immediate_stop: false
}'"
```

### Trigger critical anomaly (E-Stop warning)

```bash
docker exec -it miracle_bridge bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && ros2 topic pub --once /miracle/cnc1/anomaly miracle_msgs/msg/AnomalyAlert '{
  machine_id: \"cnc1\",
  anomaly_type: \"thermal_runaway\",
  confidence: 0.98,
  severity: 1.0,
  contributing_factors: [\"coolant_failure\", \"excessive_depth_of_cut\"],
  feature_contributions: [0.7, 0.3],
  recommended_action: \"IMMEDIATE: Activate E-Stop and check coolant system\",
  requires_immediate_stop: true
}'"
```

### Report tool wear

```bash
docker exec -it miracle_bridge bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && ros2 topic pub --once /miracle/cnc1/tool_wear miracle_msgs/msg/ToolWearEstimate '{
  machine_id: \"cnc1\",
  tool_id: \"T1_HSS_6mm\",
  wear_percentage: 72.0,
  remaining_life_minutes: 28.0,
  confidence: 0.85,
  wear_type: \"flank_wear\",
  flank_wear_mm: 0.24,
  crater_wear_mm: 0.05,
  recommended_action: \"Schedule tool change within 30 minutes\"
}'"
```

### Security alert

```bash
docker exec -it miracle_bridge bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && ros2 topic pub --once /miracle/security/alerts miracle_msgs/msg/SecurityAlert '{
  alert_id: \"SEC-001\",
  severity: \"HIGH\",
  category: \"unauthorized_access\",
  source_node: \"cnc1_controller\",
  description: \"Unauthorized parameter modification attempt detected\",
  affected_nodes: [\"cnc1\", \"mes_gateway\"],
  recommended_action: \"Review access logs and rotate credentials\",
  requires_isolation: false,
  confidence: 0.88
}'"
```

### Job status update

```bash
docker exec -it miracle_bridge bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && ros2 topic pub /miracle/cnc1/job_status miracle_msgs/msg/JobStatus '{
  job_id: \"JOB-2024-0142\",
  machine_id: \"cnc1\",
  status: \"IN_PROGRESS\",
  program_name: \"bracket_v3.nc\",
  total_lines: 1250,
  current_line: 450,
  progress: 36.0,
  estimated_remaining_sec: 480.0,
  elapsed_sec: 270.0,
  warnings: [\"Tool wear approaching threshold\"],
  errors: []
}' --rate 1"
```

---

## Work Envelope Reference

| Machine | X (mm) | Y (mm) | Z (mm) |
|---------|--------|--------|--------|
| Bantam Explorer (cnc1) | 0 - 152.4 | 0 - 101.6 | 0 - 69.85 |
| CoastRunner CR-1 (cnc2) | 0 - 88.9 | 0 - 241.3 | 0 - 76.2 |

## Robot Joint Ranges (approx radians)

| Joint | Niryo Ned2 | xArm 6 Lite |
|-------|-----------|-------------|
| J1 (base) | -2.8 to 2.8 | -6.28 to 6.28 |
| J2 (shoulder) | -1.8 to 0.6 | -2.06 to 2.09 |
| J3 (elbow) | -1.3 to 1.6 | -3.93 to 0.19 |
| J4 (wrist 1) | -2.5 to 2.5 | -6.28 to 6.28 |
| J5 (wrist 2) | -1.7 to 1.9 | -1.69 to 3.14 |
| J6 (wrist 3) | -2.5 to 2.5 | -6.28 to 6.28 |

---

## Copy-Paste Demo Scripts

All scripts below are self-contained — paste directly into a second terminal.

### Niryo Pick-and-Place Loop

Cycles through 8 waypoints: home, reach, grab, lift, swing, drop, retreat, inspect.

```bash
docker exec -it miracle_bridge bash -c "
source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash
python3 << 'PYEOF'
import rclpy, time
from miracle_msgs.msg import RobotJointState

rclpy.init()
node = rclpy.create_node('niryo_pick_place')
pub = node.create_publisher(RobotJointState, '/miracle/robots/joint_states', 10)

waypoints = [
    ([0.0, -0.3, 0.3, 0.0, 0.5, 0.0],   'open',   'homing'),
    ([1.2, -0.5, 0.7, 0.0, 1.0, 0.4],   'open',   'approaching'),
    ([1.2, -0.2, 1.0, 0.0, 1.3, 0.4],   'closed', 'gripping'),
    ([1.2, -0.5, 0.7, 0.0, 1.0, 0.4],   'closed', 'lifting'),
    ([-0.8, -0.4, 0.6, 0.3, 0.9, -0.3], 'closed', 'transporting'),
    ([-0.8, -0.2, 0.9, 0.3, 1.2, -0.3], 'open',   'releasing'),
    ([-0.8, -0.5, 0.5, 0.0, 0.8, 0.0],  'open',   'retreating'),
    ([0.4, -0.8, 1.2, 1.5, 0.6, 0.0],   'open',   'inspecting'),
]

print('Niryo pick-and-place demo starting...')
while rclpy.ok():
    for i, (pos, grip, task) in enumerate(waypoints):
        print(f'  Waypoint {i+1}/{len(waypoints)}: {task}')
        for _ in range(25):
            msg = RobotJointState()
            msg.robot_id = 'ned2'
            msg.positions = pos
            msg.velocities = [0.0] * 6
            msg.efforts = [0.0] * 6
            msg.gripper_state = grip
            msg.task_state = task
            pub.publish(msg)
            time.sleep(0.1)
    print('Loop complete, restarting...')
PYEOF
"
```

### xArm Inspection Sequence

Sweeps through inspection poses like scanning a part from different angles.

```bash
docker exec -it miracle_bridge bash -c "
source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash
python3 << 'PYEOF'
import rclpy, time
from miracle_msgs.msg import RobotJointState

rclpy.init()
node = rclpy.create_node('xarm_inspect')
pub = node.create_publisher(RobotJointState, '/miracle/robots/joint_states', 10)

waypoints = [
    ([0.0, -0.5, 0.5, 0.0, 0.3, 0.0],    'open',   'home'),
    ([0.5, -0.8, 1.0, 0.0, 0.8, 0.0],     'open',   'scan_front'),
    ([0.5, -1.0, 1.3, 0.5, 0.5, 0.3],     'open',   'scan_top'),
    ([0.0, -1.2, 1.5, 1.0, 0.3, 0.6],     'open',   'scan_detail'),
    ([-0.5, -0.8, 1.0, 0.5, 0.8, 0.0],    'open',   'scan_left'),
    ([-0.5, -1.0, 1.3, -0.5, 0.5, -0.3],  'open',   'scan_back'),
    ([0.0, -0.6, 0.8, 0.0, 1.0, 0.0],     'closed', 'pick_part'),
    ([0.0, -0.3, 0.3, 0.0, 0.5, 0.0],     'closed', 'deliver'),
    ([0.0, -0.3, 0.3, 0.0, 0.5, 0.0],     'open',   'release'),
]

print('xArm inspection sequence starting...')
while rclpy.ok():
    for i, (pos, grip, task) in enumerate(waypoints):
        print(f'  Pose {i+1}/{len(waypoints)}: {task}')
        for _ in range(30):
            msg = RobotJointState()
            msg.robot_id = 'lite6'
            msg.positions = pos
            msg.velocities = [0.0] * 6
            msg.efforts = [0.0] * 6
            msg.gripper_state = grip
            msg.task_state = task
            pub.publish(msg)
            time.sleep(0.1)
    print('Inspection loop complete, restarting...')
PYEOF
"
```

### Both Robots + CNC Coordinated Demo

Full manufacturing cell: Niryo tends the CNC, xArm inspects parts, CNC cuts a circular pocket.

```bash
docker exec -it miracle_bridge bash -c "
source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash
python3 << 'PYEOF'
import rclpy, time, math
from miracle_msgs.msg import RobotJointState, MachineState

rclpy.init()
node = rclpy.create_node('cell_coordinator')
robot_pub = node.create_publisher(RobotJointState, '/miracle/robots/joint_states', 10)
cnc_pub = node.create_publisher(MachineState, '/miracle/cnc1/state', 10)

print('Full cell coordinated demo starting...')
t = 0.0
while rclpy.ok():
    # --- Niryo: smooth tending motion with gripper toggle ---
    ned2 = RobotJointState()
    ned2.robot_id = 'ned2'
    ned2.positions = [
        0.8 * math.sin(t * 0.3),
        -0.4 + 0.2 * math.sin(t * 0.4),
        0.6 + 0.3 * math.sin(t * 0.5),
        0.2 * math.sin(t * 0.8),
        1.0 + 0.2 * math.sin(t * 0.6),
        0.3 * math.sin(t * 0.7),
    ]
    ned2.velocities = [0.0] * 6
    ned2.efforts = [0.0] * 6
    ned2.gripper_state = 'closed' if math.sin(t * 0.2) > 0 else 'open'
    ned2.task_state = 'tending'
    robot_pub.publish(ned2)

    # --- xArm: scanning / inspection sweep ---
    lite6 = RobotJointState()
    lite6.robot_id = 'lite6'
    lite6.positions = [
        -0.5 * math.sin(t * 0.25 + 1.0),
        -0.7 + 0.3 * math.sin(t * 0.35),
        1.0 + 0.3 * math.cos(t * 0.4),
        0.4 * math.sin(t * 0.6),
        -0.3 + 0.4 * math.sin(t * 0.3),
        0.2 * math.sin(t * 0.5),
    ]
    lite6.velocities = [0.0] * 6
    lite6.efforts = [0.0] * 6
    lite6.gripper_state = 'open'
    lite6.task_state = 'inspecting'
    robot_pub.publish(lite6)

    # --- CNC: circular pocket toolpath ---
    cnc = MachineState()
    cnc.machine_id = 'cnc1'
    cnc.status = 'RUNNING'
    cnc.spindle_speed = 12000.0
    cnc.feed_rate = 800.0
    cnc.axis_positions = [
        76.0 + 40.0 * math.cos(t * 0.8),
        50.0 + 30.0 * math.sin(t * 0.8),
        15.0 + 5.0 * math.sin(t * 0.2),
    ]
    cnc.axis_velocities = [
        -40.0 * math.sin(t * 0.8) * 0.8,
        30.0 * math.cos(t * 0.8) * 0.8,
        0.0,
    ]
    cnc.spindle_load = 40.0 + 8.0 * math.sin(t * 2.5)
    cnc.coolant_level = 90.0
    cnc.current_program = 'coordinated_demo.nc'
    cnc.current_line = int(t * 10) % 500
    cnc.cycle_time_elapsed = t
    cnc.cycle_time_remaining = max(0.0, 300.0 - t)
    cnc_pub.publish(cnc)

    if int(t * 10) % 100 == 0:
        print(f'  t={t:.1f}s  Niryo:{ned2.task_state}  xArm:{lite6.task_state}  CNC:line {cnc.current_line}')

    time.sleep(0.1)
    t += 0.1
PYEOF
"
```

### Niryo Smooth Wave (Continuous)

Smooth sinusoidal motion across all 6 joints — good for visual demos.

```bash
docker exec -it miracle_bridge bash -c "
source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash
python3 << 'PYEOF'
import rclpy, time, math
from miracle_msgs.msg import RobotJointState

rclpy.init()
node = rclpy.create_node('niryo_wave')
pub = node.create_publisher(RobotJointState, '/miracle/robots/joint_states', 10)

print('Niryo smooth wave demo...')
t = 0.0
while rclpy.ok():
    msg = RobotJointState()
    msg.robot_id = 'ned2'
    msg.positions = [
        0.8 * math.sin(t * 0.5),
        -0.4 + 0.3 * math.sin(t * 0.7),
        0.5 + 0.4 * math.sin(t * 0.3),
        0.3 * math.sin(t * 1.1),
        1.0 + 0.3 * math.cos(t * 0.6),
        0.5 * math.sin(t * 0.9),
    ]
    msg.velocities = [0.0] * 6
    msg.efforts = [0.0] * 6
    msg.gripper_state = 'open'
    msg.task_state = 'wave_demo'
    pub.publish(msg)
    time.sleep(0.1)
    t += 0.1
PYEOF
"
```

### CNC Spiral Pocket (Expanding)

Bantam cuts an expanding spiral with Z step-down — looks great on the dashboard.

```bash
docker exec -it miracle_bridge bash -c "
source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash
python3 << 'PYEOF'
import rclpy, time, math
from miracle_msgs.msg import MachineState

rclpy.init()
node = rclpy.create_node('cnc_spiral')
pub = node.create_publisher(MachineState, '/miracle/cnc1/state', 10)

print('Bantam spiral pocket demo...')
t = 0.0
while rclpy.ok():
    radius = 5.0 + 25.0 * (0.5 + 0.5 * math.sin(t * 0.1))
    z_depth = 10.0 + 20.0 * (0.5 + 0.5 * math.cos(t * 0.05))

    msg = MachineState()
    msg.machine_id = 'cnc1'
    msg.status = 'RUNNING'
    msg.spindle_speed = 12000.0
    msg.feed_rate = 600.0 + 200.0 * math.sin(t * 0.3)
    msg.axis_positions = [
        76.0 + radius * math.cos(t * 1.5),
        50.0 + radius * math.sin(t * 1.5),
        z_depth,
    ]
    msg.axis_velocities = [
        -radius * math.sin(t * 1.5) * 1.5,
        radius * math.cos(t * 1.5) * 1.5,
        0.0,
    ]
    msg.spindle_load = 35.0 + 15.0 * (radius / 30.0)
    msg.coolant_level = 88.0
    msg.current_program = 'spiral_pocket.nc'
    msg.current_line = int(t * 15) % 800
    msg.cycle_time_elapsed = t
    msg.cycle_time_remaining = max(0.0, 120.0 - (t % 120.0))
    pub.publish(msg)

    time.sleep(0.05)
    t += 0.05
PYEOF
"
```

### CNC Axis Validation (Tests Each Axis Individually Then Combined)

Steps through: X only, Y only, Z only, XY diagonal, full 3-axis spiral. Prints which axis is being tested so you can verify each one moves correctly.

```bash
docker exec -it miracle_bridge bash -c "
source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash
python3 << 'PYEOF'
import rclpy, time, math
from miracle_msgs.msg import MachineState

rclpy.init()
node = rclpy.create_node('cnc_axis_test')
pub = node.create_publisher(MachineState, '/miracle/cnc1/state', 10)

def send(x, y, z, label, duration=3.0):
    print(f'  >> {label}: X={x:.1f} Y={y:.1f} Z={z:.1f}')
    steps = int(duration / 0.05)
    for _ in range(steps):
        msg = MachineState()
        msg.machine_id = 'cnc1'
        msg.status = 'RUNNING'
        msg.spindle_speed = 12000.0
        msg.feed_rate = 800.0
        msg.axis_positions = [float(x), float(y), float(z)]
        msg.axis_velocities = [0.0, 0.0, 0.0]
        msg.spindle_load = 30.0
        msg.coolant_level = 90.0
        msg.current_program = 'axis_test.nc'
        msg.current_line = 0
        msg.cycle_time_elapsed = 0.0
        msg.cycle_time_remaining = 0.0
        pub.publish(msg)
        time.sleep(0.05)

print('=== CNC AXIS VALIDATION ===')
print()

# Home
send(0, 0, 0, 'HOME (origin)')

# Test 1: X axis isolation
print('--- TEST 1: X AXIS (gantry should move LEFT-RIGHT) ---')
send(0, 0, 0, 'X=0 (start)')
send(140, 0, 0, 'X=140 (far right)')
send(0, 0, 0, 'X=0 (back to left)')

# Test 2: Y axis isolation
print('--- TEST 2: Y AXIS (table should move FRONT-BACK) ---')
send(0, 0, 0, 'Y=0 (start)')
send(0, 90, 0, 'Y=90 (far back)')
send(0, 0, 0, 'Y=0 (back to front)')

# Test 3: Z axis isolation
print('--- TEST 3: Z AXIS (head should move DOWN into cut) ---')
send(0, 0, 0, 'Z=0 (top)')
send(0, 0, 60, 'Z=60 (plunged down)')
send(0, 0, 0, 'Z=0 (retracted up)')

# Test 4: XY diagonal
print('--- TEST 4: XY DIAGONAL (gantry + table together) ---')
send(0, 0, 0, 'Start corner')
send(140, 90, 0, 'Far corner')
send(0, 0, 0, 'Back to start')

# Test 5: Full 3-axis spiral
print('--- TEST 5: 3-AXIS SPIRAL (all axes moving) ---')
t = 0.0
for _ in range(200):
    x = 76.0 + 60.0 * math.cos(t)
    y = 50.0 + 40.0 * math.sin(t)
    z = 10.0 + 25.0 * (0.5 + 0.5 * math.sin(t * 0.5))
    msg = MachineState()
    msg.machine_id = 'cnc1'
    msg.status = 'RUNNING'
    msg.spindle_speed = 12000.0
    msg.feed_rate = 800.0
    msg.axis_positions = [x, y, z]
    msg.axis_velocities = [0.0, 0.0, 0.0]
    msg.spindle_load = 40.0
    msg.coolant_level = 90.0
    msg.current_program = 'axis_test.nc'
    msg.current_line = int(t * 10)
    msg.cycle_time_elapsed = t
    msg.cycle_time_remaining = 0.0
    pub.publish(msg)
    time.sleep(0.05)
    t += 0.1

# Home
send(0, 0, 0, 'HOME (done)')

print()
print('=== VALIDATION COMPLETE ===')
print('Check that:')
print('  Test 1: Only X-Axis group moved (left-right)')
print('  Test 2: Only Y-Axis group moved (front-back)')
print('  Test 3: Only Z-Axis group moved (up-down)')
print('  Test 4: X + Y moved together, Z stayed still')
print('  Test 5: All three axes moved smoothly in a spiral')
node.destroy_node()
rclpy.shutdown()
PYEOF
"
```

### Full Cell Demo: Both Robots + Both CNCs + Alerts

Everything running at once: Niryo tends Bantam, xArm tends CoastRunner, both CNCs cutting, periodic alerts.

```bash
docker exec -it miracle_bridge bash -c "
source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash
python3 << 'PYEOF'
import rclpy, time, math
from miracle_msgs.msg import RobotJointState, MachineState, SystemKPIs

rclpy.init()
node = rclpy.create_node('full_cell_demo')
robot_pub = node.create_publisher(RobotJointState, '/miracle/robots/joint_states', 10)
cnc1_pub = node.create_publisher(MachineState, '/miracle/cnc1/state', 10)
cnc2_pub = node.create_publisher(MachineState, '/miracle/cnc2/state', 10)
kpi_pub = node.create_publisher(SystemKPIs, '/miracle/system_kpis', 10)

print('=== FULL MANUFACTURING CELL DEMO ===')
print('  Niryo Ned2 -> tending Bantam CNC')
print('  xArm Lite6 -> tending CoastRunner CNC')
print('  Both CNCs cutting toolpaths')
print('  Dashboard KPIs updating')
print()

t = 0.0
while rclpy.ok():
    # --- Niryo: tending Bantam ---
    ned2 = RobotJointState()
    ned2.robot_id = 'ned2'
    phase = (t % 20.0) / 20.0
    if phase < 0.25:
        ned2.positions = [0.8*math.sin(t*0.5), -0.3, 0.4, 0.0, 0.8, 0.0]
        ned2.gripper_state = 'open'
        ned2.task_state = 'approaching'
    elif phase < 0.5:
        ned2.positions = [0.6, -0.2+0.1*math.sin(t), 0.9, 0.1*math.sin(t*2), 1.2, 0.3]
        ned2.gripper_state = 'closed'
        ned2.task_state = 'loading'
    elif phase < 0.75:
        ned2.positions = [-0.5*math.sin(t*0.4), -0.5, 0.6, 0.2*math.cos(t), 1.0, -0.2]
        ned2.gripper_state = 'closed'
        ned2.task_state = 'transporting'
    else:
        ned2.positions = [0.0, -0.4, 0.5, 0.0, 0.9, 0.0]
        ned2.gripper_state = 'open'
        ned2.task_state = 'waiting'
    ned2.velocities = [0.0] * 6
    ned2.efforts = [0.0] * 6
    robot_pub.publish(ned2)

    # --- xArm: tending CoastRunner ---
    lite6 = RobotJointState()
    lite6.robot_id = 'lite6'
    phase6 = ((t + 5.0) % 16.0) / 16.0
    if phase6 < 0.3:
        lite6.positions = [0.4*math.cos(t*0.3), -0.6, 0.8, 0.0, 0.5, 0.1*math.sin(t)]
        lite6.gripper_state = 'open'
        lite6.task_state = 'scanning'
    elif phase6 < 0.6:
        lite6.positions = [-0.3, -0.8+0.2*math.sin(t*0.5), 1.1, 0.3, 0.4, 0.2]
        lite6.gripper_state = 'closed'
        lite6.task_state = 'loading'
    else:
        lite6.positions = [0.2*math.sin(t*0.4), -0.5, 0.6, -0.1, 0.7, 0.0]
        lite6.gripper_state = 'open'
        lite6.task_state = 'idle'
    lite6.velocities = [0.0] * 6
    lite6.efforts = [0.0] * 6
    robot_pub.publish(lite6)

    # --- Bantam CNC: circular pocket ---
    cnc1 = MachineState()
    cnc1.machine_id = 'cnc1'
    cnc1.status = 'RUNNING'
    cnc1.spindle_speed = 12000.0
    cnc1.feed_rate = 800.0
    r1 = 30.0 + 15.0 * math.sin(t * 0.15)
    cnc1.axis_positions = [
        76.0 + r1 * math.cos(t * 0.8),
        50.0 + r1 * math.sin(t * 0.8),
        15.0 + 8.0 * math.sin(t * 0.1),
    ]
    cnc1.axis_velocities = [0.0, 0.0, 0.0]
    cnc1.spindle_load = 40.0 + 10.0 * math.sin(t * 2.0)
    cnc1.coolant_level = 90.0 - 0.01 * t
    cnc1.current_program = 'full_demo_pocket.nc'
    cnc1.current_line = int(t * 10) % 600
    cnc1.cycle_time_elapsed = t
    cnc1.cycle_time_remaining = max(0.0, 300.0 - (t % 300.0))
    cnc1_pub.publish(cnc1)

    # --- CoastRunner CNC: linear facing pass ---
    cnc2 = MachineState()
    cnc2.machine_id = 'cnc2'
    cnc2.status = 'RUNNING'
    cnc2.spindle_speed = 10000.0
    cnc2.feed_rate = 600.0
    sweep = 80.0 * (0.5 + 0.5 * math.sin(t * 0.3))
    cnc2.axis_positions = [
        44.0,
        20.0 + sweep * 2.5,
        30.0 + 10.0 * math.sin(t * 0.08),
    ]
    cnc2.axis_velocities = [0.0, 0.0, 0.0]
    cnc2.spindle_load = 35.0 + 5.0 * math.sin(t * 1.5)
    cnc2.coolant_level = 85.0
    cnc2.current_program = 'facing_demo.nc'
    cnc2.current_line = int(t * 8) % 400
    cnc2.cycle_time_elapsed = t
    cnc2.cycle_time_remaining = max(0.0, 200.0 - (t % 200.0))
    cnc2_pub.publish(cnc2)

    # --- KPIs: slowly drifting ---
    if int(t * 10) % 20 == 0:
        kpi = SystemKPIs()
        kpi.oee = 88.0 + 8.0 * math.sin(t * 0.02)
        kpi.availability = 96.0 + 3.0 * math.sin(t * 0.015)
        kpi.performance = 93.0 + 5.0 * math.sin(t * 0.025)
        kpi.quality = 98.0 + 1.5 * math.sin(t * 0.01)
        kpi.cpk = 1.5 + 0.3 * math.sin(t * 0.03)
        kpi.mtbf = 450.0 + 50.0 * math.sin(t * 0.008)
        kpi.mttr = 15.0 + 5.0 * math.sin(t * 0.02)
        kpi.energy_efficiency = 0.88 + 0.05 * math.sin(t * 0.012)
        kpi.schedule_adherence = 95.0 + 4.0 * math.sin(t * 0.018)
        kpi.tool_life_utilization = 70.0 + 15.0 * math.sin(t * 0.025)
        kpi.jobs_completed_today = 10 + int(t / 30.0)
        kpi.jobs_in_progress = 2
        kpi.jobs_queued = max(0, 8 - int(t / 60.0))
        kpi_pub.publish(kpi)

    if int(t * 10) % 100 == 0:
        print(f'  t={t:.0f}s  Bantam:X={cnc1.axis_positions[0]:.0f}/Y={cnc1.axis_positions[1]:.0f}/Z={cnc1.axis_positions[2]:.0f}  '
              f'CR1:Y={cnc2.axis_positions[1]:.0f}  Ned2:{ned2.task_state}  Lite6:{lite6.task_state}')

    time.sleep(0.05)
    t += 0.05
PYEOF
"
```

---

## Interactive Python Shell

For custom experimentation:

```bash
docker exec -it miracle_bridge bash -c "source /opt/ros/jazzy/setup.bash && source /miracle_ws/install/setup.bash && python3"
```

Then in the Python shell:

```python
import rclpy, math, time
from miracle_msgs.msg import RobotJointState, MachineState

rclpy.init()
node = rclpy.create_node('motion_commander')
robot_pub = node.create_publisher(RobotJointState, '/miracle/robots/joint_states', 10)
cnc_pub = node.create_publisher(MachineState, '/miracle/cnc1/state', 10)

# Send a single robot pose:
msg = RobotJointState()
msg.robot_id = 'ned2'
msg.positions = [0.5, -0.3, 0.8, 0.0, 1.2, 0.0]
msg.velocities = [0.0] * 6
msg.efforts = [0.0] * 6
msg.gripper_state = 'open'
msg.task_state = 'manual'
for _ in range(30):  # hold for 3 sec
    robot_pub.publish(msg)
    time.sleep(0.1)

# Ctrl+C then: node.destroy_node(); rclpy.shutdown()
```

---

## Troubleshooting

| Issue | Fix |
|-------|-----|
| "Connection refused" in Unity | Bridge container isn't running. Start it with the `docker run` command above |
| "port already allocated" | `docker rm miracle_bridge` or `docker stop <name>` the conflicting container |
| Connect/disconnect loop (succeeded then failed every second) | MultiThreadedExecutor crash. Rebuild image: `cd miracle_ws && docker build -f docker/Dockerfile.ros2 -t miracle_ros2:latest .` |
| "The passed message type is invalid" | Wrong container. Use `miracle_ros2:latest` (the one we built), not old compose images |
| Robot still sine-waving | Make sure Unity console shows "ROS Connection succeeded". Run Wire Dashboard again |
| SHM Transport errors | Harmless Docker warning, ignore |
| Robot doesn't respond to commands | Check `robot_id` matches exactly: `ned2` or `lite6` |
| CNC says "Using Transform motion" | ArticulationBody joints missing. Run `MIRACLE > Wire Dashboard` then re-enter Play |
| CNC all axes move one child | Same as above — Transform fallback moves one object by all offsets. Wire Dashboard fixes it |
| "Prefab instance" errors in Wire Dashboard | FBX wasn't unpacked. Should auto-unpack now. If stuck, right-click FBX in Hierarchy > Prefab > Unpack Completely, then Wire Dashboard |
| CNC doesn't respond to ROS pub | Check `machine_id` matches: `cnc1` (Bantam) or `cnc2` (CoastRunner). Check topic name matches |
