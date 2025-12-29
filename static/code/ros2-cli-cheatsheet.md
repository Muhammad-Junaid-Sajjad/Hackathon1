# ROS 2 CLI Cheat Sheet

## Topic Operations
| Command | Purpose | Example |
|---------|---------|---------|
| `ros2 topic list` | List all topics | `ros2 topic list -t` (with types) |
| `ros2 topic echo <topic>` | Print messages | `ros2 topic echo /imu/data` |
| `ros2 topic hz <topic>` | Measure frequency | `ros2 topic hz /cmd_vel` |
| `ros2 topic bw <topic>` | Measure bandwidth | `ros2 topic bw /camera/depth` |
| `ros2 topic info <topic>` | Show topic details | `ros2 topic info /joint_commands` |
| `ros2 topic pub <topic> <type> <data>` | Publish message | `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"` |

## Node Operations
| Command | Purpose | Example |
|---------|---------|---------|
| `ros2 node list` | List all nodes | `ros2 node list` |
| `ros2 node info <node>` | Show node details | `ros2 node info /camera_node` |
| `ros2 run <package> <executable>` | Launch node | `ros2 run humanoid_control joint_cmd_publisher` |

## Service Operations
| Command | Purpose | Example |
|---------|---------|---------|
| `ros2 service list` | List all services | `ros2 service list -t` (with types) |
| `ros2 service type <service>` | Show service type | `ros2 service type /compute_ik` |
| `ros2 service call <service> <type> <args>` | Call service | `ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"` |

## Parameter Operations
| Command | Purpose | Example |
|---------|---------|---------|
| `ros2 param list` | List parameters | `ros2 param list /camera_node` |
| `ros2 param get <node> <param>` | Get parameter | `ros2 param get /camera_node exposure` |
| `ros2 param set <node> <param> <value>` | Set parameter | `ros2 param set /camera_node exposure 10000` |
| `ros2 param dump <node>` | Save params to YAML | `ros2 param dump /camera_node` |
| `ros2 param load <node> <file>` | Load params from YAML | `ros2 param load /camera_node camera.yaml` |

## Interface (Message/Service) Operations
| Command | Purpose | Example |
|---------|---------|---------|
| `ros2 interface list` | List all interfaces | `ros2 interface list \| grep Image` |
| `ros2 interface show <type>` | Show definition | `ros2 interface show sensor_msgs/msg/Imu` |
| `ros2 interface proto <type>` | Generate prototype | `ros2 interface proto sensor_msgs/msg/Image` |

## Bag Operations
| Command | Purpose | Example |
|---------|---------|---------|
| `ros2 bag record <topics>` | Record topics | `ros2 bag record /camera/depth /imu/data` |
| `ros2 bag record -a` | Record all topics | `ros2 bag record -a` (use sparingly) |
| `ros2 bag play <bag>` | Replay bag | `ros2 bag play rosbag2_2024_01_15-10_30_45` |
| `ros2 bag info <bag>` | Show bag metadata | `ros2 bag info rosbag2_2024_01_15-10_30_45` |

## System Diagnosis
| Command | Purpose | Example |
|---------|---------|---------|
| `ros2 doctor` | System health check | `ros2 doctor --report` |
| `ros2 wtf` | Alias for doctor | `ros2 wtf` |
| `rqt_graph` | Visualize node graph | `rqt_graph` |
| `rqt_console` | Log message viewer | `rqt_console` |

## Transform (TF2) Operations
| Command | Purpose | Example |
|---------|---------|---------|
| `ros2 run tf2_tools view_frames` | Generate TF tree PDF | `ros2 run tf2_tools view_frames` |
| `ros2 run tf2_ros tf2_echo <parent> <child>` | Show transform | `ros2 run tf2_ros tf2_echo base_link camera_link` |
| `ros2 run tf2_ros tf2_monitor` | Monitor TF rates | `ros2 run tf2_ros tf2_monitor` |
| `ros2 topic echo /tf` | View dynamic transforms | `ros2 topic echo /tf` |
| `ros2 topic echo /tf_static` | View static transforms | `ros2 topic echo /tf_static` |

## Environment Variables
| Variable | Purpose | Example |
|----------|---------|---------|
| `ROS_DOMAIN_ID` | Network isolation (0-101) | `export ROS_DOMAIN_ID=42` |
| `ROS_LOCALHOST_ONLY` | Restrict to localhost | `export ROS_LOCALHOST_ONLY=1` |
| `RMW_IMPLEMENTATION` | DDS middleware | `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` |
| `RCUTILS_COLORIZED_OUTPUT` | Enable colored logs | `export RCUTILS_COLORIZED_OUTPUT=1` |

## Common Workflows

### Debug Missing Messages
```bash
# 1. Check if topic exists
ros2 topic list | grep <topic_name>

# 2. Check publisher count
ros2 topic info <topic_name>

# 3. Echo topic to verify data
ros2 topic echo <topic_name>

# 4. Check node connectivity
rqt_graph
```

### Capture Dataset for Training
```bash
# Record camera + IMU for 5 minutes
ros2 bag record --duration 300 --compression-mode file \
  /camera/color/image_raw /camera/depth/image_raw /imu/data

# Verify recording
ros2 bag info rosbag2_<timestamp>
```

### Test Service Without Code
```bash
# 1. Check service type
ros2 service type /compute_ik

# 2. Show message structure
ros2 interface show humanoid_control/srv/ComputeIK

# 3. Call service with test data
ros2 service call /compute_ik humanoid_control/srv/ComputeIK \
  "{target_pose: {position: {x: 0.5, y: 0.2, z: 1.2}}}"
```
