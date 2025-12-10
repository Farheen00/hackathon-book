# Quickstart Guide: Module 3 – The AI-Robot Brain (NVIDIA Isaac)

## Prerequisites

Before starting this module, ensure you have:

1. **NVIDIA Isaac Sim Installation**:
   - Isaac Sim 2023.1 or later installed
   - NVIDIA GPU with RTX series or equivalent
   - CUDA 11.8 or later with compatible drivers
   - Basic command-line familiarity

2. **ROS 2 Environment**:
   - ROS 2 Humble Hawksbill installed
   - Isaac ROS packages installed
   - Nav2 packages for navigation
   - Python 3.10+ with pip

3. **Development Environment**:
   - Text editor or IDE with Python support
   - Terminal/shell access
   - Git for version control (optional but recommended)

4. **Basic Knowledge**:
   - Completion of Module 1 (ROS 2 fundamentals) and Module 2 (simulation basics)
   - Understanding of basic computer graphics concepts
   - Familiarity with 3D coordinate systems and transformations

## Setting Up Your Environment

### 1. Isaac Sim Environment Setup

```bash
# Verify Isaac Sim installation
isaac-sim --version

# Check GPU compatibility
nvidia-smi

# Create a workspace for examples
mkdir -p ~/isaac_workspace/projects
cd ~/isaac_workspace
```

### 2. Verify Isaac ROS Installation

```bash
# Check Isaac ROS packages
ros2 pkg list | grep isaac

# Verify Isaac ROS perception nodes
ros2 component types | grep isaac_ros
```

### 3. Nav2 Setup

```bash
# Check Nav2 installation
ros2 launch nav2_bringup navigation_launch.py --show-args

# Verify navigation capabilities
ros2 pkg list | grep nav2
```

## Running Your First Isaac Sim Example

### 1. Photorealistic Environment

Create a basic environment configuration:

```json
{
  "name": "basic_office",
  "description": "Simple office environment for synthetic data generation",
  "assets": [
    {
      "name": "floor",
      "type": "plane",
      "material": "wood_floor",
      "size": [10, 10]
    },
    {
      "name": "table",
      "type": "cuboid",
      "material": "wood",
      "dimensions": [1.5, 0.8, 0.8]
    }
  ],
  "lighting": {
    "type": "dome",
    "intensity": 3000,
    "color": [1.0, 1.0, 1.0]
  },
  "domain_randomization": {
    "lighting_variation": 0.1,
    "material_variation": 0.05,
    "object_position_variation": 0.02
  }
}
```

### 2. Synthetic Data Generation

Configure synthetic data generation:

```python
# Example synthetic data generation script
import omni
from omni.isaac.synthetic_utils import SyntheticDataHelper

# Initialize synthetic data helper
synthetic_data = SyntheticDataHelper()
synthetic_data.set_resolution(640, 480)

# Configure sensor outputs
synthetic_data.enable_rgb_output()
synthetic_data.enable_depth_output()
synthetic_data.enable_segmentation_output()

# Generate dataset
dataset = synthetic_data.generate_dataset(
    num_samples=1000,
    environment_config="basic_office.json",
    output_dir="~/isaac_workspace/datasets/basic_office_dataset"
)
```

## Running Your First Isaac ROS Example

### 1. VSLAM Pipeline

Create a basic VSLAM configuration:

```yaml
# vslam_config.yaml
vslam_node:
  ros__parameters:
    # Camera parameters
    camera_matrix: [616.272, 0.0, 310.559, 0.0, 616.062, 229.153, 0.0, 0.0, 1.0]
    distortion_coefficients: [-0.41562, 0.174405, 0.0014309, 0.00109547, 0.0]

    # Processing parameters
    enable_rectification: true
    processing_rate: 30.0

    # Hardware acceleration
    use_cuda: true
    cuda_device_id: 0
```

### 2. Running the VSLAM Pipeline

```bash
# Launch Isaac ROS VSLAM pipeline
ros2 launch isaac_ros_bringup vslam.launch.py config_file:=path/to/vslam_config.yaml

# Monitor the output topics
ros2 topic echo /tracking/pose
ros2 topic echo /slam/map
```

## Running Your First Nav2 Example

### 1. Bipedal Navigation Configuration

Create a navigation configuration for bipedal robots:

```yaml
# bipedal_nav2_params.yaml
bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    default_nav_through_poses_bt_xml: "navigate_w_replanning_and_recovery.xml"
    default_nav_to_pose_bt_xml: "navigate_w_replanning_and_recovery.xml"
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_smooth_path_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_assisted_teleop_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_drive_on_heading_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_globally_consistent_localization_condition_bt_node
    - nav2_is_path_valid_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_on_amcl_node_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_truncate_path_local_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node
    - nav2_controller_cancel_bt_node
    - nav2_path_longer_on_approach_bt_node
    - nav2_wait_cancel_bt_node
    - nav2_spin_cancel_bt_node
    - nav2_back_up_cancel_bt_node
    - nav2_assisted_teleop_cancel_bt_node
    - nav2_drive_on_heading_cancel_bt_node
    - nav2_is_task_canceling_condition_bt_node

bipedal_local_planner:
  ros__parameters:
    use_sim_time: true
    # Bipedal-specific constraints
    balance_margin: 0.15  # Safety margin for bipedal balance
    step_size_max: 0.3    # Maximum step size
    step_height_max: 0.1  # Maximum step height
    stance_width: 0.2     # Distance between feet in stance phase
    gait_type: "walking"  # Walking gait for stability

### 2. Running Bipedal Navigation

```bash
# Launch Nav2 with bipedal configuration
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  params_file:=path/to/bipedal_nav2_params.yaml

# Send a navigation goal
ros2 action send_goal /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {pose: {position: {x: 2.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}, header: {frame_id: 'map'}}}"
```

## Integration Example: Complete AI-Robot System

### 1. Connecting Isaac Sim, Isaac ROS, and Nav2

Create a complete integration example:

```bash
# Terminal 1: Launch Isaac Sim environment
isaac-sim --config="basic_office.json"

# Terminal 2: Launch Isaac ROS perception pipeline
ros2 launch isaac_ros_bringup perception_pipeline.launch.py

# Terminal 3: Launch Nav2 navigation
ros2 launch nav2_bringup navigation_launch.py params_file:=bipedal_nav2_params.yaml

# Terminal 4: Monitor the complete system
ros2 topic list
ros2 run rviz2 rviz2
```

## Troubleshooting Common Issues

### Isaac Sim Environment Not Loading
- Ensure your GPU supports Isaac Sim's rendering requirements
- Check that NVIDIA drivers are properly installed: `nvidia-smi`
- Verify Isaac Sim installation: `isaac-sim --version`

### Isaac ROS Perception Nodes Not Responding
- Verify Isaac ROS packages are properly installed
- Check GPU acceleration is enabled in Isaac ROS nodes
- Ensure sensor data is being published to the correct topics

### Nav2 Navigation Fails
- Check that robot pose is properly localized in the map
- Verify navigation parameters are appropriate for bipedal constraints
- Ensure obstacle detection is working correctly

### Domain Transfer Performance Issues
- Increase synthetic dataset diversity
- Adjust domain randomization parameters
- Fine-tune model on limited real-world data

## Next Steps

1. Complete Chapter 1: Isaac Sim - Photorealistic Simulation & Synthetic Data
2. Practice with the provided Isaac Sim examples
3. Move to Chapter 2: Isaac ROS - VSLAM and Hardware-Accelerated Perception
4. Explore Chapter 3: Nav2 Path Planning - Bipedal Humanoid Navigation