# Quickstart Guide: Module 1 – The Robotic Nervous System (ROS 2)

## Prerequisites

Before starting this module, ensure you have:

1. **ROS 2 Installation**:
   - ROS 2 Humble Hawksbill or Iron Irwini installed
   - Python 3.10 or higher
   - Basic command-line familiarity

2. **Development Environment**:
   - Text editor or IDE with Python support
   - Terminal/shell access
   - Git for version control (optional but recommended)

3. **Basic Knowledge**:
   - Fundamental Python programming concepts
   - Basic understanding of Linux/Unix commands
   - Familiarity with package managers (pip, apt)

## Setting Up Your Environment

### 1. ROS 2 Environment Setup

```bash
# Source your ROS 2 installation
source /opt/ros/humble/setup.bash  # For ROS 2 Humble
# OR
source /opt/ros/iron/setup.bash    # For ROS 2 Iron

# Create a workspace for examples
mkdir -p ~/ros2_book_ws/src
cd ~/ros2_book_ws
colcon build
source install/setup.bash
```

### 2. Verify Installation

```bash
# Check ROS 2 version
ros2 --version

# Test basic ROS 2 functionality
ros2 topic list
```

## Running Your First Example

### 1. Simple Publisher/Subscriber

Create a basic publisher node:

```bash
# Navigate to your workspace
cd ~/ros2_book_ws/src

# Create a simple Python package
ros2 pkg create --build-type ament_python simple_ros_examples
cd simple_ros_examples/simple_ros_examples
```

Create `publisher_member_function.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Running the Example

```bash
# Terminal 1: Run the publisher
cd ~/ros2_book_ws
source install/setup.bash
python3 -m simple_ros_examples.publisher_member_function

# Terminal 2: Run the subscriber (in another terminal)
cd ~/ros2_book_ws
source install/setup.bash
ros2 run demo_nodes_cpp listener
```

## Testing URDF Examples

### 1. Validate URDF Syntax

```bash
# Check if a URDF file is syntactically correct
check_urdf /path/to/your/robot.urdf

# Parse and display URDF information
urdf_to_graphiz /path/to/your/robot.urdf
```

### 2. Simple Humanoid URDF Example

Create `simple_humanoid.urdf`:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.2 0.2"/>
      </geometry>
    </visual>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.5 0.2"/>
      </geometry>
    </visual>
  </link>

  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.35"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
  </link>

  <joint name="torso_to_head" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.4"/>
  </joint>
</robot>
```

Validate the URDF:

```bash
check_urdf simple_humanoid.urdf
```

## Docusaurus Book Setup

### 1. Install Dependencies

```bash
# Navigate to book directory
cd /path/to/your/book

# Install dependencies
npm install
```

### 2. Run the Book Locally

```bash
# Start development server
npm run start

# Build for production
npm run build
```

## Troubleshooting Common Issues

### ROS 2 Environment Not Found
- Ensure you've sourced the correct ROS 2 setup file
- Check that ROS 2 is properly installed: `echo $ROS_DISTRO`

### Python Package Import Errors
- Make sure your workspace is sourced: `source install/setup.bash`
- Check that the package is built: `colcon build`

### URDF Validation Errors
- Check XML syntax (properly closed tags, correct attribute names)
- Ensure all referenced files exist
- Verify joint connections reference existing links

## Next Steps

1. Complete Chapter 1: ROS 2 Fundamentals
2. Practice with the provided code examples
3. Move to Chapter 2: Python-to-ROS Control Bridge
4. Explore Chapter 3: Humanoid Robot URDF