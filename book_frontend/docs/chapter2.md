---
sidebar_position: 2
---

# Chapter 2: Python Agents with rclpy

In Chapter 1, we learned about the theoretical concepts of ROS 2 nodes, topics, and services. Now, we'll put that knowledge into practice by implementing simple Python nodes using `rclpy`, the ROS 2 client library for Python. We will create a publisher (talker) and a subscriber (listener) to demonstrate inter-node communication.

## 1. Setting up Your ROS 2 Workspace

Before we start coding, ensure your ROS 2 environment is sourced. For ROS 2 Humble Hawksbill, you would typically run:

```bash
source /opt/ros/humble/setup.bash
```

Next, navigate to your project root (e.g., `~/hackathon-1`) and create a new ROS 2 workspace and our example package:

```bash
# In your project root
mkdir -p ros2_ws/src
cd ros2_ws/src
# Create the ros2_examples package (if you haven't already)
ros2 pkg create --build-type ament_python ros2_examples
cd ros2_examples
```

*(Note: In our project structure, the `ros2_examples` package is directly in the project root. You can adapt the paths as needed.)*

## 2. Implementing the Publisher (Talker) Node

Create a new file `ros2_examples/ros2_examples/publisher_node.py` with the following content:

```python
# ros2_examples/ros2_examples/publisher_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        # Create a publisher that sends String messages to 'topic' every 0.5 seconds
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS 2: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
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

## 3. Implementing the Subscriber (Listener) Node

Create a new file `ros2_examples/ros2_examples/subscriber_node.py` with the following content:

```python
# ros2_examples/ros2_examples/subscriber_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(selfself):
        super().__init__('minimal_subscriber')
        # Create a subscription to the 'topic' that calls listener_callback
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 4. Updating `setup.py`

Modify your `ros2_examples/setup.py` file to include the entry points for your new nodes:

```python
# ros2_examples/setup.py
from setuptools import find_packages, setup

package_name = 'ros2_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Python examples for the ROS 2 book module',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'minimal_publisher = ros2_examples.publisher_node:main',
            'minimal_subscriber = ros2_examples.subscriber_node:main',
        ],
    },
)
```

## 5. Creating a Launch File

Create a launch file `ros2_examples/launch/example.launch.py` to easily start both nodes:

```python
# ros2_examples/launch/example.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_examples',
            executable='minimal_publisher',
            name='talker',
            output='screen'
        ),
        Node(
            package='ros2_examples',
            executable='minimal_subscriber',
            name='listener',
            output='screen'
        )
    ])
```

## 6. Building and Running the Example

Navigate to your ROS 2 workspace root (e.g., `~/hackathon-1/ros2_ws`) and build your package:

```bash
cd ros2_ws
colcon build --packages-select ros2_examples
```

After building, source your workspace to make the new executables available:

```bash
source install/setup.bash
```

Now, you can launch the example:

```bash
ros2 launch ros2_examples example.launch.py
```

You should see both the publisher and subscriber nodes printing messages to the console, demonstrating successful ROS 2 communication!

---

**Next**: In Chapter 3, we will explore how to describe your robot's physical structure using URDF.
