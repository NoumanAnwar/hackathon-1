---
sidebar_position: 3
---

# Chapter 3: URDF for Humanoids

In the previous chapters, we explored the ROS 2 communication graph and implemented simple Python nodes. Now, we shift our focus to representing the physical structure of a robot using the Unified Robot Description Format (URDF). URDF is an XML format used in ROS to describe all aspects of a robot, including its geometry, kinematics, and dynamics. It's crucial for visualization in tools like RViz2 and for physics simulation.

## 1. Understanding URDF Structure

A URDF file primarily consists of `<link>` and `<joint>` elements.

-   **`<link>`**: Represents a rigid body of the robot (e.g., a robot arm segment, a wheel, the base). Each link has visual (how it looks), inertial (mass properties), and collision (how it interacts physically) properties.
-   **`<joint>`**: Describes the connection between two links. Joints define the type of motion allowed between links (e.g., revolute for rotation, prismatic for sliding) and their position relative to each other.

### Basic Structure

```xml
<?xml version="1.0"?>
<robot name="my_robot">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
  </link>

  <!-- First Link -->
  <link name="link1">
    <!-- ... visual, inertial, collision properties ... -->
  </link>

  <!-- Joint connecting base_link and link1 -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="0.5"/>
  </joint>

  <!-- More links and joints... -->

</robot>
```

## 2. Our Simple Two-Link Arm Example

We will use a simple two-link robot arm to illustrate URDF concepts. The full URDF file is located at `book_frontend/static/urdf/simple_arm.urdf`.

```xml
<?xml version="1.0"?>
<robot name="simple_arm">

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
  </link>

  <link name="link1">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <material name="green">
        <color rgba="0 0.8 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.2"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="0.5"/>
  </joint>

  <link name="link2">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.04"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.5"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="0.5"/>
  </joint>

</robot>
```

## 3. Visualizing Your URDF with RViz2

To visualize your `simple_arm.urdf` file, you can use a ROS 2 launch file that starts `robot_state_publisher` and `rviz2`. The launch file for this example is `book_frontend/static/urdf/display.launch.py`.

First, ensure your ROS 2 environment is sourced (e.g., `source /opt/ros/humble/setup.bash`).

Navigate to your project's root directory and then run the launch file:

```bash
# From your project root (e.g., ~/hackathon-1)
ros2 launch book_frontend static/urdf/display.launch.py
```

This command will launch RViz2, and you should see the two-link arm model. You can manipulate the joints using the `JointStatePublisher` GUI that also typically starts with `robot_state_publisher`.

## 4. URDF for Humanoids

While our example is a simple arm, the same `<link>` and `<joint>` principles apply to complex humanoid robots. Humanoids would simply have many more links and joints, carefully arranged to mimic human anatomy, and often include more advanced elements like sensors and actuators.

---

**Next**: We have now covered the core content modules. The final steps will involve integrating the RAG chatbot and refining the book's presentation.
