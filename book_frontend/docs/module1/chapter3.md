---
id: chapter3
title: URDF for Humanoids
sidebar_position: 3
---

# Chapter 3: URDF for Humanoids - Robot Description Format

In the previous chapters, we explored the ROS 2 communication graph and implemented Python agents using rclpy. Now, we shift our focus to representing the physical structure of a robot using the Unified Robot Description Format (URDF). URDF is an XML format used in ROS to describe all aspects of a robot, including its geometry, kinematics, and dynamics. This is crucial for visualization in tools like RViz2 and for physics simulation in Gazebo.

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

## 2. Detailed URDF Components

### Visual Properties
```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="0.1 0.1 0.1"/>
    <!-- Other options: <cylinder radius="0.1" length="0.2"/> -->
    <!-- <sphere radius="0.1"/> -->
    <!-- <mesh filename="package://path/to/mesh.stl"/> -->
  </geometry>
  <material name="color">
    <color rgba="0.8 0.2 0.2 1.0"/>
  </material>
</visual>
```

### Inertial Properties
```xml
<inertial>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <mass value="0.1"/>
  <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
</inertial>
```

### Collision Properties
```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="0.1 0.1 0.1"/>
  </geometry>
</collision>
```

## 3. Our Simple Two-Link Arm Example

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
    <collision>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"/>
    </inertial>
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
    <collision>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.006"/>
    </inertial>
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
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.0005" ixy="0.0" ixz="0.0" iyy="0.0005" iyz="0.0" izz="0.002"/>
    </inertial>
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

## 4. Visualizing Your URDF with RViz2

To visualize your `simple_arm.urdf` file, you can use a ROS 2 launch file that starts `robot_state_publisher` and `rviz2`. The launch file for this example is `book_frontend/static/urdf/display.launch.py`.

First, ensure your ROS 2 environment is sourced (e.g., `source /opt/ros/humble/setup.bash`).

Navigate to your project's root directory and then run the launch file:

```bash
# From your project root (e.g., ~/hackathon-1)
ros2 launch book_frontend static/urdf/display.launch.py
```

This command will launch RViz2, and you should see the two-link arm model. You can manipulate the joints using the `JointStatePublisher` GUI that also typically starts with `robot_state_publisher`.

## 5. URDF for Humanoids

While our example is a simple arm, the same `<link>` and `<joint>` principles apply to complex humanoid robots. Humanoids would have many more links and joints, carefully arranged to mimic human anatomy:

### Humanoid Skeleton Example
```xml
<!-- Example of a simplified humanoid structure -->
<joint name="torso_to_head" type="revolute">
  <parent link="torso"/>
  <child link="head"/>
  <origin xyz="0 0 0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/> <!-- Head can nod up/down -->
  <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
</joint>

<joint name="torso_to_left_arm" type="revolute">
  <parent link="torso"/>
  <child link="left_shoulder"/>
  <origin xyz="0.1 0.2 0.1" rpy="0 0 0"/>
  <axis xyz="1 0 0"/> <!-- Shoulder rotation -->
  <limit lower="-1.57" upper="1.57" effort="15" velocity="1"/>
</joint>

<joint name="torso_to_right_arm" type="revolute">
  <parent link="torso"/>
  <child link="right_shoulder"/>
  <origin xyz="0.1 -0.2 0.1" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="-1.57" upper="1.57" effort="15" velocity="1"/>
</joint>
```

### Xacro for Complex Humanoid Descriptions
For complex humanoid robots, the XML can become very long and repetitive. Xacro (XML Macros) is used to make URDF more maintainable:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_robot">

  <xacro:property name="M_PI" value="3.1415926535897931" />

  <!-- Macro for creating a limb -->
  <xacro:macro name="limb" params="name parent *origin">
    <joint name="${name}_joint" type="revolute">
      <xacro:insert_block name="origin" />
      <parent link="${parent}"/>
      <child link="${name}_link"/>
      <axis xyz="0 1 0"/>
      <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="10" velocity="1"/>
    </joint>

    <link name="${name}_link">
      <visual>
        <geometry>
          <cylinder length="0.3" radius="0.02"/>
        </geometry>
        <material name="light_grey">
          <color rgba="0.7 0.7 0.7 1.0"/>
        </material>
      </visual>
    </link>
  </xacro:macro>

  <!-- Use the macro to create limbs -->
  <xacro:limb name="left_arm" parent="torso">
    <origin xyz="0.1 0.2 0.1" rpy="0 0 0"/>
  </xacro:limb>

  <xacro:limb name="right_arm" parent="torso">
    <origin xyz="0.1 -0.2 0.1" rpy="0 0 0"/>
  </xacro:limb>

</robot>
```

## 6. Best Practices for Humanoid URDF

- Use consistent naming conventions for joints and links
- Include proper inertial properties for accurate physics simulation
- Consider using xacro macros to reduce redundancy in complex structures
- Validate URDF files using tools like `check_urdf` from `robot_model`
- Test URDF in both visualization (RViz2) and simulation (Gazebo) environments

---

**Next**: In Module 2, we will explore Gazebo simulation environments and creating digital twins of our robots.