---
title: "Building The Robot"
datePublished: Sun Dec 22 2024 08:00:50 GMT+0000 (Coordinated Universal Time)
cuid: cm4zbjou1000s0aju0xt2ab7o
slug: building-the-robot
cover: https://cdn.hashnode.com/res/hashnode/image/upload/v1734358047173/a8a66c3b-7b5d-453f-91f5-8f7714d7742f.png
tags: robotics, ros, gazebo, ros2

---

# Introduction

In this post, we’ll explore the design of a differential-drive robot and create its URDF model. This robot will serve as the foundation for the rest of this series. Much of what I’ve learnt here comes from Josh Newans’ excellent [YouTube](https://www.youtube.com/playlist?list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT) tutorials on ROS2 and Gazebo Classic. Let’s dive in!

## What is a Differential-Drive Robot?

A differential-drive robot has two main wheels on either side to control motion, while additional caster wheels provide support and stability. By varying the rotation speed of the wheels, the robot can change direction without a steering mechanism. This simplicity makes differential-drive robots an excellent choice for beginners.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1734361385084/bbb8da45-db26-425a-9c07-23d755040091.png align="center")

## Understanding URDF

The Unified Robot Description Format (URDF) uses XML to define a robot’s physical structure, including its links (rigid parts) and joints (connections that allow movement). URDF files are essential for:

1. **Visualization**: Tools like RViz display the robot model.
    
2. **Simulation**: Software like Gazebo uses URDF for physics-based interactions.
    
3. **Kinematics**: Understanding joint relationships.
    

### Streamlining with Xacro

Instead of creating large URDF files manually, we use xacro (XML Macros) to make descriptions modular and reusable. Xacro files simplify changes, as components like sensors or inertia values can be managed in separate files and included in a main file.

Once processed, the xacro file becomes a complete URDF, which is passed to the `robot_state_publisher`. This node:

1. **Publishes the URDF**: Makes the robot description accessible via the `/robot_description` topic.
    
2. **Broadcasts Transforms**: Provides link positions and orientations for visualization and motion tracking.
    

For movable joints, their positions must be published on the `/joint_states` topic. During testing, `joint_state_publisher_gui` can simulate these joint values.

### Preparing Your Environment

Before starting, ensure you’ve installed the required tools:

```bash
sudo apt install ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui
```

We will organize our files in the `urdf/` directory within our package.

## Creating the URDF

#### Step 1: Starting with the Template

To begin with, open up `robot.urdf` from the [template](https://github.com/adoodevv/diffbot) and delete the `base_link` that is currently there. Replace it with the line

```xml
<xacro:include filename="$(find diffbot_tut)/urdf/robot.gazebo" />
```

so that your file looks like this:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro"  name="diffbot_tut">

    <xacro:include filename="$(find diffbot_tut)/urdf/robot.gazebo" />

</robot>
```

The included file doesn’t exist yet, so any attempt to launch this will fail.

#### Step 2: Defining the Base Link

In the `urdf/` directory, create a new file called `robot.gazebo`. Copy the following XML declaration and the `robot` tags (these are the same as in the previous file but without the `name` parameter). All of our links and joints will be going inside the robot tag.

```xml
<?xml version="1.0"?>
<robot>
   ... all our plugins will go here ...
</robot>
```

It is standard in ROS for the main "origin" link in a mobile robot to be called `base_link`. So we start with an empty link called `base_link` under the opening `robot` tag in `robot.urdf`.

```xml
<!-- Base link -->

<link name="base_link" />
```

#### Step 3: Adding the Chassis

Define the chassis as a simple box. Let's make it a box that is *300 × 300 × 150mm*. URDF values are in metres, so that's *0.3 × 0.3 × 0.15m*. One other thing to note here is that by default the box geometry will be centered around the link origin. We want the link origin up (in Z) by half its height (0.075m).

```xml
<!-- Chassis -->

<joint name="chassis_joint" type="fixed">
    <parent link="base_link"/>
    <child link="chassis_link"/>
    <origin xyz="-0.1 0 0"/>
</joint>

<link name="chassis_link">
    <visual>
        <origin xyz="0.15 0 0.075"/>
        <geometry>
            <box size="0.3 0.3 0.15"/>
        </geometry>
        <material name="chassis_material">
            <color rgba="0.82 0.77 0.91 1.0"/>
        </material>
    </visual>
    <collision>
        <origin xyz="0.15 0 0.075"/>
        <geometry>
            <box size="0.3 0.3 0.15"/>
        </geometry>
    </collision>
</link>
```

**Hold up, Let’s Launch and Visualize!**

To launch what we have done so far, if you made a copy of the package from the GitHub template repository, you should already have a launch file (`launch/`[`rsp.launch.py`](https://github.com/adoodevv/diffbot/blob/main/launch/rsp.launch.py)).

To see the robot in RViz:

1. Launch the robot description(note that `diffbot_tut` is the name of my package):
    
    ```bash
    ros2 launch diffbot_tut rsp.launch.py
    ```
    
    ![](https://cdn.hashnode.com/res/hashnode/image/upload/v1734459491161/0991b81a-a628-4468-ab25-e5920a36136d.png align="center")
    
2. In RViz(launch RViz in another terminal with `rviz2` command):
    
    * Set the fixed frame to `base_link`.
        
    * Add a TF display(you can enable showing names).
        
    * Add a RobotModel display and set the topic to `/robot_description`.
        

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1734460159458/fe420027-6304-47a9-a2bb-bfc662cd366a.png align="center")

You can keep RViz open as we make the changes, but after every change, we will have to restart the `robot_state_publisher` to visualize the changes in RViz.

#### Step 4: Adding Wheels

Now we want to add the drive wheels. The wheels can obviously move, so these will be connected to `base_link` via `continuous` joints. We want our wheels to be be cylinders oriented along the Y axis (left-to-right). In ROS though, cylinders by default are oriented along the Z axis (up and down). To fix this, we need to "roll" the cylinder by a quarter-turn around the X axis, so I will rotate the left wheel clockwise (negative) around X by a quarter-turn (−π/2​ radians), and the right wheel anticlockwise (+π/2​ radians).

```xml
   <!-- Left wheel -->
   <joint name="left_wheel_joint" type="continuous">
       <origin xyz="0 0.175 0" rpy="-${pi/2} 0 0"/>
       <child link="left_wheel_link"/>
       <parent link="base_link"/>
       <axis xyz="0 0 1"/>
   </joint>

   <link name="left_wheel_link">
      <visual>
         <geometry>
            <cylinder radius="0.05" length="0.04"/>
         </geometry>
         <material name="wheel_material">
            <color rgba="0.0 0.0 0.0 1.0"/>
         </material>
      </visual>
   </link>

   <!-- Right wheel -->
   <joint name="right_wheel_joint" type="continuous">
       <origin xyz="0 -0.175 0" rpy="${pi/2} 0 0"/>
       <child link="right_wheel_link"/>
       <parent link="base_link"/>
       <axis xyz="0 0 -1"/>
    </joint>

   <link name="right_wheel_link">
      <visual>
         <geometry>
            <cylinder radius="0.05" length="0.04"/>
         </geometry>
         <material name="wheel_material">
            <color rgba="0.0 0.0 0.0 1.0"/>
         </material>
      </visual>
   </link>
```

If we try to view this in RViz now we'll notice that the wheels aren't displayed correctly, since nothing is publishing their joint states. We can temporarily run `joint_state_publisher_gui` with:

```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1734461001766/d0fd71ff-9466-4db2-8132-947156ce3881.png align="center")

#### Step 5: Adding a Castor Wheel

Add a simple frictionless sphere as the castor wheel:

```xml
   <!-- Castor wheel -->
   <joint name="castor_wheel_joint" type="fixed">
        <parent link="chassis"/>
        <child link="castor_wheel_link"/>
        <origin xyz="0.24 0 0"/>
    </joint>

   <link name="castor_wheel_link">
      <visual>
         <geometry>
            <sphere radius="0.05"/>
         </geometry>
         <material name="castor_material">
               <color rgba="0.62 0.62 0.62 1.0"/>
         </material>
      </visual>
   </link>
```

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1734461229668/18374a2f-f93b-446d-a7cc-845fc8d3d165.png align="center")

### Adding Collision and Inertia

To simulate realistic interactions, add collision geometry and inertial properties. For simplicity, copy the geometry from `<visual>` to `<collision>` tags. Use xacro macros for inertia calculations, such as `inertial_box` for the chassis or `inertial_cylinder` for the wheels. Calculating inertia values can sometimes be tricky, and it’s often easier to use macros. Download the [`inertia_macros.xacro`](https://github.com/adoodevv/diffbot_tut/blob/main/urdf/inertial_macros.xacro) file from here and place it in your `urdf/` directory. Then, near the top of your `robot.urdf` just under the opening `<robot>` tag, add the following line to include them.

```xml
<xacro:include filename="inertial_macros.xacro" />
```

Example:

```xml
<link name="chassis_link">
    <visual>
        <origin xyz="0.15 0 0.075"/>
        <geometry>
            <box size="0.3 0.3 0.15"/>
        </geometry>
        <material name="chassis_material">
            <color rgba="0.82 0.77 0.91 1.0"/>
        </material>
    </visual>
    <collision>
        <origin xyz="0.15 0 0.075"/>
        <geometry>
            <box size="0.3 0.3 0.15"/>
        </geometry>
    </collision>
    <xacro:inertial_box mass="0.5" x="0.3" y="0.3" z="0.15">
        <origin xyz="0 0 0.075" rpy="0 0 0"/>
    </xacro:inertial_box>
</link>

<link name="left_wheel_link"> 
    <!-- ... visual and collision ... -->
      <xacro:inertial_cylinder mass="0.1" length="0.04" radius="0.05">
         <origin xyz="0 0 0" rpy="0 0 0"/>
      </xacro:inertial_cylinder>
</link>

<!-- Right wheel is same as left for inertia -->

<link name="castor_wheel_link">
    <!-- ... visual and collision ... -->
    <xacro:inertial_sphere mass="0.1" radius="0.05">
        <origin xyz="0 0 0" rpy="0 0 0"/>
    </xacro:inertial_sphere>
</link>
```

So go ahead and do this for all the links. To check if the collision geometry looks correct, in the RViz RobotModel display, we can untick "Visual Enabled" and tick "Collision Enabled" to see the collision geometry (it will use the colours from the visual geometry).

## What’s Next?

Your basic robot structure is ready! In the next post, we’ll spawn this robot in Gazebo and control it using the keyboard. Stay tuned as we bring this robot to life!

To view the full project as it stands at this point, click [here](https://github.com/adoodevv/diffbot_tut).