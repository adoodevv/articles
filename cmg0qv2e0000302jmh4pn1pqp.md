---
title: "Gazebo Simulation"
datePublished: Fri Sep 26 2025 11:14:06 GMT+0000 (Coordinated Universal Time)
cuid: cmg0qv2e0000302jmh4pn1pqp
slug: gazebo-simulation
cover: https://cdn.hashnode.com/res/hashnode/image/upload/v1734622479108/69411e4b-b379-44bc-832e-4d546a4c2c5a.png
tags: gazebo, ros2

---

# Introduction

The new Gazebo simulator builds on the legacy of **Gazebo Classic** but introduces key changes that affect how ROS 2 projects interact with it.

* **ros\_gz vs. gazebo\_ros\_pkgs**: ROS 2 now uses the **ros\_gz** package instead of **gazebo\_ros\_pkgs** for launch files and utilities.
    
* **Bridging vs. plugins**: In Gazebo Classic, **gazebo\_ros\_pkgs** provided plugins that ran directly inside the simulator to connect ROS and Gazebo. In the new system, **ros\_gz** mainly acts as a bridge between ROS topics and **gz-transport** topics.
    

Understanding these differences will help you transition smoothly to the updated workflow.

## Creating a Launch File

To avoid manually starting and stopping multiple programs each time we make a change, we’ll create a single **launch file** to run everything at once.

1. Inside your `launch/` directory, create a file called `robot.launch.py`.
    
2. Paste in the provided code in the [repository](https://github.com/adoodevv/diffbot_tut/blob/main/launch/robot.launch.py) and update the **package name** to match your own.
    

This file does three things:

* Includes your own `rsp.launch.py` and forces `use_sim_time` to `true`.
    
* Includes the Gazebo launch file from the **ros\_gz** package.
    
* Runs the entity-spawn node from **ros\_gz**.
    

Rebuild your workspace, close any running instances, and launch this file. Your robot should appear in Gazebo as before—only now the process is much simpler.

> **Tip:** When you close Gazebo, stop the launch script manually with `Ctrl + C`.

## Adding Gazebo Tags

To refine our simulation, we can embed `<gazebo>` tags directly in the URDF. For example, to reduce friction on a caster wheel so it can spin freely:

```xml
<gazebo reference="castor_wheel">
    <mu1>0.000001</mu1>
    <mu2>0.000001</mu2>
</gazebo>
```

Make sure your wheels already have the necessary `<gazebo>` tags from earlier steps.

## Controlling the Robot in Gazebo

Eventually, we’ll use the **ros2\_control** library to manage our robot’s motors. The advantage of `ros2_control` is that the same code works for both simulation and real hardware.

For now, we’ll keep things simple by using Gazebo’s built-in **differential-drive control plugin** to illustrate key concepts.

### How Control Works

A physical robot’s control system converts a desired velocity into motor commands, measures the motors’ actual speed, and estimates the robot’s position.

In ROS, velocity commands are published on the `/cmd_vel` topic as a **Twist** message:

* Linear velocity: `x`, `y`, `z`
    
* Angular velocity: roll, pitch, yaw
    

For a differential-drive robot, only **linear x** (forward/back) and **angular z** (turning) matter; the other four values remain zero.

The robot’s position is calculated by integrating velocity over time, a process known as **dead reckoning**, which produces an **odometry** estimate.

Within Gazebo, control is provided by a plugin (for example, `gz-sim-diff-drive-system`). This plugin publishes `joint_states` and broadcasts a transform from a frame called **odom** (the world origin) to **base\_link**, giving other nodes a live position estimate.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1758882219629/2e9cdfbf-d96b-4326-ae8a-9ac4337881fb.png align="center")

Instead of faking the joint states (with `joint_state_publisher_gui`), the Gazebo robot is spawned from `/robot_description`, and the joint\_states are published by the control plugin(`gz-sim-joint-state-publisher-system`).

## Adding the Control Plugin

To drive the robot, create a new Xacro file named [`robot.gazebo`](https://github.com/adoodevv/diffbot_tut/blob/main/urdf/robot.gazebo) and include it in your root URDF (`robot.urdf`).

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <gazebo>

        <!-- Plugins will go here! -->

    </gazebo>
</robot>
```

So then inside those `<gazebo>` tags, we will create a `<plugin>` tag, using the `gz-sim-diff-drive-system` and the others. Find the full `robot.gazebo` file with all the needed plugins [here](https://github.com/adoodevv/diffbot_tut/blob/main/urdf/robot.gazebo).

## Bridge ROS Topics

In Gazebo Classic, ROS communication relied on plugins inside **gazebo\_ros\_pkgs**.  
In the new Gazebo, bridging is handled by a **generic bridge node** from **ros\_gz**, which connects **gz-transport** topics to ROS 2 topics.

Create a file called `gz_bridge.yaml` inside a `config/` directory and reference it in your launch file. If you started from a template, this may already be in place.

## Testing Control

With everything set up, relaunch Gazebo. The robot is now ready to receive velocity commands on the `/cmd_vel` topic. The easiest way to send them is with **teleop\_twist\_keyboard**:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Follow the on-screen instructions - press **i** to move forward, for example - and watch the robot respond in Gazebo.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1758882978823/4ef45804-8fba-49d3-a9de-272cd12128a4.png align="center")

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1758883191686/bf5b8acc-6a75-4a14-8892-2a77c7785634.png align="center")

## Visualizing the Result

Because the control plugin broadcasts a transform from `odom` to `base_link`, you can visualize the robot’s motion in **RViz**:

1. Start RViz and add the **TF** and **RobotModel** displays.
    
2. Set the **Fixed Frame** to `odom`.
    

As you drive the robot in Gazebo, its movement should match in RViz in real time.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1758883269466/943e9849-b086-48ec-b097-7a33fb1f6303.png align="center")

## Let’s Make an Obstacle World

Gazebo also lets you build custom environments. Create a world file - for example, [`obstacles.world`](https://github.com/adoodevv/diffbot_tut/blob/main/worlds/obstacles.world) with construction cones - and update the `world_path` in your [launch file](https://github.com/adoodevv/diffbot_tut/blob/main/launch/robot.launch.py) to load it. Relaunch to see your new obstacle course.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1758883601869/aea62cac-6cc1-4c60-aa53-e72221860494.png align="center")

## What’s Next?

Next, we’ll add sensors such as an **IMU** and **LiDAR**, then visualize their data in both Gazebo and RViz.

You can follow the progress or grab the full example code from the project’s [GitHub repository](https://github.com/adoodevv/diffbot_tut).