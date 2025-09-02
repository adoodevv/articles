---
title: "What is a URDF?"
datePublished: Tue Sep 02 2025 17:01:51 GMT+0000 (Coordinated Universal Time)
cuid: cmf2sptvd000202l72a8g9iow
slug: what-is-a-urdf
cover: https://cdn.hashnode.com/res/hashnode/image/upload/v1756832342843/3db85c8d-fe64-4e2b-adef-844b7f7e1a9b.png
tags: robotics, urdf, gazebo

---

# Describing a Robot

When building a robot, multiple software components often need access to the same information about the robot’s physical structure. To keep things consistent and avoid duplication, it’s best to store all of this information in a single, shared location that any part of the system can reference.

In ROS, this is handled through the **robot description**, which is written in a **URDF (Unified Robot Description Format)** file. You might have seen URDFs briefly in earlier tutorials, but here we’ll dive deeper into how to create one from scratch.

At first glance, a full URDF file can look intimidating — they’re usually long and filled with lots of tags and symbols. But once you take a closer look, you’ll notice that URDFs are built from a few simple structures that repeat throughout the file. In this post, we’ll break down those building blocks and then walk through a practical example.

## Links and Joints

A URDF describes a robot as a **tree of links connected by joints**. Links represent the physical parts of the robot, while joints define how those parts move relative to each other and where they’re positioned in space.  
When creating a URDF, the first step is deciding how to break the robot into links and joints. There are two main cases where you’ll need to define them:

1. **Moving parts** – for example, the individual segments of a robotic arm.
    
2. **Fixed parts with their own reference point** – even if a component doesn’t move (like a camera or lidar), it can be useful to treat it as its own link for easier transformations.
    

Once a joint is defined, you also need to specify its type. The most common joint types are:

* **Revolute** – rotational motion with angle limits.
    
* **Continuous** – unlimited rotation (like a wheel).
    
* **Prismatic** – linear sliding motion with position limits.
    
* **Fixed** – no motion; the child link is rigidly attached to the parent.
    

## URDF Syntax

URDF files are written in **XML**, where everything is expressed in nested tags. The key tags to know are:

1. **Robot Tag**
    
    Every URDF starts with an XML declaration and a single root tag: `<robot>`. This tag defines the robot and can include attributes such as its name.
    
    ```xml
    <?xml version="1.0"?>
    <robot name="my_robot">
       ...
    </robot>
    ```
    
2. **Link Tags**
    
    A `<link>` tag defines a link’s name and optional properties. These properties include:
    
    * **Visual** – how the link looks in RViz/Gazebo (geometry, origin, material/colour).
        
    * **Collision** – geometry used for collision detection (often simpler than visual).
        
    * **Inertial** – physics properties like mass, center of gravity, and inertia matrix.
        
    
    Not every link needs all of these. You can also define multiple visual or collision tags for a single link if you want to combine shapes.
    
3. **Joint Tags**
    
    Joints specify how links are connected and move relative to each other. A joint definition includes:
    
    * **Name** – every joint should have one.
        
    * **Type** – fixed, revolute, prismatic, or continuous.
        
    * **Parent and child links** – the two links being connected.
        
    * **Origin** – the relationship between them before motion is applied.
        
    
    For moving joints, you can also set:
    
    * **Axis** – which direction it moves/rotates.
        
    * **Limits** – such as max/min angles, velocity, or effort.
        

## Xacro

URDFs can quickly get long and repetitive. **Xacro (XML macros)** is a ROS tool that helps manage this by:

* **Splitting code into multiple files** – keeping things organized.
    
* **Avoiding duplicate code** – by using properties, math expressions, and reusable macros.
    

To use Xacro, add the namespace inside the robot tag:

```xml
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
```

You can then include other files with:

```xml
<xacro:include filename="file.xacro" />
```

Xacro also supports **properties** (constants you can reuse), **math operations**, and **macros** (templates for repeated structures like inertial properties). These features keep your URDF DRY (Don’t Repeat Yourself), easier to edit, and less error-prone.

## What’s Next?

Now that we know how URDFs work, the next step is bringing our robots to life in **Gazebo**, the ROS simulation environment.

Check out [this](https://adoodevv.hashnode.dev/building-the-robot) post on building a simple robot with URDF.