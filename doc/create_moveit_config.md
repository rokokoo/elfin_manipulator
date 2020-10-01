# Creating a MoveIt! config

## The Xacro

We will create a arm + manipulator combo.

The manipulator will be a URDF sphere, since there are so many different options for manipulators and some of them require a lot more work to get them to work.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="samk_test">
    <xacro:include filename="$(find elfin_description)/urdf/elfin5.urdf.xacro" />

    <xacro:macro name="sph_tool" params="mass rad">
        <link name="sphere_tool">
            <inertial>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <mass value="${mass}"/>
                <inertia ixx="${(2/5)*(mass*rad**2)}" ixy="0.0" ixz="0.0" iyy="${(2/5)*(mass*rad**2)}" iyz="0.0" izz="${(2/5)*(mass*rad**2)}"/>
            </inertial>
            <visual>
                <origin xyz="0.0 0.0 ${rad}" rpy="0.0 0.0 0.0"/>
                <geometry>
                    <sphere radius="${rad}"/>
                </geometry>
                <material name="green">
                    <color rgba="0.0 1 0.0 1.0"/>
                </material>
            </visual>
            <collision>
                <origin xyz="0.0 0.0 ${rad}" rpy="0.0 0.0 0.0"/>
                <geometry>
                    <sphere radius="${rad}"/>
                </geometry>
            </collision>
        </link>

        <joint name="sphere_link" type="fixed">
            <child link="sphere_tool" />
            <parent link="elfin_end_link" />
        </joint>
        </xacro:macro>

    <xacro:sph_tool mass="0.1" rad="0.04" />
</robot>
```

It should look something like this in RViz

![Test URDF xacro](img/rviz_test_manipulator.png)
