"""
miracle_gazebo - Gazebo simulation assets for the MIRACLE CNC milling digital twin.

This package provides SDF world files, model definitions, and launch
configurations for simulating a CNC factory environment in Gazebo.
Included assets:

- **cnc_factory.sdf** world with industrial floor, lighting, and physics
- **cnc_machine** model with base frame, spindle, and X/Y/Z axis joints
- **cobot** collaborative robot model with shoulder, elbow, and wrist joints
- **sensor_array** model with camera, IMU, and microphone placeholder
- Launch file for spawning configurable numbers of CNC machines with
  ros_gz_bridge integration for clock, joint states, and sensor data.
"""
