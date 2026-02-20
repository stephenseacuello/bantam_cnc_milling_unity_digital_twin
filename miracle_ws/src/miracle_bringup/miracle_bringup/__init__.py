"""
miracle_bringup - Launch files and configuration for the MIRACLE CNC Digital Twin system.

This package provides launch files to bring up the full MIRACLE system stack,
including all layers from L1 (CNC machine control) through L5 (cognitive autonomy).

Launch files:
    miracle_full_system.launch.py  - Complete system bringup (all layers L1-L5)
    miracle_simulation.launch.py   - Simulation-only deployment
    miracle_physical.launch.py     - Physical hardware deployment
    cnc_machine.launch.py          - Per-machine node launcher
    cognitive_layer.launch.py      - L5 cognitive autonomy nodes
    security_layer.launch.py       - L4 security and resiliency nodes

Configuration:
    miracle_params.yaml            - Master parameter file for all nodes
    qos_profiles.yaml              - QoS profile definitions
    cnc1.yaml / cnc2.yaml / cnc3.yaml - Per-machine parameter overrides
"""
