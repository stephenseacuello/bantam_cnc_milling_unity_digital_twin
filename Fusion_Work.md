# Fusion 360 → Unity FBX Export Guide

## Your Fusion files are already organized!

Both machines already have component groups in Fusion 360. **No renaming needed** — the code auto-detects your existing Fusion names.

## What the code recognizes

### Bantam Desktop Explorer

Your Fusion hierarchy:
```
bantam tools explorer cnc milling...
  ├── Y-Axis:1        ← auto-detected as Y axis (table, front-back)
  ├── X-Axis:1        ← auto-detected as X axis (gantry, left-right)
  ├── Z-Axis:1        ← auto-detected as Z axis (head, up-down)
  └── Static:1        ← ignored (static frame)
```

### CoastRunner CR-1

Your Fusion hierarchy:
```
coast_runner_cr_1_mechani...
  ├── Subassembly XY - Stationary    ← ignored (static frame)
  ├── Subassembly YZ - Translator:1  ← auto-detected as Y axis (translator, front-back)
  ├── Subassembly Z:1                ← auto-detected as Z axis (head, up-down)
  ├── Subassembly X - Table:1        ← auto-detected as X axis (table, left-right)
  ├── Subassembly - Rear Cover:1     ← ignored (static)
  ├── Reinforcement:1                ← ignored (static)
  └── Chip Guard - Y Sides...        ← ignored (static)
```

## Export steps

### 1. Export as FBX from Fusion 360

- File → Export → select **FBX (.fbx)**
- Or use the **SimLab FBX Exporter** plugin (better hierarchy preservation)
- Settings:
  - **Scale: 0.01** (Fusion works in mm, Unity in meters)
  - **Apply Transform: ON**
  - **Binary format** (smaller file)

### 2. Import into Unity

- Bantam FBX → `Assets/Models/BantamExplorer/BantamExplorerCNC.fbx` (replace existing)
- CoastRunner FBX → `Assets/Models/CoastRunnerCR1/CoastRunnerCR1.fbx` (create folder if needed)

### 3. Run MIRACLE > Wire Dashboard

DashboardWiring will:
1. Find axis children by name (X-Axis, Subassembly X, etc.)
2. Reparent them into the correct kinematic chain
3. Add ArticulationBody prismatic joints to each
4. Controllers auto-discover them at runtime

Console should show:
```
[DashboardWiring] Bantam: Found Fusion FBX axis children: X='X-Axis', Y='Y-Axis', Z='Z-Axis'
[DashboardWiring] Bantam: Bantam chain — Root→X→Y, Root→Z
```

## All recognized names (case-insensitive)

| Axis | Recognized Patterns |
|------|-------------------|
| X | `x-axis`, `x_axis`, `x_gantry`, `xaxis`, `subassembly x` |
| Y | `y-axis`, `y_axis`, `y_table`, `yaxis`, `y_bed`, `subassembly yz` |
| Z | `z-axis`, `z_axis`, `z_head`, `zaxis`, `subassembly z` |

Fusion's `:1` suffix is automatically handled.

## Kinematic chains

**Bantam:** Y table rides on X gantry, Z head is independent (on column)
```
Root (immovable)
  ├── X-Axis (prismatic X) → Y-Axis (prismatic Z)
  └── Z-Axis (prismatic -Y)
```

**CoastRunner:** X table is independent, Z head rides on YZ translator
```
Root (immovable)
  ├── Subassembly X - Table (prismatic X)
  └── Subassembly YZ - Translator (prismatic Z) → Subassembly Z (prismatic -Y)
```

## Coordinate Mapping (Fusion 360 → Unity)

| Fusion 360 | Unity | CNC Axis | Description |
|-----------|-------|----------|-------------|
| X | X | X | Left-right (gantry travel) |
| Y | Z | Y | Front-back (table travel) |
| Z | -Y | Z | Up-down (spindle descent, inverted) |

## Verification

After import + Wire Dashboard:
1. Expand the FBX prefab in the Hierarchy — you should see named children
2. Press Play — each axis group should move independently
3. Console should show: `Found Fusion FBX axis children: X=..., Y=..., Z=...`

## Tips

- Ensure the origin of each component is at its pivot point (where the axis of motion is)
- For the spindle, the origin should be at the collet/tool holder
- Test in Fusion 360: try moving each component along its axis to verify grouping
