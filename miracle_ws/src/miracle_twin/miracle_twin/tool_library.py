"""Tool Library — central repository of cutting tool definitions.

Provides tool geometry, cutting coefficients (Altintas mechanistic model),
and wear parameters for use by CuttingSimProxy, PredictionRunner, and
AdaptiveController.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
import copy
import json
import os
import math
import time


@dataclass
class ToolDefinition:
    """Complete definition of a cutting tool for simulation."""
    # Identity
    tool_id: str
    name: str
    description: str = ''

    # Geometry
    diameter_mm: float = 6.35        # 1/4 inch
    flute_count: int = 2
    helix_angle_deg: float = 30.0
    rake_angle_deg: float = 10.0
    nose_radius_mm: float = 0.4
    overall_length_mm: float = 50.0
    flute_length_mm: float = 19.0
    shank_diameter_mm: float = 6.35

    # Material
    tool_material: str = 'HSS'       # HSS, Carbide, Ceramic, CBN, PCD
    coating: str = 'Uncoated'        # TiN, TiAlN, AlCrN, DLC, Uncoated

    # Altintas Mechanistic Cutting Coefficients (N/mm^2)
    ktc: float = 796.0   # tangential cutting
    krc: float = 168.0   # radial cutting
    kac: float = 80.0    # axial cutting
    kte: float = 14.5    # tangential edge (N/mm)
    kre: float = 10.2    # radial edge (N/mm)
    kae: float = 4.8     # axial edge (N/mm)

    # Taylor Tool Life Parameters
    taylor_C: float = 300.0          # Taylor constant
    taylor_n: float = 0.125          # speed exponent
    taylor_f_exp: float = 0.5        # feed exponent
    taylor_ap_exp: float = 0.15      # depth exponent
    vb_max_mm: float = 0.30          # maximum flank wear

    # Recommended Operating Ranges
    min_rpm: int = 1000
    max_rpm: int = 25000
    min_feed_per_tooth_mm: float = 0.01
    max_feed_per_tooth_mm: float = 0.15
    max_depth_of_cut_mm: float = 10.0
    max_width_of_cut_mm: float = 6.35

    # Deflection properties
    elastic_modulus_gpa: float = 200.0  # Young's modulus (HSS=200, Carbide=620)
    tool_overhang_mm: float = 30.0     # distance from collet to tip

    # Stability (modal parameters for chatter prediction)
    natural_freq_hz: float = 1800.0
    damping_ratio: float = 0.03
    stiffness_n_per_m: float = 8e6

    # Recommended coolant type for this tool
    recommended_coolant: str = 'flood'  # 'dry', 'mist', 'flood', 'high_pressure', 'cryogenic'

    @property
    def helix_angle_rad(self) -> float:
        return math.radians(self.helix_angle_deg)

    @property
    def rake_angle_rad(self) -> float:
        return math.radians(self.rake_angle_deg)


@dataclass
class ToolCalibrationData:
    """Per-tool empirically measured calibration offsets."""
    tool_id: str
    machine_id: str
    force_scale: float = 1.0      # multiply ktc/krc/kac by this
    edge_scale: float = 1.0       # multiply kte/kre/kae by this
    thermal_scale: float = 1.0    # multiply thermal coefficients
    wear_rate_scale: float = 1.0  # multiply wear rate
    calibration_count: int = 0    # how many calibrations applied
    total_blocks_measured: int = 0
    last_calibrated_timestamp: float = 0.0
    calibration_history: list = field(default_factory=list)  # list of (timestamp, scales_dict)

    def apply_calibration(self, force_corr: float, edge_corr: float = 1.0,
                          thermal_corr: float = 1.0, wear_corr: float = 1.0,
                          blocks: int = 0):
        """Apply a calibration update with exponential moving average."""
        alpha = 0.3  # learning rate
        self.force_scale = self.force_scale * (1 - alpha) + force_corr * alpha
        self.edge_scale = self.edge_scale * (1 - alpha) + edge_corr * alpha
        self.thermal_scale = self.thermal_scale * (1 - alpha) + thermal_corr * alpha
        self.wear_rate_scale = self.wear_rate_scale * (1 - alpha) + wear_corr * alpha
        self.calibration_count += 1
        self.total_blocks_measured += blocks
        self.last_calibrated_timestamp = time.time()
        self.calibration_history.append((
            self.last_calibrated_timestamp,
            {'force': self.force_scale, 'edge': self.edge_scale,
             'thermal': self.thermal_scale, 'wear': self.wear_rate_scale}
        ))
        # Cap history
        if len(self.calibration_history) > 50:
            self.calibration_history = self.calibration_history[-50:]


class ToolLibrary:
    """Central repository of cutting tool definitions.

    Provides lookup by tool_id and supports loading from YAML files.
    Ships with built-in tools matching common workshop inventory.
    """

    def __init__(self):
        self._tools: Dict[str, ToolDefinition] = {}
        self._calibration_data: Dict[Tuple[str, str], ToolCalibrationData] = {}
        self._calibration_file: str = ''
        self._load_builtin_tools()

    def _load_builtin_tools(self):
        """Load built-in tool definitions."""
        # These match Unity ToolDefinitionCreator presets
        self.register(ToolDefinition(
            tool_id='HSS_2F_6mm',
            name='HSS 2-Flute 6mm End Mill',
            description='General purpose HSS end mill for aluminum/steel',
            diameter_mm=6.35, flute_count=2, helix_angle_deg=30.0,
            tool_material='HSS', coating='Uncoated',
            ktc=796.0, krc=168.0, kac=80.0,
            kte=14.5, kre=10.2, kae=4.8,
            taylor_C=300.0, taylor_n=0.125,
            elastic_modulus_gpa=200.0, tool_overhang_mm=30.0,
            min_rpm=1000, max_rpm=15000,
            natural_freq_hz=1800.0, damping_ratio=0.03, stiffness_n_per_m=8e6,
            recommended_coolant='flood',
        ))

        self.register(ToolDefinition(
            tool_id='CARBIDE_4F_10mm',
            name='Carbide 4-Flute 10mm End Mill',
            description='High performance carbide for steel/titanium',
            diameter_mm=10.0, flute_count=4, helix_angle_deg=35.0,
            tool_material='Carbide', coating='TiAlN',
            ktc=1200.0, krc=250.0, kac=120.0,
            kte=20.0, kre=15.0, kae=7.0,
            taylor_C=450.0, taylor_n=0.25,
            vb_max_mm=0.30,
            elastic_modulus_gpa=620.0, tool_overhang_mm=35.0,
            min_rpm=2000, max_rpm=25000,
            max_feed_per_tooth_mm=0.12,
            natural_freq_hz=2200.0, damping_ratio=0.025, stiffness_n_per_m=12e6,
            recommended_coolant='flood',
        ))

        self.register(ToolDefinition(
            tool_id='CARBIDE_2F_3mm',
            name='Carbide 2-Flute 3mm End Mill',
            description='Micro end mill for fine features and thin walls',
            diameter_mm=3.0, flute_count=2, helix_angle_deg=30.0,
            tool_material='Carbide', coating='DLC',
            ktc=900.0, krc=200.0, kac=90.0,
            kte=12.0, kre=8.0, kae=4.0,
            taylor_C=350.0, taylor_n=0.2,
            vb_max_mm=0.15,
            elastic_modulus_gpa=620.0, tool_overhang_mm=25.0,
            min_rpm=5000, max_rpm=30000,
            max_depth_of_cut_mm=3.0,
            natural_freq_hz=3000.0, damping_ratio=0.02, stiffness_n_per_m=5e6,
            recommended_coolant='mist',  # DLC coating is self-lubricating
        ))

        self.register(ToolDefinition(
            tool_id='HSS_4F_12mm',
            name='HSS 4-Flute 12mm End Mill',
            description='Heavy roughing HSS end mill',
            diameter_mm=12.0, flute_count=4, helix_angle_deg=25.0,
            tool_material='HSS', coating='TiN',
            ktc=750.0, krc=160.0, kac=75.0,
            kte=16.0, kre=11.0, kae=5.0,
            taylor_C=280.0, taylor_n=0.1,
            vb_max_mm=0.35,
            elastic_modulus_gpa=200.0, tool_overhang_mm=40.0,
            min_rpm=500, max_rpm=10000,
            max_depth_of_cut_mm=15.0,
            natural_freq_hz=1500.0, damping_ratio=0.035, stiffness_n_per_m=15e6,
            recommended_coolant='flood',
        ))

        self.register(ToolDefinition(
            tool_id='CARBIDE_6F_20mm',
            name='Carbide 6-Flute 20mm Finishing Mill',
            description='High-flute finishing mill for superalloys',
            diameter_mm=20.0, flute_count=6, helix_angle_deg=40.0,
            tool_material='Carbide', coating='AlCrN',
            ktc=1500.0, krc=300.0, kac=150.0,
            kte=25.0, kre=18.0, kae=9.0,
            taylor_C=500.0, taylor_n=0.3,
            vb_max_mm=0.25,
            elastic_modulus_gpa=620.0, tool_overhang_mm=45.0,
            min_rpm=1500, max_rpm=20000,
            max_feed_per_tooth_mm=0.08,
            natural_freq_hz=1200.0, damping_ratio=0.04, stiffness_n_per_m=20e6,
            recommended_coolant='flood',
        ))

    def register(self, tool: ToolDefinition) -> None:
        """Register a tool definition."""
        self._tools[tool.tool_id] = tool

    def get(self, tool_id: str) -> Optional[ToolDefinition]:
        """Look up a tool by ID. Returns None if not found."""
        return self._tools.get(tool_id)

    def get_or_default(self, tool_id: str) -> ToolDefinition:
        """Look up a tool by ID, returning the default HSS 2-flute if not found."""
        return self._tools.get(tool_id, self._tools.get('HSS_2F_6mm'))

    def list_tools(self) -> List[ToolDefinition]:
        """Return all registered tools."""
        return list(self._tools.values())

    def find_by_material(self, material: str) -> List[ToolDefinition]:
        """Find tools by material type."""
        return [t for t in self._tools.values()
                if t.tool_material.lower() == material.lower()]

    def find_by_diameter(self, diameter_mm: float,
                         tolerance_mm: float = 0.5) -> List[ToolDefinition]:
        """Find tools within diameter tolerance."""
        return [t for t in self._tools.values()
                if abs(t.diameter_mm - diameter_mm) <= tolerance_mm]

    def load_from_yaml(self, yaml_path: str) -> int:
        """Load tool definitions from a YAML file. Returns count of tools loaded."""
        try:
            import yaml
        except ImportError:
            return 0
        if not os.path.exists(yaml_path):
            return 0
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        count = 0
        for tool_data in data.get('tools', []):
            tool = ToolDefinition(**tool_data)
            self.register(tool)
            count += 1
        return count

    # ------------------------------------------------------------------
    # Calibration support
    # ------------------------------------------------------------------

    def get_calibrated_tool(self, tool_id: str, machine_id: str) -> Optional[ToolDefinition]:
        """Return a ToolDefinition with calibration offsets applied."""
        base = self.get(tool_id)
        if base is None:
            return None
        cal = self._calibration_data.get((tool_id, machine_id))
        if cal is None:
            return base
        # Return a copy with scaled coefficients
        calibrated = copy.deepcopy(base)
        calibrated.ktc *= cal.force_scale
        calibrated.krc *= cal.force_scale
        calibrated.kac *= cal.force_scale
        calibrated.kte *= cal.edge_scale
        calibrated.kre *= cal.edge_scale
        calibrated.kae *= cal.edge_scale
        return calibrated

    def update_calibration(self, tool_id: str, machine_id: str,
                           force_corr: float, blocks: int = 0, **kwargs):
        """Update calibration data for a tool on a specific machine."""
        key = (tool_id, machine_id)
        if key not in self._calibration_data:
            self._calibration_data[key] = ToolCalibrationData(
                tool_id=tool_id, machine_id=machine_id)
        self._calibration_data[key].apply_calibration(
            force_corr, blocks=blocks, **kwargs)

    def save_calibrations(self, path: str) -> None:
        """Persist calibration data to JSON."""
        records = []
        for (tool_id, machine_id), cal in self._calibration_data.items():
            records.append({
                'tool_id': cal.tool_id,
                'machine_id': cal.machine_id,
                'force_scale': cal.force_scale,
                'edge_scale': cal.edge_scale,
                'thermal_scale': cal.thermal_scale,
                'wear_rate_scale': cal.wear_rate_scale,
                'calibration_count': cal.calibration_count,
                'total_blocks_measured': cal.total_blocks_measured,
                'last_calibrated_timestamp': cal.last_calibrated_timestamp,
                'calibration_history': cal.calibration_history,
            })
        with open(path, 'w') as f:
            json.dump({'calibrations': records}, f, indent=2)

    def load_calibrations(self, path: str) -> int:
        """Load calibration data from JSON. Returns count loaded."""
        if not os.path.exists(path):
            return 0
        with open(path, 'r') as f:
            data = json.load(f)
        count = 0
        for rec in data.get('calibrations', []):
            cal = ToolCalibrationData(
                tool_id=rec['tool_id'],
                machine_id=rec['machine_id'],
                force_scale=rec.get('force_scale', 1.0),
                edge_scale=rec.get('edge_scale', 1.0),
                thermal_scale=rec.get('thermal_scale', 1.0),
                wear_rate_scale=rec.get('wear_rate_scale', 1.0),
                calibration_count=rec.get('calibration_count', 0),
                total_blocks_measured=rec.get('total_blocks_measured', 0),
                last_calibrated_timestamp=rec.get('last_calibrated_timestamp', 0.0),
                calibration_history=[
                    tuple(entry) if isinstance(entry, list) else entry
                    for entry in rec.get('calibration_history', [])
                ],
            )
            self._calibration_data[(cal.tool_id, cal.machine_id)] = cal
            count += 1
        return count

    def get_calibration_summary(self) -> dict:
        """Return summary of all calibration data."""
        summary = {}
        for (tool_id, machine_id), cal in self._calibration_data.items():
            summary[f'{tool_id}@{machine_id}'] = {
                'force_scale': cal.force_scale,
                'edge_scale': cal.edge_scale,
                'thermal_scale': cal.thermal_scale,
                'wear_rate_scale': cal.wear_rate_scale,
                'calibration_count': cal.calibration_count,
                'total_blocks_measured': cal.total_blocks_measured,
                'last_calibrated_timestamp': cal.last_calibrated_timestamp,
            }
        return summary
