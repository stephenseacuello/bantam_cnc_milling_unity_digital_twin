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


# ======================================================================
# Workpiece Material Properties Database
# ======================================================================

@dataclass
class MaterialProperties:
    """Physical and machining properties of a workpiece material."""
    material_id: str                       # e.g. "6061-T6", "304-SS"
    name: str
    category: str                          # aluminum, steel, stainless, titanium, nickel_alloy, plastic, composite
    hardness_hrc: float                    # Rockwell C (or BHN equivalent)
    tensile_strength_mpa: float
    thermal_conductivity_w_mk: float
    specific_heat_j_kgk: float
    density_kg_m3: float
    machinability_rating: float            # 0-1 scale, 1 = easiest
    recommended_speed_sfm: Tuple[float, float]   # (min, max) surface feet/min
    recommended_feed_ipt: Tuple[float, float]     # (min, max) inches per tooth
    taylor_constant_c: float               # Taylor V*T^n = C constant
    taylor_exponent_n: float               # Taylor exponent
    specific_cutting_force_n_mm2: float    # Kc1.1
    chip_formation: str                    # continuous, segmented, discontinuous


class MaterialDatabase:
    """Database of workpiece material properties for cutting parameter selection.

    Ships pre-loaded with 10 common engineering materials and supports
    user-defined additions, category filtering, and machining parameter
    recommendations.
    """

    VALID_CATEGORIES = {
        'aluminum', 'steel', 'stainless', 'titanium',
        'nickel_alloy', 'plastic', 'composite',
    }

    def __init__(self):
        self._materials: Dict[str, MaterialProperties] = {}
        self._load_builtin_materials()

    # ------------------------------------------------------------------
    # Built-in material data
    # ------------------------------------------------------------------

    def _load_builtin_materials(self):
        """Pre-load 10 common engineering materials."""
        builtins = [
            MaterialProperties(
                material_id='6061-T6', name='Aluminum 6061-T6',
                category='aluminum', hardness_hrc=15.0,
                tensile_strength_mpa=310.0,
                thermal_conductivity_w_mk=167.0,
                specific_heat_j_kgk=896.0,
                density_kg_m3=2710.0,
                machinability_rating=0.90,
                recommended_speed_sfm=(800.0, 1500.0),
                recommended_feed_ipt=(0.003, 0.010),
                taylor_constant_c=600.0, taylor_exponent_n=0.28,
                specific_cutting_force_n_mm2=700.0,
                chip_formation='continuous',
            ),
            MaterialProperties(
                material_id='7075-T6', name='Aluminum 7075-T6',
                category='aluminum', hardness_hrc=17.0,
                tensile_strength_mpa=572.0,
                thermal_conductivity_w_mk=130.0,
                specific_heat_j_kgk=960.0,
                density_kg_m3=2810.0,
                machinability_rating=0.85,
                recommended_speed_sfm=(600.0, 1200.0),
                recommended_feed_ipt=(0.003, 0.008),
                taylor_constant_c=550.0, taylor_exponent_n=0.26,
                specific_cutting_force_n_mm2=800.0,
                chip_formation='continuous',
            ),
            MaterialProperties(
                material_id='1018', name='AISI 1018 Low Carbon Steel',
                category='steel', hardness_hrc=12.0,
                tensile_strength_mpa=440.0,
                thermal_conductivity_w_mk=51.9,
                specific_heat_j_kgk=486.0,
                density_kg_m3=7870.0,
                machinability_rating=0.70,
                recommended_speed_sfm=(300.0, 600.0),
                recommended_feed_ipt=(0.003, 0.008),
                taylor_constant_c=350.0, taylor_exponent_n=0.20,
                specific_cutting_force_n_mm2=1400.0,
                chip_formation='continuous',
            ),
            MaterialProperties(
                material_id='4140', name='AISI 4140 Alloy Steel',
                category='steel', hardness_hrc=28.0,
                tensile_strength_mpa=655.0,
                thermal_conductivity_w_mk=42.7,
                specific_heat_j_kgk=473.0,
                density_kg_m3=7850.0,
                machinability_rating=0.55,
                recommended_speed_sfm=(200.0, 450.0),
                recommended_feed_ipt=(0.002, 0.006),
                taylor_constant_c=280.0, taylor_exponent_n=0.18,
                specific_cutting_force_n_mm2=1800.0,
                chip_formation='continuous',
            ),
            MaterialProperties(
                material_id='D2', name='AISI D2 Tool Steel',
                category='steel', hardness_hrc=60.0,
                tensile_strength_mpa=1850.0,
                thermal_conductivity_w_mk=20.0,
                specific_heat_j_kgk=460.0,
                density_kg_m3=7700.0,
                machinability_rating=0.25,
                recommended_speed_sfm=(60.0, 150.0),
                recommended_feed_ipt=(0.001, 0.003),
                taylor_constant_c=150.0, taylor_exponent_n=0.12,
                specific_cutting_force_n_mm2=3000.0,
                chip_formation='segmented',
            ),
            MaterialProperties(
                material_id='304-SS', name='304 Stainless Steel',
                category='stainless', hardness_hrc=20.0,
                tensile_strength_mpa=515.0,
                thermal_conductivity_w_mk=16.2,
                specific_heat_j_kgk=500.0,
                density_kg_m3=8000.0,
                machinability_rating=0.45,
                recommended_speed_sfm=(150.0, 350.0),
                recommended_feed_ipt=(0.002, 0.006),
                taylor_constant_c=220.0, taylor_exponent_n=0.15,
                specific_cutting_force_n_mm2=2100.0,
                chip_formation='continuous',
            ),
            MaterialProperties(
                material_id='316-SS', name='316 Stainless Steel',
                category='stainless', hardness_hrc=22.0,
                tensile_strength_mpa=580.0,
                thermal_conductivity_w_mk=14.0,
                specific_heat_j_kgk=500.0,
                density_kg_m3=8000.0,
                machinability_rating=0.40,
                recommended_speed_sfm=(120.0, 300.0),
                recommended_feed_ipt=(0.002, 0.005),
                taylor_constant_c=200.0, taylor_exponent_n=0.14,
                specific_cutting_force_n_mm2=2200.0,
                chip_formation='continuous',
            ),
            MaterialProperties(
                material_id='Ti-6Al-4V', name='Titanium 6Al-4V',
                category='titanium', hardness_hrc=36.0,
                tensile_strength_mpa=950.0,
                thermal_conductivity_w_mk=6.7,
                specific_heat_j_kgk=526.0,
                density_kg_m3=4430.0,
                machinability_rating=0.22,
                recommended_speed_sfm=(50.0, 150.0),
                recommended_feed_ipt=(0.002, 0.005),
                taylor_constant_c=120.0, taylor_exponent_n=0.12,
                specific_cutting_force_n_mm2=2500.0,
                chip_formation='segmented',
            ),
            MaterialProperties(
                material_id='Inconel-718', name='Inconel 718',
                category='nickel_alloy', hardness_hrc=40.0,
                tensile_strength_mpa=1240.0,
                thermal_conductivity_w_mk=11.4,
                specific_heat_j_kgk=435.0,
                density_kg_m3=8190.0,
                machinability_rating=0.15,
                recommended_speed_sfm=(30.0, 100.0),
                recommended_feed_ipt=(0.001, 0.004),
                taylor_constant_c=90.0, taylor_exponent_n=0.10,
                specific_cutting_force_n_mm2=3200.0,
                chip_formation='segmented',
            ),
            MaterialProperties(
                material_id='PEEK', name='PEEK (Polyetheretherketone)',
                category='plastic', hardness_hrc=5.0,
                tensile_strength_mpa=100.0,
                thermal_conductivity_w_mk=0.25,
                specific_heat_j_kgk=2180.0,
                density_kg_m3=1310.0,
                machinability_rating=0.95,
                recommended_speed_sfm=(500.0, 1000.0),
                recommended_feed_ipt=(0.004, 0.012),
                taylor_constant_c=800.0, taylor_exponent_n=0.35,
                specific_cutting_force_n_mm2=250.0,
                chip_formation='continuous',
            ),
        ]
        for mat in builtins:
            self._materials[mat.material_id] = mat

    # ------------------------------------------------------------------
    # Lookup
    # ------------------------------------------------------------------

    def get_material(self, material_id: str) -> MaterialProperties:
        """Look up a material by ID. Raises KeyError if not found."""
        if material_id not in self._materials:
            raise KeyError(f"Unknown material: {material_id!r}")
        return self._materials[material_id]

    def get_materials_by_category(self, category: str) -> List[MaterialProperties]:
        """Return all materials belonging to *category*."""
        cat = category.lower()
        return [m for m in self._materials.values() if m.category == cat]

    # ------------------------------------------------------------------
    # Machining parameter recommendations
    # ------------------------------------------------------------------

    def get_recommended_params(self, material_id: str,
                               tool_diameter_mm: float,
                               num_flutes: int) -> dict:
        """Compute recommended RPM, feed, depth, and width for a material/tool pair.

        Parameters
        ----------
        material_id : str
        tool_diameter_mm : float  (must be > 0)
        num_flutes : int  (must be > 0)

        Returns
        -------
        dict with keys: rpm, feed_mmpm, depth_mm, width_mm
        """
        if tool_diameter_mm <= 0:
            raise ValueError("tool_diameter_mm must be > 0")
        if num_flutes <= 0:
            raise ValueError("num_flutes must be > 0")

        mat = self.get_material(material_id)

        # Use midpoint of recommended SFM range
        mid_sfm = (mat.recommended_speed_sfm[0] + mat.recommended_speed_sfm[1]) / 2.0
        # SFM → m/min: SFM * 0.3048
        mid_mpm = mid_sfm * 0.3048
        # RPM = (cutting speed m/min * 1000) / (pi * D_mm)
        rpm = (mid_mpm * 1000.0) / (math.pi * tool_diameter_mm)
        rpm = round(rpm)

        # Feed per tooth — midpoint in inches, convert to mm
        mid_ipt = (mat.recommended_feed_ipt[0] + mat.recommended_feed_ipt[1]) / 2.0
        fz_mm = mid_ipt * 25.4  # mm/tooth
        feed_mmpm = round(rpm * num_flutes * fz_mm, 1)

        # Depth of cut: 1x diameter for aluminum/plastic, 0.5x for others
        if mat.category in ('aluminum', 'plastic'):
            depth_mm = round(tool_diameter_mm * 1.0, 2)
        else:
            depth_mm = round(tool_diameter_mm * 0.5, 2)

        # Width of cut: 50 % of diameter (general guideline)
        width_mm = round(tool_diameter_mm * 0.5, 2)

        return {
            'rpm': rpm,
            'feed_mmpm': feed_mmpm,
            'depth_mm': depth_mm,
            'width_mm': width_mm,
        }

    # ------------------------------------------------------------------
    # Taylor tool life
    # ------------------------------------------------------------------

    def get_taylor_life(self, material_id: str,
                        cutting_speed_mpm: float,
                        feed_mmrev: float) -> float:
        """Estimate tool life in minutes using the Taylor equation.

        Uses the extended form: V * T^n = C, solved for T, with a feed
        correction factor.  A higher feed shortens tool life linearly.

        Returns tool life in minutes (always >= 0).
        """
        mat = self.get_material(material_id)
        if cutting_speed_mpm <= 0 or feed_mmrev <= 0:
            return 0.0

        # Baseline mid-feed for normalisation (convert midpoint IPT to mm/rev)
        mid_ipt = (mat.recommended_feed_ipt[0] + mat.recommended_feed_ipt[1]) / 2.0
        ref_feed = mid_ipt * 25.4  # mm/rev (approximation for single-flute)

        # Feed correction: life scales inversely with feed ratio
        feed_ratio = feed_mmrev / ref_feed if ref_feed > 0 else 1.0

        # Taylor: T = (C / V) ^ (1/n)
        n = mat.taylor_exponent_n
        if n <= 0:
            return 0.0
        base_life = (mat.taylor_constant_c / cutting_speed_mpm) ** (1.0 / n)

        # Apply feed correction
        life = base_life / feed_ratio
        return max(life, 0.0)

    # ------------------------------------------------------------------
    # Material comparison
    # ------------------------------------------------------------------

    def compare_materials(self, id1: str, id2: str) -> dict:
        """Compare two materials and return relative metrics.

        Returns dict with keys:
            machinability_ratio  — mat1 / mat2  (>1 means mat1 easier)
            speed_ratio          — recommended speed mat1 / mat2
            feed_ratio           — recommended feed mat1 / mat2
            cutting_force_ratio  — Kc mat1 / mat2  (>1 means mat1 harder to cut)
        """
        m1 = self.get_material(id1)
        m2 = self.get_material(id2)

        def _mid(t: Tuple[float, float]) -> float:
            return (t[0] + t[1]) / 2.0

        speed_mid1 = _mid(m1.recommended_speed_sfm)
        speed_mid2 = _mid(m2.recommended_speed_sfm)
        feed_mid1 = _mid(m1.recommended_feed_ipt)
        feed_mid2 = _mid(m2.recommended_feed_ipt)

        return {
            'machinability_ratio': m1.machinability_rating / m2.machinability_rating if m2.machinability_rating else float('inf'),
            'speed_ratio': speed_mid1 / speed_mid2 if speed_mid2 else float('inf'),
            'feed_ratio': feed_mid1 / feed_mid2 if feed_mid2 else float('inf'),
            'cutting_force_ratio': m1.specific_cutting_force_n_mm2 / m2.specific_cutting_force_n_mm2 if m2.specific_cutting_force_n_mm2 else float('inf'),
        }

    # ------------------------------------------------------------------
    # Custom materials
    # ------------------------------------------------------------------

    def add_custom_material(self, props: MaterialProperties) -> None:
        """Add a user-defined material to the database."""
        self._materials[props.material_id] = props

    # ------------------------------------------------------------------
    # Search
    # ------------------------------------------------------------------

    def search_materials(self, query: str) -> List[MaterialProperties]:
        """Fuzzy search materials by name or ID (case-insensitive substring match)."""
        q = query.lower()
        return [
            m for m in self._materials.values()
            if q in m.material_id.lower() or q in m.name.lower()
        ]
