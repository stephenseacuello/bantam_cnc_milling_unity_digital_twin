"""
Cutting Simulation Proxy — Python port of Unity's CuttingForceEngine + ToolWearModel.

Provides force prediction, wear estimation, and RUL calculation for the
PredictionRunner to replace hardcoded values with real simulation results.
"""

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional


@dataclass
class CuttingCoefficients:
    """Altintas mechanistic cutting coefficients for 6061-T6 + HSS."""
    Ktc: float = 796.0   # N/mm² tangential shearing
    Krc: float = 168.0   # N/mm² radial shearing
    Kac: float = 80.0    # N/mm² axial shearing
    Kte: float = 14.5    # N/mm tangential edge
    Kre: float = 10.2    # N/mm radial edge
    Kae: float = 4.8     # N/mm axial edge
    wear_force_multiplier: float = 12.5  # mm⁻¹


@dataclass
class ToolState:
    """Current tool state."""
    diameter_mm: float = 6.35
    flute_count: int = 2
    helix_angle_deg: float = 30.0
    flank_wear_vb: float = 0.02  # mm
    cutting_time_min: float = 0.0


@dataclass
class GCodeBlock:
    """Simplified G-code block for simulation."""
    feed_rate_mmpm: float = 0.0  # mm/min
    spindle_rpm: float = 0.0
    axial_depth_mm: float = 1.5
    radial_depth_mm: float = 3.175
    length_mm: float = 10.0  # segment length


@dataclass
class BlockPrediction:
    """Prediction result for a single G-code block."""
    peak_force_n: float = 0.0
    avg_force_n: float = 0.0
    power_w: float = 0.0
    torque_nm: float = 0.0
    mrr_mm3pm: float = 0.0
    wear_after_block_mm: float = 0.0
    temperature_rise_c: float = 0.0


@dataclass
class SimulationResult:
    """Full simulation result."""
    block_predictions: List[BlockPrediction] = field(default_factory=list)
    total_cutting_time_min: float = 0.0
    final_wear_mm: float = 0.0
    remaining_useful_life_hours: float = 0.0
    confidence: float = 0.0
    health_index: float = 0.0
    trend_data: List[float] = field(default_factory=list)
    recommended_action: str = ''


class CuttingSimProxy:
    """Python proxy for cutting force + wear simulation.

    Ports the Altintas mechanistic force model and 3-stage Taylor wear model
    from the Unity C# implementation to Python for use by PredictionRunner.
    """

    # Taylor equation constants (6061-T6 + HSS)
    TAYLOR_N = 0.125
    TAYLOR_A = 0.5
    TAYLOR_B = 0.15
    TAYLOR_C = 300.0
    REFERENCE_SPEED = 100.0  # m/min

    # Wear model constants
    VB0 = 0.02       # Initial wear (mm)
    VB1 = 0.08       # Wear at end of break-in (mm)
    T1 = 2.0         # Break-in period (min)
    C2 = 0.004       # Steady-state wear rate (mm/min at V=100)
    VBMAX = 0.30     # End-of-life criterion (mm)

    def __init__(self, coefficients: Optional[CuttingCoefficients] = None):
        self.coeffs = coefficients or CuttingCoefficients()

    def calculate_forces(
        self,
        spindle_rpm: float,
        feed_per_tooth: float,
        axial_depth: float,
        radial_depth: float,
        tool_diameter: float = 6.35,
        flute_count: int = 2,
        helix_angle_rad: float = None,
        flank_wear_vb: float = 0.0,
    ) -> Dict[str, float]:
        """Calculate cutting forces using the full Altintas mechanistic model.

        Returns dict with: fx_peak, fy_peak, fz_peak, fx_avg, fy_avg, fz_avg,
                          power_w, torque_nm, mrr, specific_cutting_energy
        """
        if helix_angle_rad is None:
            helix_angle_rad = math.radians(30.0)

        if spindle_rpm <= 0 or feed_per_tooth <= 0 or axial_depth <= 0 or radial_depth <= 0:
            return {k: 0.0 for k in [
                'fx_peak', 'fy_peak', 'fz_peak',
                'fx_avg', 'fy_avg', 'fz_avg',
                'power_w', 'torque_nm', 'mrr',
                'specific_cutting_energy',
            ]}

        R = tool_diameter / 2.0
        n_disk = max(1, int(axial_depth / 0.1))
        dz = axial_depth / n_disk

        # Engagement boundaries (conventional milling)
        ratio_ae_d = min(max(radial_depth / tool_diameter, 0.0), 1.0)
        phi_start = math.acos(1.0 - 2.0 * ratio_ae_d)
        phi_exit = math.pi

        # Wear-adjusted edge coefficients
        wear_factor = 1.0 + self.coeffs.wear_force_multiplier * flank_wear_vb
        Kte_w = self.coeffs.Kte * wear_factor
        Kre_w = self.coeffs.Kre * wear_factor
        Kae_w = self.coeffs.Kae * wear_factor

        fx_sum = fy_sum = fz_sum = 0.0
        fx_peak = fy_peak = fz_peak = 0.0
        angle_steps = 360
        two_pi = 2.0 * math.pi

        for a in range(angle_steps):
            phi = a * two_pi / angle_steps
            fx = fy = fz = 0.0

            for j in range(flute_count):
                for k in range(n_disk):
                    z = k * dz
                    lag_angle = (2.0 * math.tan(helix_angle_rad) * z) / tool_diameter
                    phi_j = phi - j * (two_pi / flute_count) - lag_angle
                    phi_j = phi_j % two_pi
                    if phi_j < 0:
                        phi_j += two_pi

                    if phi_start <= phi_j <= phi_exit:
                        h = feed_per_tooth * math.sin(phi_j)
                        if h <= 0:
                            continue

                        dFt = (self.coeffs.Ktc * h + Kte_w) * dz
                        dFr = (self.coeffs.Krc * h + Kre_w) * dz
                        dFa = (self.coeffs.Kac * h + Kae_w) * dz

                        cos_phi = math.cos(phi_j)
                        sin_phi = math.sin(phi_j)
                        fx += -dFt * cos_phi - dFr * sin_phi
                        fy += dFt * sin_phi - dFr * cos_phi
                        fz += dFa

            fx_sum += fx
            fy_sum += fy
            fz_sum += fz
            fx_peak = max(fx_peak, abs(fx))
            fy_peak = max(fy_peak, abs(fy))
            fz_peak = max(fz_peak, abs(fz))

        fx_avg = fx_sum / angle_steps
        fy_avg = fy_sum / angle_steps
        fz_avg = fz_sum / angle_steps

        V = math.pi * tool_diameter * spindle_rpm / 1000.0
        Fc = math.sqrt(fx_avg**2 + fy_avg**2)
        power = abs(Fc) * V / 60.0
        omega = 2.0 * math.pi * spindle_rpm / 60.0
        torque = power / omega if omega > 0 else 0.0
        mrr = radial_depth * axial_depth * feed_per_tooth * flute_count * spindle_rpm
        kc_specific = power * 60.0 / mrr if mrr > 0 else 0.0

        return {
            'fx_peak': fx_peak, 'fy_peak': fy_peak, 'fz_peak': fz_peak,
            'fx_avg': fx_avg, 'fy_avg': fy_avg, 'fz_avg': fz_avg,
            'power_w': power, 'torque_nm': torque, 'mrr': mrr,
            'specific_cutting_energy': kc_specific,
        }

    def update_wear(
        self,
        vb_current: float,
        cutting_time_min: float,
        v_mpm: float,
        fz_mm: float,
        ap_mm: float,
        dt_min: float,
        tool_temperature_c: float = 20.0,
    ) -> tuple:
        """Update wear for one time step. Returns (new_vb, new_cutting_time)."""
        if dt_min <= 0 or v_mpm <= 0:
            return vb_current, cutting_time_min

        cutting_time_min += dt_min
        speed_factor = v_mpm / self.REFERENCE_SPEED
        thermal_factor = min(
            3.0, max(1.0, 1.0 + 0.035 * max(0.0, tool_temperature_c - 20.0))
        )

        if cutting_time_min < self.T1:
            # Stage 1: Break-in
            vb = self.VB0 + (self.VB1 - self.VB0) * math.sqrt(
                cutting_time_min / self.T1
            )
        elif vb_current < 0.25:
            # Stage 2: Steady-state
            rate = self.C2 * speed_factor * thermal_factor
            rate *= (max(fz_mm, 0.01) / 0.05) ** 0.3
            rate *= (max(ap_mm, 0.1) / 1.0) ** 0.1
            vb = vb_current + rate * dt_min
        else:
            # Stage 3: Accelerated
            c3 = 0.1 * speed_factor * thermal_factor
            vb = vb_current + vb_current * c3 * dt_min

        return min(vb, 0.50), cutting_time_min

    def taylor_life_prediction(
        self, v_mpm: float, fz_mm: float, ap_mm: float
    ) -> float:
        """Predict tool life in minutes using Taylor equation."""
        denom = (
            v_mpm
            * (max(fz_mm, 0.001) ** self.TAYLOR_A)
            * (max(ap_mm, 0.01) ** self.TAYLOR_B)
        )
        if denom <= 0:
            return float('inf')
        return (self.TAYLOR_C / denom) ** (1.0 / self.TAYLOR_N)

    def simulate_program(
        self,
        gcode_blocks: List[GCodeBlock],
        tool_state: Optional[ToolState] = None,
        parameter_overrides: Optional[Dict[str, float]] = None,
    ) -> SimulationResult:
        """Simulate an entire G-code program and predict forces, wear, and RUL.

        Args:
            gcode_blocks: List of G-code blocks to simulate.
            tool_state: Current tool state (wear, cutting time).
            parameter_overrides: Optional dict to override spindle_rpm, feed_rate, etc.

        Returns:
            SimulationResult with per-block predictions and overall RUL.
        """
        tool = tool_state or ToolState()
        overrides = parameter_overrides or {}

        vb = tool.flank_wear_vb
        cutting_time = tool.cutting_time_min
        helix_rad = math.radians(tool.helix_angle_deg)

        block_predictions = []
        health_trend = []
        total_time = 0.0

        for block in gcode_blocks:
            rpm = overrides.get('spindle_rpm', block.spindle_rpm)
            feed = overrides.get('feed_rate', block.feed_rate_mmpm)
            ap = overrides.get('axial_depth', block.axial_depth_mm)
            ae = overrides.get('radial_depth', block.radial_depth_mm)

            if rpm <= 0 or feed <= 0:
                block_predictions.append(BlockPrediction())
                continue

            fz = feed / (rpm * tool.flute_count)
            v_mpm = math.pi * tool.diameter_mm * rpm / 1000.0

            # Calculate forces
            forces = self.calculate_forces(
                spindle_rpm=rpm,
                feed_per_tooth=fz,
                axial_depth=ap,
                radial_depth=ae,
                tool_diameter=tool.diameter_mm,
                flute_count=tool.flute_count,
                helix_angle_rad=helix_rad,
                flank_wear_vb=vb,
            )

            # Time for this block
            block_time_min = block.length_mm / feed if feed > 0 else 0.0
            total_time += block_time_min

            # Update wear
            vb, cutting_time = self.update_wear(
                vb, cutting_time, v_mpm, fz, ap, block_time_min
            )

            # Simple thermal estimate
            temp_rise = forces['power_w'] * 0.05  # Simplified

            peak_force = math.sqrt(
                forces['fx_peak'] ** 2
                + forces['fy_peak'] ** 2
                + forces['fz_peak'] ** 2
            )
            avg_force = math.sqrt(
                forces['fx_avg'] ** 2
                + forces['fy_avg'] ** 2
                + forces['fz_avg'] ** 2
            )

            pred = BlockPrediction(
                peak_force_n=peak_force,
                avg_force_n=avg_force,
                power_w=forces['power_w'],
                torque_nm=forces['torque_nm'],
                mrr_mm3pm=forces['mrr'],
                wear_after_block_mm=vb,
                temperature_rise_c=temp_rise,
            )
            block_predictions.append(pred)
            health_trend.append(max(0.0, 1.0 - vb / self.VBMAX))

        # Calculate RUL
        if len(gcode_blocks) > 0 and total_time > 0:
            last_block = gcode_blocks[-1]
            rpm = overrides.get('spindle_rpm', last_block.spindle_rpm)
            feed = overrides.get('feed_rate', last_block.feed_rate_mmpm)
            ap = overrides.get('axial_depth', last_block.axial_depth_mm)

            if rpm > 0 and feed > 0:
                fz = feed / (rpm * tool.flute_count)
                v_mpm = math.pi * tool.diameter_mm * rpm / 1000.0
                taylor_life = self.taylor_life_prediction(v_mpm, fz, ap)
                remaining_life_min = max(0.0, taylor_life - cutting_time)
                rul_hours = remaining_life_min / 60.0
            else:
                rul_hours = 1000.0
        else:
            rul_hours = 1000.0

        health_index = max(0.0, 1.0 - vb / self.VBMAX)

        # Confidence based on data quality
        confidence = min(0.95, 0.6 + 0.01 * len(gcode_blocks))

        # Recommended action
        if vb >= 0.25:
            action = 'Replace tool immediately'
        elif vb >= 0.15:
            action = 'Schedule tool replacement soon'
        elif vb >= 0.10:
            action = 'Monitor tool wear closely'
        elif rul_hours < 24:
            action = 'Schedule maintenance within 24 hours'
        elif rul_hours < 168:
            action = 'Plan maintenance within one week'
        else:
            action = 'Continue normal operation'

        return SimulationResult(
            block_predictions=block_predictions,
            total_cutting_time_min=total_time,
            final_wear_mm=vb,
            remaining_useful_life_hours=rul_hours,
            confidence=confidence,
            health_index=health_index,
            trend_data=health_trend[-10:] if health_trend else [health_index],
            recommended_action=action,
        )
