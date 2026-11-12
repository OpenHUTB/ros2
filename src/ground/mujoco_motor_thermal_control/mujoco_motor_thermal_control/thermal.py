"""基于集总热容的电机温升模型。"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ThermalParameters:
    heat_capacity_j_per_c: float = 28.0
    winding_resistance_ohm: float = 0.42
    cooling_w_per_c: float = 0.22
    torque_constant_nm_per_a: float = 0.18
    idle_current_a: float = 0.25


@dataclass
class MotorThermalModel:
    temperature_c: float = 25.0
    parameters: ThermalParameters = ThermalParameters()

    def update(self, torque_nm: float, dt: float, ambient_c: float = 25.0) -> float:
        """Advance winding temperature by one explicit Euler step."""
        if dt <= 0:
            raise ValueError("dt must be positive")
        p = self.parameters
        current_a = abs(torque_nm) / p.torque_constant_nm_per_a + p.idle_current_a
        copper_loss_w = current_a * current_a * p.winding_resistance_ohm
        cooling_w = p.cooling_w_per_c * (self.temperature_c - ambient_c)
        self.temperature_c += (copper_loss_w - cooling_w) * dt / p.heat_capacity_j_per_c
        return self.temperature_c
