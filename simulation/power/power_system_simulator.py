#!/usr/bin/env python3
"""
Power System Simulator - URC Machiato 2026

MOCK IMPLEMENTATION - Simulates rover power systems including battery management,
solar charging, thermal control, and power distribution.

This is a software simulation of the physical power system hardware and should be
replaced with actual power management hardware when available.

Author: URC 2026 Autonomy Team
"""

import logging
import time
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)


class PowerSource(Enum):
    """Types of power sources."""

    BATTERY = "battery"
    SOLAR = "solar"
    EXTERNAL = "external"


class PowerConsumer(Enum):
    """Types of power consumers."""

    DRIVE_SYSTEM = "drive_system"
    ARM_SYSTEM = "arm_system"
    SCIENCE_PAYLOAD = "science_payload"
    COMPUTING = "computing"
    COMMUNICATION = "communication"
    THERMAL_CONTROL = "thermal_control"
    LIGHTING = "lighting"


@dataclass
class BatteryCell:
    """Individual battery cell model."""

    voltage: float  # Volts
    capacity: float  # Ah
    temperature: float  # °C
    state_of_charge: float  # 0-1
    internal_resistance: float  # ohms
    max_charge_current: float  # A
    max_discharge_current: float  # A


@dataclass
class SolarPanel:
    """Solar panel model."""

    area: float  # m²
    efficiency: float  # 0-1
    max_power: float  # Watts
    temperature_coefficient: float  # %/°C
    current_temperature: float  # °C
    orientation: Tuple[float, float, float]  # yaw, pitch, roll


class PowerSystemSimulator:
    """
    MOCK IMPLEMENTATION - Complete power system simulation.

    Simulates:
    - Battery pack with multiple cells
    - Solar panel charging system
    - Power distribution to subsystems
    - Thermal management and cooling
    - Power budgeting and optimization
    - Fault detection and emergency shutdown

    This replaces physical power management hardware for software development.
    """

    def __init__(self, config: Optional[Dict] = None):
        self.logger = logging.getLogger(__name__)

        # Default power system configuration
        if config is None:
            config = self._get_default_config()

        self._load_configuration(config)
        self._initialize_state()

        self.logger.info(" MOCK Power System Simulator initialized")

    def _get_default_config(self) -> Dict:
        """Get default power system configuration."""
        return {
            "name": "URC_Power_System_Simulator",
            "battery_pack": {
                "series_cells": 12,  # 12S configuration
                "parallel_strings": 2,  # 2P configuration
                "nominal_voltage": 48.0,  # Volts (4V per cell * 12)
                "capacity": 100.0,  # Ah total pack capacity
                "cell_capacity": 100.0,  # Ah per cell
                "max_charge_voltage": 54.0,  # Volts
                "min_discharge_voltage": 36.0,  # Volts
                "max_charge_current": 50.0,  # Amps
                "max_discharge_current": 200.0,  # Amps
                "initial_soc": 0.8,  # 80% state of charge
            },
            "solar_panels": [
                {
                    "name": "main_panel",
                    "area": 2.0,  # m²
                    "efficiency": 0.22,  # 22%
                    "max_power": 400.0,  # Watts
                    "orientation": [0.0, 0.0, 0.0],  # yaw, pitch, roll
                },
                {
                    "name": "aux_panel",
                    "area": 1.0,
                    "efficiency": 0.20,
                    "max_power": 200.0,
                    "orientation": [0.0, 0.0, 0.0],
                },
            ],
            "thermal_management": {
                "heat_sink_area": 1.0,  # m²
                "cooling_power": 100.0,  # Watts available for cooling
                "target_temperature": 35.0,  # °C
                "max_temperature": 60.0,  # °C
            },
            "power_distribution": {
                "bus_voltage": 48.0,  # Main bus voltage
                "efficiency": 0.95,  # DC-DC converter efficiency
                "protection_thresholds": {
                    "overcurrent": 250.0,  # Amps
                    "overvoltage": 55.0,  # Volts
                    "undervoltage": 35.0,  # Volts
                },
            },
        }

    def _load_configuration(self, config: Dict):
        """Load power system configuration."""
        self.name = config["name"]
        self.battery_config = config["battery_pack"]
        self.solar_config = config["solar_panels"]
        self.thermal_config = config["thermal_management"]
        self.distribution_config = config["power_distribution"]

    def _initialize_state(self):
        """Initialize power system state."""
        # Battery pack
        self.cells = []
        total_cells = (
            self.battery_config["series_cells"]
            * self.battery_config["parallel_strings"]
        )

        for i in range(total_cells):
            cell = BatteryCell(
                voltage=4.0,  # Nominal cell voltage
                capacity=self.battery_config["cell_capacity"],
                temperature=25.0,
                state_of_charge=self.battery_config["initial_soc"],
                internal_resistance=0.01,  # ohms
                max_charge_current=10.0,
                max_discharge_current=50.0,
            )
            self.cells.append(cell)

        # Solar panels
        self.solar_panels = []
        for panel_config in self.solar_config:
            panel = SolarPanel(
                area=panel_config["area"],
                efficiency=panel_config["efficiency"],
                max_power=panel_config["max_power"],
                temperature_coefficient=-0.004,  # -0.4%/°C
                current_temperature=25.0,
                orientation=tuple(panel_config["orientation"]),
            )
            self.solar_panels.append(panel)

        # Power state
        self.bus_voltage = self.distribution_config["bus_voltage"]
        self.bus_current = 0.0
        self.total_power_consumption = 0.0
        self.solar_power_generated = 0.0

        # Power consumers (power draw in Watts)
        self.power_consumers = {
            PowerConsumer.DRIVE_SYSTEM: 0.0,
            PowerConsumer.ARM_SYSTEM: 0.0,
            PowerConsumer.SCIENCE_PAYLOAD: 0.0,
            PowerConsumer.COMPUTING: 50.0,  # Base computing power
            PowerConsumer.COMMUNICATION: 10.0,  # Base comms power
            PowerConsumer.THERMAL_CONTROL: 0.0,
            PowerConsumer.LIGHTING: 0.0,
        }

        # Thermal management
        self.ambient_temperature = 25.0
        self.cooling_active = False
        self.cooling_power_used = 0.0

        # System state
        self.enabled = True
        self.charging_enabled = True
        self.emergency_shutdown = False
        self.faults = []

        # Performance tracking
        self.last_update_time = time.time()
        self.uptime_hours = 0.0

    def set_power_consumption(
        self, consumer: PowerConsumer, power_watts: float
    ) -> bool:
        """
        Set power consumption for a subsystem.

        Args:
            consumer: Power consumer type
            power_watts: Power consumption in watts

        Returns:
            bool: True if power allocation successful
        """
        if not self.enabled:
            return False

        # Validate power limits
        max_available = self.get_available_power()
        if power_watts > max_available:
            self.logger.warning(
                f"Power request {power_watts}W exceeds available {max_available}W"
            )
            return False

        self.power_consumers[consumer] = power_watts
        self.logger.debug(f"Set {consumer.value} power consumption: {power_watts}W")
        return True

    def get_available_power(self) -> float:
        """Get currently available power in watts."""
        # Calculate total battery power capability
        pack_voltage = self.get_pack_voltage()
        pack_current_limit = self.battery_config["max_discharge_current"]

        battery_power_limit = pack_voltage * pack_current_limit

        # Add solar power contribution
        solar_power = sum(panel.max_power for panel in self.solar_panels)

        # Total available power (with some reserve margin)
        available_power = (battery_power_limit + solar_power) * 0.8

        return available_power

    def get_pack_voltage(self) -> float:
        """Get current battery pack voltage."""
        # Calculate series voltage
        series_voltage = sum(
            cell.voltage for cell in self.cells[: self.battery_config["series_cells"]]
        )
        return series_voltage

    def get_state_of_charge(self) -> float:
        """Get battery state of charge (0-1)."""
        # Average SOC across all cells
        total_soc = sum(cell.state_of_charge for cell in self.cells)
        return total_soc / len(self.cells)

    def get_total_energy_remaining(self) -> float:
        """Get total energy remaining in watt-hours."""
        pack_capacity = self.battery_config["capacity"]  # Ah
        pack_voltage = self.battery_config["nominal_voltage"]
        soc = self.get_state_of_charge()

        return pack_capacity * pack_voltage * soc

    def update_simulation(self, dt: float):
        """
        Update power system simulation.

        Args:
            dt: Time step in seconds
        """
        if not self.enabled:
            return

        current_time = time.time()

        # Calculate total power consumption
        total_consumption = sum(self.power_consumers.values())

        # Calculate solar power generation
        solar_power = self._calculate_solar_power()
        self.solar_power_generated = solar_power

        # Net power flow (positive = consumption, negative = generation)
        net_power_flow = total_consumption - solar_power

        # Update battery state
        if net_power_flow > 0:
            # Discharging
            self._discharge_battery(net_power_flow, dt)
        else:
            # Charging (from solar)
            if self.charging_enabled:
                self._charge_battery(-net_power_flow, dt)

        # Update thermal management
        self._update_thermal_model(dt)

        # Update power distribution
        self._update_power_distribution(dt)

        # Check for faults and safety limits
        self._check_safety_limits()

        # Update uptime
        self.uptime_hours += dt / 3600.0

        self.last_update_time = current_time

    def get_power_state(self) -> Dict:
        """Get complete power system state."""
        return {
            "enabled": self.enabled,
            "charging_enabled": self.charging_enabled,
            "emergency_shutdown": self.emergency_shutdown,
            "battery": {
                "pack_voltage": self.get_pack_voltage(),
                "state_of_charge": self.get_state_of_charge(),
                "energy_remaining": self.get_total_energy_remaining(),
                "capacity": self.battery_config["capacity"],
                "nominal_voltage": self.battery_config["nominal_voltage"],
                "cell_count": len(self.cells),
                "cell_temperatures": [cell.temperature for cell in self.cells],
                "cell_soc": [cell.state_of_charge for cell in self.cells],
            },
            "solar": {
                "total_power_generated": self.solar_power_generated,
                "panels": [
                    {
                        "name": panel_config["name"],
                        "power": panel.max_power,
                        "temperature": 25.0,  # Simplified
                    }
                    for panel_config in self.solar_config
                ],
            },
            "power_consumers": {
                consumer.value: power
                for consumer, power in self.power_consumers.items()
            },
            "power_distribution": {
                "bus_voltage": self.bus_voltage,
                "bus_current": self.bus_current,
                "total_consumption": sum(self.power_consumers.values()),
                "efficiency": self.distribution_config["efficiency"],
            },
            "thermal": {
                "ambient_temperature": self.ambient_temperature,
                "cooling_active": self.cooling_active,
                "cooling_power_used": self.cooling_power_used,
                "battery_avg_temperature": np.mean(
                    [cell.temperature for cell in self.cells]
                ),
            },
            "system": {
                "uptime_hours": self.uptime_hours,
                "total_energy_consumed": self.total_power_consumption,
                "faults": self.faults.copy(),
            },
            "mock": True,
            "simulated": True,
        }

    def emergency_shutdown_power(self) -> bool:
        """Execute emergency power shutdown."""
        self.logger.warning(" POWER SYSTEM EMERGENCY SHUTDOWN")
        self.emergency_shutdown = True
        self.enabled = False

        # Cut power to all consumers
        for consumer in self.power_consumers:
            self.power_consumers[consumer] = 0.0

        # Stop cooling
        self.cooling_active = False
        self.cooling_power_used = 0.0

        return True

    def enable_power_system(self) -> bool:
        """Enable power system operation."""
        if not self.emergency_shutdown:
            self.enabled = True
            self.logger.info("Power system enabled")
            return True
        else:
            self.logger.warning(
                "Cannot enable power system while in emergency shutdown"
            )
            return False

    def enable_charging(self, enable: bool) -> bool:
        """Enable or disable battery charging."""
        self.charging_enabled = enable
        self.logger.info(f"Battery charging {'enabled' if enable else 'disabled'}")
        return True

    def _calculate_solar_power(self) -> float:
        """Calculate current solar power generation."""
        total_power = 0.0

        # Simplified solar calculation (would use actual solar model in real implementation)
        # Assume 60% of max power under good conditions
        solar_efficiency = 0.6

        for panel in self.solar_panels:
            power = panel.max_power * solar_efficiency

            # Temperature derating
            temp_derating = 1.0 + panel.temperature_coefficient * (
                panel.current_temperature - 25.0
            )
            power *= max(0.0, temp_derating)

            total_power += power

        return total_power

    def _discharge_battery(self, power_demand: float, dt: float):
        """Simulate battery discharge."""
        # Calculate current draw
        pack_voltage = self.get_pack_voltage()
        current_draw = power_demand / pack_voltage

        # Limit current draw
        max_current = self.battery_config["max_discharge_current"]
        current_draw = min(current_draw, max_current)

        # Update each cell
        for cell in self.cells:
            # Peukert's law approximation for capacity at different discharge rates
            effective_capacity = cell.capacity * (
                1.0 - 0.1 * np.log10(current_draw / 10.0)
            )

            # Update state of charge
            energy_used = (
                current_draw * pack_voltage * dt / len(self.cells) / 3600.0
            )  # Wh
            capacity_used = energy_used / cell.voltage  # Ah
            cell.state_of_charge = max(
                0.0, cell.state_of_charge - capacity_used / effective_capacity
            )

            # Update voltage (simplified model)
            cell.voltage = 3.7 + 0.3 * cell.state_of_charge  # Rough Li-ion model

            # Update temperature (heating from discharge)
            heat_generated = current_draw * current_draw * cell.internal_resistance * dt
            temp_rise = heat_generated / 10.0  # Simplified thermal mass
            cell.temperature += temp_rise

        # Update total energy consumption
        energy_used = power_demand * dt / 3600.0  # Wh
        self.total_power_consumption += energy_used

    def _charge_battery(self, charge_power: float, dt: float):
        """Simulate battery charging."""
        if not self.charging_enabled:
            return

        # Calculate charge current
        pack_voltage = self.get_pack_voltage()
        charge_current = charge_power / pack_voltage

        # Limit charge current
        max_charge = self.battery_config["max_charge_current"]
        charge_current = min(charge_current, max_charge)

        # Update each cell
        for cell in self.cells:
            # Update state of charge
            capacity_added = charge_current * dt / len(self.cells) / 3600.0  # Ah
            cell.state_of_charge = min(
                1.0, cell.state_of_charge + capacity_added / cell.capacity
            )

            # Update voltage
            cell.voltage = 3.7 + 0.3 * cell.state_of_charge

            # Update temperature (slight heating from charging)
            heat_generated = (
                charge_current * charge_current * cell.internal_resistance * dt * 0.5
            )
            temp_rise = heat_generated / 10.0
            cell.temperature += temp_rise

    def _update_thermal_model(self, dt: float):
        """Update thermal management simulation."""
        # Calculate average battery temperature
        avg_battery_temp = np.mean([cell.temperature for cell in self.cells])

        # Thermal management logic
        target_temp = self.thermal_config["target_temperature"]
        max_temp = self.thermal_config["max_temperature"]

        if avg_battery_temp > target_temp + 5.0:
            # Activate cooling
            self.cooling_active = True
            self.cooling_power_used = min(
                self.thermal_config["cooling_power"],
                (avg_battery_temp - target_temp) * 10.0,  # Proportional control
            )
        elif avg_battery_temp < target_temp - 5.0:
            # Deactivate cooling
            self.cooling_active = False
            self.cooling_power_used = 0.0

        # Update power consumer for cooling
        self.power_consumers[PowerConsumer.THERMAL_CONTROL] = self.cooling_power_used

        # Ambient temperature variation (simplified)
        self.ambient_temperature += np.random.normal(0, 0.1) * dt
        self.ambient_temperature = np.clip(self.ambient_temperature, -10, 50)

        # Heat transfer to ambient
        for cell in self.cells:
            heat_transfer = (cell.temperature - self.ambient_temperature) * 0.1 * dt
            cell.temperature -= heat_transfer

    def _update_power_distribution(self, dt: float):
        """Update power distribution simulation."""
        total_load = sum(self.power_consumers.values())

        # Calculate bus current
        self.bus_current = total_load / self.bus_voltage

        # Apply efficiency losses
        efficiency = self.distribution_config["efficiency"]
        actual_power_delivered = total_load / efficiency

        # Update bus voltage (slight droop under load)
        voltage_droop = self.bus_current * 0.01  # 10mV per amp
        self.bus_voltage = self.distribution_config["bus_voltage"] - voltage_droop

    def _check_safety_limits(self):
        """Check safety limits and trigger faults."""
        thresholds = self.distribution_config["protection_thresholds"]

        # Overcurrent protection
        if self.bus_current > thresholds["overcurrent"]:
            if "overcurrent" not in self.faults:
                self.faults.append("overcurrent")
                self.logger.error(" OVERCURRENT FAULT DETECTED")

        # Over/under voltage protection
        if self.bus_voltage > thresholds["overvoltage"]:
            if "overvoltage" not in self.faults:
                self.faults.append("overvoltage")
                self.logger.error(" OVERVOLTAGE FAULT DETECTED")

        if self.bus_voltage < thresholds["undervoltage"]:
            if "undervoltage" not in self.faults:
                self.faults.append("undervoltage")
                self.logger.error(" UNDERVOLTAGE FAULT DETECTED")

        # Low battery protection
        soc = self.get_state_of_charge()
        if soc < 0.1:
            if "low_battery" not in self.faults:
                self.faults.append("low_battery")
                self.logger.warning(" LOW BATTERY WARNING")

        if soc < 0.05:
            self.emergency_shutdown_power()
            self.logger.critical(" BATTERY CRITICAL - EMERGENCY SHUTDOWN")

        # Cell imbalance detection
        cell_socs = [cell.state_of_charge for cell in self.cells]
        soc_spread = max(cell_socs) - min(cell_socs)

        if soc_spread > 0.1:  # 10% SOC difference
            if "cell_imbalance" not in self.faults:
                self.faults.append("cell_imbalance")
                self.logger.warning(" BATTERY CELL IMBALANCE DETECTED")

        # Thermal protection
        max_cell_temp = max(cell.temperature for cell in self.cells)
        if max_cell_temp > 60.0:
            if "thermal_runaway" not in self.faults:
                self.faults.append("thermal_runaway")
                self.emergency_shutdown_power()
                self.logger.critical(" THERMAL RUNAWAY DETECTED - EMERGENCY SHUTDOWN")
