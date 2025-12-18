#!/usr/bin/env python3
"""
URC Band Manager - Dynamic band switching and interference monitoring

Manages URC competition band restrictions and provides dynamic switching
between 900MHz sub-bands and 2.4GHz with interference monitoring.
"""

import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from typing import Dict, List, Any, Optional
import time
import threading

from autonomy_interfaces.srv import GetSubsystemStatus


class URCBandManager(Node):
    """
    Manages URC band switching and interference monitoring.

    Handles the competition requirements:
    - 900 MHz band: 8 MHz max bandwidth, sub-band switching
    - 2.4 GHz band: No restrictions but interference monitoring
    - Frequency hopping and automatic channel selection
    """

    def __init__(self):
        super().__init__('urc_band_manager')

        # URC Band configuration (from competition rules)
        self.band_config = {
            '900mhz': {
                'max_bandwidth_mhz': 8.0,
                'sub_bands': {
                    'low': {
                        'range_mhz': (902, 910),
                        'active': False,
                        'interference_level': 0.0,
                        'last_used': 0
                    },
                    'mid': {
                        'range_mhz': (911, 919),
                        'active': False,
                        'interference_level': 0.0,
                        'last_used': 0
                    },
                    'high': {
                        'range_mhz': (920, 928),
                        'active': False,
                        'interference_level': 0.0,
                        'last_used': 0
                    }
                },
                'current_subband': None,
                'frequency_hopping': True,
                'channel_selection': True
            },
            '2_4ghz': {
                'max_bandwidth_mhz': None,  # No FCC limit
                'channels': [1, 6, 11],  # Non-overlapping channels
                'current_channel': 1,
                'interference_tolerant': True,
                'frequency_hopping': True,
                'channel_selection': True,
                'interference_level': 0.0
            }
        }

        # Current band state
        self.current_band = 'unknown'
        self.band_switching_enabled = True
        self.interference_monitoring = True

        # Interference monitoring
        self.interference_history = {
            '900mhz': {subband: [] for subband in ['low', 'mid', 'high']},
            '2_4ghz': []
        }
        self.interference_threshold = 0.7  # 70% interference triggers switch

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('initial_band', '900mhz'),
                ('initial_subband', 'low'),
                ('band_switching_enabled', True),
                ('interference_monitoring', True),
                ('interference_check_rate', 2.0),  # Hz
                ('band_switch_cooldown', 30.0),    # seconds between switches
                ('interference_threshold', 0.7),
                ('frequency_hopping_rate', 10.0)   # Hz
            ]
        )

        # Get parameters
        initial_band = self.get_parameter('initial_band').value
        initial_subband = self.get_parameter('initial_subband').value
        self.band_switching_enabled = self.get_parameter('band_switching_enabled').value
        self.interference_monitoring = self.get_parameter('interference_monitoring').value
        self.interference_check_rate = self.get_parameter('interference_check_rate').value
        self.band_switch_cooldown = self.get_parameter('band_switch_cooldown').value
        self.interference_threshold = self.get_parameter('interference_threshold').value
        self.frequency_hopping_rate = self.get_parameter('frequency_hopping_rate').value

        # Initialize band
        self.set_band(initial_band, initial_subband)

        # Setup timers
        self.create_timer(1.0 / self.interference_check_rate, self._interference_check_callback)
        if self.band_config['900mhz']['frequency_hopping']:
            self.create_timer(1.0 / self.frequency_hopping_rate, self._frequency_hopping_callback)

        # Setup services
        self.create_service(
            GetSubsystemStatus,
            '/urc_band_manager/get_status',
            self._handle_get_status
        )

        self.last_band_switch = time.time()

        self.get_logger().info(f"URC Band Manager initialized - Band: {self.current_band}")

    def set_band(self, band: str, subband: Optional[str] = None) -> bool:
        """
        Set the current URC band configuration.

        Args:
            band: '900mhz' or '2_4ghz'
            subband: For 900MHz, 'low', 'mid', or 'high'

        Returns:
            bool: Success of band switch
        """
        if band not in ['900mhz', '2_4ghz']:
            self.get_logger().error(f"Invalid band: {band}")
            return False

        # Check cooldown period
        if self.band_switching_enabled and time.time() - self.last_band_switch < self.band_switch_cooldown:
            self.get_logger().warn("Band switch cooldown active, ignoring request")
            return False

        old_band = self.current_band
        self.current_band = band

        if band == '900mhz':
            if subband and subband in self.band_config['900mhz']['sub_bands']:
                # Deactivate all subbands
                for sb_name in self.band_config['900mhz']['sub_bands']:
                    self.band_config['900mhz']['sub_bands'][sb_name]['active'] = False

                # Activate selected subband
                self.band_config['900mhz']['sub_bands'][subband]['active'] = True
                self.band_config['900mhz']['current_subband'] = subband
                self.band_config['900mhz']['sub_bands'][subband]['last_used'] = time.time()

                self.get_logger().info(
                    f"Switched to 900MHz {subband} band "
                    f"({self.band_config['900mhz']['sub_bands'][subband]['range_mhz']} MHz)"
                )
            else:
                self.get_logger().error(f"Invalid 900MHz subband: {subband}")
                return False

        elif band == '2_4ghz':
            self.get_logger().info(f"Switched to 2.4GHz band (channel {self.band_config['2_4ghz']['current_channel']})")

        # Notify competition bridge of band change
        self._notify_band_change(old_band, band, subband)

        self.last_band_switch = time.time()
        return True

    def get_band_info(self) -> Dict[str, Any]:
        """Get current band configuration and status."""
        return {
            'current_band': self.current_band,
            '900mhz': {
                'current_subband': self.band_config['900mhz']['current_subband'],
                'max_bandwidth_mhz': self.band_config['900mhz']['max_bandwidth_mhz'],
                'sub_bands': self.band_config['900mhz']['sub_bands']
            },
            '2_4ghz': {
                'current_channel': self.band_config['2_4ghz']['current_channel'],
                'interference_level': self.band_config['2_4ghz']['interference_level']
            },
            'band_switching_enabled': self.band_switching_enabled,
            'interference_monitoring': self.interference_monitoring,
            'last_switch_time': self.last_band_switch
        }

    def measure_interference(self, band: str, subband: Optional[str] = None) -> float:
        """
        Measure interference level for a specific band/subband.

        Returns:
            float: Interference level (0.0 = no interference, 1.0 = severe interference)
        """
        # This would integrate with actual radio hardware/signal analysis
        # For now, simulate interference measurement
        import random
        interference = random.uniform(0.0, 0.3)  # Low interference baseline

        # Add some realistic variation based on band
        if band == '2_4ghz':
            # 2.4GHz has more potential interference from WiFi/other teams
            interference += random.uniform(0.0, 0.4)
        elif band == '900mhz' and subband:
            # 900MHz generally has less interference
            interference += random.uniform(0.0, 0.2)

        return min(1.0, interference)

    def _interference_check_callback(self):
        """Periodic interference monitoring and band switching."""
        if not self.interference_monitoring:
            return

        # Measure interference on all bands
        current_interference = {}

        # Check 900MHz subbands
        for subband in ['low', 'mid', 'high']:
            interference = self.measure_interference('900mhz', subband)
            current_interference[f'900mhz_{subband}'] = interference

            # Store in history
            self.interference_history['900mhz'][subband].append({
                'timestamp': time.time(),
                'level': interference
            })

            # Keep only recent history
            if len(self.interference_history['900mhz'][subband]) > 50:
                self.interference_history['900mhz'][subband].pop(0)

        # Check 2.4GHz
        interference_2_4 = self.measure_interference('2_4ghz')
        current_interference['2_4ghz'] = interference_2_4
        self.band_config['2_4ghz']['interference_level'] = interference_2_4

        self.interference_history['2_4ghz'].append({
            'timestamp': time.time(),
            'level': interference_2_4
        })
        if len(self.interference_history['2_4ghz']) > 50:
            self.interference_history['2_4ghz'].pop(0)

        # Check if we need to switch bands due to interference
        if self.band_switching_enabled:
            self._check_band_switch_needed(current_interference)

    def _check_band_switch_needed(self, interference_levels: Dict[str, float]):
        """Determine if band switching is needed due to interference."""
        current_band = self.current_band

        if current_band == '900mhz':
            current_subband = self.band_config['900mhz']['current_subband']
            current_key = f'900mhz_{current_subband}'
            current_interference = interference_levels.get(current_key, 0.0)

            if current_interference > self.interference_threshold:
                # Current subband has high interference, find better option
                best_subband = None
                best_interference = current_interference

                for subband in ['low', 'mid', 'high']:
                    if subband != current_subband:
                        interference = interference_levels.get(f'900mhz_{subband}', 1.0)
                        if interference < best_interference:
                            best_subband = subband
                            best_interference = interference

                if best_subband:
                    self.get_logger().warn(
                        f"High interference on {current_subband} subband ({current_interference:.2f}), "
                        f"switching to {best_subband} subband ({best_interference:.2f})"
                    )
                    self.set_band('900mhz', best_subband)

                # If all 900MHz subbands have high interference, consider switching to 2.4GHz
                elif interference_levels.get('2_4ghz', 1.0) < self.interference_threshold:
                    self.get_logger().warn(
                        "High interference on all 900MHz subbands, switching to 2.4GHz"
                    )
                    self.set_band('2_4ghz')

        elif current_band == '2_4ghz':
            current_interference = interference_levels.get('2_4ghz', 0.0)

            if current_interference > self.interference_threshold:
                # Try switching 2.4GHz channels first
                self._switch_2_4ghz_channel()

                # If still high interference, consider switching back to 900MHz
                best_900mhz = min(
                    [(subband, interference_levels.get(f'900mhz_{subband}', 1.0))
                     for subband in ['low', 'mid', 'high']],
                    key=lambda x: x[1]
                )

                if best_900mhz[1] < current_interference:
                    self.get_logger().warn(
                        f"High interference on 2.4GHz ({current_interference:.2f}), "
                        f"switching to 900MHz {best_900mhz[0]} subband ({best_900mhz[1]:.2f})"
                    )
                    self.set_band('900mhz', best_900mhz[0])

    def _switch_2_4ghz_channel(self):
        """Switch to a different 2.4GHz channel with lower interference."""
        current_channel = self.band_config['2_4ghz']['current_channel']
        available_channels = [ch for ch in self.band_config['2_4ghz']['channels'] if ch != current_channel]

        if available_channels:
            new_channel = available_channels[0]  # Simple selection, could be more sophisticated
            self.band_config['2_4ghz']['current_channel'] = new_channel
            self.get_logger().info(f"Switched 2.4GHz to channel {new_channel}")

    def _frequency_hopping_callback(self):
        """Implement frequency hopping within current band."""
        if not self.band_config['900mhz']['frequency_hopping'] or self.current_band != '900mhz':
            return

        # Simple frequency hopping - could be more sophisticated
        current_subband = self.band_config['900mhz']['current_subband']
        subbands = ['low', 'mid', 'high']

        # Find next subband in sequence
        current_index = subbands.index(current_subband)
        next_index = (current_index + 1) % len(subbands)
        next_subband = subbands[next_index]

        # Only hop if next subband has acceptable interference
        next_interference = 0.0
        if self.interference_history['900mhz'][next_subband]:
            recent_measurements = self.interference_history['900mhz'][next_subband][-5:]
            next_interference = sum(m['level'] for m in recent_measurements) / len(recent_measurements)

        if next_interference < self.interference_threshold:
            self.set_band('900mhz', next_subband)

    def _notify_band_change(self, old_band: str, new_band: str, subband: Optional[str]):
        """Notify other components of band changes."""
        # This would publish to relevant topics or call services
        # For now, just log the change
        self.get_logger().info(f"Band changed: {old_band} -> {new_band}" + (f" ({subband})" if subband else ""))

    def _handle_get_status(self, request, response):
        """Handle subsystem status requests."""
        response.subsystem_name = 'urc_band_manager'
        response.status = 'operational'
        response.details = str(self.get_band_info())
        response.timestamp = time.time()

        return response


def main(args=None):
    rclpy.init(args=args)

    band_manager = URCBandManager()

    try:
        rclpy.spin(band_manager)
    except KeyboardInterrupt:
        pass
    finally:
        band_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


