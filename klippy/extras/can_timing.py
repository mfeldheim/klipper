# CAN-specific timing optimizations for Klipper
#
# Copyright (C) 2025  Enhanced CAN Timing
#
# This file may be distributed under the terms of the GNU GPLv3 license.

class CANTimingOptimizer:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.can_mcus = {}
        
        # CAN-specific timing parameters
        self.can_sync_interval = config.getfloat('can_sync_interval', 0.7839)
        self.can_rtt_tolerance = config.getfloat('can_rtt_tolerance', 0.002)
        self.can_retry_threshold = config.getint('can_retry_threshold', 100)
        
        # Register for MCU connection events
        self.printer.register_event_handler("klippy:mcu_identify", 
                                           self._handle_mcu_identify)
        
    def _handle_mcu_identify(self, mcu):
        # Check if this is a CAN MCU
        serial = mcu.get_serialport()
        if hasattr(serial, '_canbus_iface'):
            self.can_mcus[mcu.get_name()] = mcu
            self._optimize_can_mcu(mcu)
            
    def _optimize_can_mcu(self, mcu):
        # Apply CAN-specific optimizations
        clocksync = mcu._clocksync
        
        # Mark as CAN MCU for timing adjustments
        if not hasattr(clocksync.serial, '_canbus_iface'):
            clocksync.serial._canbus_iface = True
            
        # Enhanced prediction variance for CAN network delays
        base_variance = (.001 * clocksync.mcu_freq)**2
        clocksync.prediction_variance = base_variance * 1.5  # 50% higher for CAN
        
    def get_status(self, eventtime):
        status = {}
        for name, mcu in self.can_mcus.items():
            clocksync = mcu._clocksync
            rtt = getattr(clocksync, 'min_half_rtt', 0) * 2000  # Convert to ms
            status[name] = {
                'rtt_ms': round(rtt, 3),
                'freq_drift': round(clocksync.clock_est[2] - clocksync.mcu_freq, 1),
                'is_can_optimized': True
            }
        return status

def load_config(config):
    return CANTimingOptimizer(config)
