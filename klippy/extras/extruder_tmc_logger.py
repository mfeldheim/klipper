# Extruder TMC Logger - Log cs_actual and sg_result from extruder steppers
#
# Copyright (C) 2024  Michel Feldheim <m.feldheim@opendi.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import os
import time
import threading

class ExtruderTMCLogger:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        
        # Configuration
        self.log_file_path = config.get('log_file', '/tmp/extruder_tmc.log')
        self.log_interval = config.getfloat('log_interval', 0.05, minval=0.01)  # 50ms default
        self.enabled = config.getboolean('enabled', False)
        
        # Internal state
        self.extruder_tmcs = {}  # Dict of extruder_name -> TMC object
        self.log_timer = None
        self.log_file = None
        self.lock = threading.Lock()
        
        # Register event handlers
        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self.printer.register_event_handler("klippy:disconnect", self._handle_disconnect)
        
        # Register G-code commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("EXTRUDER_TMC_LOG_START", self.cmd_start_logging,
                              desc="Start logging extruder TMC data")
        gcode.register_command("EXTRUDER_TMC_LOG_STOP", self.cmd_stop_logging,
                              desc="Stop logging extruder TMC data")
        gcode.register_command("EXTRUDER_TMC_LOG_STATUS", self.cmd_log_status,
                              desc="Show extruder TMC logging status")
    
    def _handle_connect(self):
        """Called when Klipper connects - discover extruder TMC drivers"""
        self._discover_extruder_tmcs()
        if self.enabled and self.extruder_tmcs:
            self._start_logging()
    
    def _handle_disconnect(self):
        """Called when Klipper disconnects - stop logging"""
        self._stop_logging()
    
    def _discover_extruder_tmcs(self):
        """Find all extruder steppers with TMC drivers"""
        self.extruder_tmcs.clear()
        
        # Look for extruder objects
        for i in range(10):  # Check extruder, extruder1, extruder2, etc.
            extruder_name = 'extruder' if i == 0 else f'extruder{i}'
            try:
                extruder = self.printer.lookup_object(extruder_name, None)
                if extruder is None:
                    continue
                
                # Check if extruder has a stepper with TMC driver
                if hasattr(extruder, 'extruder_stepper') and extruder.extruder_stepper:
                    stepper_name = extruder.extruder_stepper.stepper.get_name()
                    tmc_obj = self._find_tmc_for_stepper(stepper_name)
                    if tmc_obj:
                        self.extruder_tmcs[extruder_name] = {
                            'tmc': tmc_obj,
                            'stepper_name': stepper_name
                        }
                        logging.info("ExtruderTMCLogger: Found TMC driver for %s (%s)", 
                                   extruder_name, stepper_name)
            except Exception as e:
                logging.debug("ExtruderTMCLogger: Error checking %s: %s", extruder_name, e)
        
        # Also check extruder_stepper objects
        try:
            objects = self.printer.lookup_objects('extruder_stepper')
            for name, obj in objects:
                if hasattr(obj, 'extruder_stepper'):
                    stepper_name = obj.extruder_stepper.stepper.get_name()
                    tmc_obj = self._find_tmc_for_stepper(stepper_name)
                    if tmc_obj:
                        self.extruder_tmcs[name] = {
                            'tmc': tmc_obj,
                            'stepper_name': stepper_name
                        }
                        logging.info("ExtruderTMCLogger: Found TMC driver for %s (%s)", 
                                   name, stepper_name)
        except Exception as e:
            logging.debug("ExtruderTMCLogger: Error checking extruder_steppers: %s", e)
        
        logging.info("ExtruderTMCLogger: Discovered %d extruder TMC drivers", 
                    len(self.extruder_tmcs))
    
    def _find_tmc_for_stepper(self, stepper_name):
        """Find TMC driver object for a given stepper name"""
        # Try different TMC driver types
        tmc_types = ['tmc2130', 'tmc2208', 'tmc2209', 'tmc2240', 'tmc2660', 'tmc5160']
        
        for tmc_type in tmc_types:
            try:
                tmc_name = f"{tmc_type} {stepper_name}"
                tmc_obj = self.printer.lookup_object(tmc_name, None)
                if tmc_obj and hasattr(tmc_obj, 'mcu_tmc'):
                    # Check if this TMC supports the registers we need
                    fields = tmc_obj.mcu_tmc.get_fields()
                    drv_status_fields = fields.all_fields.get("DRV_STATUS", {})
                    if "cs_actual" in drv_status_fields and "sg_result" in drv_status_fields:
                        return tmc_obj
            except Exception:
                continue
        return None
    
    def _start_logging(self):
        """Start the logging timer and open log file"""
        if self.log_timer is not None:
            return  # Already logging
        
        try:
            # Create log directory if needed
            log_dir = os.path.dirname(self.log_file_path)
            if log_dir and not os.path.exists(log_dir):
                os.makedirs(log_dir)
            
            # Open log file
            self.log_file = open(self.log_file_path, 'a')
            self.log_file.write(f"\n# ExtruderTMCLogger started at {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            self.log_file.write("# timestamp,extruder,cs_actual,sg_result\n")
            self.log_file.flush()
            
            # Start timer
            reactor = self.printer.get_reactor()
            self.log_timer = reactor.register_timer(self._log_data, reactor.NOW)
            
            logging.info("ExtruderTMCLogger: Started logging to %s", self.log_file_path)
        except Exception as e:
            logging.error("ExtruderTMCLogger: Failed to start logging: %s", e)
            if self.log_file:
                self.log_file.close()
                self.log_file = None
    
    def _stop_logging(self):
        """Stop the logging timer and close log file"""
        if self.log_timer is not None:
            self.printer.get_reactor().unregister_timer(self.log_timer)
            self.log_timer = None
        
        if self.log_file is not None:
            self.log_file.write(f"# ExtruderTMCLogger stopped at {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            self.log_file.close()
            self.log_file = None
        
        logging.info("ExtruderTMCLogger: Stopped logging")

    def _log_data(self, eventtime):
        """Timer callback to log TMC data"""
        if not self.extruder_tmcs or self.log_file is None:
            return eventtime + self.log_interval

        try:
            with self.lock:
                timestamp = time.time()
                for extruder_name, info in self.extruder_tmcs.items():
                    try:
                        tmc_obj = info['tmc']
                        # Read DRV_STATUS register
                        status = tmc_obj.mcu_tmc.get_register_raw("DRV_STATUS")
                        reg_val = status["data"]
                        fields = tmc_obj.mcu_tmc.get_fields()

                        # Extract cs_actual and sg_result
                        cs_actual = fields.get_field("cs_actual", reg_val)
                        sg_result = fields.get_field("sg_result", reg_val)

                        # Write to log file
                        log_line = f"{timestamp:.6f},{extruder_name},{cs_actual},{sg_result}\n"
                        self.log_file.write(log_line)

                    except Exception as e:
                        logging.debug("ExtruderTMCLogger: Error reading %s: %s", extruder_name, e)

                self.log_file.flush()

        except Exception as e:
            logging.error("ExtruderTMCLogger: Error in log_data: %s", e)

        return eventtime + self.log_interval

    # G-code command handlers
    def cmd_start_logging(self, gcmd):
        """Start logging command"""
        if not self.extruder_tmcs:
            gcmd.respond_info("No extruder TMC drivers found")
            return

        if self.log_timer is not None:
            gcmd.respond_info("Logging already active")
            return

        # Override log file path if specified
        log_path = gcmd.get('FILE', self.log_file_path)
        if log_path != self.log_file_path:
            self.log_file_path = log_path

        self._start_logging()
        gcmd.respond_info(f"Started extruder TMC logging to {self.log_file_path}")

    def cmd_stop_logging(self, gcmd):
        """Stop logging command"""
        if self.log_timer is None:
            gcmd.respond_info("Logging not active")
            return

        self._stop_logging()
        gcmd.respond_info("Stopped extruder TMC logging")

    def cmd_log_status(self, gcmd):
        """Show logging status command"""
        status = "active" if self.log_timer is not None else "inactive"
        gcmd.respond_info(f"Extruder TMC logging: {status}")
        gcmd.respond_info(f"Log file: {self.log_file_path}")
        gcmd.respond_info(f"Log interval: {self.log_interval:.3f}s")
        gcmd.respond_info(f"Found {len(self.extruder_tmcs)} extruder TMC drivers:")
        for name, info in self.extruder_tmcs.items():
            gcmd.respond_info(f"  {name} -> {info['stepper_name']}")

def load_config(config):
    return ExtruderTMCLogger(config)
