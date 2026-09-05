# heat_soak.py
import time
import statistics
from collections import deque

class HeatSoak:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')

        # Configuration options
        self.default_temp = config.getfloat('default_temperature', 55.0)
        self.default_tolerance = config.getfloat('default_tolerance', 0.05)
        self.default_step = config.getfloat('default_step', 0.01)
        self.default_threshold = config.getfloat('default_threshold', 0.0015)
        self.safety_z = config.getfloat('safety_z', -1.0)
        self.fast_feed = config.getfloat('fast_feed_rate', 7000.0)  # mm/min
        self.slow_feed = config.getfloat('slow_feed_rate', 120.0)   # mm/min
        self.z_height = config.getfloat('z_height', 0.5)
        self.interval = config.getfloat('interval', 10.0)
        self.default_consecutive = config.getint('default_consecutive', 10)
        self.cancel_heat_soak = False

        # Register the G-code command
        self.gcode.register_command("HEAT_SOAK", self.cmd_HEAT_SOAK,
                                    desc=self.cmd_HEAT_SOAK_help)
        self.gcode.register_command("HEAT_SOAK_CANCEL", self.cmd_HEAT_SOAK_CANCEL,
                                    desc=self.cmd_HEAT_SOAK_CANCEL_help)

    cmd_HEAT_SOAK_help = "Heat soak routine for bed thermal stabilization"
    cmd_HEAT_SOAK_CANCEL_help = "Cancel the ongoing heat soak routine"
    def cmd_HEAT_SOAK(self, gcmd):
        """
        Heat soak routine.
        Usage: HEAT_SOAK TEMPERATURE=55 THRESHOLD=0.0015 CONSECUTIVE=10 INTERVAL=10
        """
        # Get toolhead object when command is called (not during init)
        toolhead = self.printer.lookup_object('toolhead')

        temp = gcmd.get_float("TEMPERATURE", self.default_temp)
        threshold = gcmd.get_float("THRESHOLD", self.default_threshold)
        consecutive = gcmd.get_int("CONSECUTIVE", self.default_consecutive)
        interval = gcmd.get_float("INTERVAL", self.interval)

        self.gcode.respond_info("Starting heat soak: temp=%.1f, threshold=%.4f" %
                                (temp, threshold))
        self.cancel_heat_soak = False

        # 1. Heat bed and wait until target temperature
        self.gcode.run_script_from_command("M140 S%.1f" % temp)
        self.gcode.run_script_from_command("M190 S%.1f" % temp)  # wait until stable

        # 2. Home all axes
        self.gcode.run_script_from_command("G28")

        # 3. Move Z axis to configured probe height
        self.gcode.run_script_from_command("G1 Z%.3f" % self.z_height)

        # 4. Measure Z homing distance every 5 seconds using cartographer probe

        # Set the Z height for measurements
        gcmd._params['Z'] = '%.3f' % self.z_height
        self.gcode.respond_info("Setting probe Z to %.3f" % self.z_height)

        # Initialize variables for Z deviation tracking
        start_time = time.time()
        max_offset = 0.0
        iteration = 0
        z_history = deque(maxlen=self.default_consecutive)
        initial_z = None
        previous_temp = None

        while True:
            iteration += 1
            if self.printer.is_shutdown():
                self.gcode.respond_info("Heat soak cancelled due to printer shutdown")
                break
            # Measure Z homing distance using cartographer probe
            try:
                # Use the cartographer probe to measure the current Z position
                probe = self.printer.lookup_object('probe')
                if probe is None:
                    raise self.gcode.error("Cartographer probe not available")

                # Perform a probe to get the current Z position
                probed_position = probe.run_probe(gcmd)
                current_z = probed_position[2]

                # Get bed temperature
                heater_bed = self.printer.lookup_object('heater_bed')
                bed_temp = heater_bed.get_status(0)['temperature']
                elapsed = time.time() - start_time

                if previous_temp is None:
                    temp_delta = 0.0
                else:
                    temp_delta = abs(bed_temp - previous_temp)
                previous_temp = bed_temp

                if initial_z is None:
                    initial_z = current_z
                    offset = 0.0
                else:
                    offset = abs(current_z - initial_z)
                    max_offset = max(max_offset, offset)

                z_history.append(current_z)
                z_range = max(z_history) - min(z_history) if len(z_history) == self.default_consecutive else float('inf')

                log_this = (len(z_history) == self.default_consecutive and z_range <= threshold) or (iteration % 10 == 0)

                if log_this:
                    self.gcode.respond_info("Iteration %d: Elapsed %.1fs, Bed Temp %.1f°C, Temp Delta %.4f, Bed Z position: %.6f, Offset %.4f, Z range %.6f" % (iteration, elapsed, bed_temp, temp_delta, current_z, offset, z_range))

                # Additional debug logs for z_history
                if iteration % 10 == 0:
                    history_str = ", ".join("%.6f" % z for z in z_history)
                    self.gcode.respond_info("Z history: [%s]" % history_str)
                    self.gcode.respond_info("Z range calc: max=%.6f, min=%.6f, range=%.6f" % (max(z_history) if z_history else 0, min(z_history) if z_history else 0, z_range))

                # Check if Z variation is below threshold over consecutive measurements
                if len(z_history) == self.default_consecutive and z_range <= threshold:
                    break

            except Exception as e:
                if self.printer.is_shutdown():
                    self.gcode.respond_info("Heat soak cancelled due to printer shutdown")
                else:
                    self.gcode.respond_error("Error during heat soak: %s" % str(e))
                break

            if self.cancel_heat_soak:
                self.gcode.respond_info("Heat soak cancelled by user")
                break

            # Wait for the configured interval before next measurement
            time.sleep(interval)

        total_time = time.time() - start_time
        self.gcode.respond_info("Heat soak complete! Total time: %.1f seconds, Max Offset: %.4f" % (total_time, max_offset))

        # Move Z axis to 10mm after heat soak completes
        self.gcode.run_script_from_command("G0 Z10")

    def cmd_HEAT_SOAK_CANCEL(self, gcmd):
        self.cancel_heat_soak = True
        self.gcode.respond_info("Heat soak cancellation requested")

def load_config(config):
    return HeatSoak(config)