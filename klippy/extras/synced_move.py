# Synchronized movement between any two steppers
#
# Copyright (C) 2024 Augment Agent <augment@augmentcode.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging

class SyncedMove:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.stepper = None
        self.synced_stepper = None
        self.original_trapq = None
        
        # Register commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("SYNC_STEPPER_MOTION", self.cmd_SYNC_STEPPER_MOTION,
                             desc=self.cmd_SYNC_STEPPER_MOTION_help)
        
        logging.info("SyncedMove '%s' initialized", self.name)

    cmd_SYNC_STEPPER_MOTION_help = "Sync one stepper to another's motion queue"
    def cmd_SYNC_STEPPER_MOTION(self, gcmd):
        stepper_name = gcmd.get('STEPPER')
        motion_queue = gcmd.get('MOTION_QUEUE', '')
        
        logging.info("SYNC_STEPPER_MOTION: stepper=%s, motion_queue=%s", 
                    stepper_name, motion_queue)
        
        try:
            if motion_queue:
                self._sync_to_stepper(stepper_name, motion_queue)
            else:
                self._unsync_stepper(stepper_name)
        except Exception as e:
            logging.error("SYNC_STEPPER_MOTION failed: %s", str(e))
            raise gcmd.error("Sync failed: %s" % str(e))

    def _sync_to_stepper(self, stepper_name, motion_queue_name):
        # Find the stepper to sync
        stepper_obj = self.printer.lookup_object(stepper_name, None)
        if stepper_obj is None:
            logging.error("Stepper '%s' not found", stepper_name)
            raise self.printer.command_error("'%s' is not a valid stepper." % stepper_name)
        
        # Find the motion queue source (could be extruder, manual_stepper, etc.)
        motion_queue_obj = self.printer.lookup_object(motion_queue_name, None)
        if motion_queue_obj is None:
            # Try looking for manual stepper with the name
            try:
                motion_queue_obj = self.printer.lookup_object('manual_stepper ' + motion_queue_name, None)
                logging.info("Found manual stepper: %s", motion_queue_name)
            except:
                pass
        
        if motion_queue_obj is None:
            logging.error("Motion queue '%s' not found", motion_queue_name)
            raise self.printer.command_error("'%s' is not a valid motion queue." % motion_queue_name)
        
        # Check if motion queue object has get_trapq method
        if not hasattr(motion_queue_obj, 'get_trapq'):
            logging.error("Object '%s' does not have get_trapq method", motion_queue_name)
            raise self.printer.command_error("'%s' cannot be used as motion queue." % motion_queue_name)
        
        # Get the stepper object (different for extruders vs manual steppers)
        logging.info("Stepper object type: %s", type(stepper_obj))

        if hasattr(stepper_obj, 'extruder_stepper'):
            actual_stepper = stepper_obj.extruder_stepper.stepper  # For extruders
            logging.info("Using extruder_stepper.stepper for %s", stepper_name)
        elif hasattr(stepper_obj, 'stepper'):
            actual_stepper = stepper_obj.stepper  # For other steppers
            logging.info("Using stepper for %s", stepper_name)
        elif hasattr(stepper_obj, 'rail') and hasattr(stepper_obj.rail, 'get_steppers'):
            actual_stepper = stepper_obj.rail.get_steppers()[0]  # For manual steppers
            logging.info("Using manual stepper for %s", stepper_name)
        elif hasattr(stepper_obj, 'get_steppers'):
            # Try direct get_steppers method
            steppers = stepper_obj.get_steppers()
            if steppers:
                actual_stepper = steppers[0]
                logging.info("Using direct stepper access for %s", stepper_name)
            else:
                logging.error("No steppers found in %s", stepper_name)
                raise self.printer.command_error("No steppers found for '%s'." % stepper_name)
        else:
            logging.error("Cannot find stepper object for %s", stepper_name)
            logging.error("Available attributes: %s", [attr for attr in dir(stepper_obj) if not attr.startswith('_')])
            raise self.printer.command_error("Cannot access stepper for '%s'." % stepper_name)
        
        # Store original trapq for restoration
        self.stepper = actual_stepper
        self.synced_stepper = stepper_obj
        self.original_trapq = actual_stepper.get_trapq()
        
        logging.info("Original trapq: %s", self.original_trapq)
        
        # Check rotation distances for compatibility
        stepper_rotation_dist = getattr(stepper_obj, 'rotation_distance', None)
        motion_queue_rotation_dist = getattr(motion_queue_obj, 'rotation_distance', None)

        if stepper_rotation_dist and motion_queue_rotation_dist:
            ratio = stepper_rotation_dist / motion_queue_rotation_dist
            logging.info("Rotation distance ratio: stepper=%.6f, motion_queue=%.6f, ratio=%.6f",
                        stepper_rotation_dist, motion_queue_rotation_dist, ratio)

            if abs(ratio - 1.0) > 0.01:  # More than 1% difference
                logging.warning("Large rotation distance difference detected! This may cause step generation errors.")
                logging.warning("Consider using steppers with similar rotation distances for sync moves.")

        # Get current positions for debugging
        current_stepper_pos = actual_stepper.get_commanded_position()
        logging.info("Current stepper position before sync: %s", current_stepper_pos)

        # Set stepper position and sync to motion queue
        if hasattr(stepper_obj, 'last_position'):
            # For extruders
            sync_position = [stepper_obj.last_position, 0., 0.]
            actual_stepper.set_position(sync_position)
            logging.info("Set extruder position to: %s (last_position: %s)", sync_position, stepper_obj.last_position)
        else:
            # For manual steppers or others
            sync_position = [0., 0., 0.]
            actual_stepper.set_position(sync_position)
            logging.info("Set stepper position to: %s", sync_position)

        # Get motion queue position for debugging
        if hasattr(motion_queue_obj, 'get_commanded_position'):
            motion_queue_pos = motion_queue_obj.get_commanded_position()
            logging.info("Motion queue position: %s", motion_queue_pos)

        # Sync to the motion queue
        target_trapq = motion_queue_obj.get_trapq()
        original_trapq = actual_stepper.get_trapq()
        actual_stepper.set_trapq(target_trapq)

        logging.info("Synced stepper '%s' to motion queue '%s'", stepper_name, motion_queue_name)
        logging.info("Original trapq: %s -> Target trapq: %s", original_trapq, target_trapq)

    def _unsync_stepper(self, stepper_name):
        if self.stepper is None:
            logging.warning("No stepper currently synced")
            return
        
        # Restore original trapq
        self.stepper.set_trapq(self.original_trapq)
        
        logging.info("Unsynced stepper '%s', restored original trapq: %s", 
                    stepper_name, self.original_trapq)
        
        # Clear references
        self.stepper = None
        self.synced_stepper = None
        self.original_trapq = None

def load_config_prefix(config):
    return SyncedMove(config)
