# Micro-controller clock synchronization
#
# Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math
import threading

RTT_AGE = .000010 / (60. * 60.)
DECAY = 1. / 30.
TRANSMIT_EXTRA = .001


class ClockSync:
    def __init__(self, reactor, max_concurrent_queries=4):
        self.reactor = reactor
        self.serial = None  # Serial will be provided in connect()
        self.semaphore = threading.Semaphore(max_concurrent_queries)
        self.lock = threading.Lock()
        self.mcu_freq = 1.0
        self.last_clock = 0
        self.clock_est = (0.0, 0.0, 0.0)
        self.min_half_rtt = float("inf")
        self.min_rtt_time = 0.0
        self.time_avg = self.time_variance = 0.0
        self.clock_avg = self.clock_covariance = 0.0
        self.prediction_variance = 0.0
        self.last_prediction_time = 0.0
        self.queries_pending = 0

    def connect(self, serial):
        self.serial = serial
        self.mcu_freq = serial.msgparser.get_constant_float('CLOCK_FREQ')
        # Load initial clock and frequency
        params = serial.send_with_response('get_uptime', 'uptime')
        self.last_clock = (params['high'] << 32) | params['clock']
        self.clock_avg = self.last_clock
        self.time_avg = params['#sent_time']
        self.clock_est = (self.time_avg, self.clock_avg, self.mcu_freq)
        self.prediction_variance = (.001 * self.mcu_freq) ** 2
        # Enable periodic get_clock timer
        for i in range(8):
            self.reactor.pause(self.reactor.monotonic() + 0.050)
            params = self.serial.send_with_response("get_clock", "clock")
            self._handle_clock(params)

    def sync_clock(self):
        """Synchronize the clock using the semaphore to limit concurrency."""
        threads = []
        for _ in range(4):  # Adjust this if needed for multiple queries
            t = threading.Thread(target=self._query_clock)
            threads.append(t)
            t.start()
        for t in threads:
            t.join()

    def _query_clock(self):
        with self.semaphore:
            with self.lock:
                self.queries_pending += 1
            try:
                params = self.serial.send_with_response("get_clock", "clock")
                self._handle_clock(params)
            finally:
                with self.lock:
                    self.queries_pending -= 1

    def _handle_clock(self, params):
        last_clock = self.last_clock
        clock_delta = (params["clock"] - last_clock) & 0xFFFFFFFF
        self.last_clock = clock = last_clock + clock_delta

        sent_time = params["#sent_time"]
        receive_time = params["#receive_time"]
        half_rtt = 0.5 * (receive_time - sent_time)
        aged_rtt = (sent_time - self.min_rtt_time) * RTT_AGE
        if half_rtt < self.min_half_rtt + aged_rtt:
            self.min_half_rtt = half_rtt
            self.min_rtt_time = sent_time

        exp_clock = (
                (sent_time - self.time_avg) * self.clock_est[2] + self.clock_avg
        )
        clock_diff2 = (clock - exp_clock) ** 2
        if clock_diff2 > 25.0 * self.prediction_variance and clock_diff2 > (
                0.000500 * self.mcu_freq
        ) ** 2:
            if clock > exp_clock and sent_time < self.last_prediction_time + 10.0:
                return
            self.prediction_variance = (0.001 * self.mcu_freq) ** 2
        else:
            self.last_prediction_time = sent_time
            self.prediction_variance = (
                    (1.0 - DECAY)
                    * (self.prediction_variance + clock_diff2 * DECAY)
            )

        diff_sent_time = sent_time - self.time_avg
        self.time_avg += DECAY * diff_sent_time
        self.time_variance = (1.0 - DECAY) * (
                self.time_variance + diff_sent_time ** 2 * DECAY
        )
        diff_clock = clock - self.clock_avg
        self.clock_avg += DECAY * diff_clock
        self.clock_covariance = (1.0 - DECAY) * (
                self.clock_covariance + diff_sent_time * diff_clock * DECAY
        )

        new_freq = self.clock_covariance / self.time_variance
        self.serial.set_clock_est(
            new_freq,
            self.time_avg + TRANSMIT_EXTRA,
            int(self.clock_avg - 3.0 * math.sqrt(self.prediction_variance)),
            clock,
        )
        self.clock_est = (
            self.time_avg + self.min_half_rtt,
            self.clock_avg,
            new_freq,
        )

    def print_time_to_clock(self, print_time):
        return int(print_time * self.mcu_freq)

    def clock_to_print_time(self, clock):
        return clock / self.mcu_freq

    # system time conversions
    def get_clock(self, eventtime):
        sample_time, clock, freq = self.clock_est
        return int(clock + (eventtime - sample_time) * freq)

    def estimate_clock_systime(self, reqclock):
        sample_time, clock, freq = self.clock_est
        return float(reqclock - clock) / freq + sample_time

    def estimated_print_time(self, eventtime):
        return self.clock_to_print_time(self.get_clock(eventtime))

    # misc commands
    def clock32_to_clock64(self, clock32):
        last_clock = self.last_clock
        clock_diff = (clock32 - last_clock) & 0xffffffff
        clock_diff -= (clock_diff & 0x80000000) << 1
        return last_clock + clock_diff

    def is_active(self):
        with self.lock:
            return self.queries_pending > 0

    def dump_debug(self):
        sample_time, clock, freq = self.clock_est
        return ("clocksync state: mcu_freq=%d last_clock=%d"
                " clock_est=(%.3f %d %.3f) min_half_rtt=%.6f min_rtt_time=%.3f"
                " time_avg=%.3f(%.3f) clock_avg=%.3f(%.3f)"
                " pred_variance=%.3f" % (
                    self.mcu_freq, self.last_clock, sample_time, clock, freq,
                    self.min_half_rtt, self.min_rtt_time,
                    self.time_avg, self.time_variance,
                    self.clock_avg, self.clock_covariance,
                    self.prediction_variance))

    def stats(self, eventtime):
        sample_time, clock, freq = self.clock_est
        return "freq=%d" % (freq,)

    def calibrate_clock(self, print_time, eventtime):
        return (0., self.mcu_freq)


# Clock syncing code for secondary MCUs (whose clocks are sync'ed to a
# primary MCU)
class SecondarySync(ClockSync):
    def __init__(self, serial, main_sync, max_concurrent_queries=4):
        # Initialize the parent ClockSync class
        super().__init__(serial, max_concurrent_queries)
        self.main_sync = main_sync
        self.clock_adj = (0.0, 1.0)  # Offset and frequency adjustment
        self.last_sync_time = 0.0

    def connect(self, serial):
        # Call the parent connect method
        super().connect(serial)
        self.clock_adj = (0.0, self.mcu_freq)  # Initialize clock adjustment
        self._calibrate_to_main_sync()  # Calibrate this clock with the main sync

    def _calibrate_to_main_sync(self):
        curtime = self.serial.reactor.monotonic()
        main_print_time = self.main_sync.estimated_print_time(curtime)
        local_print_time = self.estimated_print_time(curtime)
        self.clock_adj = (main_print_time - local_print_time, self.mcu_freq)

    def print_time_to_clock(self, print_time):
        adjusted_offset, adjusted_freq = self.clock_adj
        return int((print_time - adjusted_offset) * adjusted_freq)

    def clock_to_print_time(self, clock):
        """Convert this MCU's clock to print time, adjusted for main MCU sync."""
        adjusted_offset, adjusted_freq = self.clock_adj
        return clock / adjusted_freq + adjusted_offset

    def estimated_print_time(self, eventtime):
        """
        Estimate the print time for a given system time.
        """
        adjusted_offset, adjusted_freq = self.clock_adj
        sample_time, clock, freq = self.clock_est
        base_print_time = self.clock_to_print_time(
            clock + (eventtime - sample_time) * freq
        )
        return base_print_time + adjusted_offset

    def dump_debug(self):
        adjusted_offset, adjusted_freq = self.clock_adj
        return "%s clock_adj=(%.3f %.3f)" % (
            ClockSync.dump_debug(self), adjusted_offset, adjusted_freq)

    def stats(self, eventtime):
        adjusted_offset, adjusted_freq = self.clock_adj
        return "%s adj=%d" % (ClockSync.stats(self, eventtime), adjusted_freq)
