import math
import threading

RTT_AGE = 0.000010 / (60.0 * 60.0)
DECAY = 1.0 / 30.0
TRANSMIT_EXTRA = 0.001


class ClockSync:
    def __init__(self, serial, max_concurrent_queries=4):
        self.serial = serial
        self.semaphore = threading.Semaphore(max_concurrent_queries)
        self.mcu_freq = 1.0
        self.last_clock = 0
        self.clock_est = (0.0, 0.0, 0.0)
        self.min_half_rtt = float("inf")
        self.min_rtt_time = 0.0
        self.time_avg = self.time_variance = 0.0
        self.clock_avg = self.clock_covariance = 0.0
        self.prediction_variance = 0.0
        self.last_prediction_time = 0.0

    def connect(self):
        self.mcu_freq = self.serial.get_constant_float("CLOCK_FREQ")
        params = self.serial.send_with_response("get_uptime", "uptime")
        self.last_clock = (params["high"] << 32) | params["clock"]
        self.clock_avg = self.last_clock
        self.time_avg = params["#sent_time"]
        self.clock_est = (self.time_avg, self.clock_avg, self.mcu_freq)
        self.prediction_variance = (0.001 * self.mcu_freq) ** 2

        # Perform initial clock synchronization
        for _ in range(8):
            params = self.serial.send_with_response("get_clock", "clock")
            self._handle_clock(params)

    def sync_clock(self):
        with self.semaphore:
            params = self.serial.send_with_response("get_clock", "clock")
            self._handle_clock(params)

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
                self.time_variance + diff_sent_time**2 * DECAY
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

    def get_clock(self, eventtime):
        sample_time, clock, freq = self.clock_est
        return int(clock + (eventtime - sample_time) * freq)

    def estimate_clock_systime(self, reqclock):
        sample_time, clock, freq = self.clock_est
        return float(reqclock - clock) / freq + sample_time

    def dump_debug(self):
        sample_time, clock, freq = self.clock_est
        return (
            f"clocksync state: mcu_freq={self.mcu_freq} last_clock={self.last_clock} "
            f"clock_est=({sample_time:.3f} {clock} {freq:.3f}) min_half_rtt={self.min_half_rtt:.6f} "
            f"min_rtt_time={self.min_rtt_time:.3f} time_avg={self.time_avg:.3f} "
            f"time_variance={self.time_variance:.3f} clock_avg={self.clock_avg:.3f} "
            f"clock_covariance={self.clock_covariance:.3f} pred_variance={self.prediction_variance:.3f}"
        )


class SecondarySync(ClockSync):
    def __init__(self, serial, main_sync, max_concurrent_queries=4):
        super().__init__(serial, max_concurrent_queries)
        self.main_sync = main_sync
        self.clock_adj = (0.0, 1.0)
        self.last_sync_time = 0.0

    def connect(self):
        super().connect()
        self.clock_adj = (0.0, self.mcu_freq)
        self._calibrate_to_main_sync()

    def _calibrate_to_main_sync(self):
        curtime = self.serial.reactor.monotonic()
        main_print_time = self.main_sync.estimated_print_time(curtime)
        local_print_time = self.estimated_print_time(curtime)
        self.clock_adj = (main_print_time - local_print_time, self.mcu_freq)

    def print_time_to_clock(self, print_time):
        adjusted_offset, adjusted_freq = self.clock_adj
        return int((print_time - adjusted_offset) * adjusted_freq)

    def clock_to_print_time(self, clock):
        adjusted_offset, adjusted_freq = self.clock_adj
        return clock / adjusted_freq + adjusted_offset

    def dump_debug(self):
        adjusted_offset, adjusted_freq = self.clock_adj
        return (
            f"{super().dump_debug()} clock_adj=({adjusted_offset:.3f} {adjusted_freq:.3f})"
        )

    def stats(self, eventtime):
        adjusted_offset, adjusted_freq = self.clock_adj
        return f"{super().stats(eventtime)} adj_freq={adjusted_freq}"
