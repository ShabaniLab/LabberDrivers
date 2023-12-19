import logging
import pyvisa
from typing import List, Tuple
from time import sleep

from VICurveTracer import BiasGenerator

logger = logging.getLogger(f"VICurveTracer.{__name__}")

RAMP_TEMPLATE = (
    "*CLS;"
    ":PROG:REP 0;"
    "SLOP {sweep_time:.1f};INT {sweep_time:.1f};"
    "EDIT:STAR;"
    ":SOUR:LEV {value:.6E};"
    ":PROG:EDIT:END;"
    ":PROG:RUN"
)


class Driver:
    """Yokogawa GS200 as bias generator."""

    def __init__(self, address):
        self._rm = rm = pyvisa.ResourceManager()
        self._rsc = rsc = rm.open_resource(
            address, write_termination="\n", read_termination="\n"
        )
        self._ramps = []
        if rsc.query(":SOUR:FUNC?") != "VOLT":
            raise RuntimeError(f"Yokogawa GS200 ({address}) is not in voltage mode.")
        if rsc.query(":OUTP?") != "1":
            raise RuntimeError(f"Yokogawa GS200 ({address}) output is off.")

        # Guard indicated we started a program and hance checking if it is
        # complete make sense
        self._maybe_ramping = False

        # Direct the signal marking the beginning of a program execution the
        # rear BNC output
        rsc.clear()
        rsc.write(":ROUT:BNCO TRIG")

    def close(self):
        self._rsc.close()

    def select_range(self, value, load_resistance):
        if value < 12e-3:
            r = 10e-3
        elif value < 120e-3:
            r = 100e-3
        elif value < 1.2:
            r = 1
        elif value < 12:
            r = 10
        elif value < 32:
            r = 30
        else:
            raise ValueError(f"No admissible range for {value}")

        self._rsc.write(f":SOUR:RANG {r}")
        resp = self._rsc.query(":SOUR:RANG?", delay=0.2)
        if float(resp) != r:
            raise RuntimeError(
                f"Failed to set range (after setting value is {resp}," f"expected {r}"
            )

    def current_value(self):
        """Get the current value of the output."""
        return float(self._rsc.query(":SOUR:LEV?"))

    def goto_value(self, value, slope, wait=False):
        """Ramp to specified `value` with speed `slope`.

        If wait=True, this function blocks until the ramp finishes.
        """
        rsc = self._rsc

        error = rsc.query(":STAT:ERR?")  # <integer>,<character string>
        err_code, err_msg = error.split(",", 1)
        if int(err_code) != 0:
            logger.error(f"{err_msg} (code={err_code})")
        curr_value = self.current_value()
        # Program cannot be shorter than 0.1
        sweep_time = round(abs(value - curr_value) / slope, 1)
        if sweep_time < 0.1:
            rsc.write(f":SOUR:LEV {value}")
        else:
            rsc.write(RAMP_TEMPLATE.format(sweep_time=sweep_time, value=value))
            self._maybe_ramping = True

        if wait:
            while self.is_ramping():
                sleep(0.01)

    def prepare_ramps(self, ramps: List[Tuple[float, float, float]]):
        """Prepare ramps by creating a program executing them in series.

        Parameters
        ----------
        ramps : List[Tuple[float, float, float]]
            Each ramp is described by a start point, a stop point and a slope.

        """
        progs = []
        for start, stop, slope in ramps:

            # Yoko GS200 Slope resolution is 1 decimal place
            sweep_time = round(abs(start - stop) / slope, 1)
            if sweep_time < 0.1:
                raise ValueError(
                    f"GS200 sweeps must be at least 100ms long. Time is {sweep_time},"
                    f" start: {start}, stop: {stop}, slope: {slope}"
                )
            progs.append(RAMP_TEMPLATE.format(sweep_time=sweep_time, value=stop))
        self._ramps = progs

    def start_ramp(self, index):
        """Start the specified ramp."""
        self._rsc.write(self._ramps[index])
        self._maybe_ramping = True

    def is_ramping(self):
        """Check is the program is done executing."""
        if self._maybe_ramping:
            # Check for bit 7 EOP (End of program) in the extended event register
            # We are not ramping if the bit is set
            ramping = (int(self._rsc.query(":STAT:EVEN?")) & 128) == 0
            self._maybe_ramping = ramping
            return ramping
        else:
            return False

    def get_admissible_reset_rate(self, reset_rate, amplitude):
        """Get an admissible reset rate.

        This avoids issue for too fast reset for the system to handle.

        """
        sweep_time = round(amplitude / reset_rate, 1)
        if sweep_time < 0.1:
            return amplitude / 0.1
        else:
            return reset_rate

    def get_sweep_resolution(self):
        return {"time": 0.1}

    @classmethod
    def support_continuous_sweeping(self) -> bool:
        """"""
        return True


# Can used for debugging by commenting the import of BiasSource
# if __name__ == "__main__":
#     y = Driver("GPIB::13::INSTR")
#     try:
#         print(y.is_ramping())
#         y.goto_value(-1, 10)
#     finally:
#         y.close()
