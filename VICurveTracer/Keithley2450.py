from math import copysign
from threading import Thread
from time import sleep
from typing import List, Tuple

import pyvisa
from VICurveTracer import BiasGenerator


class Driver:
    """Keithley 2450 as bias generator."""

    def __init__(self, address):
        self._rm = rm = pyvisa.ResourceManager()
        self._rsc = rsc = rm.open_resource(
            address, write_termination="\n", read_termination="\n"
        )
        self._ramps = []
        if rsc.query(":SOUR:FUNC?") != "VOLT":
            raise RuntimeError(f"Keithley 2450 ({address}) is not in voltage mode.")
        if rsc.query(":OUTP?") != "1":
            raise RuntimeError(f"Keithley 2450 ({address}) output is off.")
        self._worker_thread = None

    def close(self):
        self._rsc.close()

    def select_range(self, value, load_resistance):
        if value < 21e-3:
            r = 20e-3
        elif value < 210e-3:
            r = 200e-3
        elif value < 2.1:
            r = 2
        elif value < 21:
            r = 20
        elif value < 210:
            r = 200
        else:
            raise ValueError(f"No admissible range for {value}")

        resp = self._rsc.query(f":SOUR:VOLT:RANG {r};:SOUR:VOLT:RANG?")
        if float(resp) != r:
            raise RuntimeError(
                f"Failed to set range (after setting value is {resp}," f"expected {r}"
            )

        curr_limit = 1.1 * r / load_resistance
        resp = self._rsc.query(
            f":SENS:CURR:RANG:AUTO 1;:SOUR:VOLT:ILIMIT {curr_limit};:SOUR:VOLT:ILIMIT?"
        )
        if float(resp) != curr_limit:
            raise RuntimeError(
                f"Failed to set current limit (after setting value is {resp},"
                f"expected {curr_limit}"
            )

    def current_value(self):
        """Get the current value of the output."""
        return float(self._rsc.query(":SOUR:VOLT?"))

    def goto_value(self, value, slope):
        """Go to the specified value immediately."""
        rsc = self._rsc
        curr_value = self.current_value()
        step = slope * 0.05
        step = copysign(step, value - curr_value)
        if abs(value - curr_value) < step:
            rsc.write(f":SOUR:VOLT {value}")
        else:
            self._worker_thread = Thread(target=self._go_to, args=(value, step))
            self._worker_thread.start()
            sleep(0.02)

    def is_ramping(self):
        """Check is the program is done executing."""
        if self._worker_thread is not None and self._worker_thread.is_alive():
            return True
        else:
            return False

    def get_admissible_reset_rate(self, reset_rate, amplitude):
        """Get an admissible reset rate.

        This avoids issue for too fast reset for the system to handle.
        """
        return reset_rate

    @classmethod
    def support_continuous_sweeping(self) -> bool:
        """"""
        return False

    def _go_to(self, value, step):
        rsc = self._rsc
        cval = self.current_value()
        while abs(value - cval) > abs(step):
            rsc.write(f":SOUR:VOLT {cval + step}")
            sleep(0.05)
            cval = self.current_value()
        rsc.write(f":SOUR:VOLT {value}")


# Can used for debugging by commenting the import of BiasSource
# if __name__ == "__main__":
#     y = Driver("GPIB::13::INSTR")
#     try:
#         print(y.is_ramping())
#         y.goto_value(-1, 10)
#     finally:
#         y.close()
