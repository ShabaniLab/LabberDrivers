import pyvisa
from typing import List, Tuple

from VICurveTracer import BiasGenerator

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
    """Yokogawa GS200 as bias generator.

    """

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

    def list_ranges(self):
        return "10mV, 100mV, 1V, 10V, 30V"

    def get_range(self):
        return float(self._rsc.query(":SOUR:RANG?"))

    def set_range(self, value):
        if value not in (10e-3, 100e-3, 1, 10, 30):
            raise ValueError("Invalid range specified.")
        resp = self._rsc.query(f":SOUR:RANG {value};:SOUR:RANG?")
        if float(resp) != value:
            raise RuntimeError(
                f"Failed to set range (after setting value is {resp},"
                f"expected {value}"
            )

    def current_value(self):
        """Get the current value of the output.

        """
        return float(self._rsc.query(":SOUR:LEV?"))

    def goto_value(self, value, slope):
        """Go to the specified value immediately.

        """
        rsc = self._rsc
        curr_value = self.current_value()
        # Program cannot be shorter than 0.1
        sweep_time = max(abs(value - curr_value) / slope, 0.1)
        rsc.write(RAMP_TEMPLATE.format(sweep_time=sweep_time, value=value))
        self._maybe_ramping = True

    def prepare_ramps(self, ramps: List[Tuple[float, float, float]]):
        """Prepare ramps by creating a program executing them in series.

        Parameters
        ----------
        ramps : List[Tuple[float, float, float]]
            Each ramp is described by a start point, a stop point and a slope.

        """
        progs = []
        for start, stop, slope in ramps:
            sweep_time = abs(start - stop) / slope
            if sweep_time < 0.1:
                raise ValueError("GS200 sweeps must be at least 100ms long.")
            progs.append(RAMP_TEMPLATE.format(sweep_time=sweep_time, value=stop))
        self._ramps = progs

    def start_ramp(self, index):
        """Start the specified ramp.

        """
        self._rsc.write(self._ramps[index])
        self._maybe_ramping = True

    def is_ramping(self):
        """Check is the program is done executing.

        """
        if self._maybe_ramping:
            # Check for bit 7 EOP (End of program) in the extended event register
            # We are not ramping if the bit is set
            ramping = (int(self._rsc.query(":STAT:EVEN?")) & 128) == 0
            self._maybe_ramping = ramping
            return ramping
        else:
            return False


# Can used for debugging by commenting the import of BiasSource
if __name__ == "__main__":
    y = Driver("GPIB::13::INSTR")
    try:
        print(y.is_ramping())
        y.goto_value(-1, 10)
    finally:
        y.close()
