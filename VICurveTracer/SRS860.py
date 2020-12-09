import pyvisa

import numpy as np

from VICurveTracer import LockIn


class Driver(LockIn):
    """"""

    def __init__(self, address):
        self._rm = rm = pyvisa.ResourceManager()
        self._rsc = rsc = rm.open_resource(
            address, write_termination="\n", read_termination="\n"
        )

    def close(self):
        self._rsc.close()

    def get_amplitude(self):
        return float(self._rsc.query("SLVL?"))

    def set_amplitude(self, value):
        self._rsc.write(f"SLVL {value}")

    def list_tcs(self):
        """"""
        return [
            1e-6,
            3e-6,
            10e-6,
            30e-6,
            100e-6,
            300e-6,
            1e-3,
            3e-3,
            10e-3,
            30e-3,
            100e-3,
            300e-3,
            1,
            3,
            10,
            30,
            100,
            300,
            1e3,
            3e3,
            10e3,
            30e3,
        ]

    def get_tc(self):
        return self.list_tcs()[int(self._rsc.query("OFLT?"))]

    def set_tc(self, value):
        if value not in self.list_tcs():
            raise ValueError(
                f"Admissible Tc for SR860 are: {self.list_tcs}, got {value}"
            )
        return self.list_tcs().index(value)

    def get_frequency(self):
        return float(self._rsc.query("FREQ?"))

    def set_frequency(self, value):
        self._rsc.write(f"FREQ {value}")

    def read_value(self):
        """Read a complex value from the lock-in.

        This method should not wait before taking a measurement.

        The framework ensures a proper wait time.

        """
        # get complex value in one instrument reading
        resp = self._rsc.query("SNAP? 0,1").strip()
        vals = resp.split(",")
        # Sometimes, we receive the value twice
        # 0.12e-3,4.56e-70.12e-3,4.56e-7 instead of 0.12e-3,4.56e-7
        if len(vals) > 2:
            vals = resp[: len(resp) // 2].split(",")

        # Also, sometimes we receive an additional "-" at the end of a value
        # 0.12e-3,4.56e-7- instead of 0.12e-3,4.56e-7
        if vals[1][-1] == "-":
            vals[1] = vals[1][:-1]

        # return complex values
        return complex(float(vals[0].strip()), float(vals[1].strip()))
