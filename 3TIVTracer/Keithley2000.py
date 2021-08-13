import pyvisa
from pyvisa import util

import numpy as np

from IVtracer2 import VoltMeter


class Driver(VoltMeter):
    """Keithley 2000 as VICurveTracer volt-meter."""

    def __init__(self, address):
        self._points = 0
        self._rm = rm = pyvisa.ResourceManager()
        self._rsc = rsc = rm.open_resource(
            address, write_termination="\n", read_termination="\n"
        )
        # Disable auto-ranging
        rsc.clear()
        rsc.write(":VOLT:DC:RANG:AUTO 0")
        # Ensure the continuous triggering is disabled
        rsc.write(":INIT:CONT 0")

    def close(self):
        self._rsc.close()

    def list_ranges(self):
        """List valid ranges for the Keithley 2000."""
        return "100mV, 1V, 10V, 100V, 1000V"

    def get_range(self):
        """Return teh currently active range."""
        return float(self._rsc.query(":VOLT:DC:RANG?"))

    def set_range(self, value):
        """Set the range."""
        if value not in (100e-3, 1, 10, 100, 1000):
            raise ValueError("Invalid range specified.")
        resp = self._rsc.query(f":VOLT:DC:RANG {value};:VOLT:DC:RANG?")
        if float(resp) != value:
            raise RuntimeError(
                f"Failed to set range (after setting value is {resp},"
                f"expected {value}"
            )

    def list_acquisition_rates(self):
        """List the valid acquisition rate.

        The Keithley works in terms of PLC. In the US we get 60Hz and 50Hz in Europe.
        We assume we are in the US.

        """
        return "6kHz, 600Hz, 60Hz, 6Hz"

    def get_acquisition_rate(self):
        """Return the current acquisition rate."""
        return float(self._rsc.query(":VOLT:DC:NPLC?")) * 60

    def set_acquisition_rate(self, value):
        """Set the acquistion rate."""
        if value not in (6, 60, 600, 6000):
            raise ValueError("Invalid range specified.")
        resp = self._rsc.query(f":VOLT:DC:NPLC {value/60:.2f};:VOLT:DC:NPLC?")
        if float(resp) * 60 != value:
            raise RuntimeError(
                f"Failed to set range (after setting value is {float(resp)*60},"
                f"expected {value}"
            )

    def set_acquisition_mode(self, value):
        """Switch between continuous and point by point acquisition mode."""
        rsc = self._rsc
        if value == "continuous":
            # Use digitized voltage, with a sampling rate of 1 kHz and auto aperture
            rsc.clear()
        else:
            # Use ASCII for single point transfer
            rsc.write(':FUNC "VOLT:DC";:FORM:DATA ASC;:INIT:CONT 0')  # Data format

    def get_averaging_time(self):
        """"""
        rsc = self._rsc
        nplc = int(rsc.query(":VOLT:DC:NPLC?"))
        average = int(rsc.query(":VOLT:AVER:STAT?"))
        count = int(rsc.query(":VOLT:AVER:COUN?"))
        return round((count if average else 1) * nplc / 60, 3)

    def set_averaging_time(self, value):
        """"""
        rsc = self._rsc
        # Use repeating average
        rsc.write(":VOLT:AVER:TCON REP")
        nplc = value / (1 / 60)
        err_10 = nplc % 10
        err_1 = nplc % 1
        err_01 = ((10 * nplc) % 1) / 10
        # Try to prefer larger nplc if possible (those factor are pure guesses)
        index = np.argmin([err_10, err_1 * 1.5, err_01 * 3])
        if index == 0:
            avg = int(nplc // 10)
            nplc = 10
        elif index == 1:
            avg = int(nplc // 1)
            nplc = 1
        else:
            avg = int((10 * nplc) // 1)
            nplc = 0.1

        rsc.write(f":VOLT:DC:NPLC {nplc}")
        if avg > 1:
            rsc.write(f":VOLT:AVER:STAT 1;:VOLT:AVER:COUN {avg}")
        else:
            rsc.write(f":VOLT:AVER:STAT 0;:VOLT:AVER:COUN 1")

        # Ensure the continuous triggering is disabled
        rsc.write(":INIT:CONT 0")

        return nplc * 1 / 60 * avg

    def prepare_acquisition(self, points):
        """Prepare the device to measure a series of points."""
        if points > 1024:
            raise ValueError("Keithley 2000 is limited to 1024 points.")
        self._points = points = int(points)
        rsc = self._rsc
        # Ensure the continuous triggering is disabled
        rsc.write(":INIT:CONT 0")
        rsc.write(
            f":DATA:POIN {points:d};"  # Number of points in the buffer
            f":SAMP:COUN {points:d};"  # Number of points to acquire on a trigger
            ":FORM:DATA DREAL;"  # Data format
            ":TRIG:SOUR EXT;"  # Trigger source
            ":TRIG:COUN 1;"  # Number of trigger to expect
            ":TRIG:DEL 0;"  # No trigger delay
            ":DATA:FEED SENS1;"  # Origin of the data
            ":VOLT:AVER:STAT 0"  # Disable filtering
            ":DISP:ENAB 0"  # Turn display off
        )

    def arm_device(self):
        """Make the device ready for a trigger."""
        rsc = self._rsc
        # Clear the status register as otherwise the service request will fail
        rsc.write(":STATUS:PRESET;*CLS")
        # Clear the buffer, enable service request on buffer full, enable buffer
        # storage.
        rsc.write(":TRAC:CLE;:STAT:MEAS:ENAB 512;*SRE 1;:DATA:FEED:CONT NEXT;:INIT")

    def wait_for_data_ready(self):
        """Retrive the data collected after a trigger is received."""
        self._rsc.wait_for_srq(timeout=60000)

    def retrieve_data(self):
        """Retrive the data collected after a trigger is received."""
        self._rsc.write(":DISP:ENAB 1")  # Turn display on
        self._rsc.write(":DATA:DATA?")
        block = self._rsc.read_bytes(self._points * 8 + 3)

        return util.from_binary_block(
            block, 2, self._points * 8, "d", False, np.ndarray
        )

    def read_value(self):
        return float(self._rsc.query(":READ?"))


# Can used for debugging by commenting the import of BiasSource
# if __name__ == "__main__":
#     k = Driver("GPIB::16::INSTR")
#     try:
#         k.prepare_acquisition(201)
#         k.arm_device()
#         print(k._rsc.query(":SAMP:COUN?"))
#         k.wait_for_data_ready()
#         # from time import sleep

#         # sleep(10)
#         print(k.retrieve_data())
#     finally:
#         k.close()
