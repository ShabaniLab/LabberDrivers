import pyvisa
from pyvisa import util

import numpy as np

from VICurveTracer import VoltMeter

SRATE = 1000

class Driver(VoltMeter):
    """Keithley 6500 as VICurveTracer volt-meter.

    """

    def __init__(self, address):
        self._points = 0
        self._rm = rm = pyvisa.ResourceManager()
        self._rsc = rsc = rm.open_resource(
            address, write_termination="\n", read_termination="\n"
        )
        self._acq_rate = 100
        # Use digitized voltage, with a sampling rate of 1 kHz and auto aperture
        rsc.clear()
        rsc.write(f':DIG:FUNC "VOLT";:DIG:VOLT:SRATE {SRATE};DIG:VOLT:APER AUTO')

    def close(self):
        self._rsc.close()

    def list_ranges(self):
        """List valid ranges for the Keithley 2000.

        """
        return "100mV, 1V, 10V, 100V, 1000V"

    def get_range(self):
        """Return teh currently active range.

        """
        return float(self._rsc.query(":DIG:VOLT:DC:RANG?"))

    def set_range(self, value):
        """Set the range.

        """
        if value not in (100e-3, 1, 10, 100, 1000):
            raise ValueError("Invalid range specified.")
        resp = self._rsc.query(f":DIG:VOLT:DC:RANG {value};:VOLT:DC:RANG?")
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
        return "1kHz, 500Hz, 250Hz, 200Hz, 100Hz, 50Hz, 20Hz, 10Hz"

    def get_acquisition_rate(self):
        """Return the current acquisition rate.

        """
        return self._acq_rate

    def set_acquisition_rate(self, value):
        """Set the acquistion rate.

        """
        rate = int(round(value))
        if rate not in (10, 20, 50, 100, 200, 250, 500, 1000):
            raise ValueError("Invalid rate specified.")
        self._acq_rate = rate

    def prepare_acquisition(self, points):
        """Prepare the device to measure a series of points.

        """
        self._points = points = int(points)*SRATE/self._acq_rate
        rsc = self._rsc
        # Ensure the continuous triggering is disabled
        rsc.write(":TRIG:CONT 0")
        rsc.write(
            f":TRAC:POIN {points:d};"  # Number of points in the buffer
            ":FORM:DATA SREAL;"  # Data format
            ":TRIG:EXT:IN:EDGE FALL;" #
            ':TRIG:LOAD "Empty";'  # Empty the trigger model
            ":TRIG:BLOC:BUFF:CLEAR 1;"  # Clear buffer
            ":TRIG:BLOC:WAIT 2, EXT, ENTER;"
            f':TRIG:BLOC:MDIG 3, "defbuffer1", {points}'
            ":DISP:SCR PROC"  # Minimize display processor use
        )

    def arm_device(self):
        """Make the device ready for a trigger.

        """
        rsc = self._rsc
        # Clear the status register as otherwise the service request will fail
        rsc.write(":STATUS:CLE;*CLS")
        # Enable service request on buffer full, init trigger model
        rsc.write(":STAT:OPER:MAP 0, 4918, 4917;:STAT:OPER:ENAB 1;*SRE 128;:INIT")

    def wait_for_data_ready(self):
        """Retrive the data collected after a trigger is received.

        """
        self._rsc.wait_for_srq(timeout=60000)

    def retrieve_data(self):
        """Retrive the data collected after a trigger is received.

        """
        self._rsc.write(":DISP:SRC HOME")  # Turn display on
        self._rsc.write(f":TRAC:DATA? 1, {self._points}")
        block = self._rsc.read_bytes(self._points * 4 + 3)

        data = util.from_binary_block(
            block, 2, self._points * 4, "f", False, np.ndarray
        )
        n_avg = SRATE/self._acq_rate
        return np.average(data.reshape((-1, n_avg)), axis=-1)


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