import pyvisa
import numpy as np

from VICurveTracer import VoltMeter


class Driver(VoltMeter):
    """Keithley 2000 as VICurveTracer volt-meter.

    """

    def __init__(self, address):
        self._points = 0
        self._rm = rm = pyvisa.ResourceManager()
        self._rsc = rsc = rm.open_resource(
            address, write_termination="\n", read_termination="\n"
        )
        # Disable auto-ranging
        rsc.write(":VOLT:DC:RANG:AUTO 0")

    def close(self):
        self._rsc.close()

    def list_ranges(self):
        """List valid ranges for the Keithley 2000.

        """
        return "100mV, 1V, 10V, 100V, 1000V"

    def get_range(self):
        """Return teh currently active range.

        """
        return float(self._rsc.query(":VOLT:DC:RANG?"))

    def set_range(self, value):
        """Set the range.

        """
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
        """Return the current acquisition rate.

        """
        return float(self._rsc.query(":VOLT:DC:NPLC?")) * 60

    def set_acquisition_rate(self, value):
        """Set the acquistion rate.

        """
        if value not in (6, 60, 600, 6000):
            raise ValueError("Invalid range specified.")
        resp = self._rsc.query(f":VOLT:DC:NPLC? {value/60:.2f};:VOLT:DC:NPLC?")
        if float(resp) * 60 != value:
            raise RuntimeError(
                f"Failed to set range (after setting value is {float(resp)*60},"
                f"expected {value}"
            )

    def prepare_acquistion(self, points):
        """Prepare the device to measure a series of points.

        """
        if points > 1024:
            raise ValueError("Keithley 2000 is limited to 1024 points.")
        self._points = points
        rsc = self._rsc
        # Ensure the continuous triggering is disabled
        rsc.write(":INIT:CONT 0")
        # Set the number of points (in the buffer and to acquire for a trigger)
        # the data format, the trigger source, the points origin
        rsc.write(
            f":DATA:POIN {points:d};:SAMP:COUN {points:d};:DATA:FORM DREAL;"
            ":TRIG:SOUR EXT;:TRIG:COUN 1;:DATA:FEED SENS1"
        )

    def arm_device(self):
        """Make the device ready for a trigger.

        """
        rsc = self._rsc
        # Clear the status register as otherwise the service request will fail
        rsc.query(":STAT:MEAS?")
        # Clear the buffer, enable service request on buffer full, enable buffer
        # storage.
        self.write(":TRAC:CLE;:STAT:MEAS:ENAB 512;*SRE 1;:DATA:FEED:CONT NEXT")

    def wait_for_data_ready(self):
        """Retrive the data collected after a trigger is received.

        """
        self._rsc.wait_for_srq(timeout=1200000)

    def retrieve_data(self):
        """Retrive the data collected after a trigger is received.

        """
        return self._rsc.query_binary_values(
            ":DATA:DATA?", "d", container=np.ndarray, data_points=self._points,
        )
