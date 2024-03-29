import logging
import pyvisa
from pyvisa import util

import numpy as np

from VICurveTracer import VoltMeter

SRATE = 1000

logger = logging.getLogger(f"VICurveTracer.{__name__}")
logger.setLevel(logging.DEBUG)


class Driver(VoltMeter):
    """Keithley 6500 as VICurveTracer volt-meter."""

    def __init__(self, address):
        self._points = 0
        self._acq_rate = 100
        self._rm = pyvisa.ResourceManager()
        self._address = address
        self.open()

    def open(self):
        self._rsc = rsc = self._rm.open_resource(
            self._address, write_termination="\n", read_termination="\n"
        )

        rsc.clear()
        # Setup signals to allow SRQ on buffer full
        rsc.write(":STAT:OPER:MAP 0, 4918, 4917;")

    def close(self):
        self._rsc.close()

    def get_autorange(self):
        return int(self._rsc.query(":SENS:VOLT:DC:RANG:AUTO?"))

    def set_autorange(self, value):
        self._rsc.write(f":SENS:VOLT:DC:RANG:AUTO {int(value)}")
        return self.get_autorange()

    def get_range(self):
        """Return the currently active range."""
        return float(self._rsc.query(":SENS:VOLT:DC:RANG?"))

    def set_range(self, value):
        """Set the range."""
        self._rsc.write(f":SENS:VOLT:DC:RANG {value}")
        return self.get_range()

    def list_acquisition_rates(self):
        """List the valid acquisition rate.

        The Keithley works in terms of PLC. In the US we get 60Hz and 50Hz in Europe.
        We assume we are in the US.

        """
        return "1kHz, 500Hz, 250Hz, 200Hz, 100Hz, 50Hz, 20Hz, 10Hz"

    def get_acquisition_rate(self):
        """Return the current acquisition rate."""
        return self._acq_rate

    def set_acquisition_rate(self, value):
        """Set the acquistion rate."""
        rate = int(round(value, 1))
        if rate not in (10, 20, 50, 100, 200, 250, 500, 1000):
            raise ValueError("Invalid rate specified.")
        self._acq_rate = rate

    def set_acquisition_mode(self, value):
        """Switch between continuous and point by point acquisition mode."""
        rsc = self._rsc
        if value == "continuous":
            # Use digitized voltage, with a sampling rate of 1 kHz and auto aperture
            rsc.clear()
            rsc.write(f':DIG:FUNC "VOLT";:DIG:VOLT:SRATE {SRATE};:DIG:VOLT:APER AUTO')
        else:
            # Use ASCII for single point transfer
            rsc.write(':FUNC "VOLT:DC";:FORM:DATA ASC;')  # Data format

    def get_nplc(self):
        """Get the number of power-line cycles."""
        return float(self._rsc.query(":VOLT:DC:NPLC?"))

    def set_nplc(self, value):
        """Set the number of power-line cycles."""
        self._rsc.write(f":VOLT:DC:NPLC {value}")
        return value

    def get_filter_enabled(self):
        return int(self._rsc.query(":VOLT:AVER:STAT?"))

    def set_filter_enabled(self, value):
        self._rsc.write(f":VOLT:AVER:STAT {int(value)}")
        return self.get_filter_enabled()

    def get_filter_type(self):
        return self._rsc.query(":VOLT:AVER:TCON?")

    def set_filter_type(self, value):
        self._rsc.write(f":VOLT:AVER:TCON {value}")
        return value

    def get_filter_count(self):
        return int(self._rsc.query(":VOLT:AVER:COUN?"))

    def set_filter_count(self, value):
        value = int(value)
        self._rsc.write(f":VOLT:AVER:COUN {value}")
        return value

    def prepare_acquisition(self, points):
        """Prepare the device to measure a series of points.

        Apply to continuous mode acquisition.

        """
        self._points = points = int(round(points * SRATE / self._acq_rate, 1))
        rsc = self._rsc
        # Ensure the continuous triggering is disabled
        # rsc.write(":TRIG:CONT 0")
        rsc.write(
            f":TRAC:POIN {points};"  # Number of points in the buffer
            ":FORM:DATA SREAL;"  # Data format
            ":TRIG:EXT:IN:EDGE FALL;"  #
            ':TRIG:LOAD "Empty";'  # Empty the trigger model
            ":TRIG:BLOC:BUFF:CLEAR 1;"  # Clear buffer
            ":TRIG:BLOC:WAIT 2, EXT, ENTER;"
            f':TRIG:BLOC:MDIG 3, "defbuffer1", {points};'
        )

    def arm_device(self):
        """Make the device ready for a trigger.

        Apply to continuous mode acquisition.

        """
        rsc = self._rsc

        # Type of event we want to be notified about
        event_type = pyvisa.constants.EventType.service_request
        # Mechanism by which we want to be notified
        event_mech = pyvisa.constants.EventMechanism.queue
        # Enable service request detection
        rsc.enable_event(event_type, event_mech)

        # Clear the status register as otherwise the service request will fail
        rsc.write(":STATUS:CLE;*CLS;:DISP:SCR PROC")  # Minimize display processor use
        # Enable service request on buffer full, init trigger model
        rsc.write(":STAT:OPER:ENAB 1;*SRE 128;:INIT")

    def wait_for_data_ready(self):
        """Retrive the data collected after a trigger is received.

        Apply to continuous mode acquisition.

        """
        rsc = self._rsc

        # Type of event we want to be notified about
        event_type = pyvisa.constants.EventType.service_request
        # Mechanism by which we want to be notified
        event_mech = pyvisa.constants.EventMechanism.queue

        # Wait for service request
        response = rsc.wait_on_event(event_type, 30000)
        assert (response.event_type, response.timed_out) == (event_type, False)

        # Disbale further events detection
        rsc.disable_event(event_type, event_mech)

    def retrieve_data(self):
        """Retrive the data collected after a trigger is received.

        Apply to continuous mode acquisition.

        """
        self._rsc.write(":DISP:SCR HOME")  # Turn display on
        self._rsc.write(f":TRAC:DATA? 1, {self._points}")
        block = self._rsc.read_bytes(self._points * 4 + 3)

        data = util.from_binary_block(
            block, 2, self._points * 4, "f", False, np.ndarray
        )
        n_avg = int(round(SRATE / self._acq_rate, 1))
        cutoff = len(data) - len(data) % n_avg
        return np.average(data[:cutoff].reshape((-1, n_avg)), axis=-1)

    def read_value(self):
        i = 0
        while i < 10:
            try:
                return float(self._rsc.query(":READ?"))
            except pyvisa.errors.VisaIOError:
                self.close()
                self.open()
                i += 1


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
