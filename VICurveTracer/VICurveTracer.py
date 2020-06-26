#!/usr/bin/env python
import importlib
import logging
from math import acos, atan2, copysign, cos, sin, sqrt
from multiprocessing import RLock
from threading import Event, Thread
from time import sleep, time
from typing import List, Tuple

import numpy as np

import InstrumentDriver
from pyvisa import ResourceManager


class BiasGenerator:
    """Bias current generator for VICurveTracer.

    """

    def __init__(self, address):
        pass

    def close(self):
        pass

    def list_ranges(self):
        """
        """
        pass

    def get_range(self):
        """
        """
        pass

    def set_range(self, value):
        """
        """
        pass

    def current_value(self):
        """
        """
        pass

    def goto_value(self, value, slope):
        """
        """
        pass

    def prepare_ramps(self, ramps: List[Tuple[float, float, float]]):
        """
        """
        pass

    def start_ramp(self, index):
        """
        """
        pass

    def is_ramping(self):
        """
        """
        pass


class VoltMeter:
    """Voltmeter for VICurveTracer.

    """

    def __init__(self, address):
        pass

    def close():
        pass

    def list_ranges(self):
        """
        """
        pass

    def get_range(self):
        """
        """
        pass

    def set_range(self, value):
        """
        """
        pass

    def list_acquisition_rates(self):
        """
        """
        pass

    def get_acquisition_rate(self):
        """
        """
        pass

    def set_acquisition_rate(self, value):
        """
        """
        pass

    def prepare_acquistion(self, points):
        """
        """
        pass

    def arm_device(self):
        """
        """
        pass

    def wait_for_data_ready(self):
        """
        """
        pass

    def retrieve_data(self):
        """
        """
        pass


class Driver(VISA_Driver):
    """ This class implements the VICurveTracer driver.

    This assumes that the source will trigger the meter when starting a ramp.

    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._source: BiasGenerator = None
        self._meter: VoltMeter = None
        self._lock = RLock()
        self._source_extrema = 0
        self._source_reset_rate = 1
        self._source_load_resistance = 1e6
        self._dmm_points = 601

    def performOpen(self, options={}):
        """Perform the operation of opening the instrument connection.

        """
        s_model = self.getValue("Source: model")
        s_add = self.getValue("Source: VISA address")
        cls = importlib.import_module(s_model).Driver
        self._source = cls(s_add)
        ext = self.getValue("Source: extrema")
        re_rate = self.getValue("Source: reset rate")
        points = self.getValue("DMM: number of points")
        ac_rate = self.getValue("DMM: acquisition rate")
        self._source.prepare_ramps(
            [(-value, value, 2 * value * points / ac_rate,), (value, -value, re_rate),]
        )

        m_model = self.getValue("DMM: model")
        m_add = self.getValue("DMM: VISA address")
        cls = importlib.import_module(m_model).Driver
        self._meter = cls(m_add)
        self._meter.prepare_acquistion(self.getValue("DMM: number of points"))

    def performClose(self, bError=False, options={}):
        """Perform the close instrument connection operation.

        """
        self._source.close()
        self._meter.close()

    def performSetValue(self, quant, value, sweepRate=0.0, options={}):
        """Perform the Set Value instrument operation.

        This function should return the actual value set by the instrument

        """
        q_name = quant.name
        update_ramps = False
        ext, points, ac_rate, re_rate = None

        if q_name in ("Source: VISA address", "DMM: VISA address",):
            pass

        elif q_name in ("Source: Model", "DMM: Model",):
            pass

        elif q_name == "Source: range":
            self._source.set_range(value)
        elif q_name == "Source: extrema":
            self._source_extrema = value
            update_ramps = True
            ext = value
        elif q_name == "Source: reset rate":
            self._source_reset_rate = value
            update_ramps = True
            re_rate = value
        elif q_name == "Source: load resistance":
            self._source_load_resistance = value
        elif q_name == "DMM: range":
            self._meter.set_range(value)
        elif q_name == "DMM: number of points":
            self._dmm_points = value
            update_ramps = True
            points = value
            self._meter.prepare_acquistion(points)
        elif q_name == "DMM: acquisition rate":
            self._meter.set_acquisition_rate(value)
            update_ramps = True
            ac_rate = value

        if update_ramps:
            ext = ext or self.getValue("Source: extrema")
            re_rate = re_rate or self.getValue("Source: reset rate")
            points = points or self.getValue("DMM: number of points")
            ac_rate = ac_rate or self.getValue("DMM: acquisition rate")
            # Center the points in the window of acquisition
            val = ext + ext / (points - 1)
            self._source.prepare_ramps(
                [(-val, val, 2 * val * points / ac_rate,), (val, -val, re_rate),]
            )

        return value

    def performGetValue(self, quant, options={}):
        """Perform the Get Value instrument operation.

        """
        q_name = quant.name

        if qname == "VI curve":
            ext = self._source_extrema
            reset = self._source_reset_rate
            ac_rate = self.getValue("DMM: acquisition rate")
            points = self._dmm_points
            # Center the points in the window of acquisition
            init = ext + ext / (points - 1)

            # Should only happen on the first scan since we reset the value after
            # setting
            while self._source.is_ramping():
                sleep(0.01)
            if self._source.current_value() != -init:
                self._source.goto_value(-init, reset)
                while self._source.is_ramping():
                    sleep(0.01)

            # The DMM is preconfigured for the right number of points, so simply arm
            self._meter.arm_device()

            # Start the ramp.
            self._source.start_ramp(0)

            # Wait for the data
            self._meter.wait_for_data_ready()

            # Reset the source so that it has time to reset
            self._source.start_ramp(1)

            # Retrive the data
            data = self._meter.retrieve_data()

            return quant.getTraceDict(data, x=np.linspace(-ext, ext, self._dmm_points))

        # For quantities corresponding to software only parameters simply
        # return the value
        if q_name in (
            "Source: extrema",
            "Source: reset rate",
            "Source: load resistance",
            "DMM: number of points",
        ):
            return quant.getValue()

        elif q_name == "Source: list ranges":
            return self._source.list_ranges()

        elif q_name == "Source: range":
            return self._source.get_range()

        elif q_name == "DMM: list ranges":
            return self._meter.list_ranges()

        elif q_name == "DMM: range":
            return self._meter.get_range()

        elif q_name == "DMM: list acquisition rates":
            return self._meter.list_acquisition_rates()

        elif q_name == "DMM: acquisition rate":
            return self._meter.get_acquisition_rate()

        else:
            raise KeyError("Unknown quantity: %s" % q_name)


if __name__ == "__main__":
    pass
