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

logger = logging.getLogger(__name__)
handler = logging.FileHandler(
    r"C:\Users\Shabani_Lab\Documents\MagnetDebug\log.txt", mode="w"
)
handler.setLevel(logging.DEBUG)
logger.addHandler(handler)
logger.critical("Test handler")


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


class Driver(InstrumentDriver.InstrumentWorker):
    """ This class implements the VICurveTracer driver.

    This assumes that the source will trigger the meter when starting a ramp.

    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._source: BiasGenerator = None
        self._meter: VoltMeter = None
        self._meter_1: VoltMeter = None
        self._lock = RLock()
        self._cache = None

    def performOpen(self, options={}):
        """Perform the operation of opening the instrument connection.

        """
        super().performOpen(options)
        with self._lock:
            # Start the source
            s_model = self.getValue("Source: Model")
            s_add = self.getValue("Source: VISA address")
            cls = importlib.import_module(s_model).Driver
            self._source = cls(s_add)

            # Start the first meter
            m_model = self.getValue("DMMs: Model")
            m_add = self.getValue("DMM: VISA address")
            cls = importlib.import_module(m_model).Driver
            self._meter = cls(m_add)

            # Start the second meter
            m_add = self.getValue("DMM_1: VISA address")
            cls = importlib.import_module(m_model).Driver
            self._meter_1 = cls(m_add)

            ext = self.getValue("Source: extrema")
            re_rate = self.getValue("Source: reset rate")

            points = int(self.getValue("DMMs: number of points"))
            ac_rate = self.readValueFromOther("DMMs: acquisition rate")

            # Center the points in the window of acquisition and add padding
            self._prepare_ramp(ext, points, ac_rate, re_rate)

            self._meter.prepare_acquisition(points)
            self._meter_1.prepare_acquisition(points)

    def performClose(self, bError=False, options={}):
        """Perform the close instrument connection operation.

        """
        self._source.close()
        self._meter.close()
        self._meter_1.close()

    def performSetValue(self, quant, value, sweepRate=0.0, options={}):
        """Perform the Set Value instrument operation.

        This function should return the actual value set by the instrument

        """
        q_name = quant.name
        update_ramps = False
        ext = points = ac_rate = re_rate = None

        if q_name in (
            "Source: VISA address",
            "DMM: VISA address",
            "DMM_1: VISA address",
        ):
            pass

        elif q_name in ("Source: Model", "DMM: Model", "DMM_1: Model"):
            pass

        elif q_name == "Source: range":
            with self._lock:
                self._source.set_range(value)
        elif q_name == "Source: extrema":
            update_ramps = True
            ext = value
        elif q_name == "Source: reset rate":
            update_ramps = True
            re_rate = value
        elif q_name == "Source: load resistance":
            pass

        elif q_name == "DMM: range":
            with self._lock:
                self._meter.set_range(value)
        elif q_name == "DMM_1: range":
            with self._lock:
                self._meter_1.set_range(value)
        elif q_name == "DMMs: number of points":
            update_ramps = True
            points = int(value)
            with self._lock:
                self._meter.prepare_acquisition(points)
                self._meter_1.prepare_acquisition(points)
        elif q_name == "DMMs: acquisition rate":
            with self._lock:
                self._meter.set_acquisition_rate(value)
                self._meter_1.set_acquisition_rate(value)
            update_ramps = True
            ac_rate = value
        elif q_name in ("DMMs: effective rate factor", "DMMs: delay"):
            update_ramps = True

        if update_ramps:
            ext = ext or self.getValue("Source: extrema")
            re_rate = re_rate or self.getValue("Source: reset rate")
            points = points or self.getValue("DMMs: number of points")
            ac_rate = ac_rate or self.getValue("DMMs: acquisition rate")
            # Center the points in the window of acquisition and add padding
            self._prepare_ramp(ext, points, ac_rate, re_rate)

        return value

    def performGetValue(self, quant, options={}):
        """Perform the Get Value instrument operation.

        """
        q_name = quant.name

        if q_name == "VI curve":
            with self._lock:
                ext = self.getValue("Source: extrema")
                reset = self.getValue("Source: reset rate")
                points = self.getValue("DMMs: number of points")

                # Repeat if necessary to mitigate random timeouts
                i = 0
                while i < 20:
                    # Center the points in the window of acquisition
                    init = self.ramp_extrema(ext, points, self.padding_points)

                    # Should only happen on the first scan since we reset the value after
                    # setting
                    while self._source.is_ramping():
                        sleep(0.01)
                    curr = self._source.current_value()
                    logger.critical(f"{curr}")
                    if curr != -init:
                        self._source.goto_value(-init, reset)
                        while self._source.is_ramping():
                            sleep(0.01)

                    # The DMMs is preconfigured for the right number of points, so simply arm
                    self._meter.arm_device()
                    self._meter_1.arm_device()

                    # Start the ramp.
                    self._source.start_ramp(0)
                    sleep(0.1)

                    # Wait for the data
                    self._meter.wait_for_data_ready()
                    self._meter_1.wait_for_data_ready()
                    while self._source.is_ramping():
                        sleep(0.01)

                    # Retrieve the data
                    try:
                        data = self._meter.retrieve_data()
                        data_1 = self._meter_1.retrieve_data()
                        break
                    except Exception:
                        i += 1

                # Reset the source so that it has time to reset
                self._source.start_ramp(1)

                self._cache = quant.getTraceDict(
                    data_1, x=np.linspace(-ext, ext, points),
                )

                return quant.getTraceDict(data, x=np.linspace(-ext, ext, points))

        elif q_name == "VI curve - 2":
            if self._cache is not None:
                data = self._cache
                self._cache = None
                return data
            else:
                raise RuntimeError("Get VI curve first.")

        # For quantities corresponding to software only parameters simply
        # return the value
        elif q_name in (
            "Source: Model",
            "Source: VISA address",
            "DMMs: Model",
            "DMM: VISA address",
            "DMM_1: VISA address",
            "Source: extrema",
            "Source: reset rate",
            "Source: load resistance",
            "DMMs: number of points",
            "DMMs: effective rate factor",
            "DMMs: delay",
        ):
            return quant.getValue()

        elif q_name == "Source: list ranges":
            with self._lock:
                return self._source.list_ranges()

        elif q_name == "Source: range":
            with self._lock:
                return self._source.get_range()

        elif q_name == "DMM: list ranges":
            with self._lock:
                return self._meter.list_ranges()

        elif q_name == "DMM: range":
            with self._lock:
                return self._meter.get_range()

        elif q_name == "DMM_1: list ranges":
            with self._lock:
                return self._meter_1.list_ranges()

        elif q_name == "DMM_1: range":
            with self._lock:
                return self._meter_1.get_range()

        elif q_name == "DMMs: list acquisition rates":
            with self._lock:
                return self._meter.list_acquisition_rates()

        elif q_name == "DMMs: acquisition rate":
            with self._lock:
                return self._meter.get_acquisition_rate()

        else:
            raise KeyError("Unknown quantity: %s" % q_name)

    @staticmethod
    def ramp_extrema(sweep_extrema, points, padding_points):
        """Extrema of the ramp.
        
        The value is chosen so that points are centered in each
        acquisition segment.

        """
        half_step = sweep_extrema / (points - 1)
        return sweep_extrema + (2 * padding_points + 1) * half_step

    @staticmethod
    def ramp_speed(sweep_extrema, points, data_rate, data_rate_factor):
        """Sweep rate given the amplitude, number of points and selected ac rate.

        """
        # Adjusted by comparing curves on very different ranges so that they
        # agree and coompared to a conventional measurement
        # Take into account the slowness of the DMM
        return 2 * sweep_extrema * data_rate * data_rate_factor / points

    @property
    def padding_points(self):
        return int(
            self.getValue("DMMs: acquisition rate")
            * self.getValue("DMMs: effective rate factor")
            * self.getValue("DMMs: delay")
        )

    def _prepare_ramp(self, ext, points, data_rate, reset_rate):
        """Prepare a ramp by centering teh points and adding padding.
        
        """
        # Center the points in the window of acquisition
        pad = self.padding_points
        val = self.ramp_extrema(ext, points, pad)
        self._source.prepare_ramps(
            [
                (
                    -val,
                    val,
                    self.ramp_speed(
                        val,
                        points + 2 * pad,
                        data_rate,
                        self.getValue("DMMs: effective rate factor"),
                    ),
                ),
                (val, -val, reset_rate),
            ]
        )


if __name__ == "__main__":
    pass
