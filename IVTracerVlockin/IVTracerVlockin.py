#!/usr/bin/env python
import importlib
import logging
from math import acos, atan2, copysign, cos, sin, sqrt
from multiprocessing import RLock
from threading import Event, Thread
from time import sleep
from typing import Dict, List, Tuple

import InstrumentDriver
import numpy as np
from scipy import constants as cs

logger = logging.getLogger(__name__)
# handler = logging.FileHandler(
#     r"C:\Users\Shabani_Lab\Documents\MagnetDebug\log.txt", mode="w"
# )
# handler.setLevel(logging.DEBUG)
# logger.addHandler(handler)
# logger.critical("Test handler")


class BiasGenerator:
    """Bias current generator for VICurveTracer."""

    def __init__(self, address):
        raise NotImplementedError

    def close(self):
        raise NotImplementedError

    def select_range(self, voltage, load_resistance):
        """"""
        raise NotImplementedError

    def current_value(self):
        """"""
        raise NotImplementedError

    def goto_value(self, value, slope):
        """"""
        raise NotImplementedError

    def prepare_ramps(self, ramps: List[Tuple[float, float, float]]):
        """"""
        raise NotImplementedError

    def start_ramp(self, index):
        """"""
        raise NotImplementedError

    def is_ramping(self):
        """"""
        raise NotImplementedError

    def get_admissible_reset_rate(self, reset_rate, amplitude):
        """"""
        raise NotImplementedError

    def get_sweep_resolution(self) -> Dict[str, float]:
        """ """
        raise NotImplementedError

    def support_continuous_sweeping(self) -> bool:
        """"""
        raise NotImplementedError


class VoltMeter:
    """Voltmeter for VICurveTracer."""

    def __init__(self, address):
        raise NotImplementedError

    def close(self):
        raise NotImplementedError

    def list_ranges(self):
        """"""
        raise NotImplementedError

    def get_range(self):
        """"""
        raise NotImplementedError

    def set_range(self, value):
        """"""
        raise NotImplementedError

    # Continuous or point by point
    def set_acquisition_mode(self, value):
        """"""
        raise NotImplementedError

    def get_averaging_time(self):
        """"""
        raise NotImplementedError

    def set_averaging_time(self, value):
        """"""
        raise NotImplementedError

    def list_acquisition_rates(self):
        """"""
        raise NotImplementedError

    def get_acquisition_rate(self):
        """"""
        raise NotImplementedError

    def set_acquisition_rate(self, value):
        """"""
        raise NotImplementedError

    def prepare_acquistion(self, points):
        """"""
        raise NotImplementedError

    def arm_device(self):
        """"""
        raise NotImplementedError

    def wait_for_data_ready(self):
        """"""
        raise NotImplementedError

    def retrieve_data(self):
        """"""
        raise NotImplementedError

    def read_value(self):
        """"""
        raise NotImplementedError


class LockIn:
    """"""

    def __init__(self, address):
        raise NotImplementedError

    def get_amplitude(self):
        raise NotImplementedError

    def set_amplitude(self, value):
        raise NotImplementedError

    def list_tcs(self):
        """"""
        raise NotImplementedError

    def get_tc(self):
        raise NotImplementedError

    def set_tc(self, value):
        raise NotImplementedError

    def get_frequency(self):
        raise NotImplementedError

    def set_frequency(self, value):
        raise NotImplementedError

    def read_value(self):
        """Read a complex value from the lock-in.

        This method should not wait before taking a measurement.

        The framework ensures a proper wait time.

        """
        raise NotImplementedError


class Driver(InstrumentDriver.InstrumentWorker):
    """This class implements the VICurveTracer driver.

    This assumes that the source will trigger the meter when starting a ramp.

    Using the Yoko GS200 and the Keithley 6500 we can expect the bias current
    to be offset by 1-3% of the max value.

    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._source: BiasGenerator = None
        self._meter1: VoltMeter = None
        self._meter2: VoltMeter = None
        self._li: LockIn = None
        self._Vli: LockIn = None
        self._lock = RLock()
        # In Point by point (with Lock-in) this is used to store the LI trace
        # acquired at the same time as the IV.
        self._bias = None
        self._Vdc = None
        self._realbias = None
        self._didv_trace = None
        self._li_trace = None
        self._Vac = None
        # Padding points help overcome issues with respect of the ramp by the
        # source
        self._padding_points = 0

    def performOpen(self, options={}):
        """Perform the operation of opening the instrument connection."""
        super().performOpen(options)
        with self._lock:
            # Get the acquisition mode
            acq_mode = self.getValue("Acquisition mode")

            # Start the source
            s_model = self.getValue("Source: Model")
            s_add = self.getValue("Source: VISA address")
            cls = importlib.import_module(s_model).Driver
            self._source = cls(s_add)
            if (
                not self._source.support_continuous_sweeping()
                and acq_mode == "Continuous"
            ):
                acq_mode = self.setValue(
                    "Acquisition mode", "Point by point (without Lock-in)"
                )

            # Start the meter
            m1_model = self.getValue("DMM1: Model")
            m1_add = self.getValue("DMM1: VISA address")
            cls = importlib.import_module(m1_model).Driver
            self._meter1 = cls(m1_add)

            m2_model = self.getValue("VoltageDMM: Model")
            m2_add = self.getValue("VoltageDMM: VISA address")
            cls = importlib.import_module(m2_model).Driver
            self._meter2 = cls(m2_add)

            # Start the lock-in
            li_model = self.getValue("Lock-In: Model")
            li_add = self.getValue("Lock-In: VISA address")
            if acq_mode == "Point by point (with Lock-in)":
                if li_model and li_add:
                    cls = importlib.import_module(li_model).Driver
                    self._li = cls(li_add)

            Vli_model = self.getValue("VLock-In: Model")
            Vli_add = self.getValue("VLock-In: VISA address")
            if acq_mode == "Point by point (with Lock-in)":
                if Vli_model and Vli_add:
                    cls = importlib.import_module(li_model).Driver
                    self._Vli = cls(Vli_add)
                else:
                    raise RuntimeError(
                        "No valid information for Lock-in even though the mode "
                        "requires a lock-in. Missing:\n"
                        + ("- Address\n" if not li_add else "")
                        + ("- Model" if not li_model else "")
                    )
            else:
                self._Vli = None



            # Start the voltage lock-in
            # li_model = self.getValue("Lock-In: Model")
            # li_add = self.getValue("Lock-In: VISA address")
            # if acq_mode == "Point by point (with Lock-in)":
            #     if li_model and li_add:
            #         cls = importlib.import_module(li_model).Driver
            #         self._li = cls(li_add)
            #     else:
            #         raise RuntimeError(
            #             "No valid information for Lock-in even though the mode "
            #             "requires a lock-in. Missing:\n"
            #             + ("- Address\n" if not li_add else "")
            #             + ("- Model" if not li_model else "")
            #         )
            # else:
            #     self._li = None

            self._change_meter_acquisition_mode(acq_mode)
            if acq_mode == "Continuous":
                ext = self.getValue("Source: extrema")
                re_rate = self.getValue("Source: reset rate")
                points = self.getValue("Number of points")
                ac_rate = self.readValueFromOther("DMM: acquisition rate")

                # Center the points in the window of acquisition and add padding
                self._prepare_ramp(ext, points, ac_rate, re_rate)

    def performClose(self, bError=False, options={}):
        """Perform the close instrument connection operation."""
        self._source.close()
        self._meter1.close()
        self._meter2.close()
        if self._li:
            self._li.close()
            self._Vli.close()


    def performSetValue(self, quant, value, sweepRate=0.0, options={}):
        """Perform the Set Value instrument operation.

        This function should return the actual value set by the instrument

        """
        q_name = quant.name
        update_ramps = False
        ext = points = ac_rate = re_rate = None

        if q_name in (
            "Source: VISA address",
            "DMM1: VISA address",
            "VoltageDMM: VISA address",
            "Lock-In: VISA address",
            "VLock-In: VISA address",
            "Source: Model",
            "DMM1: Model",
            "VoltageDMM: Model",
            "Lock-In: Model",
            "VLock-In: Model",
            "DC Voltage divider",
            "AC Voltage divider",
            "Trans-impedance amplifier: gain",
            "Voltage amplifier: gain",
            "Inline resistance",

            "Bias offset",
            "DMM: averaging time Lock-in",
        ):
            pass

        elif q_name == "Acquisition mode":
            if value == "Point by point (with Lock-in)":
                li_model = self.getValue("Lock-In: Model")
                li_add = self.getValue("Lock-In: VISA address")
                if li_model and li_add:
                    cls = importlib.import_module(li_model).Driver
                    self._li = cls(li_add)

                Vli_model = self.getValue("VLock-In: Model")
                Vli_add = self.getValue("VLock-In: VISA address")
                if Vli_model and Vli_add:
                    cls = importlib.import_module(Vli_model).Driver
                    self._Vli = cls(Vli_add)


                else:
                    raise RuntimeError(
                        "No valid information for Lock-in even though the mode "
                        "requires a lock-in. Missing:\n"
                        + ("- Address\n" if not li_add else "")
                        + ("- Model" if not li_model else "")
                    )
            # elif value == "Point by point (with 2 Lock-in)":
            #     li_model = self.getValue("Lock-In: Model")
            #     li_add = self.getValue("Lock-In: VISA address")
            #     if li_model and li_add:
            #         cls = importlib.import_module(li_model).Driver
            #         self._li = cls(li_add)
            #     else:
            #         raise RuntimeError(
            #             "No valid information for Lock-in even though the mode "
            #             "requires a lock-in. Missing:\n"
            #             + ("- Address\n" if not li_add else "")
            #             + ("- Model" if not li_model else "")
            #         )
            #     li_model = self.getValue("Lock-In: Model")
            #     li_add = self.getValue("Lock-In: VISA address")
            #     if li_model and li_add:
            #         cls = importlib.import_module(li_model).Driver
            #         self._li = cls(li_add)
            #     else:
            #         raise RuntimeError(
            #             "No valid information for Lock-in even though the mode "
            #             "requires a lock-in. Missing:\n"
            #             + ("- Address\n" if not li_add else "")
            #             + ("- Model" if not li_model else "")
            #         )
            elif self._li:
                self._li.close()
                self._Vli.close()
            if value == "Continuous" and not self._source.support_continuous_sweeping():
                raise ValueError(
                    "The selected source does not support continuous sweeps"
                )
            self._change_meter_acquisition_mode(value)
        elif q_name == "Source: range":
            with self._lock:
                self._source.set_range(value)
        elif q_name == "Source: extrema":
            update_ramps = True
            ext = value
        elif q_name == "Source: reset rate":
            update_ramps = True
            re_rate = value
        elif q_name == "DMM: range":
            with self._lock:
                self._meter1.set_range(value)
                self._meter2.set_range(value)
        elif q_name == "Number of points":
            update_ramps = True
            points = int(value)
        elif q_name == "DMM: acquisition rate":
            update_ramps = True
            ac_rate = value
        elif q_name == "DMM: averaging time":
            with self._lock:
                self._meter1.set_averaging_time(value)
                self._meter2.set_averaging_time(value)
        elif q_name == "Lock-in: frequency":
            with self._lock:
                self._li.set_frequency(value)
                self._Vli.set_frequency(value)
                # XXX deal with LI voltage
        elif q_name == "Lock-in: amplitude":
            with self._lock:
                self._li.set_amplitude(value)
                self._Vli.set_frequency(value)
                # XXX deal with LI voltage
        elif q_name == "Lock-in: time constant":
            with self._lock:
                self._li.set_tc(value)
                self._Vli.set_tc(value)
                self._meter1.set_averaging_time(
                    self.getValue("DMM: averaging time Lock-in")
                )
                self._meter2.set_averaging_time(
                    self.getValue("DMM: averaging time Lock-in")
                )
                # XXX deal with LI voltage
        elif q_name == "Lock-in: settling time":
            with self._lock:
                self._meter1.set_averaging_time(
                    self.getValue("DMM: averaging time Lock-in")
                )
                self._meter2.set_averaging_time(
                    self.getValue("DMM: averaging time Lock-in")
                )

        if self.getValue("Acquisition mode") == "Continuous" and update_ramps:
            ext = ext or self.getValue("Source: extrema")
            re_rate = re_rate or self.getValue("Source: reset rate")
            points = points or self.getValue("Number of points")
            ac_rate = ac_rate or self.getValue("DMM: acquisition rate")
            # Center the points in the window of acquisition and add padding
            self._prepare_ramp(ext, points, ac_rate, re_rate)

        return value

    def performGetValue(self, quant, options={}):
        """Perform the Get Value instrument operation."""
        q_name = quant.name

        if q_name == "IV curve":
            acq_mode = self.getValue("Acquisition mode")
            ext = self.getValue("Source: extrema")
            points = self.getValue("Number of points")
            offset = self.getValue("Bias offset")
            with self._lock:
                if acq_mode == "Continuous":
                    data = self._perform_continuous_acquisition()
                else:
                    data = self._perform_point_by_point_acquisition(
                        "without" not in acq_mode
                    )
                bias = (np.linspace(ext, -ext, int(points)) / self.getValue("DC Voltage divider"))
                self._bias = bias

                return quant.getTraceDict(
                    data,
                    x=bias,
                )

        if q_name == "Vdc":
                return quant.getTraceDict(
                    self._Vdc,
                    x=self._bias,
                )

        if q_name == "Vac":
                return quant.getTraceDict(
                    self._Vac,
                    x=self._bias,
                )

        if q_name == "dIdV vs V curve":
            return quant.getTraceDict(
                self._li_trace, x=self._bias
            )


        if q_name == "dsigma vs V curve":
            return quant.getTraceDict(
                self._didv_trace, x=self._bias
            )



        if q_name == "Real voltagebias":
            return quant.getTraceDict(
                self._realbias, x=self._bias
            )


        # For quantities corresponding to software only parameters simply
        # return the value
        elif q_name in (
            "Acquisition mode",
            "Source: Model",
            "Source: VISA address",
            "DMM1: Model",
            "DMM1: VISA address",
            "VoltageDMM: Model",
            "VoltageDMM: VISA address",
            "Lock-In: Model",
            "Lock-In: VISA address",
            "VLock-In: Model",
            "VLock-In: VISA address",
            # XXX duplicate Li voltage entries
            "Source: extrema",
            "Source: reset rate",
            "Number of points",
            "DC Voltage divider",
            "AC Voltage divider",
            "Trans-impedance amplifier: gain",
            "Voltage amplifier: gain",
            "Inline resistance",

            "Bias offset",
            "DMM: averaging time Lock-in",
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
                return self._meter1.list_ranges()

        elif q_name == "DMM: range":
            with self._lock:
                return self._meter1.get_range()

        elif q_name == "DMM: list acquisition rates":
            with self._lock:
                return self._meter1.list_acquisition_rates()

        elif q_name == "DMM: acquisition rate":
            with self._lock:
                return self._meter1.get_acquisition_rate()

        elif q_name == "DMM: averaging time":
            with self._lock:
                return self._meter1.get_averaging_time()

        elif q_name == "Lock-in: frequency":
            with self._lock:
                return self._li.get_frequency()

        elif q_name == "Lock-in: amplitude":
            with self._lock:
                return self._li.get_amplitude()

        elif q_name == "Lock-in: list time constants":
            with self._lock:
                return self._li.list_tcs()

        elif q_name == "Lock-in: time constant":
            with self._lock:
                value = self._li.get_tc()
                return value

        elif q_name == "Lock-in: settling time":
            with self._lock:
                value = self.getValue("Lock-in: settling time")
            return value
        else:
            raise KeyError("Unknown quantity: %s" % q_name)

    @staticmethod
    def ramp_extrema(sweep_extrema, points, padding_points):
        """Extrema of the ramp for continuous acquisition.

        The value is chosen so that points are centered in each
        acquisition segment.

        """
        half_step = sweep_extrema / (points - 1)
        return sweep_extrema + (2 * padding_points + 1) * half_step

    @staticmethod
    def ramp_speed(sweep_extrema, points, data_rate):
        """Sweep rate given the amplitude, number of points and selected ac rate."""
        # Adjusted by comparing curves on very different ranges so that they
        # agree and coompared to a conventional measurement
        return 2 * sweep_extrema * data_rate / points

    def _prepare_ramp(self, ext, points, data_rate, reset_rate):
        """Prepare a ramp by centering the points and adding padding."""
        # Get the source sweep resolution
        res = self._source.get_sweep_resolution()
        if "time" in res:  # The Yokogawa has a finite time resolution of 100 ms
            possible_pads = np.arange(2, max(10, int(0.05 * points)))
            extremas = self.ramp_extrema(ext, points, possible_pads)
            times = (
                2
                * extremas
                / self.ramp_speed(extremas, points + 2 * possible_pads, data_rate)
            )
            # This selects the padding giving us the ramping time leading to the
            # smallest error
            padding = possible_pads[np.argmin(times % res["time"])]
        else:
            raise ValueError("Unsupported resolution format")
        self._padding_points = padding

        # Center the points in the window of acquisition
        val = self.ramp_extrema(ext, points, padding)

        self._source.select_range(val, self.getValue("Source: load resistance"))

        self._source.prepare_ramps(
            [
                (-val, val, self.ramp_speed(val, points + 2 * padding, data_rate)),
                (
                    val,
                    -val,
                    self._source.get_admissible_reset_rate(reset_rate, 2 * val),
                ),
            ]
        )
        self._meter.prepare_acquisition(points + 2 * padding)

    def _change_meter_acquisition_mode(self, mode):
        """Change the meter configuation based on the acquisition mode."""
        if "Point by point" in mode:
            self._meter1.set_acquisition_mode("point by point")
            self._meter2.set_acquisition_mode("point by point")
            if "without" not in mode:
                self._meter1.set_averaging_time(
                    self.getValue("DMM: averaging time Lock-in")
                )
                self._meter2.set_averaging_time(
                    self.getValue("DMM: averaging time Lock-in")
                )
            else:
                if "without" in mode:
                    self._meter1.set_averaging_time(self.getValue("DMM: averaging time"))
                    self._meter2.set_averaging_time(self.getValue("DMM: averaging time"))
        else:
            ext = self.getValue("Source: extrema")
            re_rate = self.getValue("Source: reset rate")
            points = self.getValue("Number of points")
            ac_rate = self.getValue("DMM: acquisition rate")
            self._meter.set_acquisition_mode("continuous")
            self._meter.set_acquisition_rate(ac_rate)
            self._prepare_ramp(ext, points, ac_rate, re_rate)

    def _perform_continuous_acquisition(self):
        """Perform a continuous acquisition."""
        ext = self.getValue("Source: extrema")
        reset = self.getValue("Source: reset rate")
        points = self.getValue("Number of points")
        # Center the points in the window of acquisition
        init = self.ramp_extrema(ext, points, self._padding_points)

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

        # The DMM is preconfigured for the right number of points, so simply arm
        self._meter.arm_device()

        # Start the ramp.
        self._source.start_ramp(0)
        sleep(0.1)

        # Wait for the data
        self._meter.wait_for_data_ready()
        while self._source.is_ramping():
            sleep(0.01)

        # Retrieve the data
        data = self._meter.retrieve_data()
        # Remove padding
        data = data[self._padding_points : -self._padding_points]

        # Reset the source so that it has time to reset
        self._source.start_ramp(1)

        return data

    def _perform_point_by_point_acquisition(self, with_li: bool):
        """Perform a point by point acquisition."""
        ext = self.getValue("Source: extrema")
        reset = self.getValue("Source: reset rate")
        points = self.getValue("Number of points")
        offset = self.getValue("Bias offset")
        #both_li = with_li and "2" in self.getValue("Acquisition mode")

        set_points = np.linspace(ext + offset * self.getValue("DC Voltage divider"),
                    -ext + offset * self.getValue("DC Voltage divider"), int(points))
        dmm_vals1 = np.empty_like(set_points)
        dmm_vals2 = np.empty_like(set_points)
        Vac = np.empty_like(set_points)
        if with_li:
            li_vals = np.empty_like(set_points, dtype=complex)
            Vli_vals = np.empty_like(set_points, dtype=complex)
            # XXX create storage for LI 2 value
        t = self.getValue("Lock-in: settling time") * self.getValue(
            "Lock-in: time constant"
        )
        source = self._source
        dmm1 = self._meter1
        VoltageDMM = self._meter2
        li = self._li
        Vli = self._Vli
        # XXX get LI

        # Ensure we are using the proper range
        self._source.select_range(
            # 10 resistance to ground of the divider
            ext + abs(offset) * self.getValue("DC Voltage divider"),
            10 * self.getValue("DC Voltage divider")
        )

        # Should only happen on the first scan since we reset the value after
        # setting
        #while self._source.is_ramping():
            #sleep(3)
        # Go to the first point and wait
        source.goto_value(set_points[0], reset)
        sleep(3)
        #while source.is_ramping():
            #sleep(3)

        for i, v in enumerate(set_points):
            source.goto_value(v, reset)
            sleep(t)
            dmm_vals1[i] = dmm1.read_value()
            dmm_vals2[i] = VoltageDMM.read_value()
            if with_li:
                li_vals[i] = li.read_value()
                Vli_vals[i] = Vli.read_value()
        # Go to the first point and wait
        source.goto_value(set_points[0], reset)

        # Convert to current
        dmm_vals1 /= self.getValue("Trans-impedance amplifier: gain")
        dmm_vals2 /= self.getValue("Voltage amplifier: gain")

        if with_li:
            self._Vdc = dmm_vals2
            self._li_trace = li_vals
            # Convert AC measurement to conductance
            sigma = (abs(li_vals)
                / self.getValue("Trans-impedance amplifier: gain")
            )

            Vac = abs(Vli_vals)


            self._Vac = Vac

            self._didv_trace = (
                sigma
                / (2 * cs.e ** 2 / cs.h)
                / Vac
            )
            self._realbias = (set_points / self.getValue("DC Voltage divider")
            - offset
            - dmm_vals1 * self.getValue("Inline resistance")
            )
        # Conversion for both li

        return dmm_vals1


if __name__ == "__main__":
    pass
