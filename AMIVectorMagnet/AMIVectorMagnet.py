#!/usr/bin/env python
from math import sin, cos, sqrt, atan2, acos
from threading import Thread, Event
from multiprocessing import RLock
from time import time, sleep
from typing import Optional, Union, cast

import numpy as np
from scipy_rotation import Rotation
from pyvisa import ResourceManager, resources

from InstrumentDriver import InstrumentWorker


class BasePowerSupply:
    """Base class for power supplies use in the vector magnet system.

    Parameters
    ----------
    rm : visa.ResourceManager
        Resource manager to use to connect to the instrument.
    address : str
        Visa address of the instrument.
    conversion_factor : float
        Conversion factor between T and whatever unit the power supply is
        using.

    """

    require_conversion = False

    def __init__(
        self, rm: ResourceManager, address: str, conversion_factor: Optional[float]
    ) -> None:
        self._driver = cast(resources.MessageBasedResource, rm.open_resource(address))
        if self.require_conversion and conversion_factor is None:
            raise ValueError(f"{type(self)} requires a conversion factor.")
        self.conversion_factor = conversion_factor

    def close(self) -> None:
        self._driver.close()

    def read_value(self) -> float:
        """Read the output"""
        raise NotImplementedError

    def start_sweep(
        self,
        target: Union[float, np.ndarray],
        rate: Union[float, np.ndarray],
        times: Union[float, np.ndarray] = None,
    ) -> None:
        """Start sweeping the field.

        Parameters
        ----------
        target : float or np.ndarray
            Target value expressed in T.
        rate : float or np.ndarray
            Sweeping rate expressed in T/min.
        time : np.ndarray
            Times at which to change the sweeping rate.

        """
        raise NotImplementedError

    def stop_sweep(self) -> None:
        """Stop the currently running sweep if any."""
        raise NotImplementedError

    def check_if_sweeping(self) -> bool:
        """Check if the magnet is currently sweeping."""
        raise NotImplementedError


class AMIPowerSupply(BasePowerSupply):
    """Single axis driver for AMI430 power supply."""

    def __init__(self, rm, address: str, conversion_factor: None = None):
        super().__init__(rm, address, conversion_factor)
        self._lock = RLock()
        self._address = address
        self._rm = rm
        with self._lock:
            self._driver.timeout = 2000
            self._driver.write_termination = "\n"
            self._driver.read_termination = "\r\n"
            # Set the field unit in T
            sleep(1)
            self._driver.read()
            self._driver.read()
            self._driver.write("CONF:FIELD:UNITS 1")
            self._driver.write("CONF:RAMP:RATE:UNITS 1")

        self._stop_sweeping = Event()
        self._curr_val = 0
        self.sweep_resolution = 0.0001

    def read_value(self) -> float:
        """Read the field currently applied by the power supply."""
        return float(self._driver.query("FIELD:MAGnet?"))

    def set_target_field(self, field: float) -> None:
        """Set the sweeping rate in T/min."""
        self._driver.write(f"CONF:FIELD:TARGET {field}")

    def set_rate(self, rate: float) -> None:
        """Set the sweeping rate in T/min.

        This commad is only used for simple ramps and as a consequence we use
        the largest field admissible for the full sphere as limit.

        """
        self._driver.write(f"CONF:RAMP:RATE:FIELD 1,{rate},3")

    def start_sweep(
        self,
        target: Union[float, np.ndarray],
        rate: Union[float, np.ndarray],
        times: Optional[np.ndarray] = None,
    ) -> None:
        """Start sweeping the field.

        Parameters
        ----------
        target : float
            Target value expressed in T.
        rate : float or np.ndarray
            Sweeping rate expressed in T/min.
        time : np.ndarray
            Times at which to change the sweeping rate.

        """
        self._stop_sweeping.clear()
        if times is None:
            if rate is not None:
                self.set_rate(rate)
            self.set_target_field(target)
            self._driver.write("RAMP")

        else:
            raise NotImplementedError()
        if times is not None:
            target_mask = np.zeros_like(target)
            if isinstance(target, float):
                target = target * np.ones_like(times)
                target_mask[0] = 1
            else:
                start = target[0]
                stop = target[-1]
                imin = min(np.argmin(target), np.argmax(target))
                imax = max(np.argmax(target), np.argmin(target))
                # Simple ramp we are better off specifying only the final target
                if imin == 0 and imax == len(target) - 1:
                    target[0] = stop
                    target_mask[0] = 1
                # We go through one max
                elif imin == 0 and imax != len(target) - 1:
                    target_mask[0] = 1
                    target_mask[imax + 1] = 1
                    target[0] = target[imax]
                    target[imax + 1] = stop
                # We go through one min
                elif imin != 0 and imax == len(target) - 1:
                    target_mask[0] = 1
                    target_mask[imin + 1] = 1
                    target[0] = target[imin]
                    target[imin + 1] = target[-1]
                # Assume we never have more than on extra min and max
                else:
                    target_mask[0] = 1
                    target_mask[imax + 1] = 1
                    target_mask[imin + 1] = 1
                    target[0] = target[imin]
                    target[imin + 1] = target[imax]
                    target[imax + 1] = stop
            self._sweeping_thread = Thread(
                target=self._adjust_output, args=(times, target_mask, target, rate)
            )
            self._sweeping_thread.start()

    def stop_sweep(self) -> None:
        """Stop the currently running sweep if any."""
        self._driver.write("PAUSE")
        while not self._driver.query("STATE?") == "3":
            sleep(0.1)

    def check_if_sweeping(self) -> bool:
        """Check if the magnet is currently sweeping."""
        if self._driver.query("STATE?") == "1":
            return True
        else:
            return False

    def _adjust_output(self, times, targets_mask, targets, rates):
        """Adjust continuously the sweeping rate based on a time array."""
        start = time()
        # XXX Could be optimized to make better use of the rates
        for t, m, f, r in zip(times, targets_mask, targets, rates):
            while time() - start < t:
                sleep(0.001)
            if m:
                self.set_target_field(f)
            self.set_rate(r)
            self._driver.write("RAMP")
            if self._stop_sweeping.is_set():
                return


class Keithley2400(BasePowerSupply):
    """Driver to use a Keithley 2400 as a magnet power supply"""

    conversion_factor: float

    def __init__(self, rm: ResourceManager, address: str, conversion_factor: float):
        super().__init__(rm, address, conversion_factor)
        self._lock = RLock()
        self._address = address
        self._rm = rm
        with self._lock:
            self._driver.timeout = 2
            self._driver.write_termination = "\n"
            self._driver.read_termination = "\n"
            if self._driver.query(":OUTP:STAT?") == "0":
                msg = "The output of {} is off, please turn it on."
                raise RuntimeError(msg.format(address))

        self._sweeping_thread: Optional[Thread] = None
        self._stop_sweeping = Event()
        self._curr_val = 0

    def read_value(self) -> float:
        """Read the current output current and convert."""
        if self._sweeping_thread and self._sweeping_thread.is_alive():
            return self._curr_val / self.conversion_factor

        with self._lock:
            try:
                return float(self._driver.query(":SOUR:CURR?")) / self.conversion_factor
            except Exception:
                self._driver.close()
                self._driver = cast(
                    resources.MessageBasedResource,
                    self._rm.open_resource(self._address),
                )
                self._driver.timeout = 2
                self._driver.write_termination = "\n"
                self._driver.read_termination = "\n"
                return self.read_value()

    def start_sweep(
        self,
        target: Union[float, np.ndarray],
        rate: Union[float, np.ndarray],
        times: Optional[np.ndarray] = None,
    ):
        """Start a sweep managed by the computer for simplicity.

        Parameters
        ----------
        target : float
            Target value in T.
        rate : float | np.ndarray
            Rate at which to update the output.
        times : np.ndarray | None
            Times at which to change the value (those are expected to be
            regularly spaced).

        """
        current = self.read_value()
        # Convert the rate in T/s
        rate /= 60.0 / 0.995  # taking into account a weird drift
        interval = 0.1  # Time interval between values update

        if times is None:
            # we update the value of the output every 100 ms
            if rate == 0.0:
                return
            step_number = int(round(np.abs((target - current) / (rate * interval)))) + 1
            values = np.linspace(current, target, step_number)
            times = np.linspace(0, interval * (step_number - 1), step_number)

        else:
            assert isinstance(target, np.ndarray)

            if len(times) == 1:
                times = np.array(times[0], times[0])
                values = np.array(target[0], target[0])
            else:
                # Compute the points in time at which to update the value of the
                # output
                intervals = abs(times[1] - times[0])
                val_times = [
                    np.linspace(
                        t, t + intervals - interval, int(round(intervals / interval))
                    )
                    for t in times
                ]

                # Compute the value of the output to set at each time.
                values = []
                last_val = current
                for time_slice, t in zip(val_times, target):  # type: ignore
                    values.append(np.linspace(last_val, t, len(time_slice) + 1)[:-1])
                    last_val = t

                # Add the missing last point to times and values
                val_times.append(np.array([val_times[-1][-1] + interval]))
                values.append(np.array([target[-1]]))  # type: ignore
                times = np.concatenate(val_times)
                values = np.concatenate(values)

        # convert the values to the proper unit
        values *= self.conversion_factor
        self._stop_sweeping.clear()
        self._sweeping_thread = Thread(target=self._update_output, args=(times, values))
        self._sweeping_thread.start()

    def stop_sweep(self):
        """Stop a currently running sweep."""
        if self._sweeping_thread:
            self._stop_sweeping.set()
            self._sweeping_thread.join()

    def check_if_sweeping(self):
        """Check if a sweep is currently underway."""
        return self._sweeping_thread.is_alive() if self._sweeping_thread else False

    def _update_output(self, times, values):
        """Update the output of the Keithley

        Parameters
        ----------
        times : np.ndarray
            Time at which to update the output value (start at zero)
        values : np.ndarray
            Values of the output to set at each time (start at the current
            value)

        """
        stop_ev = self._stop_sweeping
        driv = self._driver
        cmd = ":SOUR:CURR {:.5f}"
        start = time()
        for t, val in zip(times[1:], values[1:]):
            while time() - start < t:
                sleep(0.001)
            self._curr_val = val
            with self._lock:
                driv.write(cmd.format(val))
            if stop_ev.is_set():
                break


MODELS = {"AMI Model 430": AMIPowerSupply, "Keithley 2400": Keithley2400}


def xyz_axis_to_angles(x, y, z):
    """Convert an xyz representation of a vector to two angles.

    The first angle is the inclination in the spherical coordinate system, the
    second is the azimuthal angle counted from x.

    """
    theta = np.rad2deg(atan2(sqrt(x ** 2 + y ** 2), z))
    phi = np.rad2deg(atan2(y, x))
    return theta, phi


class Converter:
    """Object handling a basis change.

    The new basis is described by the direction of the new z axis and
    the angle between the x axis of the old basis and the x axis to use.


    """

    def __init__(self, axis, phi_offset, z_offset):

        axis_format, axis = axis
        if axis_format == "xyz":
            axis = xyz_axis_to_angles(*axis)
        # Rotation to apply to go from the original coordinates to the new
        # coordinates
        # The rotations describe how to pass from the original frame to the
        # new frame as a consequence to transform a vector in the original
        # frame to the new frame we need to apply the inverse rotations in the
        # reverse order
        coordinate_change = [
            Rotation.from_euler("z", axis[1], degrees=True),
            Rotation.from_euler("y", axis[0], degrees=True),
            Rotation.from_euler("z", phi_offset - axis[1], degrees=True),
        ]
        self._forward_change = (
            coordinate_change[2].inv()
            * coordinate_change[1].inv()
            * coordinate_change[0].inv()
        )
        self._backward_change = (
            coordinate_change[0] * coordinate_change[1] * coordinate_change[2]
        )
        self._z_offset = z_offset

    def to_new_basis(self, vec, no_offset=False):
        """Convert a vector expressed in the old basis."""
        new = self._forward_change.apply(vec)
        if not no_offset:
            if len(new.shape) == 1:
                new[2] -= self._z_offset
            else:
                new[:, 2] -= self._z_offset
        return new

    def from_new_basis(self, vec, no_offset=False):
        """Convert a vector expressed in the new basis."""
        if not no_offset:
            if not hasattr(vec, "shape") or len(vec.shape) == 1:
                vec[2] += self._z_offset
            else:
                vec[:, 2] += self._z_offset
        return self._backward_change.apply(vec)

    def convert_to_xyz(self, x, y, z):
        return self.from_new_basis(np.array([x, y, z]))

    def convert_from_xyz(self, x, y, z):
        return self.to_new_basis(np.array((x, y, z)))


class CylindricalConverter(Converter):
    """Object handling a basis change.

    In the new basis the coordinates are expressed in cylindrical coordinates.

    """

    def convert_to_xyz(self, r, phi, z):
        phi = np.deg2rad(phi)
        return self.from_new_basis(np.array([r * cos(phi), r * sin(phi), z]))

    def convert_from_xyz(self, x, y, z):
        x, y, z = self.to_new_basis(np.array((x, y, z)))
        return np.array((sqrt(x ** 2 + y ** 2), np.rad2deg(atan2(y, x)), z))

    def convert_rate_to_xyz_rates(self, axis, rate, state=None, target=None):
        """Convert a rate expressed in the new basis to the old basis.

        Parameters
        ----------
        axis : {'r', 'phi', 'z'}
            Axis being ramped in the new basis.
        rate : float
            Rate at which to ramp the ramped axis expressed in T/min or deg/min
        state : 3-tuple, optional
            Current state of the magnet, only required for phi ramp.
        target : float, optional
            Target value of phi, only required for phi ramp.

        """
        if axis == "z":
            rates = dict.fromkeys(("r", "phi", "z"), 0)
            rates[axis] = rate
            return self.convert_to_xyz(**rates)

        if state is None:
            msg = (
                "In cylindrical, the state must be specified for r and phi"
                " (state: {})"
            )
            raise ValueError(msg.format(state))

        if axis == "r":
            phi = np.deg2rad(state[1])
            x_rate = rate * np.cos(phi)
            y_rate = rate * np.sin(phi)
            return self.from_new_basis(np.array([x_rate, y_rate, 0]), no_offset=True)

        if state is None or target is None:
            msg = (
                "For phi ramp the state and target value must be "
                "specified (state: {}, target: {})"
            )
            raise ValueError(msg.format(state, target))

        # Convert the rate to rad/min
        rate = rate * np.pi / 180

        # Compute the intermediate angles spaced by one degree
        phis = np.deg2rad(
            np.linspace(state[1], target, int(round(abs(target - state[1]))) + 1)
        )
        # If there is less than 1deg between the starting point and the
        # target we end end with an array in which the target is missing
        # so add it.
        if len(phis) == 1:
            phis = np.append(phis, target)

        # We keep the target to set at each time (value to reach by the next
        # time)
        x_vals = state[0] * np.cos(phis)[1:]
        y_vals = state[0] * np.sin(phis)[1:]
        z_vals = np.zeros_like(phis)[1:]

        # We keep the rate to set at the beginning of each time interval
        x_rate = -rate * state[0] * np.sin(phis)[:-1]
        y_rate = rate * state[0] * np.cos(phis)[:-1]
        z_rate = np.zeros_like(phis)[:-1]

        # We need the times in second
        return (
            (phis[:-1] - phis[0]) / rate * 60,
            self.from_new_basis(np.array([x_vals, y_vals, z_vals]).T).T,
            self.from_new_basis(np.array([x_rate, y_rate, z_rate]).T, no_offset=True).T,
        )


class SphericalConverter(Converter):
    """Object handling a basis change.

    In the new basis the coordinates are expressed in spherical coordinates.

    """

    def convert_to_xyz(self, r, theta, phi):
        theta = np.deg2rad(theta)
        phi = np.deg2rad(phi)
        x, y, z = r * cos(phi) * sin(theta), r * sin(phi) * sin(theta), r * cos(theta)
        return self.from_new_basis(np.array((x, y, z)))

    def convert_from_xyz(self, x, y, z):
        x, y, z = self.to_new_basis((x, y, z))
        r = sqrt(x ** 2 + y ** 2 + z ** 2)
        if not r:
            return 0, 0, 0
        theta = np.rad2deg(acos(z / r))
        phi = np.rad2deg(atan2(y, x))
        return r, theta, phi

    def convert_rate_to_xyz_rates(self, axis, rate, state=None, target=None):
        """Convert a rate expressed in the new basis to the old basis.

        Parameters
        ----------
        axis : {'r', 'theta', 'phi'}
            Axis being ramped in the new basis.
        rate : float
            Rate at which to ramp the ramped axis expressed in T/min or deg/min
        state : 3-tuple, optional
            Current state of the magnet, only required for theta/phi ramp.
        target : float, optional
            Target value of phi, only required for theta/phi ramp.

        """
        if state is None:
            msg = "In sphericaL, the state must be specified (state: {})"
            raise ValueError(msg.format(state))

        if axis == "r":
            x_rate = rate * np.sin(state[1]) * cos(state[2])
            y_rate = rate * np.sin(state[1]) * sin(state[2])
            z_rate = rate * np.cos(state[1])
            return self.from_new_basis(
                np.array([x_rate, y_rate, z_rate]), no_offset=True
            )

        if state is None or target is None:
            msg = (
                "For theta/phi ramp the state and target value must be "
                "specified (state: {}, target: {})"
            )
            raise ValueError(msg.format(state, target))

        # Convert the rate to rad/min
        rate = np.deg2rad(rate)

        if axis == "theta":
            # Compute the intermediate angles spaced by one degree
            angles = np.linspace(
                state[1], target, int(round(abs(target - state[1]))) + 1
            )
            # If there is less than 1deg between the starting point and the
            # target we end end with an array in which the target is missing
            # so add it.
            if len(angles) == 1:
                angles = np.append(angles, target)
            angles = np.deg2rad(angles)
            phi = np.deg2rad(state[2])

            # We keep the target to set at each time (value to reach by the
            # next  time)
            x_vals = (state[0] * np.sin(angles) * cos(phi))[1:]
            y_vals = (state[0] * np.sin(angles) * sin(phi))[1:]
            z_vals = (state[0] * np.cos(angles))[1:]

            # We keep the rate to set at the beginning of each time interval
            x_rate = (rate * state[0] * np.cos(angles) * cos(phi))[:-1]
            y_rate = (rate * state[0] * np.cos(angles) * sin(phi))[:-1]
            z_rate = (-rate * state[0] * np.sin(angles))[:-1]

        elif axis == "phi":
            # Compute the intermediate angles spaced by one degree
            angles = np.linspace(
                state[2], target, int(round(abs(target - state[2]))) + 1
            )
            # If there is less than 1deg between the starting point and the
            # target we end end with an array in which the target is missing
            # so add it.
            if len(angles) == 1:
                angles = np.append(angles, target)
            angles = np.deg2rad(angles)
            theta = np.deg2rad(state[1])

            # We keep the target to set at each time (value to reach by the
            # next time)
            x_vals = (state[0] * sin(theta) * np.cos(angles))[1:]
            y_vals = (state[0] * sin(theta) * np.sin(angles))[1:]
            z_vals = (state[0] * cos(theta) * np.ones_like(angles))[1:]

            # We keep the rate to set at the beginning of each time interval
            x_rate = (-rate * state[0] * sin(theta) * np.sin(angles))[:-1]
            y_rate = (rate * state[0] * sin(theta) * np.cos(angles))[:-1]
            z_rate = np.zeros_like(angles)[:-1]

        # We need the times in second
        return (
            (angles[:-1] - angles[0]) / rate * 60,
            self.from_new_basis(np.array([x_vals, y_vals, z_vals]).T).T,
            self.from_new_basis(np.array([x_rate, y_rate, z_rate]).T, no_offset=True).T,
        )


class Driver(InstrumentWorker):
    """ This class implements the vector magnet driver"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._power_supplies = {"x": None, "y": None, "z": None}
        self._visa_rm = None
        self._lock = RLock()
        self._converter = None

    def performOpen(self, options={}):
        """Perform the operation of opening the instrument connection."""
        super().performOpen(options)
        for axis in ("X", "Y", "Z"):
            add = self.getValue("Power supply {} axis: VISA address".format(axis))
            model = self.getValue("Power supply {} axis: Model".format(axis))
            if not add or not model:
                continue  # The user will specify the address later
            self._start_power_supply_driver(axis, add, model)

        ref_mode = self.getValue("Reference specification mode")
        if ref_mode == "XYZ":
            values = (
                self.getValue("Direction x"),
                self.getValue("Direction y"),
                self.getValue("Direction z"),
            )
        elif ref_mode == "Spherical":
            values = (self.getValue("Direction theta"), self.getValue("Direction phi"))
        else:
            values = (
                self.getValue("Direction XZangle"),
                self.getValue("Direction YZangle"),
            )
        self._update_reference_frame(ref_mode, values)

    def performClose(self, bError=False, options={}):
        """Perform the close instrument connection operation."""
        for axis in ("x", "y", "z"):
            supply = self._power_supplies[axis]
            supply.close()
            qname = "Power supply {} axis: Driver running"
            self.setValue(qname.format(axis.upper()), False)
        super().performClose()

    def performSetValue(self, quant, value, sweepRate=0.0, options={}):
        """Perform the Set Value instrument operation.

        This function should return the actual value set by the instrument

        """
        q_name = quant.name
        seen = True

        if q_name in (
            "Power supply X axis: VISA address",
            "Power supply Y axis: VISA address",
            "Power supply Z axis: VISA address",
        ):
            axis = q_name[13]
            model = self.getValue("Power supply %s axis: Model" % axis)
            if value and model:
                self._start_power_supply_driver(axis, value, model)

        elif q_name in (
            "Power supply X axis: Model",
            "Power supply Y axis: Model",
            "Power supply Z axis: Model",
        ):
            axis = q_name[13]
            add = self.getValue("Power supply %s axis: VISA address" % axis)
            if value and add:
                self._start_power_supply_driver(axis, add, value)

        elif q_name in (
            "Power supply X axis: Conversion factor (T->native)",
            "Power supply Y axis: Conversion factor (T->native)",
            "Power supply Z axis: Conversion factor (T->native)",
        ):
            axis = q_name[13]
            psu = self._power_supplies[axis.lower()]
            if psu is not None:
                psu.conversion_factor = value

        elif q_name in (
            "Max field",
            "Max rate: X",
            "Max rate: Y",
            "Max rate: Z",
            "Field X rate",
            "Field Y rate",
            "Field Z rate",
            "Field magnitude rate",
            "Theta rate",
            "Phi rate",
            "Ramping mode",
        ):
            pass  # Nothing to do for pure software values

        elif q_name == "Reference specification mode":
            # Nothing we will keep xyz and theta phi in sync so that
            # representations agree
            pass

        elif q_name in ("Direction x", "Direction y", "Direction z"):
            values = {k: self.getValue("Direction %s" % k) for k in ("x", "y", "z")}
            values[q_name[-1]] = value
            self._update_reference_frame("XYZ", values)

        elif q_name in ("Direction theta", "Direction phi"):
            if "theta" in q_name:
                theta = value
                phi = self.getValue("Direction phi")
            else:
                theta = self.getValue("Direction theta")
                phi = value
            self._update_reference_frame("Spherical", (theta, phi))

        elif q_name in ("Direction XZangle", "Direction YZangle"):
            if "XZangle" in q_name:
                xz = value
                yz = self.getValue("Direction YZangle")
            else:
                xz = self.getValue("Direction XZangle")
                yz = value
            self._update_reference_frame("Plane", (xz, yz))

        elif q_name == "Phi offset":
            mode = self.getValue("Specification mode")
            theta = self.getValue("Direction theta")
            phi = self.getValue("Direction phi")
            z_offset = self.getValue("Bz offset")
            self._create_converter(mode, theta, phi, value, z_offset)
            self._update_fields()

        elif q_name == "Bz offset":
            current = quant.value
            z_coil = self.getValue("Z Coil field")
            mode = self.getValue("Specification mode")
            theta = self.getValue("Direction theta")
            phi = self.getValue("Direction phi")
            phi_offset = self.getValue("Phi offset")
            self._create_converter(mode, theta, phi, phi_offset, value)
            self._update_fields()
            if sweepRate != 0.0:
                self._power_supplies["z"].start_sweep(
                    z_coil - current + value, sweepRate
                )

        elif q_name == "Specification mode":
            theta = self.getValue("Direction theta")
            phi = self.getValue("Direction phi")
            phi_offset = self.getValue("Phi offset")
            z_offset = self.getValue("Bz offset")
            self._create_converter(value, theta, phi, phi_offset, z_offset)
            self._update_fields()

        else:
            seen = False

        if seen:
            return value

        mode = self.getValue("Specification mode")
        max_rates = self._get_max_rates()
        rate = sweepRate
        if mode == "XYZ":
            if q_name not in ("Field X", "Field Y", "Field Z"):
                raise KeyError()

            values = {k.lower(): self.getValue("Field %s" % k) for k in ("X", "Y", "Z")}
            values[q_name[-1].lower()] = value
            targets = self._converter.from_new_basis(
                [values[k] for k in ("x", "y", "z")]
            )
            self.log("{}".format(targets), level=1000)
            if any(t > self.getValue("Max field") for t in targets):
                raise ValueError(
                    "The requested field is too large. Coil "
                    "fields would be: %s" % targets
                )
            rates = [0, 0, 0]
            rates[["X", "Y", "Z"].index(q_name[-1])] = rate
            rates = self._converter.from_new_basis(rates, no_offset=True)
            self._validate_rates(rates, max_rates)

            self._ramp_fields(targets, rates)

        elif mode == "Cylindrical":
            if q_name not in ("Field magnitude", "Phi", "Field Z"):
                raise KeyError()

            key = {"Field magnitude": "r", "Phi": "phi", "Field Z": "z"}[q_name]

            state = (
                self.getValue("Field magnitude"),
                self.getValue("Phi"),
                self.getValue("Field Z"),
            )

            if key == "phi":
                times, targets, rates = self._converter.convert_rate_to_xyz_rates(
                    key, rate, state, value
                )
                self._validate_rates(rates, max_rates)
                self._validate_targets(targets)

                self._ramp_fields(targets, rates)
            else:
                values = {k: v for k, v in zip(("r", "phi", "z"), state)}
                values[key] = value
                targets = self._converter.convert_to_xyz(
                    *[values[k] for k in ("r", "phi", "z")]
                )
                self._validate_targets(targets)

                rates = self._converter.convert_rate_to_xyz_rates(key, rate, state)
                self._validate_rates(rates, max_rates)

                self._ramp_fields(targets, rates)

        else:
            if q_name not in ("Field magnitude", "Theta", "Phi"):
                raise KeyError()

            key = {"Field magnitude": "r", "Theta": "theta", "Phi": "phi"}[q_name]

            state = (
                self.getValue("Field magnitude"),
                self.getValue("Theta"),
                self.getValue("Phi"),
            )

            if key in ("theta", "phi"):
                times, targets, rates = self._converter.convert_rate_to_xyz_rates(
                    key, rate, state, value
                )
                self._validate_targets(targets)
                self._validate_rates(rates, max_rates)

                self._ramp_fields(targets, rates)
            else:

                # Determine the target value
                values = {k: v for k, v in zip(("r", "theta", "phi"), state)}
                values[key] = value
                vec = [values[k] for k in ("r", "theta", "phi")]
                targets = self._converter.convert_to_xyz(*vec)

                # Check that we respect the magnet bound
                self._validate_targets(targets)

                # Compute the rates for each axis
                rates = self._converter.convert_rate_to_xyz_rates(key, rate, state)
                self._validate_rates(rates, max_rates)

                self._ramp_fields(targets, rates)

        return value

    def performGetValue(self, quant, options={}):
        """Perform the Get Value instrument operation."""
        q_name = quant.name
        # For quantities corresponding to software only parameters simply
        # return the value
        if q_name in (
            "Power supply X axis: VISA address",
            "Power supply Y axis: VISA address",
            "Power supply Z axis: VISA address",
            "Power supply X axis: Model",
            "Power supply Y axis: Model",
            "Power supply Z axis: Model",
            "Power supply X axis: Conversion factor (T->native)",
            "Power supply Y axis: Conversion factor (T->native)",
            "Power supply Z axis: Conversion factor (T->native)",
            "Power supply X axis: Driver running",
            "Power supply Y axis: Driver running",
            "Power supply Z axis: Driver running",
            "Max field",
            "Max rate: X",
            "Max rate: Y",
            "Max rate: Z",
            "Specification mode",
            "Reference specification mode",
            "Direction x",
            "Direction y",
            "Direction z",
            "Direction theta",
            "Direction phi",
            "Direction XZangle",
            "Direction YZangle",
            "Phi offset",
            "Bz offset",
            "Field X rate",
            "Field Y rate",
            "Field Z rate",
            "Field magnitude rate",
            "Theta rate",
            "Phi rate",
            "Ramping mode",
        ):
            return quant.getValue()

        elif q_name in ("X Coil field", "Y Coil field", "Z Coil field"):
            return self._power_supplies[q_name[0].lower()].read_value()

        elif q_name in (
            "Field X",
            "Field Y",
            "Field Z",
            "Field magnitude",
            "Theta",
            "Phi",
        ):
            self._update_fields()
            return self.getValue(q_name)

        else:
            raise KeyError("Unknown quantity: %s" % q_name)

    def checkIfSweeping(self, quant, options={}):
        return any(
            supply.check_if_sweeping() for supply in self._power_supplies.values()
        )

    def performStopSweep(self, quant, options={}):
        for supply in self._power_supplies.values():
            supply.stop_sweep()

    def _get_max_rates(self):
        """Get the maximum rates of the coil."""
        return (
            self.getValue("Max rate: X"),
            self.getValue("Max rate: Y"),
            self.getValue("Max rate: Z"),
        )

    def _start_power_supply_driver(self, axis, address, model):
        """Start a power supply driver."""
        if not self._visa_rm:
            self._visa_rm = ResourceManager()
        if model not in MODELS:
            msg = "No power supply driver for {}, available drivers " "are {}"
            raise KeyError(msg.format(model, list(MODELS)))
        driver_cls = MODELS[model]
        cmd = "Power supply %s axis: Conversion factor (T->native)"
        cf = self.getValue(cmd % axis)
        self._power_supplies[axis.lower()] = driver_cls(self._visa_rm, address, cf)
        qname = "Power supply {} axis: Driver running"
        self.setValue(qname.format(axis), True)

    def _create_converter(self, mode, theta, phi, phi_offset, z_offset):
        """Create a converter"""
        args = (("angles", (theta, phi)), phi_offset, z_offset)
        if mode == "XYZ":
            self._converter = Converter(*args)
        elif mode == "Cylindrical":
            self._converter = CylindricalConverter(*args)
        else:
            self._converter = SphericalConverter(*args)

    def _update_fields(self):
        """Update the values of the fields after a change of basis/mode."""
        real_values = {}
        for k, v in self._power_supplies.items():
            real_values[k] = v.read_value()
        new_basis = self._converter.convert_from_xyz(**real_values)
        mode = self.getValue("Specification mode")
        if mode == "XYZ":
            names = ("Field X", "Field Y", "Field Z")
        elif mode == "Cylindrical":
            names = ("Field magnitude", "Phi", "Field Z")
        elif mode == "Spherical":
            names = ("Field magnitude", "Theta", "Phi")
        for name, value in zip(names, new_basis):
            self.setValue(name, value)

    def _update_reference_frame(self, mode, values):
        """Update the reference frame definition in all representations.

        XXX: currently the plane representation is not updated !

        """
        if mode == "XYZ":
            theta, phi = xyz_axis_to_angles(**values)
            self.setValue("Direction theta", theta)
            self.setValue("Direction phi", phi)
        elif mode == "Spherical":
            theta, phi = values
            self.setValue("Direction x", sin(theta) * cos(phi))
            self.setValue("Direction y", sin(theta) * sin(phi))
            self.setValue("Direction z", cos(theta))
        elif mode == "Plane":
            xz, yz = values
            xzrot = Rotation.from_euler("y", -xz, degrees=True)
            yzrot = Rotation.from_euler("x", yz, degrees=True)
            x, y, z = (xzrot * yzrot).apply([0, 0, 1])
            self.setValue("Direction x", x)
            self.setValue("Direction y", y)
            self.setValue("Direction z", z)
            theta, phi = xyz_axis_to_angles(x, y, z)
            self.setValue("Direction theta", theta)
            self.setValue("Direction phi", phi)
        else:
            raise ValueError("Unknown reference frame specification")

        theta = self.getValue("Direction theta")
        phi = self.getValue("Direction phi")
        phi_offset = self.getValue("Phi offset")
        z_offset = self.getValue("Bz offset")
        mode = self.getValue("Specification mode")
        self._create_converter(mode, theta, phi, phi_offset, z_offset)
        self._update_fields()

    def _ramp_fields(self, targets, rates):
        for axis, t, r in zip(("x", "y", "z"), targets, rates):
            if r == 0.0:
                continue
            psu = self._power_supplies[axis]
            psu.start_sweep(t, r)

    def _validate_targets(self, targets):
        max_field = self.getValue("Max field")
        if any(np.any(np.greater(field, max_field)) for field in targets):
            raise ValueError(
                "Field would exceed max field: %s" % np.max(targets, axis=1)
            )

    def _validate_rates(self, rates, max_rates):
        if any(np.any(np.greater(r, max_r)) for r, max_r in zip(rates, max_rates)):
            raise ValueError("Rate would exceed max rate: %s" % np.max(rates, axis=1))


if __name__ == "__main__":
    pass
