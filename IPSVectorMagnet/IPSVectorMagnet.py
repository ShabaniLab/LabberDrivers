#!/usr/bin/env python
from math import sin, cos, sqrt, atan2, acos
from threading import Thread, Event, RLock
from time import time, sleep

import numpy as np
from scipy.spatial.transform import Rotation
from pyvisa import ResourceManager

from VISA_Driver import VISA_Driver


class IPSPowerSupply:
    """[summary]

    Parameters
    ----------
    object : [type]
        [description]

    """

    def __init__(self, driver, axis):
        self._driver = driver  # for IPS we get the Labber driver
        self._axis = axis.upper()
        self.sweep_resolution = 1e-4
        self._sweeping_thread = None
        self._stop_sweeping = Event()

    def read_value(self):
        """Read the field currently applied by the power supply.

        """
        get_cmd = 'READ:DEV:GRP{}:PSU:SIG:FLD'.format(self._axis)
        return float(self._do_read(get_cmd)[:-1])

    def set_rate(self, rate):
        """Set the sweeping rate in T/min.

        """
        self._do_write('SET:DEV:GRP{}:PSU:SIG:RFST:{}'.format(self._axis,
                                                              rate*60.0))

    def start_sweep(self, target, rate, times=None):
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
        if rate is not None:
            self.set_rate(rate if time is None else rate[0])
        self._do_write('SET:DEV:GRP{}:PSU:SIG:FSET:{}'.format(self._axis,
                                                                target))
        self._do_write('SET:DEV:GRP{}:PSU:ACTN:RTOS'.format(self._axis))

        if times is not None:
            self._sweeping_thread = Thread(target=self._adjust_rate,
                                           args=(time, rate))
            self._sweeping_thread.start()

    def stop_sweep(self):
        """Stop the currently running sweep if any.

        """
        self._stop_sweeping.set()
        self._do_write('SET:DEV:GRP{}:PSU:ACTN:HOLD'.format(self._axis))

    def check_if_sweeping(self):
        """Check if the magnet is currently sweeping.

        """
        cmd = 'READ:DEV:GRP{}:PSU:SIG:FSET'.format(self._axis)
        target = float(self._do_read(cmd)[:-1])
        current_value = self.read_value()
        if abs(target - current_value) < self.sweep_resolution:
            status = self._do_read('READ:DEV:GRP{}:PSU:ACTN'.format(self._axis)
                                   )
            # check that power supply is in hold mode
            if status == 'HOLD':
                return False
        return True

    def _do_write(self, msg):
        """Write a value to the Oxford IPS.

        """
        with self._driver._lock:
            answer = self._driver.askAndLog(msg, False)
        if not answer.endswith('VALID'):
            if answer.endswith('N/A'):
                raise ValueError('A wrong board id was used. Use READ:SYS:CAT '
                                 'to get the valid board ids.')
            msg = 'The operation failed the iPS answered: {}'
            raise RuntimeError(msg.format(answer))

    def _do_read(self, get_cmd):
        """Read a value from the Oxford IPS.

        """
        with self._driver._lock:
            answer = self._driver.askAndLog(get_cmd)
        # strip first character
        if answer.endswith('N/A'):
            raise ValueError('A wrong board id was used. Use READ:SYS:CAT to'
                             ' get the valid board ids.')
        elif answer.endswith('INVALID'):
            msg = 'The iPS failed to answer to {}'
            raise RuntimeError(msg.format(get_cmd))
        elif answer.endswith('N/A'):
            msg = ('The iPS does not appear to support this feature. It may'
                   ' be because a wrong UID was used. Use READ:SYS:CAT to'
                   ' get the valid board ids.\niTC answer {}')
            raise ValueError(msg.format(answer))
        # Extract the numeric answer with its unit
        return answer[len(get_cmd) + 1:].split(':', 1)[0]

    def _adjust_rate(self, times, rates):
        """Adjust continuously the sweeping rate based on a time array.

        """
        start = time()
        times *= 60
        for t, r in zip(times[1:], rates[:1]):
            while time() - start < t:
                sleep(0.001)
            self.set_rate(r)
            if self._stop_sweeping.is_set():
                return


class CustomPowerSupply:
    """Base class for power supplies other than the Oxford power supply.

    Parameters
    ----------
    rm : visa.ResourceManager
        Resource manager to sue to connect to the instrument.
    address : str
        Visa address of the instrument.
    conversion_factor : float
        Conversion factor between T and whatever unit the power supply is
        using.

    """

    def __init__(self, rm, address, conversion_factor):
        self._driver = rm.open_resource(address)
        self.conversion_factor = conversion_factor

    def close(self):
        self._driver.close()

    def read_value(self):
        """Read the output

        """
        raise NotImplementedError

    def start_sweep(self, target, rate, times=None):
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
        raise NotImplementedError


    def stop_sweep(self):
        """Stop the currently running sweep if any.

        """
        raise NotImplementedError

    def check_if_sweeping(self):
        """Check if the magnet is currently sweeping.

        """
        raise NotImplementedError


class Keithley2450(CustomPowerSupply):
    """Driver to use a Keithley 2450 as a magnet power supply

    """
    def __init__(self, rm, address, conversion_factor):
        super().__init__(rm, address, conversion_factor)
        self._driver.write_termination = '\n'
        self._driver.read_termination = '\n'
        if self._driver.query(':OUTP:STAT?') == '0':
            msg = 'The output of {} is off, please turn it on.'
            raise RuntimeError(msg.format(address))
        mode = self._driver.query(':SOUR:FUNC?')
        if mode != 'CURR':
            msg = 'The output of {} is not set to current mode (read {}).'
            raise RuntimeError(msg.format(address, mode))

        self._sweeping_thread = None
        self._stop_sweeping = Event()

    def read_value(self):
        """Read the current output current and convert.

        """
        return float(self._driver.query(':SOUR:CURR?'))*self.conversion_factor

    def start_sweep(self, target, rate, times=None):
        """Start a sweep managed by the computer for simplicity.

        Parameters
        ----------
        target : float
            Target value in T.
        rate : float | np.ndarray
            Rate at which to update the output.
        times : np.ndarray
            Times at which to change the range (those are expected to be
            regularly spaced).

        """
        current = self.read_value()
        if times is None:
            # we update the value of the output every 10 ms
            step_number = int(round(abs(target-current)/(rate/60/100))) + 1
            values = np.linspace(current, target,
                                 step_number)
            times = np.linspace(0, 0.01*(step_number-1), step_number)
        else:
            # Compute the points in time at which to update the value of the
            # output
            intervals = times[1] - times[0]
            val_times = [np.linspace(t, t+intervals-0.01,
                                     int(round(100*(intervals))))
                         for t in times]

            # Compute the value of the output to set at each time.
            values = []
            last_val = current
            for time_slice, r in zip(val_times, rate):
                time_slice = time_slice - time_slice[0]
                values.append(last_val + r*time_slice)
                last_val = values[-1][-1] + 0.01*r

            # Add the missing the last point to times and values
            val_times.append(np.array([val_times[-1][-1] + 0.01]))
            values.append(np.array([target]))
            times = np.concatenate(val_times)
            values = np.concatenate(values)

        # convert the values to the proper unit
        values /= self._conversion_factor
        self._stop_sweeping.clear()
        self._sweeping_thread = Thread(target=self._update_output,
                                      args=(times, values))
        self._sweeping_thread.start()

    def stop_sweep(self):
        """Stop a currently running sweep.

        """
        if self._sweeping_thread:
            self._stop_sweeping.set()
            self._sweeping_thread.join()

    def check_if_sweeping(self):
        """Check if a sweep is currently underway.

        """
        return (self._sweeping_thread.is_alive()
                if self._sweeping_thread else False)

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
        cmd = ':SOUR:CURR {}'
        start = time()
        for t, val in zip(times[1:], values[1:]):
            while time() - start < t:
                sleep(0.001)
            driv.write(cmd.format(val))
            if stop_ev.is_set():
                break


MODELS = {'Keithley 2450': Keithley2450}


def xyz_axis_to_angles(x, y, z):
    """Convert an xyz representation of a vector to two angles.

    The first angle is the inclination in the spherical coordinate system, the
    second is the azimuthal angle counted from x.

    """
    theta = atan2(sqrt(x**2 + y**2), z)
    phi = atan2(y, x)
    return theta, phi


class Converter:
    """Object handling a basis change.

    The new basis is described by the direction of the new z axis and
    the angle between the x axis of the old basis and the x axis to use.


    """
    def __init__(self, axis, phi_offset, z_offset):

        axis_format, axis = axis
        if axis_format == 'xyz':
            axis = xyz_axis_to_angles(*axis)
        # Rotation to apply to go from the original coordinates to the new
        # coordinates
        # The rotations describe how to pass from the original frame to the
        # new frame as a consequence to transform a vector in the original
        # frame to the new frame we need to apply the inverse rotations in the
        # reverse order
        coordinate_change = [Rotation.from_euler('z', axis[1]),
                             Rotation.from_euler('y', axis[0]),
                             Rotation.from_euler('z', phi_offset - axis[1])]
        self._forward_change = (coordinate_change[2].inv() *
                                coordinate_change[1].inv() *
                                coordinate_change[0].inv())
        self._backward_change = (coordinate_change[0] *
                                 coordinate_change[1] *
                                 coordinate_change[2])
        self._z_offset = z_offset

    def to_new_basis(self, vec, no_offset=False):
        """Convert a vector expressed in the old basis.

        """
        new = self._forward_change.apply(vec)
        if not no_offset:
            new[2] -= self._z_offset
        return new

    def from_new_basis(self, vec, no_offset=False):
        """Convert a vector expressed in the new basis.

        """
        if not no_offset:
            vec[2] += self._z_offset
        return self._backward_change.apply(vec)


class CylindricalConverter(Converter):
    """Object handling a basis change.

    In the new basis the coordinates are expressed in cylindrical coordinates.

    """

    def convert_to_xyz(self, r, phi, z):
        return self.from_new_basis(np.array(r*cos(phi), r*sin(phi), z))

    def convert_from_xyz(self, x, y, z):
        x, y, z = self.to_new_basis(np.array((x, y, z)))
        return np.array((sqrt(x**2 + y**2), atan2(y, x), z))

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
        if axis in ('r', 'z'):
            rates = dict.fromkeys(('r', 'phi', 'z'), 0)
            rates[axis] = rate
            return self.convert_to_xyz(**rates)

        if state is None or target is None:
            msg = ('For phi ramp the state and target value must be '
                    'specified (state: {}, target: {})')
            raise ValueError(msg.format(state, target))

        # Compute the intermediate angles spaced by one degree, and omit the
        # last one since we won't have to update the rate
        phis = np.linspace(state[1], target,
                           int(round(target - state[1])) + 1)[:-1]
        x_rate = -state[0]*np.sin(phis)*rate
        y_rate = state[0]*np.cos(phis)*rate
        z_rate = np.zeros_like(phis)

        return ((phis - phis[0])/rate,
                self.from_new_basis(np.array(x_rate, y_rate, z_rate).T))


class SphericalConverter(Converter):
    """Object handling a basis change.

    In the new basis the coordinates are expressed in spherical coordinates.

    """

    def convert_to_xyz(self, r, theta, phi):
        x, y, z = r*cos(phi)*sin(theta), r*sin(phi)*sin(theta), r*cos(theta)
        return self.from_new_basis(np.array((x, y, z)))

    def convert_from_xyz(self, x, y, z):
        x, y, z = self.to_new_basis((x, y, z))
        r = sqrt(x**2 + y**2 + z**2)
        if not r:
            return 0, 0, 0
        theta = acos(z/r)
        phi = atan2(y, x)
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
        if axis == 'r':
            rates = dict.fromkeys(('r', 'phi', 'z'), 0)
            rates[axis] = rate
            return self.convert_to_xyz(**rates)

        if state is None or target is None:
            msg = ('For theta/phi ramp the state and target value must be '
                    'specified (state: {}, target: {})')
            raise ValueError(msg.format(state, target))

        if axis == 'theta':
            # Compute the intermediate angles spaced by one degree, and omit
            # the last one since we won't have to update the rate
            angles = np.linspace(state[1], target,
                                 int(round(target - state[1])) + 1)[:-1]
            x_rate = state[0]*np.cos(angles)*cos(state[2])*rate
            y_rate = state[0]*np.cos(angles)*sin(state[2])*rate
            z_rate = -state[0]*np.sin(angles)

        elif axis == 'phi':
            # Compute the intermediate angles spaced by one degree, and omit
            # the last one since we won't have to update the rate
            angles = np.linspace(state[2], target,
                                 int(round(target - state[2])) + 1)[:-1]
            x_rate = -state[0]*sin(state[1])*np.sin(angles)*rate
            y_rate = state[0]*sin(state[1])*np.cos(angles)*rate
            z_rate = np.zeros_like(angles)

        return ((angles - angles[0])/rate,
                self.from_new_basis(np.array(x_rate, y_rate, z_rate).T,
                                    no_offset=True))


class Driver(VISA_Driver):
    """ This class implements the Oxford Mercury IPS driver"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._target_xyz = ()
        self._power_supplies = {'x': None, 'y': None, 'z': None}
        self._visa_rm = None
        self._lock = RLock()
        self._converter = None

    def performOpen(self, options={}):
        """Perform the operation of opening the instrument connection.

        """
        super().performOpen(options)
        for axis in ('X', 'Y', 'Z'):
            opt = 'Custom B{} magnet power supply'.format(axis.lower())
            add = self.getValue(
                'Power supply {} axis: VISA address'.format(axis))
            model = self.getValue(
                'Power supply {} axis: VISA address'.format(axis))
            if options[opt]:
                if not add or not model:
                    continue  # The user will specify the address later
                self._start_power_supply_driver(axis, add, model)
            else:
                self._power_supplies[axis.lower()] = IPSPowerSupply(self, axis)

    def performClose(self, bError=False, options={}):
        """Perform the close instrument connection operation.

        """
        for axis in ('x', 'y', 'z'):
            supply = self._power_supplies[axis]
            if not isinstance(supply, IPSPowerSupply):
                supply.close()
                qname = 'Power supply {} axis: Driver running'
                self.setValue(qname.format(axis.upper()), False)
        super().performClose()

    def performSetValue(self, quant, value, sweepRate=0.0, options={}):
        """Perform the Set Value instrument operation.

        This function should return the actual value set by the instrument

        """
        q_name = quant.name

        if q_name in ('Power supply X axis: VISA address',
                          'Power supply Y axis: VISA address',
                          'Power supply Z axis: VISA address'):
            axis = q_name[13]
            model = self.getValue('Power supply %s axis: Model' % axis)
            if value and model:
                self._start_power_supply_driver(axis, value, model)
            else:
                supply = self._power_supplies[axis.lower()]
                if not isinstance(supply, IPSPowerSupply):
                    supply.close()
                    self._power_supplies[axis.lower()] = None

        elif q_name in ('Power supply X axis: Model',
                            'Power supply Y axis: Model',
                            'Power supply Z axis: Model'):
            axis = q_name[13]
            add = self.getValue('Power supply %s axis: VISA address' % axis)
            if value and model:
                self._start_power_supply_driver(axis, add, value)
            else:
                supply = self._power_supplies[axis.lower()]
                if not isinstance(supply, IPSPowerSupply):
                    supply.close()
                    self._power_supplies[axis.lower()] = None


        elif q_name in ('Power supply X axis: Conversion factor (T->native)',
                            'Power supply Y axis: Conversion factor (T->native)',
                            'Power supply Z axis: Conversion factor (T->native)',
                            ):
            axis = q_name[13]
            psu = self._power_supplies[axis.lower()]
            if psu is not None:
                psu.conversion_factor = value

        elif q_name in ('Max field',
                            'Max rate: X', 'Max rate: Y', 'Max rate: Z',
                            'Field X rate', 'Field Y rate', 'Field Z rate',
                            'Field magnitude rate', 'Theta rate', 'Phi rate'):
            pass  # Nothing to do for pure software values

        elif q_name == 'Reference specification mode':
            # Nothing we will keep xyz and theta phi in sync so that
            # representations agree
            pass

        elif q_name in ('Direction x', 'Direction y', 'Direction z'):
            values = {k: self.getValue('Direction %s' % k)
                      for k in ('x', 'y', 'z')}
            values[q_name[-1]] = value
            theta, phi = xyz_axis_to_angles(**values)
            self.setValue('Direction theta', theta)
            self.setValue('Direction phi', phi)
            phi_offset = self.getValue('Phi offset')
            z_offset = self.getValue('Bz offset')
            mode = self.getValue('Specification mode')
            self._create_converter(mode, theta, phi, phi_offset, z_offset)
            self._update_fields()

        elif q_name in ('Direction theta', 'Direction phi'):
            self.setValue('Direction x', sin(theta)*cos(phi))
            self.setValue('Direction y', sin(theta)*sin(phi))
            self.setValue('Direction z', cos(theta))
            if 'theta' in q_name:
                theta = value
                phi = self.getValue('Direction phi')
            else:
                theta = self.getValue('Direction theta')
                phi = value
            mode = self.getValue('Specification mode')
            phi_offset = self.getValue('Phi offset')
            z_offset = self.getValue('Bz offset')
            self._create_converter(mode, theta, phi, phi_offset, z_offset)
            self._update_fields()

        elif q_name == 'Phi offset':
            mode = self.getValue('Specification mode')
            theta = self.getValue('Direction theta')
            phi = self.getValue('Direction phi')
            z_offset = self.getValue('Bz offset')
            self._create_converter(mode, theta, phi, value, z_offset)
            self._update_fields()

        elif q_name == 'Bz offset':
            mode = self.getValue('Specification mode')
            theta = self.getValue('Direction theta')
            phi = self.getValue('Direction phi')
            phi_offset = self.getValue('Phi offset')
            self._create_converter(mode, theta, phi, phi_offset, value)
            self._update_fields()

        elif q_name == 'Specification mode':
            theta = self.getValue('Direction theta')
            phi = self.getValue('Direction phi')
            phi_offset = self.getValue('Phi offset')
            z_offset = self.getValue('Bz offset')
            self._create_converter(value, theta, phi, phi_offset, z_offset)
            self._update_fields()

        mode = self.getValue('Specification mode')
        max_rates = self._get_max_rates()
        if mode == 'XYZ':
            if q_name not in ('Field X', 'Field Y', 'Field Z'):
                raise KeyError()
            rate = sweepRate or self.getValue('Field %s rate' % q_name[-1])
            values = {k: self.getValue('Field %s' % k)
                      for k in ('X', 'Y', 'Z')}
            values[q_name[-1].lower()] = value
            targets = self._converter.from_new_basis([values[k]
                                                      for k in ('x', 'y', 'z')]
                                                     )
            if any(t > self.getValue('Max field') for t in targets):
                raise ValueError('The requested field is too large. Coil '
                                 'fields would be: %s' % targets)
            rates = [0, 0, 0]
            rates[['X', 'Y', 'Z'].index(q_name[-1])] = rate
            rates = self.from_new_basis(rates)
            if any(r > max_r for r, max_r in zip(rates, max_rates)):
                raise ValueError('Rate would exceed max rate: %s' % rates)

            for axis, t, r in zip(('x', 'y', 'z'), targets, rates):
                psu = self._power_supplies[axis]
                psu.start_sweep(t, r)

        elif mode == 'Cylindrical':
            if q_name not in ('Field magnitude', 'Phi', 'Field Z'):
                raise KeyError()
            rate = sweepRate or self.getValue(q_name + ' rate')
            values = {k: self.getValue(name)
                      for k, name in zip(('r', 'phi', 'z'),
                                         ('Field magnitude', 'Phi', 'Field Z'))
                      }
            key = {'Field magnitude': 'r',
                   'Phi': 'phi',
                   'Field Z': 'z'}[q_name]
            values[key] = value
            targets =\
                self._converter.convert_to_xyz([values[k]
                                                for k in ('r', 'phi', 'z')]
                                                )
            if any(t > self.getValue('Max field') for t in targets):
                raise ValueError('The requested field is too large. Coil '
                                 'fields would be: %s' % targets)

            if key == 'phi':
                state = (self.getValue('Field magnitude'),
                         self.getValue('Phi'),
                         self.getValue('Field Z'))
                times, rates =\
                    self._converter.convert_rate_to_xyz_rates(key, rate,
                                                              state, value)
                if any(np.any(np.greater(r, max_r))
                       for r, max_r in zip(rates, max_rates)):
                    raise ValueError('Rate would exceed max rate: %s' % rates)

                for axis, t, r in zip(('x', 'y', 'z'), targets, rates):
                    psu = self._power_supplies[axis]
                    psu.start_sweep(t, r, times)
            else:
                rates = [0, 0, 0]
                rates[['r', 'phi', 'z'].index(key)] = rate
                rates = self._converter.convert_rate_to_xyz_rates(key, rates)
                if any(r > max_r for r, max_r in zip(rates, max_rates)):
                    raise ValueError('Rate would exceed max rate: %s' % rates)

                for axis, t, r in zip(('x', 'y', 'z'), targets, rates):
                    psu = self._power_supplies[axis]
                    psu.start_sweep(t, r)

        else:
            if q_name not in ('Field magnitude', 'Theta', 'Phi'):
                raise
            rate = sweepRate or self.getValue(q_name + ' rate')
            values = {k: self.getValue(name)
                      for k, name in zip(('r', 'theta', 'phi'),
                                         ('Field magnitude', 'Theta', 'Phi'))
                      }
            key = {'Field magnitude': 'r',
                   'Theta': 'theta',
                   'Phi': 'phi'}[q_name]
            values[key] = value
            targets =\
                self._converter.convert_to_xyz([values[k]
                                                for k in ('r', 'theta', 'phi')]
                                                )
            if any(t > self.getValue('Max field') for t in targets):
                raise ValueError('The requested field is too large. Coil '
                                 'fields would be: %s' % targets)

            if key in ('theta', 'phi'):
                state = (self.getValue('Field magnitude'),
                         self.getValue('Theta'),
                         self.getValue('Phi'))
                times, rates =\
                    self._converter.convert_rate_to_xyz_rates(key, rate,
                                                              state, value)
                if any(np.any(np.greater(r, max_r))
                       for r, max_r in zip(rates, max_rates)):
                    raise ValueError('Rate would exceed max rate: %s' % rates)

                for axis, t, r in zip(('x', 'y', 'z'), targets, rates):
                    psu = self._power_supplies[axis]
                    psu.start_sweep(t, r, times)
            else:
                rates = [0, 0, 0]
                rates[['r', 'theta', 'phi'].index(key)] = rate
                rates = self._converter.convert_rate_to_xyz_rates(key, rates)
                if any(r > max_r for r, max_r in zip(rates, max_rates)):
                    raise ValueError('Rate would exceed max rate: %s' % rates)

                for axis, t, r in zip(('x', 'y', 'z'), targets, rates):
                    psu = self._power_supplies[axis]
                    psu.start_sweep(t, r)

        return value

    def performGetValue(self, quant, options={}):
        """Perform the Get Value instrument operation.

        """
        q_name = quant.name
        # For quantities corresponding to software only parameters simply
        # return the value
        if q_name in ('Power supply X axis: VISA address',
                          'Power supply Y axis: VISA address',
                          'Power supply Z axis: VISA address',
                          'Power supply X axis: Model',
                          'Power supply Y axis: Model',
                          'Power supply Z axis: Model',
                          'Power supply X axis: Conversion factor (T->native)',
                          'Power supply Y axis: Conversion factor (T->native)',
                          'Power supply Z axis: Conversion factor (T->native)',
                          'Power supply X axis: Driver running',
                          'Power supply Y axis: Driver running',
                          'Power supply Z axis: Driver running',
                          'Max field',
                          'Max rate: X',
                          'Max rate: Y',
                          'Max rate: Z',
                          'Specification mode',
                          'Reference specification mode',
                          'Direction x',
                          'Direction y',
                          'Direction z',
                          'Phi offset',
                          'Bz offset',
                          'Field X rate',
                          'Field Y rate',
                          'Field Z rate',
                          'Field magnitude rate',
                          'Theta rate',
                          'Phi rate',
                          ):
            return quant.getValue()

        elif q_name in ('X Coil field', 'Y Coil field', 'Z Coil field'):
            return self._power_supplies[q_name[0].lower()].read_value()

        elif q_name in ('Field X', 'Field Y', 'Field Z', 'Field magnitude',
                        'Theta', 'Phi'):
            self._update_fields()
            return self.getValue(q_name)

        else:
            raise KeyError('Unknown quantity: %s' % q_name)

    def checkIfSweeping(self, quant, options={}):
        return any(supply.check_if_sweeping()
                   for supply in self._power_supplies.values())

    def performStopSweep(self, quant, options={}):
        for supply in self._power_supplies.values():
            supply.stop_sweep()

    def _get_max_rates(self):
        """Get the maximum rates of the coil.

        """
        return (self.getValue('Max rate: X'),
                self.getValue('Max rate: Y'),
                self.getValue('Max rate: Z'))


    def _start_power_supply_driver(self, axis, address, model):
        """Start a power supply driver.

        """
        if not self._visa_rm:
            self._visa_rm = ResourceManager()
        if model not in MODELS:
            msg = ('No power supply driver for {}, available drivers '
                    'are {}')
            raise KeyError(msg.format(model, list(MODELS)))
        driver_cls = MODELS[model]
        cmd = 'Power supply %s axis:  Conversion factor (T->native)'
        cf = model = self.getValue(cmd % axis)
        self._power_supplies[axis.lower()] = model(self._visa_rm, add)
        qname = 'Power supply {} axis: Driver running'
        self.setValue(qname.format(axis), True)

    def _create_converter(self, mode, theta, phi, phi_offset, z_offset):
        """Create a converter

        """
        args = (('angles', (theta, phi)), phi_offset, z_offset)
        if mode == 'XYZ':
            self._converter = Converter(*args)
        elif mode == 'Cylindrical':
            self._converter = CylindricalConverter(*args)
        else:
            self._converter = SphericalConverter(*args)

    def _update_fields(self):
        """Update the values of the fields after a change of basis/mode.

        """
        real_values = {}
        for k, v in self._power_supplies:
            real_values[k] = v.read_value()
        new_basis = self._converter.convert_from_xyz(**real_values)
        if self.getValue('Specification mode') == 'XYZ':
            names = ('Field X', 'Field Y', 'Field Z')
        if self.getValue('Specification mode') == 'Cylindrical':
            names = ('Field magnitude', 'Phi', 'Field Z')
        if self.getValue('Specification mode') == 'Spherical':
            names = ('Field magnitude', 'Theta', 'Phi')
        for name, value in zip(names, new_basis):
            self.setValue(name, value)

if __name__ == '__main__':
    pass
