from ctypes import (CDLL, Structure, POINTER,
                    c_float, c_int, c_ubyte, c_ushort, c_uint, c_ulonglong,
                    c_void_p, c_char_p)
import InstrumentDriver


class ListMode(Structure):
    """Structure represnting the list mode.

    """
    _fields_ = [(name, c_ubyte)
                for name in ('sss_mode', 'weep_dir', 'tri_waveform',
                             'hw_trigger', 'step_on_hw_trig', 'return_to_start',
                             'trig_out_enable', 'trig_out_on_cycle')]


class OperateStatus(Structure):
    """Structure representing the operate status.

    """
    _fields_ = [(name, c_ubyte)
                for name in ('rf1_lock_mode', 'rf1_loop_gain', 'device_access',
                             'rf2_standby', 'rf1_standby', 'auto_pwr_disable',
                             'alc_mode', 'rf1_out_enable', 'ext_ref_lock_enable',
                             'ext_ref_detect', 'ref_out_select', 'list_mode_running',
                             'rf1_mode', 'over_temp', 'harmonic_ss')]


class PllStatus(Structure):
    """Structure representing the PLL status.

    """
    _fields_ = [(name, c_ubyte)
                for name in ('sum_pll_ld', 'crs_pll_ld', 'fine_pll_ld',
                             'crs_ref_pll_ld', 'crs_aux_pll_ld',
                             'ref_100_pll_ld', 'ref_10_pll_ld', 'rf2_pll_ld')]


class DeviceStatus(Structure):
    """Structure representing the device status.

    """
    _fields_ = [('list_mode', ListMode),
                ('operate_status', OperateStatus),
                ('pll_status', PllStatus)]


class RFParameters(Structure):
    """Structure representing the RF parameters.

    """
    _fields_ = [('rf1_freq', c_ulonglong),
                ('start_freq', c_ulonglong),
                ('stop_freq', c_ulonglong),
                ('step_freq', c_ulonglong),
                ('sweep_dwell_time', c_uint),
                ('sweep_cycles', c_uint),
                ('buffer_points', c_uint),
                ('rf_level', c_float),
                ('rf2_freq', c_ushort)]


class Driver(InstrumentDriver.InstrumentWorker):
    """ This class implements a simple signal generator driver"""


    def performOpen(self, options={}):
        """Perform the operation of opening the instrument connection"""
        self._lib = CDLL('sc5511a.dll')
        self._set_signatures()
        # The expected format for the serial number is: 10001A4C
        addr = self.getAddress()
        handle = self._lib.sc5511a_open_device(addr.encode('utf-8'))
        if not handle:
            msg = 'Failed to connect to the instrument with serial number: %s'
            raise RuntimeError(msg % addr)
        self._handle = handle

    def performClose(self, bError=False, options={}):
        """Perform the close instrument connection operation"""
        self._lib.sc5511a_close_device(self._handle)

    def performSetValue(self, quant, value, sweepRate=0.0, options={}):
        """Perform the Set Value instrument operation.
        
        This function should return the actual value set by the instrument
        
        """
        qname = quant.name
        if qname == 'Frequency':
            value = int(round(value))
            ret = self._lib.sc5511a_set_freq(self._handle, value)
            self._check_return('Frequency', ret)
        elif qname == 'Amplitude':
            ret = self._lib.sc5511a_set_level(self._handle, value)
            self._check_return('Amplitude', ret)
        elif qname == 'Output':
            ret = self._lib.sc5511a_set_output(self._handle, value)
            self._check_return('Output', ret)
        else:
            raise RuntimeError('Unknown attribute %s' % qname)
        return value

    def performGetValue(self, quant, options={}):
        """Perform the Get Value instrument operation"""
        qname = quant.name
        params = RFParameters()
        self._lib.sc5511a_get_rf_parameters(self._handle, params)
        if qname == 'Frequency':
            return float(params.rf1_freq)
        elif qname == 'Amplitude':
            return float(params.rf_level)
        elif qname == 'Output':
            status = DeviceStatus()
            self._lib.sc5511a_get_device_status(self._handle, status)
            return bool(status.operate_status.rf1_out_enable)
        else:
            raise RuntimeError('Unknown attribute %s' % qname)

    def _check_return(self, name, ret_code):
        """Check the return code for library function call.

        """
        if ret_code == 0:
            pass
        else:
            raise RuntimeError('An error occured setting %s: %d' % (name, ret_code))

    def _set_signatures(self):
        """Set the signature of the DLL functions we use.

        """
        self._lib.sc5511a_open_device.argtypes = [c_char_p]
        self._lib.sc5511a_open_device.restype = c_void_p

        self._lib.sc5511a_close_device.argtypes = [c_void_p]
        self._lib.sc5511a_close_device.restype = c_int

        self._lib.sc5511a_set_freq.argtypes = [c_void_p, c_ulonglong]
        self._lib.sc5511a_set_freq.restype = c_int

        self._lib.sc5511a_set_level.argtypes = [c_void_p, c_float]
        self._lib.sc5511a_set_level.restype = c_int

        self._lib.sc5511a_set_output.argtypes = [c_void_p, c_ubyte]
        self._lib.sc5511a_set_output.restype = c_int

        self._lib.sc5511a_get_rf_parameters.argtypes = [c_void_p, POINTER(RFParameters)]
        self._lib.sc5511a_get_rf_parameters.restype = c_int

        self._lib.sc5511a_get_device_status.argtypes = [c_void_p, POINTER(DeviceStatus)]
        self._lib.sc5511a_get_device_status.restype = c_int
