from ctypes import (CDLL, Structure, POINTER,
                    c_float, c_int, c_ubyte, c_ushort, c_uint, c_ulonglong,
                    c_void_p, c_char_p, c_long, c_uint8, c_uint16, c_uint32,
                    byref, c_longdouble)
import InstrumentDriver


class ListMode(Structure):
    """Structure represnting the list mode.

    """
    _fields_ = [(name, c_uint8)
                for name in ('sweep_mode', 'sweep_dir', 'tri_waveform',
                             'hw_trigger', 'step_on_hw_trig', 'return_to_start',
                             'trig_out_enable', 'trig_out_on_cycle')]


class OperateStatus(Structure):
    """Structure representing the operate status.

    """
    _fields_ = [(name, c_uint8)
                for name in ('rf1_lock_mode', 'rf1_loop_gain', 'device_access',
                             'device_standby', 'auto_pwr_disable',
                             'output_enable', 'ext_ref_lock_enable',
                             'ext_ref_detect', 'ref_out_select', 'list_mode_running',
                             'rf_mode', 'over_temp', 'harmonic_ss', 'pci_clk_enable', 'sweep_on_pwrup')]


class PllStatus(Structure):
    """Structure representing the PLL status.

    """
    _fields_ = [(name, c_uint8)
                for name in ('sum_pll_ld', 'crs_pll_ld', 'fine_pll_ld',
                             'crs_ref_pll_ld', 'crs_aux_pll_ld',
                             'ref_100_pll_ld', 'ref_10_pll_ld')]


class DeviceStatus(Structure):
    """Structure representing the device status.

    """
    _fields_ = [('list_mode', ListMode),
                ('operate_status', OperateStatus),
                ('pll_status', PllStatus)]


class RFParameters(Structure):
    """Structure representing the RF parameters.

    """
    _fields_ = [('frequency', c_longdouble),
                ('sweep_start_freq', c_longdouble),
                ('sweep_stop_freq', c_longdouble),
                ('sweep_step_freq', c_longdouble),
                ('sweep_dwell_time', c_uint32),
                ('sweep_cycles', c_uint32),
                ('buffer_points', c_uint32),
                ('rf_phase_offset', c_float),
                ('power_level', c_float),
                ('atten_value', c_float),
                ('level_dac_value', c_uint16)
                ]


class Driver(InstrumentDriver.InstrumentWorker):
    """ This class implements a simple signal generator driver"""


    def performOpen(self, options={}):
        """Perform the operation of opening the instrument connection"""
        self._lib = CDLL('sc5520a_uhfs.dll')
        self._set_signatures()
        # The expected format for the serial number is: 10001A4C
        addr = self.getAddress()
        # handle = self._lib.sc5520a_uhfsOpenDevice(1, addr.encode('utf-8'),0)
        handle = c_void_p()
        status = self._lib.sc5520a_uhfsOpenDevice(1, addr.encode('utf-8'),0, byref(handle)) 
        # raise TypeError(handle)
        # assert status == SCI_SUCCESS  (SCI_SUCCESS = 0)

        # raise TypeError(handle)
        if not handle:
            msg = 'Failed to connect to the instrument with serial number: %s'
            raise RuntimeError(msg % addr)
        self._handle = handle
        self._addr = addr

    def performClose(self, bError=False, options={}):
        """Perform the close instrument connection operation"""
        self._lib.sc5520a_uhfsCloseDevice(self._handle)

    def performSetValue(self, quant, value, sweepRate=0.0, options={}):
        """Perform the Set Value instrument operation.
        
        This function should return the actual value set by the instrument
        
        """
        qname = quant.name
        if qname == 'Frequency':
            ret = self._lib.sc5520a_uhfsSetFrequency(self._handle, value)
            self._check_return('Frequency', ret)
        elif qname == 'Amplitude':
            ret = self._lib.sc5520a_uhfsSetPowerLevel(self._handle, value)
            self._check_return('Amplitude', ret)
        elif qname == 'Output':
            ret = self._lib.sc5520a_uhfsSetOutputEnable(self._handle, value)
            self._check_return('Output', ret)
        else:
            raise RuntimeError('Unknown attribute %s' % qname)
        return value

    def performGetValue(self, quant, options={}):
        """Perform the Get Value instrument operation"""
        qname = quant.name
        params = RFParameters()
        self._lib.sc5520a_uhfsFetchRfParameters(self._handle, params)
        if qname == 'Frequency':
            return float(params.frequency)
        elif qname == 'Amplitude':
            return float(params.power_level)
        elif qname == 'Output':
            status = DeviceStatus()
            self._lib.sc5520a_uhfsFetchDeviceStatus(self._handle, status)
            return bool(status.operate_status.output_enable)
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
        self._lib.sc5520a_uhfsOpenDevice.argtypes = [c_int, c_char_p, c_uint8, POINTER(c_void_p)]
        self._lib.sc5520a_uhfsOpenDevice.restype = c_long

        self._lib.sc5520a_uhfsCloseDevice.argtypes = [c_void_p]
        self._lib.sc5520a_uhfsCloseDevice.restype = c_int

        self._lib.sc5520a_uhfsSetFrequency.argtypes = [c_void_p, c_longdouble]
        self._lib.sc5520a_uhfsSetFrequency.restype = c_uint

        self._lib.sc5520a_uhfsSetPowerLevel.argtypes = [c_void_p, c_float]
        self._lib.sc5520a_uhfsSetPowerLevel.restype = c_int

        self._lib.sc5520a_uhfsSetOutputEnable.argtypes = [c_void_p, c_ubyte]
        self._lib.sc5520a_uhfsSetOutputEnable.restype = c_int

        self._lib.sc5520a_uhfsFetchRfParameters.argtypes = [c_void_p, POINTER(RFParameters)]
        self._lib.sc5520a_uhfsFetchRfParameters.restype = c_int

        self._lib.sc5520a_uhfsFetchDeviceStatus.argtypes = [c_void_p, POINTER(DeviceStatus)]
        self._lib.sc5520a_uhfsFetchDeviceStatus.restype = c_int
