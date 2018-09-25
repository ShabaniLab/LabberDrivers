#!/usr/bin/env python
import qdac
from InstrumentDriver import InstrumentWorker


class Driver(InstrumentWorker):
    """ This class implements the QDevil QDAC driver"""

    def performOpen(self, options={}):
        """Perform the operation of opening the instrument connection"""
        self._daq = qdac.qdac(self.getAddress())
        self._daq.__enter__()

    def performClose(self, bError=False, options={}):
        """Perform the close instrument connection operation"""
        self._daq.__exit__(None, None, None)

    def performSetValue(self, quant, value, sweepRate=0.0, options={}):
        """Perform the Set Value instrument operation. This function should
        return the actual value set by the instrument"""
        channel, qname = quant.name.split('.')
        ch_id = int(channel.strip('Output'))
        if qname == 'Voltage':
            self._daq.setDCVoltage(ch_id, value)
        elif qname == 'VoltageRange':
            self._daq.setVoltageRange(ch_id, float(value.strip('V')))
        else:
            raise RuntimeError('Unknown attribute %s' % qname)
        return value

    def performGetValue(self, quant, options={}):
        """Perform the Get Value instrument operation"""
        channel, qname = quant.name.split('.')
        ch_id = int(channel.strip('Output'))
        if qname == 'Voltage':
            return self._daq.getDCVoltage(ch_id)
        elif qname == 'MeasuredCurrent':
            return self._daq.getCurrentReading(ch_id)
        elif qname == 'VoltageRange':
            return self._daq.voltageRange[ch_id]
        else:
            raise RuntimeError('Unknown attribute %s' % qname)
