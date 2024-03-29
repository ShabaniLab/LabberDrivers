#!/usr/bin/env python

from VISA_Driver import VISA_Driver


class Driver(VISA_Driver):
    """ This class implements the Oxford Mercury IPS driver"""


    def performSetValue(self, quant, value, sweepRate=0.0, options={}):
        """Perform the Set Value instrument operation. This function should
        return the actual value set by the instrument"""
        # check quantity
        if quant.name == 'Magnetic Field':
            if sweepRate != 0:
                # sweep rate should be in T/min
                self._do_write('SET:DEV:GRPZ:PSU:SIG:RFST:'+ str(sweepRate*60.0))
            self._do_write('SET:DEV:GRPZ:PSU:SIG:FSET:'+  str(value))
            self._do_write('SET:DEV:GRPZ:PSU:ACTN:RTOS')
        else:
            # run standard VISA case
            value = VISA_Driver.performSetValue(self, quant, value, sweepRate, options)
        return value


    def performGetValue(self, quant, options={}):
        """Perform the Get Value instrument operation"""
       # value = float(self._do_read(quant.get_cmd)[:-1])
        answer = self.askAndLog(quant.get_cmd, False)
        quantity = answer[len(quant.get_cmd) + 1:].split(':', 1)[0]
        if quantity.lower() in ('o','on','off'):
            return quant.getValueFromCmdString(quantity)
        return float(self._do_read(quant.get_cmd)[:-1])
        
    def checkIfSweeping(self, quant, options={}):
        self.wait(0.1)
        # check that power supply is in hold mode
        if self._do_read('READ:DEV:GRPZ:PSU:ACTN') == 'HOLD':
            return False
        return True

    def performStopSweep(self, quant, options={}):
        self._do_write('SET:DEV:GRPZ:PSU:ACTN:HOLD')


    def _do_write(self, msg):
        """
        """
        answer = self.askAndLog(msg, False)
        if not answer.endswith('VALID'):
            if answer.endswith('N/A'):
                raise ValueError('A wrong board id was used. Use READ:SYS:CAT '
                                 'to get the valid board ids.')
            msg = 'The operation failed the iPS answered: {}'
            raise RuntimeError(msg.format(answer))

    def _do_read(self, get_cmd):
        """
        """
        answer = self.askAndLog(get_cmd)
        # strip first character
        if answer.endswith('N/A'):
            raise ValueError('A wrong board id was used. Use READ:SYS:CAT to'
                             ' get the valid board ids.')
        elif answer.endswith('INVALID'):
            msg = 'The ITC failed to answer to {}, its answer was {}'
            raise RuntimeError(msg.format(get_cmd, answer))
        elif answer.endswith('N/A'):
            msg = ('The iTC does not appear to support this feature. It may'
                   ' be because a wrong UID was used. Use READ:SYS:CAT to'
                   ' get the valid board ids.\niTC answer {}')
            raise ValueError(msg.format(answer))
        # Extract the numeric answer with its unit
        return answer[len(get_cmd) + 1:].split(':', 1)[0]

if __name__ == '__main__':
    pass
