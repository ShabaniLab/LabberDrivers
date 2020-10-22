#!/usr/bin/env python
from VISA_Driver import VISA_Driver


class Driver(VISA_Driver):
    """ This class implements the Oxford Mercury IPS driver"""

    def performGetValue(self, quant, options={}):
        """Perform the Get Value instrument operation

        Since the ITC return long messages rather than a simple value,
        we need to properly parse them.

        """
        get_cmd = quant.get_cmd
        answer = self.askAndLog(get_cmd, False)
        if answer.endswith('N/A'):
            raise ValueError('A wrong board id was used. Use READ:SYS:CAT to'
                             ' get the valid board ids.')
        elif answer.endswith('INVALID'):
            msg = 'The ITC failed to answer to {}'
            raise RuntimeError(msg.format(get_cmd))
        elif answer.endswith('N/A'):
            msg = ('The iTC does not appear to support this feature. It may'
                   ' be because a wrong UID was used. Use READ:SYS:CAT to'
                   ' get the valid board ids.\niTC answer {}')
            raise ValueError(msg.format(answer))
        # Extract the numeric answer with its unit
        quantity = answer[len(get_cmd) + 1:].split(':', 1)[0]
        if quantity.lower() in ('on','off'):
            return quant.getValueFromCmdString(quantity)
        return float(quantity.rstrip(quant.unit))

    def performSetValue(self, quant, value, sweepRate=0.0, options={}):
        """Perform the Set Value instrument operation

        The ITC always answers to indicate a command succeeded

        """
        set_cmd = quant.set_cmd + str(value) + quant.unit
        answer = self.askAndLog(set_cmd, False)
        if not answer.endswith('VALID'):
            msg = 'The operation failed the iTC answered: {}'
            raise RuntimeError(msg.format(answer))
        elif answer.endswith('N/A'):
            raise ValueError('A wrong board id was used. Use READ:SYS:CAT to'
                             ' get the valid board ids.')
        return value



if __name__ == '__main__':
    pass
