#!/usr/bin/env python

import InstrumentDriver
from VISA_Driver import VISA_Driver
import numpy as np

class Driver(VISA_Driver):
    """ This class implements the Agilen 33250 AWG"""


    def performGetValue(self, quant, options={}):
        """Perform the Get Value instrument operation.

        """
        if quant.name in ('BufferValues',):
            self.write('FORM:DATA ASCII')
            self.com.query("status:measurement?")
            self.write('trace:clear')
            self.write("status:measurement:enable 512; *sre 1")
            self.com.write("data:feed:control next")
            self.com.wait_for_srq(timeout=1200000)
            value  = self.com.query_ascii_values('data:data?')
        else:
            # for all other cases, call VISA driver
            value = VISA_Driver.performGetValue(self, quant, options=options)
        return value


if __name__ == '__main__':
    pass
