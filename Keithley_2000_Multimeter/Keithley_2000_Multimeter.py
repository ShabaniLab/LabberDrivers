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
            self.wait_for_srq()
            value  = self.query_binary_values('data:data?')
        else:
            # for all other cases, call VISA driver
            value = VISA_Driver.performGetValue(self, quant, options=options)
        return value

    def performArm(self, quant_names, options={}):
        """Prepare the system for the next trigger.

        """
        self.write('trace:clear')
        self.write("status:measurement:enable 512; *sre 1")
        self.write("feed:control next")
        self.write('FORM:DATA DREAL;FORM:BORD NORM')
        self.write('initiate')

if __name__ == '__main__':
    pass
