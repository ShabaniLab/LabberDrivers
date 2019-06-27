#!/usr/bin/env python

import time
import numpy as np
from VISA_Driver import VISA_Driver

class Driver(VISA_Driver):
    """ The SRS 86x driver re-implements the VISA driver with extra options"""

    def performGetValue(self, quant, options={}):
        """Perform the Get Value instrument operation"""
        # perform special getValue for reading complex value
        name = str(quant.name)
        if name == 'Value':
            # get complex value in one instrument reading
            sCmd = 'SNAP? 0,1'
            sAns = self.askAndLog(sCmd).strip()
            lData =  sAns.split(',')

            #Sometimes, we receive the value twice
            #(0.12e-3,4.56e-70.12e-3,4.56e-7 instead of 0.12e-3,4.56e-7)
            #This is a simple fix:
            if len(lData) > 2:
                lData =  sAns[:int(len(sAns)/2)].split(',')
            #Also, sometimes we receive an additional "-" at the end of a value
            #(0.12e-3,4.56e-7- instead of 0.12e-3,4.56e-7)
            #Hence, another simple fix:
            if lData[1][-1] == "-":
                lData[1] = ldata[1][:-1]

            # return complex values
            return complex(float(lData[0].strip()), float(lData[1].strip()))
        elif name == 'Capture-data':
            # Start capturing data
            trig = self.getValue('Capture-trigger mode')

            trigger_mode = {'Immediate': 'IMM',
                            'Trigger-start': 'TRIG',
                            'Trigger-sample': 'SAMP'}[trig]

            self.com.write('CAPTURESTART ONESHOT, %s' % trigger_mode)

            # Wait for the buffer to be full
            tic = time.time()
            while not int(self.com.query('CAPTURESTAT?')) & 4:
                if time.time() - tic > 120:
                    raise RuntimeError('Data acquisition timed out.')
                time.sleep(0.1)

            # Get the data
            size = int(round(self.readValueFromOther('Capture-buffer size')))
            if size > 64:
                raise RuntimeError('Buffer with more 64 kB are not supported.')
            else:
                return self.com.query_binary_values('CAPTUREGET? 0, %d' % size,
                                                    container=np.ndarray)

        elif name in ('Capture-trigger mode',):
            return quant.getValue()
        else:
            # run the generic visa driver case
            return VISA_Driver.performGetValue(self, quant, options=options)

    def performSetValue(self, quant, value, sweepRate=0.0, options={}):
        """Perform the Set Value instrument operation

        """
        if quant.name in ('Capture-trigger mode',):
            return value
        else:
            return VISA_Driver.performSetValue(self, quant, value, options=options)


if __name__ == '__main__':
    pass
