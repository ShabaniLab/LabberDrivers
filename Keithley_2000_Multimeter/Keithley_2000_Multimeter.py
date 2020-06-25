#!/usr/bin/env python

import InstrumentDriver
from VISA_Driver import VISA_Driver
import numpy as np


class Driver(VISA_Driver):
    """ This class implements the Keithley DMM2000.

    """

    def performArm(quant_names, options={}):
        """Prepare the system for a single trigger and enable serial request.

        """
        self.com.query("status:measurement?")
        self.write("trace:clear")
        self.write("status:measurement:enable 512; *sre 1")
        self.com.write("data:feed:control next")

    def performGetValue(self, quant, options={}):
        """Perform the Get Value instrument operation.

        """
        if quant.name == "Buffer values":
            nsamples = self.readValueFromOther("Sample count")
            buffer_size = self.readValueFromOther("Buffer size")
            precision = self.readValueFromOther("Buffer precision")
            self.com.wait_for_srq(timeout=1200000)
            value = self.com.query_binary_values(
                "data:data?",
                datatype="f" if precision == "Single" else "d",
                container=np.ndarray,
                data_points=buffer_size,
            )
            # Remove points from the buffer we do not care about
            value = value[:nsamples]
        else:
            # for all other cases, call VISA driver
            value = VISA_Driver.performGetValue(self, quant, options=options)
        return value


if __name__ == "__main__":
    pass
