#!/usr/bin/env python
from VISA_Driver import VISA_Driver


class Driver(VISA_Driver):
    r"""Oxford Triton driver.

    This driver relies on the TCPIP::SOCKET protocol. The default port is 33576.

    This driver assumes that the fridge is setup such that we will naturally
    have the right access level. Currently the driver only reads information
    so an access level of GUEST is sufficient.
    The default access level is controlled by the following entry of the registry:
    Initial TritonAPI user access level

    It also assumes a default assignment of the thermometers:
    - MC -> MC RuOx
    - Cooldown -> MC Cernox

    The expected termchar is \r\n

    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Dictionary storing the UID
        self._uids = {}

    def performOpen(self, options={}):
        """Perform the operation of opening the instrument connection.

        """
        super().performOpen(options)
        answer = self.askAndLog("READ:SYS:USER")
        if "DENIED" in answer:
            raise RuntimeError(
                "Insufficient privilege please update the"
                " 'Initial TritonAPI user access level' registry to GUEST."
            )
        answer = self.askAndLog("READ:SYS:DR:CHAN:COOL", False)
        if answer.endswith("INVALID"):
            raise RuntimeError(f"Unexpected answer {answer}")
        self._uids["MC Cernox"] = answer.rsplit(":", 1)[1]
        answer = self.askAndLog("READ:SYS:DR:CHAN:MC", False)
        if answer.endswith("INVALID"):
            raise RuntimeError(f"Unexpected answer {answer}")
        self._uids["MC RuOx"] = answer.rsplit(":", 1)[1]

    def performGetValue(self, quant, options={}):
        """Perform the Get Value instrument operation

        Since the ITC return long messages rather than a simple value,
        we need to properly parse them.

        """
        get_cmd = quant.get_cmd.format(**self._uids)
        answer = self.askAndLog(get_cmd, False)
        if answer.endswith("N/A"):
            raise ValueError(f"Requested quantity is not available ({answer}).")
        elif answer.endswith("INVALID"):
            raise RuntimeError(
                f"The Triton failed to answer to {get_cmd}, answer was {answer}"
            )
        # Extract the numeric answer with its unit
        quantity = answer.rsplit(":", 1)[1]
        if quantity.lower() in ("on", "off"):
            return quantity

        if quant.name.endswith("heater range"):
            return quantity[:-2]  # Strip the unit

        quantity = quantity[: -len(quant.unit)]
        factor = 1
        if not quantity[-1].isdigit():
            prefix = quantity[-1]
            quantity = quantity[:-1]
            if prefix == "m":
                factor = 1e-3
            else:
                raise ValueError(f"Unknown unit prefix {prefix} in {answer}")

        return float(quantity) * factor

    def performSetValue(self, quant, value, sweepRate=0.0, options={}):
        """Perform the Set Value instrument operation.

        This function should return the actual value set by the instrument

        """
        q_name = quant.name
        cmd = quant.getCmdStringFromValue(value)
        if q_name.endswith("heater range"):
            cmd = cmd.replace('"', "")

        answer = self.askAndLog(quant.set_cmd + cmd)

        if not answer.endswith(":VALID"):
            raise RuntimeError("Instrument answered: {} to {}".format(answer, cmd))


if __name__ == "__main__":
    pass
