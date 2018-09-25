# qdac.py
# Copyright QDevil ApS, March-July 2018
import serial
import time

class Waveform:
    # Enum-like class defining the built-in waveform types
    sine = 1
    square = 2
    triangle = 3
    all = [sine, square, triangle]

class Generator:
    # Enum-like class defining the waveform generators
    DC = 0
    generator1 = 1
    generator2 = 2
    generator3 = 3
    generator4 = 4
    generator5 = 5
    generator6 = 6
    generator7 = 7
    generator8 = 8
    AWG = 9
    pulsetrain = 10
    functionGenerators = [generator1, generator2, generator3, generator4, generator5, generator6,
           generator7, generator8]
    syncGenerators = functionGenerators + [AWG, pulsetrain]
    all = [DC] + syncGenerators

class qdac():
    # Main QDAC instance class
    noChannel = 0
    debugMode = False

    def __init__(self, port, verbose=False):
        # Constructor
        # port: Serial port for QDAC
        # verbose: Print serial communication during operation. Useful for debugging
        self.port = port
        self.verbose = verbose
        self.channelNumbers = range(1, 49)
        self.syncChannels = range(1, 6)
        self.voltageRange = {ch: 10.0 for ch in self.channelNumbers} # Assumes that QDAC has power-on values
        self.currentRange = {ch: 100e-6 for ch in self.channelNumbers} # Assumes that QDAC has power-on values

    def __enter__(self):
        self.sport = serial.Serial(port=self.port, baudrate=460800, bytesize=serial.EIGHTBITS,
                                   parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.5)
        return self

    def __exit__(self, type, value, traceback):
        self.sport.close()

    def flush(self):
        # Purges the serial port input buffer
        while True:
            response = self.sport.read(1)
            if not response:
                break

    def getSerialNumber(self):
        # Returns the QDAC unit serial number
        reply = self._checkForError(self._sendReceive(b'sernum'))
        try:
            return reply.decode("utf-8")
        except:
            raise Exception("Error response from QDAC: <%s>" % reply)

    def getNumberOfBoards(self):
        # Returns number of 8-channel boards
        reply = self._checkForError(self._sendReceive(b"boardNum"))
        try:
            self.numBoards = int(reply.split(b":", 1)[1])
            self.channelNumbers = range(1, self.numBoards*8+1)
            self.syncChannels = range(1, self.numBoards)
            return self.numBoards
        except:
            raise Exception("Error response from QDAC: <%s>" % reply)

    def getNumberOfChannels(self):
        # Returns number of channels on the QDAC unit
        return self.getNumberOfBoards()*8

    def setVoltageRange(self, channel, theRange):
        # Set the voltage output range of a QDAC channel
        # range must be 1.0 or 10.0 (unit is V)
        if self.debugMode == False:
            self._validateChannel(channel)
        if theRange == 10.0:
            rangeFlag = 0
        elif theRange == 1.0:
            rangeFlag = 1
        else:
            raise Exception("Invalid voltage range %d" % theRange)
        self.voltageRange[channel] = theRange
        return self._checkForError(self._sendReceive(b"vol %d %d" % (channel, rangeFlag))).decode("utf-8")

    def setCurrentRange(self, channel, theRange):
        # Set the current sensing range of a QDAC channel
        # range must be 1e-6 or 100e-6 (unit is A)
        if self.debugMode == False:
            self._validateChannel(channel)
        if theRange == 1e-6:
            rangeFlag = 0
        elif theRange == 100e-6:
            rangeFlag = 1
        else:
            raise Exception("Invalid current range %e" % theRange)
        self.currentRange[channel] = theRange
        return self._checkForError(self._sendReceive(b"cur %d %d" % (channel, rangeFlag))).decode("utf-8")

    def getDCVoltage(self, channel):
        # Gets the DC voltage that is currently set a QDAC channel
        # This only works if setChannelOutput has been set to Generator.DC, which is the power-on setting!!
        if self.debugMode == False:
            self._validateChannel(channel)
        reply = self._checkForError(self._sendReceive(b"set %d" % channel))
        try:
            return float(reply.split(b":", 1)[1].split()[1])
        except:
            raise Exception("Error response from QDAC: <%s>" % reply)

    def setDCVoltage(self, channel, volts):
        # Set the immediate DC voltage of a QDAC channel
        # This only works if setChannelOutput has been set to Generator.DC, which is the power-on setting!!
        if self.debugMode == False:
            self._validateChannel(channel)
            self._validateVoltage(channel, volts)
        return self._checkForError(self._sendReceive(b"set %d %e" % (channel, volts))).decode("utf-8")

    def setRawDAC(self, channel, dacValue):
        # Sets the DAC output of a channel as a raw integer
        # value range from -524288 to 524287
        if dacValue < -524288 or dacValue > 524287:
            raise Exception("Invalid dac value: %d" % dacValue)
        return self._checkForError(self._sendReceive(b"dac %d %d" % (channel, dacValue))).decode("utf-8")

    def getRawDAC(self, channel):
        # Gets the DAC output of a channel as a raw integer
        # value range from -524288 to 524287
        self._validateChannel(channel)
        reply = self._checkForError(self._sendReceive(b"dac %d" % channel))
        try:
            return int(reply.split(b":")[1].split()[1])
        except:
            raise Exception("Error response from QDAC: <%s>" % reply)

    def setCalibrationChannel(self, channel):
        # Connect a QDAC channel to the Calibration output. Useful for testing the output performance
        # Set channel to 0 to disconnect all channels from the Calibration output
        if self.debugMode == False:
            if channel != qdac.noChannel:
                self._validateChannel(channel)
        return self._checkForError(self._sendReceive(b"cal %d" % channel)).decode("utf-8")

    def getCalibrationChannel(self):
        return int(self._sendReceive(b"cal").split(':')[1].strip())
    def readTemperature(self, board, position):
        # Read the temperature in Celsius inside the QDAC at different positions
        # Board: 0-5
        # Channel: 0-2
        # board 0 is channel 1-8, board 1 is channel 9-16, etc.
        if self.debugMode == False:
            if board not in [0,1,2,3,4,5] or position not in [0,1,2]:
                raise Exception("readTemperature: Invalid board %d or position %d" % (board, position))
        reply = self._checkForError(self._sendReceive(b"tem %d %d" % (board, position)))
        try:
            return float(reply.split(b":", 1)[1])
        except:
            raise Exception("Error response from QDAC: <%s>" % reply)

    def defineFunctionGenerator(self, generator, waveform, period, dutycycle=0, repetitions=-1):
        # Define a function generator
        # generator: Generator.generator1, ..., Generator.generator8
        # waveform: Waveform.sine, Waveform.square, Waveform.triangle
        # period: Number of samples in waveform period
        # dutycycle: 0-100, used for square and triangle waveforms to define shape
        # repetitions: How many times the waveform is repeated. -1 means infinite
        # Note: The amplitude is always max. range of the channel. Set the amplitude in setChannelOutput
        if generator not in Generator.functionGenerators:
            raise Exception("Invalid generator number (must be 1-8): %d" % generator)
        if waveform not in Waveform.all:
            raise Exception("Invalid waveform: %d" % waveform)
        if period < 1:
            raise Exception("Invalid waveform period: %d" % period)
        if repetitions < -1 or repetitions > 0x7FFFFFFF:
            raise Exception("Invalid number of repetitions: %d" % repetitions)
        if dutycycle < 0 or dutycycle > 100:
            raise Exception("Invalid dutycycle: %d" % dutycycle)
        return self._checkForError(self._sendReceive(b"fun %d %d %d %d %d" % (generator, waveform, period, dutycycle, repetitions))).decode("utf-8")

    def getFunctionGenerator(self, generator):
        if not self.debugMode:
            if generator not in Generator.functionGenerators:
                raise Exception("Invalid generator number (must be 1-8): %d" % generator)
        return self._checkForError(self._sendReceive(b"fun %d" % generator)).decode("utf-8")

    def definePulsetrain(self, lowDuration, highDuration, lowVolts, highVolts, repetitions=-1):
        # Define a pulse train function generator
        # The generator is always Generator.pulsetrain
        # lowDuration, highDuration, lowVolts, highVolts defines the pulsetrain
        # repetitions: How many times the waveform is repeated. -1 means infinite
        if not self.debugMode:
            if lowDuration < 0 or lowDuration > 0x7FFFFFFF:
                raise Exception("Invalid lowDuration: %d" % lowDuration)
            if highDuration < 0 or highDuration > 0x7FFFFFFF:
                raise Exception("Invalid highDuration: %d" % highDuration)
            if lowVolts < -10.0 or lowVolts > 10.0:
                raise Exception("Invalid lowVolts: %f" % lowVolts)
            if highVolts < -10.0 or highVolts > 10.0:
                raise Exception("Invalid highVolts: %f" % highVolts)
            if repetitions < -1 or repetitions > 0xFFFFFFFF:
                raise Exception("Invalid number of repetitions: %d" % repetitions)
        return self._checkForError(self._sendReceive(b"pul %d %d %e %e %d" % (lowDuration, highDuration, lowVolts, highVolts, repetitions))).decode("utf-8")

    def getPulsetrain(self):
        return self._checkForError(self._sendReceive(b"pul")).decode("utf-8")

    def defineAWG(self, samples, repetitions=-1): # Sample rate is 1kS/s
        # Define a pulse train function generator
        # The generator is always Generator.AWG
        # samples: An array of volt, defines the pulsetrain samples at 1000 samples per second. Max 8000 samples allowed
        # repetitions: How many times the waveform is repeated. -1 means infinite
        if len(samples) == 0 or len(samples) > 8000:
            raise Exception("Invalid number of samples in AWG definition")
        for idx in range(0, len(samples), 64):
            cmd = ("awg 0 0 " + " ".join(["%e" % v for v in samples[idx:idx+64]])).encode('ascii')
            self._sendReceive(cmd)
        return self._checkForError(self._sendReceive(b"run %d" % repetitions)).decode("utf-8")

    def setChannelOutput(self, channel, generator, amplitude=1.0, offset=0):
        # Defines the output for a channel
        # generator: Generator.DC, Generator.generator1, .., Generator.generator8, Generator.AWG, Generator.pulsetrain
        # amplitude: Scaling of the waveform
        # offset: Voltage offset
        if not self.debugMode:
            self._validateChannel(channel)
            self._validateVoltage(channel, amplitude)
            self._validateVoltage(channel, offset)
        if generator not in Generator.all:
            raise Exception("Invalid generator number (must be 0-10): %d" % generator)
        return self._checkForError(self._sendReceive(b"wav %d %d %e %e" % (channel, generator, amplitude, offset))).decode("utf-8")

    def getChannelOutput(self, channel):
        if not self.debugMode:
            self._validateChannel(channel)
        return self._checkForError(self._sendReceive(b"wav %d" %channel)).decode("utf-8")

    def setSyncOutput(self, syncChannel, generator, delay=0, pulseLength=1):
        # Set output on a sync output channel
        # syncChannel: 0-5
        # generator: The generator that the sync channel follows
        # delay: milliseconds delay for the sync
        # pulseLength: Length in milliseconds of the sync pulse
        if syncChannel not in self.syncChannels:
            raise Exception("Invalid sync channel (must be 0-5): %d" % syncChannel)
        if generator not in Generator.all:
            raise Exception("Invalid generator number (must be 0-10): %d" % generator)
        if delay < 0 or delay > 268435455:
            raise Exception("Invalid sync channel delay: %d ms" % delay)
        if pulseLength < 1:
            raise Exception("Invalid sync channel pulse length: %d ms" % pulseLength)
        return self._checkForError(self._sendReceive(b"syn %d %d %d %d" % (syncChannel, generator, delay, pulseLength))).decode("utf-8")

    def setSyncOutputOff(self, syncChannel):
        # Turns off output on a sync output channel
        # syncChannel: 0-5
        if syncChannel not in self.syncChannels:
            raise Exception("Invalid sync channel (must be 0-5): %d" % syncChannel)
        return self._checkForError(self._sendReceive(b"syn %d %d %d %d" % (syncChannel, 0, 0, 0))).decode("utf-8")

    def getCurrentReading(self, channel):
        # Reads current from a DAC channel. Unit is in Amps
        if self.debugMode == False:
            self._validateChannel(channel)
        reply = self._checkForError(self._sendReceive(b"get %d" % channel))
        try:
            return float(reply.split(b":", 1)[1][:-2])*1e-6
        except:
            raise Exception("Error response from QDAC: <%s>" % reply)

    def getRawCurrentADCreading(self, channel):
        # Reads current from a DAC channel. Unit is in Amps
        if self.debugMode == False:
            self._validateChannel(channel)
        reply = self._checkForError(self._sendReceive(b"adc %d" % channel))
        try:
            return int(reply.split(b":", 1)[1])
        except:
            raise Exception("Error response from QDAC: <%s>" % reply)

    def waitForSync(self, generator, timeout=-1):
        # Software wait for the beginning of a generator signal
        # generator: The generator that the sync waits for
        # timeout: Max number of seconds to wait. -1 = infinite
        beginTime = time.time()
        if generator not in Generator.syncGenerators:
            raise Exception("Invalid generator number (must be 1-10): %d" % generator)
        self._checkForError(self._sendReceive(b"ssy %d" % generator))
        while True:
            response = self._readLine(failOnTimeout=False)
            if response and b"#" in response:
                return True
            if timeout > 0 and time.time() - beginTime > timeout:
                return False
            else:
                time.sleep(0.001)

    def goToBootloader(self):
        # Soft restart system and enter bootlaoder
        return self._checkForError(self._sendReceive(b"bootloader")).decode("utf-8")

    def setVoltageCalibration(self, channel, theRange, samplesPerVolt, offsetSamples):
        # range must be 1.0 or 10.0 (unit is V)
        # Note: Data is not stored in persisted memory. When instrument is rebooted, it is set back to factory values.
        if self.debugMode == False:
            self._validateChannel(channel)
        if theRange == 10.0:
            rangeFlag = 0
        elif theRange == 1.0:
            rangeFlag = 1
        else:
            raise Exception("Invalid voltage range %f" % theRange)
        return self._checkForError(self._sendReceive(b"vcal %d %d %e %e" % (channel, rangeFlag, samplesPerVolt, offsetSamples))).decode("utf-8")

    def getVoltageCalibration(self, channel, theRange):
        # Range must be 1.0 or 10.0 (unit is V)
        if theRange == 10.0:
            rangeFlag = 0
        elif theRange == 1.0:
            rangeFlag = 1
        else:
            raise Exception("Invalid voltage range %f" % theRange)
        return self._checkForError(self._sendReceive(b"vcal %d %d" % (channel, rangeFlag))).decode("utf-8") # TODO: Proper formatting processing of response

    def setCurrentCalibration(self, channel, theRange, microampsPerSample, offsetMicroamps):
        # range must be 1e-6 or 100e-6 (unit is A)
        # Note: Data is not stored in persisted memory. When instrument is rebooted, it is set back to factory values.
        if self.debugMode == False:
            self._validateChannel(channel)
        if theRange == 1e-6:
            rangeFlag = 0
        elif theRange == 100e-6:
            rangeFlag = 1
        else:
            raise Exception("Invalid current range %e" % theRange)
        return self._checkForError(self._sendReceive(b"ical %d %d %e %e" % (channel, rangeFlag, microampsPerSample, offsetMicroamps))).decode("utf-8")

    def getCurrentCalibration(self, channel, theRange):
        # range must be 1e-6 or 100e-6 (unit is A)
        if theRange == 1e-6:
            rangeFlag = 0
        elif theRange == 100e-6:
            rangeFlag = 1
        else:
            raise Exception("Invalid current range: %e" % theRange)
        return self._checkForError(self._sendReceive(b"ical %d %d" % (channel, rangeFlag))).decode("utf-8") # TODO: Proper formatting processing of response

    def setBaudrate(self, baudrate):
        # Changes the communication speed with the QDAC
        BaudratesAllowed = [600, 1200, 2400, 4800, 9600, 14400, 19200, 38400, 57600, 115200, 230400, 460800, 921600]
        if baudrate in BaudratesAllowed:
            reply = self._checkForError(self._sendReceive(b"brt %d" % baudrate)).decode("utf-8")
            if (reply == 'Changing BAUD rate to: %d' % baudrate):
                self.sport.close()
                self.sport = serial.Serial(port=self.port, baudrate=baudrate, bytesize=serial.EIGHTBITS,
                                           parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.5)
            else:
                raise Exception("Failed to change baud rate: %s" % reply)
        else:
            raise Exception("Invalid baud rate: %d" % baudrate)

    def restart(self):
        # Reboot the QDAC firmware
        self.goToBootloader()
        time.sleep(2)
        self.sport.close()
        self.sport = serial.Serial(port=self.port, baudrate=115200, bytesize=serial.EIGHTBITS,
                                   parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.5)
        self.sport.write(b'\x01\x05\xa5\x50\x04')
        self.sport.close()
        self.sport = serial.Serial(port=self.port, baudrate=460800, bytesize=serial.EIGHTBITS,
                                   parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.5)
        time.sleep(0.5)
        self.flush()

    def setDebugMode(self, value):
        self.debugMode = value

    def _checkForError(self, message):
        if message[0:5] == b"Error":
            raise Exception("Error response from QDAC: <%s>" % message)
        return message

    def _validateChannel(self, channel):
        if channel not in self.channelNumbers:
            raise Exception("Invalid channel number: %d" % channel)

    def _validateVoltage(self, channel, volts):
        if volts < 0.001-self.voltageRange[channel] or volts > self.voltageRange[channel]-0.001:
            raise Exception("Invalid voltage: %f" % volts)

    def _sendReceive(self, msg):
        if self.verbose:
            print(msg)
        self.sport.write(msg + b"\n")
        reply = self._readLine()
        return reply

    def _readLine(self, failOnTimeout=True):
        out = b""
        c = b""
        while True:
            c = self.sport.read(1)
            if c:
                if c != b"\n":
                    out += c
                else:
                    break
            else:
                if failOnTimeout and self.verbose:
                    raise Exception("Timeout!")
                break
        if self.verbose and out:
            print(out)
        return out
