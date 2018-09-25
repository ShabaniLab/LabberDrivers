import os
from configparser import ConfigParser

# Number of channel of the DAC
CHANNEL_NUMBER = 48

parser = ConfigParser()

parser['General settings'] = {'name': "QDevil DAC",
                              'version': "0.1",
                              'driver_path': "QDevilQDAC"}
parser['Model and options'] = {'check_model': False}

for ch_id in range(1, CHANNEL_NUMBER+1):
    parser['Output%d.Voltage' % ch_id] =\
        {'datatype': 'DOUBLE',
         'unit': 'V',
         'group': 'Output%d' % ch_id,
         'sweep_cmd': "***REPEAT SET***"}
    parser['Output%d.VoltageRange' % ch_id] =\
        {'datatype': 'COMBO',
         'combo_def_1': '10V',
         'cmd_def_1': 10.0,
         'combo_def_2': '1V',
         'cmd_def_2': 1.0,
         'group': 'Output%d' % ch_id}
    parser['Output%d.MeasuredCurrent' % ch_id] =\
        {'datatype': 'DOUBLE',
         'unit': 'A',
         'permission': "READ",
         'group': 'Output%d' % ch_id}

path = os.path.join(os.path.dirname(__file__), 'QDevilQDAC.ini')

with open(path, 'w') as f:
    parser.write(f)
