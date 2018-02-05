"""
# SPDX-License-Identifier: GPL-2.0
tdc_config.py - tdc user-specified values

Copyright (C) 2017 Lucas Bates <lucasb@mojatatu.com>
"""

# Dictionary containing all values that can be substituted in executable
# commands.
NAMES = {
          # Substitute your own tc path here
          'TC': '/sbin/tc',
          # Name of veth devices to be created for the namespace
          'DEV0': 'v0p0',
          'DEV1': 'v0p1',
          'DEV2': '',
          'BATCH_FILE': './batch.txt',
          # Name of the namespace to use
          'NS': 'tcut'
        }


ENVIR = { }

# put customizations in tdc_config_local.py
try:
    from tdc_config_local import *
except ImportError as ie:
    pass

try:
    NAMES.update(EXTRA_NAMES)
except NameError as ne:
    pass
