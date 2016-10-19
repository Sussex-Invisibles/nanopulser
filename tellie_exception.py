#!/usr/bin/env python
#
# tellie_exception:
#
# TellieException
#
# Exceptions raised by the tellie modules
#
# Author: Matt Mottram
#         <m.mottram@sussex.ac.uk>
#
# History:
# 2013/03/08: First instance
#
###########################################
###########################################

class TellieException(Exception):
    """General exception for the Tellie command modules"""

    def __init__(self, error):
        Exception.__init__(self, error)


class TellieSerialException(Exception):
    """Exception when communicating with the Serial Port"""

    def __init__(self, error):
        Exception.__init__(self, error)


class ThreadException(Exception):
    """Exception raised specific to threading issues"""

    def __init__(self, error):
        Exception.__init__(self, error)
