#!/usr/bin/env python
#
# serial_command
#
# SerialCommand
#
# Command functions for controlling comercial
# LED driver circuit
#
# Author: <j.waterfield@sussex.ac.uk>
#
# History:
# 2016/10/10: First instance
###########################################
###########################################

import serial
import re
import sys
import time
import tellie_exception
import tellie_logger
import parameters


_max_pulse_height = 16383
_max_pulse_width = 16383
_max_lo = 255.
_max_pulse_delay = 256.020
_min_pulse_delay = 0.1
_max_trigger_delay = 1275
_max_fibre_delay = 127.5
_max_pulse_number = 65025
_max_pulse_number_upper = 255
_max_pulse_number_lower = 255
_max_temp_probe = 64

_cmd_fire_continuous = "a"
_cmd_read_single_lower = "r"
_cmd_read_single_upper = "m"
_cmd_fire_average_lower = "s"
_cmd_fire_average_upper = "U"
_cmd_fire_series = "g"
_buffer_end_sequence = "K"
_cmd_stop = "X"
_cmd_channel_clear = "C"
_cmd_channel_select_single_start = "I"
_cmd_channel_select_single_end = "N"
_cmd_channel_select_many_start = "J"
_cmd_channel_select_many_end = "E"
_cmd_ph_hi = "L"
_cmd_ph_lo = "M"
_cmd_ph_end = "P"
_cmd_pw_hi = "Q"
_cmd_pw_lo = "R"
_cmd_pw_end = "S"
_cmd_pn_hi = "H"
_cmd_pn_lo = "G"
_cmd_pd = "u"
_cmd_td = "d"
_cmd_fd = "e"
_cmd_temp_select_lower = "n"
_cmd_temp_read_lower = "T"
_cmd_temp_select_upper = "f"
_cmd_temp_read_upper = "k"
_cmd_disable_ext_trig = "B"
_cmd_enable_ext_trig = "A"
_cmd_fire_average_ext_trig_lower = "p"
_cmd_fire_average_ext_trig_upper = "b"
_cmd_fire_ext_trig = "F"


class SerialCommand(object):
    """Serial command object.
    Base class, different chips then inheret from this.
    """

    def __init__(self, port_name=None):
        """Initialise the serial command"""
        if not port_name:
# find portname with: python -m serial.tools.list_ports
 #           self._port_name = "/dev/ttyUSB0" # linux
             self._port_name = "COM4"         # windows
        else:
            self._port_name = port_name
        # This is the same as a sleep, but with the advantage of breaking
        # if enough characters are seen in the buffer.
        self._port_timeout = 0.3
        self._serial = None
        self.logger = tellie_logger.TellieLogger.get_instance()
        try:
            self._serial = serial.Serial(port=self._port_name,
                                         timeout=self._port_timeout)
            self.logger.debug("Serial connection open: %s" % self._serial)
        except serial.SerialException, e:
            message = ("Could not find port %s\n Find portname with: "
                       "python -m serial.tools.list_ports" %(self._port_name))
            raise tellie_exception.TellieSerialException(e)
        # cache current settings - remove need to re-command where possible
        self._current_ph = None
        self._current_fd = None
        self._current_pn = None
        self._current_pd = None
        self._current_td = None
        # information on whether the channel is being fired
        self._firing = False  # must wait for firing to complete
        # if a new channel is selected should force setting all new parameters
        # restriction only lifted once a fire command has been called
        self._force_setting = False
        # send a reset, to ensure the RTS is set to false
        self.reset()
        # Slave mode can be re-instated later if required.
        #self._send_command(_cmd_disable_ext_trig)

    def __del__(self):
        """Deletion function"""
        if self._serial:
            # Stop accecpting external trigs
            #self._send_command(_cmd_disable_ext_trig)
            self._serial.close()

    def _check_clear_buffer(self):
        """Many commands expect an empty buffer, fail if they are not!
        """
        buffer_read = self._serial.read(100)
        if buffer_read != "":
            raise tellie_exception.TellieException(
                "Buffer not clear: %s" % (buffer_read))

    def _send_command(self, command, readout=True, buffer_check=None):
        """Send a command to the serial port.
        Command can be a chr/str (single write) or a list.
        Lists are used for e.g. a high/low bit command where
        the high bit could finish with an endline (i.e. endstream)"""
        self.logger.debug("_send_command:%s" % command)
        if type(command) is str:
            command = [command]
        if type(command) is not list:
            raise tellie_exception.TellieException(
                "Command is not a list: %s %s" % (command, type(command)))
        try:
            for c in command:
                self._serial.write(c)
        except:
            raise tellie_exception.TellieException(
                "Lost connection with TELLIE control!")
        if not buffer_check:  # assume returns same as input
            buffer_check = ''
            for c in command:
                buffer_check += c
        if readout is True:
            # One read command (with default timeout of 0.1s) should be
            # enough to get all the chars from the readout.
            buffer_read = self._serial.read(len(buffer_check))
            if str(buffer_read) != str(buffer_check):
                self.logger.debug(
                    "problem reading buffer, send %s, read %s" % (command,
                                                                  buffer_read))
                # clear anything else that might be in there
                time.sleep(0.1)
                remainder = self._serial.read(100)
                self._serial.write("X")  # send a stop
                time.sleep(0.1)
                self._serial.write("C")  # send a clear
                time.sleep(0.1)
                self._serial.read(100)
                message = ("Unexpected buffer output:\nsaw: %s, remainder "
                           "%s\nexpected: %s" % (buffer_read, remainder,
                                                 buffer_check))
                self.logger.warn(message)
                raise tellie_exception.TellieException(message)
            else:
                self.logger.debug("success reading buffer:%s" % buffer_read)
        else:
            self.logger.debug("not a readout command")

    def _send_setting_command(self, command, buffer_check=None,
                              while_fire=False):
        """Send non-firing command.
        All of these should have a clear buffer before being used.  Can set
        while_fire to True to allow a non-fire command to be sent while firing
        (will cause PIN readout to be flushed to buffer).
        """
        self.logger.debug("Send non-firing command")
        if self._firing is True:
            if while_fire is False:
                raise tellie_exception.TellieException(
                    "Cannot run command, in firing mode")
            else:
                # Assume that we CANNOT readout the buffer here!
                self._send_command(command=command, readout=False)
        else:
            self._check_clear_buffer()
            self._send_command(command=command, buffer_check=buffer_check)

    def reset(self):
        """Send a reset command!

        Assumes that the port is open (which it is by default)
        """
        self.logger.debug("Reset!")
        self._serial.setRTS(True)
        # sleep, just in case
        time.sleep(3.0)
        self._serial.setRTS(False)
        # close the port and reopen?
        time.sleep(3.0)

    def check_firing(self):
        if self._firing:
            if _buffer_end_sequence in self.read_buffer():
                self._firing = False
        return self._firing

    def enable_external_trig(self, while_fire=False):
        """Tell TELLIE to fire on any external trigger.

        Can send a fire command while already in fire mode if required."""
        self.logger.debug("Enable ext triggering mode")
        if self._firing is True and while_fire is False:
            raise tellie_exception.TellieException(
                "Cannot set ext. trig, already in firing mode")
        self._send_command(_cmd_enable_ext_trig)

    def trigger_single(self):
        """Fire single pulse upon receiving an external trigger.

        """
        if self._firing is True:
            if _buffer_end_sequence in self.read_buffer():
                self._firing = False
            else:
                raise tellie_exception.TellieException(
                    "Cannot fire, already in firing mode")
        self._send_command(_cmd_fire_ext_trig, False)
        self._firing = True

    def fire(self, while_fire=False):
        """Fire tellie, place class into firing mode.
        Can send a fire command while already in fire mode if required."""
        self.logger.debug("Fire!")
        if self._firing is True and while_fire is False:
            if _buffer_end_sequence in self.read_buffer():
                self._firing = False
            else:
                raise tellie_exception.TellieException(
                    "Cannot fire, already in firing mode")
        self.check_ready()
        buffer_check = _cmd_fire_series
        if (self._current_pn * self._current_pd) < 500:
            buffer_check += _buffer_end_sequence
            self._send_command(_cmd_fire_series, buffer_check=buffer_check)
        else:
            self._send_command(_cmd_fire_series, buffer_check=buffer_check)
            self._firing = True
        self._force_setting = False

    def fire_sequence(self, while_fire=False):
        """Fire in sequence mode, can only be done for a single channel.
        """
        self.logger.debug("Fire sequence!")
        self.check_ready()
        self._send_command(_cmd_fire_series, False)
        self._firing = True
        self._force_setting = False

    def fire_single(self):
        """Fire single pulse
        """
        if self._firing is True:
            if _buffer_end_sequence in self.read_buffer():
                self._firing = False
            else:
                raise tellie_exception.TellieException(
                    "Cannot fire, already in firing mode")
        if self._channel <= 56:  # up to box 7
            cmd = _cmd_read_single_lower
        else:
            cmd = _cmd_read_single_upper
        self._send_command(cmd, False)
        self._firing = True
        pin = self.read_pin(self._channel[0])
        while not pin:
            pin = self.read_pin(self._channel[0])
        return pin

    def fire_continuous(self):
        """Fire Tellie in continous mode.
        """
        if self._firing is True:
            if _buffer_end_sequence in self.read_buffer():
                self._firing = False
            else:
                raise tellie_exception.TellieException(
                    "Cannot fire, already in firing mode")
        self._send_command(_cmd_fire_continuous, False)
        self._firing = True
        self._force_setting = False

    def read_buffer(self, n=100):
        return self._serial.read(n)

    def stop(self):
        """Stop firing tellie"""
        self.logger.debug("Stop firing!")
        self._send_command(_cmd_stop, False)
        buffer_contents = self._serial.read(100)
        self._firing = False
        return buffer_contents

    def check_ready(self):
        """Check that all settings have been set"""
        not_set = []
        if self._current_ph is None:
            not_set += ["Pulse height"]
        #if self._current_fd is None:
        #    not_set += ["Fibre delay"]
        if self._current_pn is None:
            not_set += ["Pulse number"]
        if self._current_pd is None:
            not_set += ["Pulse delay"]
        #if self._current_td is None:
        #    not_set += ["Trigger delay"]
        if not_set != []:
            raise tellie_exception.TellieException(
                "Undefined options: %s" % (", ".join(opt for opt in not_set)))

    def clear_settings(self):
        """Clear settings that affect all channels"""
        self._current_pw = None
        self._current_ph = None
        self._current_fd = None
        self._current_pn = None
        self._current_pd = None
        self._current_td = None

    def set_pulse_height(self, par):
        """Set the pulse height for the selected channel"""
        if par == self._current_ph and not self._force_setting:
            pass  # same as current setting
        else:
            self.logger.debug("Set pulse height %s %s" % (par, type(par)))
            command, buffer_check = command_pulse_height(par)
            self._send_setting_command(command=command,
                                       buffer_check=buffer_check)
            self._current_ph = par

    def set_fibre_delay(self, par):
        """Set the fibre (channel) delay for the selected channel"""
        if len(self._channel) != 1:
            raise tellie_exception.TellieException(
                "Cannot set parameter with channels set as %s" %
                (self._channel))
        if par == self._current_fd[self._channel[0]] and \
           not self._force_setting:
            pass
        else:
            self.logger.debug("Set Fibre delay %s %s" % (par, type(par)))
            command, buffer_check = command_fibre_delay(par)
            self._send_channel_setting_command(command=command,
                                               buffer_check=buffer_check)
            self._current_fd[self._channel[0]] = par

    def set_pulse_number(self, par):
        """Set the number of pulses to fire (global setting)"""
        if par == self._current_pn and not self._force_setting:
            pass
        else:
            self.logger.debug("Set pulse number %s %s" % (par, type(par)))
            command, buffer_check = command_pulse_number(par)
            self._send_setting_command(command=command,
                                       buffer_check=buffer_check)
            self._current_pn = par

    def set_pulse_delay(self, par):
        """Set the delay between pulses (global setting)"""
        if par == self._current_pd and not self._force_setting:
            pass
        else:
            self.logger.debug("Set pulse delay %s %s" % (par, type(par)))
            command, buffer_check = command_pulse_delay(par)
            self._send_setting_command(command=command,
                                       buffer_check=buffer_check)
            self._current_pd = par

    def set_trigger_delay(self, par):
        """Set the trigger delay (global setting)"""
        if par == self._current_td and not self._force_setting:
            pass
        else:
            self.logger.debug("Set trigger delay %s %s" % (par, type(par)))
            command, buffer_check = command_trigger_delay(par)
            self._send_setting_command(command=command,
                                       buffer_check=buffer_check)
            self._current_td = par

    def disable_external_trigger(self):
        """Disable the external trigger"""
        self._send_command(command="B")


##################################################
# Command options and corresponding buffer outputs
#

def command_pulse_height(par):
    """Get the command to set a pulse height"""
    if par > _max_pulse_height or par < 0:
        raise tellie_exception.TellieException(
            "Invalid pulse height: %s" % par)
    hi = par >> 8
    lo = par & 255
    command = [_cmd_ph_hi+chr(hi)]
    command += [_cmd_ph_lo+chr(lo)]
    command += [_cmd_ph_end]
    buffer_check = _cmd_ph_hi + _cmd_ph_lo + _cmd_ph_end
    return command, buffer_check


def command_pulse_width(par):
    """Get the command to set a pulse width"""
    if par > _max_pulse_width or par < 0:
        raise tellie_exception.TellieException(
            "Invalid pulse width: %s %s %s" % (par,
                                               _max_pulse_width,
                                               par > _max_pulse_width))
    hi = par >> 8
    lo = par & 255
    command = [_cmd_pw_hi+chr(hi)]
    command += [_cmd_pw_lo+chr(lo)+_cmd_pw_end]
    buffer_check = _cmd_pw_hi + _cmd_pw_lo + _cmd_pw_end
    return command, buffer_check


def command_pulse_number(par):
    """Get the command to set a pulse number"""
    if par > _max_pulse_number or par < 0:
        raise tellie_exception.TellieException("Invalid pulse number: %s" %
                                               (par))
    par = int(par)
    adjusted, actual_par, hi, lo = parameters.pulse_number(par)
    if adjusted is True:
        raise tellie_exception.TellieException("Invalid pulse number: %s" %
                                               (par))
    command = [_cmd_pn_hi+chr(hi)]
    command += [_cmd_pn_lo+chr(lo)]
    buffer_check = _cmd_pn_hi + _cmd_pn_lo
    return command, buffer_check


def command_pulse_delay(par):
    """Get the command to set a pulse delay"""
    if par > _max_pulse_delay or par < 0:
        raise tellie_exception.TellieException("Invalid pulse delay: %s" % par)
    ms = int(par)
    us = int((par-ms)*250)
    command = [_cmd_pd+chr(ms)]
    command += [chr(us)]
    buffer_check = _cmd_pd
    return command, buffer_check


def command_trigger_delay(par):
    """Get the command to set a trigger delay"""
    if par > _max_trigger_delay or par < 0:
        raise tellie_exception.TellieException("Invalid trigger delay: %s" %
                                               par)
    command = [_cmd_td+chr(par/5)]
    buffer_check = _cmd_td
    return command, buffer_check


def command_fibre_delay(par):
    """Get the command to set a fibre delay"""
    if par > _max_fibre_delay or par < 0:
        raise tellie_exception.TellieException("Invalid fibre delay: %s" %
                                               par)
    adjusted, adj_delay, setting = parameters.fibre_delay(par)
    print "COMMAND", par, adjusted, adj_delay, setting
    if adjusted is True:
        raise tellie_exception.TellieException("Invalid delay: %s" %
                                               (par))
    command = [_cmd_fd+chr(setting)]
    buffer_check = _cmd_fd
    return command, buffer_check
