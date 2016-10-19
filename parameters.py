#######################
# parameters.py:
#  Any parameters that may need checking
#
import math

max_pulse_number = 65025
max_fibre_delay = 63.75 #ns
max_trigger_delay = 1275 #ns


def pulse_number(number):
    adjusted = False
    if type(number)!=int:
        raise Exception("PN must be an integer")
    if number > max_pulse_number:
        raise Exception("PN must be < %d.  You set %d" % (65025, number))
        #number = max_pulse_number
        #adjusted = True
    hi = -1
    lo = -1
    diff = 100000 # bigger than max pn
    for i in range(1, 256):
        #assume hi is i
        lo_check = number/i
        if lo_check > 255:
            lo_check = 255
        check = i * lo_check
        if math.fabs(check - number) < diff:
            diff = math.fabs(check - number)
            hi = i
            lo = lo_check
        if check == number:
            break
    actual_par = hi * lo
    if actual_par != number:
        adjusted = True
    return adjusted, actual_par, hi, lo


def trigger_delay(delay):
    adjusted = False
    delay = float(delay)
    if delay > max_trigger_delay or delay < 0:
        raise Exception("TD must be >%s and <%s" % (0, max_trigger_delay))
    parameter = int(round(delay)/5)
    adj_delay = parameter * 5
    if delay != adj_delay:
        adjusted = True
    return adjusted, adj_delay, parameter


def fibre_delay(delay):
    adjusted = False
    delay = float(delay)
    if delay > max_fibre_delay or delay < 0:
        raise Exception("FD must be >%s and <%s" % (0, max_fibre_delay))
    parameter = int(round(delay * 4.))
    adj_delay = float(parameter) / 4.
    if delay != adj_delay:
        adjusted = True
    return adjusted, adj_delay, parameter
