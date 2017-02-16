import serial_command
import argparse
import time


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-n", dest="pulse_num", type=int,
                        help="pulse_number")
    parser.add_argument("-p", dest="pulse_height", type=int,
                        help="pulse_height")
    parser.add_argument("-d", dest="pulse_delay", type=float,
                        help="pulse_number")
    args = parser.parse_args()
    sc = serial_command.SerialCommand()
    sc.set_pulse_height(args.pulse_height)
    sc.set_pulse_number(1000)
    sc.set_pulse_delay(args.pulse_delay)
    npulses = 0
    sc.fire()
    npulses += 1000.
    if args.pulse_num % 1000. != 0.:
        raise ValueError("Pulse Number must be a multiple of 10000")
    fire = True
    while fire and npulses < args.pulse_num:
        if sc.check_firing():
            pass
        else:
            npulses += 1000.
            sc.fire()
    print "Number of pulses", npulses, "time active", args.pulse_delay*1e-3*npulses

