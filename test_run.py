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
    sc.set_pulse_number(args.pulse_num)
    sc.set_pulse_delay(args.pulse_delay)
    sc.fire()
    fire = True
    while fire:
        if sc.check_firing():
            pass
        else:
            sc.fire()

