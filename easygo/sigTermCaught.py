#!/usr/bin/python

from time import sleep
import signal
import sys


def sigterm_handler(_signo, _stack_frame):
    # Raises SystemExit(0):
    sys.exit(0)

if sys.argv[1] == "handle_signal":
    signal.signal(signal.SIGTERM, sigterm_handler)

try:
    print "Hello"
    i = 0
    while True:
        i += 1
        print "Iteration #%i" % i
        sleep(1)
finally:
    print "Goodbye"
