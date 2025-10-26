#!/usr/bin/env python3
import time
from rrclite_sdk import Board  # replace 'rrclite_sdk' with the filename where your SDK code is

def main():
    # Initialize board on correct USB port
    board = Board(device="/dev/ttyUSB1")
    board.enable_reception(True)  # start receiving feedback from board

    # Blink LED: 0.2s on, 0.2s off, repeat 3 times
    board.set_led(on_time=0.2, off_time=0.2, repeat=3, led_id=1)

    # Buzzer beep: 1500Hz, 0.3s on, 0.1s off, repeat 2 times
    board.set_buzzer(freq=1500, on_time=0.3, off_time=0.1, repeat=2)

    print("Commands sent: LED blink and buzzer beep!")

    # Keep script alive to ensure commands are sent
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting...")

if __name__ == "__main__":
    main()
