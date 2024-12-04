import time
from time import sleep
import machine
import math
from machine import Pin, ADC, PWM
# For help: https://docs.micropython.org/en/latest/library/machine.PWM.html

# From micropython-servo 1.0.1 on PyPi.org
class Servo:
        def __init__(self, pin_id, min_us=544.0, max_us=2400.0, min_deg=0.0, max_deg=180.0, freq=50):
                self.pwm = machine.PWM(machine.Pin(pin_id))
                self.pwm.freq(freq)
                self.current_us = 0.0
                self._slope = (min_us - max_us) / (math.radians(min_deg) - math.radians(max_deg))
                self._offset = min_us

        def write(self, deg):
                self.write_rad(math.radians(deg))

        def read(self):
                return math.degrees(self.read_rad())

        def write_rad(self, rad):
                self.write_us(rad * self._slope + self._offset)

        def read_rad(self):
                return (self.current_us - self._offset) / self._slope

        def write_us(self, us):
                self.current_us = us
                self.pwm.duty_ns(int(self.current_us * 1000.0))

        def read_us(self):
                return self.current_us

        def off(self):
                self.pwm.duty_ns(0)

# From DIYables-MicroPython-Joystick 0.1.0 on PyPi.org
class Joystick:
        def __init__(self, pin_x, pin_y, pin_button, mode=Pin.PULL_UP):
                # Initialize the button
                self.btn_pin = Pin(pin_button, Pin.IN, mode)
                self.mode = mode
                self.debounce_time = 0
                self.count = 0
                self.count_mode = 'COUNT_FALLING'

                # Set initial state based on pull mode
                if self.mode == Pin.PULL_DOWN:
                        self.unpressed_state = 0
                        self.pressed_state = 1
                else:
                        self.unpressed_state = 1
                        self.pressed_state = 0

                self.previous_steady_state = self.btn_pin.value()
                self.last_steady_state = self.previous_steady_state
                self.last_flickerable_state = self.previous_steady_state
                self.last_debounce_time = 0

                # Initialize ADC for X and Y axes if pins are provided
                if pin_x is not None:
                        self.adc_x = ADC(Pin(pin_x))
                if pin_y is not None:
                        self.adc_y = ADC(Pin(pin_y))

        def set_debounce_time(self, time_ms):
                self.debounce_time = time_ms

        def read_button_state(self):
                return self.last_steady_state

        def is_pressed(self):
                return self.previous_steady_state == self.unpressed_state and self.last_steady_state == self.pressed_state

        def is_released(self):
                return self.previous_steady_state == self.pressed_state and self.last_steady_state == self.unpressed_state

        def set_press_count_mode(self, mode):
                if mode in ['COUNT_BOTH', 'COUNT_FALLING', 'COUNT_RISING']:
                        self.count_mode = mode

        def get_press_count(self):
                return self.count

        def reset_press_count(self):
                self.count = 0

        def read_x(self):
                if hasattr(self, 'adc_x'):
                        return self.adc_x.read_u16() >> 4  # Shift the bits right by 4, this is to convert back to 12-bit ADC of Pico
                else:
                        return None

        def read_y(self):
                if hasattr(self, 'adc_y'):
                        return self.adc_y.read_u16() >> 4  # Shift the bits right by 4, this is to convert back to 12-bit ADC of Pico
                else:
                        return None

        def loop(self):
                current_state = self.btn_pin.value()
                current_time = time.ticks_ms()

                if current_state != self.last_flickerable_state:
                        self.last_debounce_time = current_time
                        self.last_flickerable_state = current_state

                if time.ticks_diff(current_time, self.last_debounce_time) >= self.debounce_time:
                        self.previous_steady_state = self.last_steady_state
                        self.last_steady_state = current_state

                if self.previous_steady_state != self.last_steady_state:
                        if self.count_mode == 'COUNT_BOTH':
                                self.count += 1
                        elif (self.count_mode == 'COUNT_FALLING' and
                              self.previous_steady_state == self.pressed_state and
                              self.last_steady_state == self.unpressed_state):
                                self.count += 1
                        elif (self.count_mode == 'COUNT_RISING' and
                              self.previous_steady_state == self.unpressed_state and
                              self.last_steady_state == self.pressed_state):
                                self.count += 1

# Servo Pins
servo_pin = Pin(0)
servo2_pin = Pin(1)
servo3_pin = Pin(2)


# Initialize Servos
servo1 = Servo(servo_pin)
servo2 = Servo(servo2_pin)
servo3 = Servo(servo3_pin)

# Joystick Pins (ADC Pins)
x_Pin = 26
y_Pin = 27
button = 28

# Initialize Joystick
joystick = Joystick(x_Pin, y_Pin, button)

# Define variables
x_val = 0
y_val = 0
x_normalized = 0.0
y_normalized = 0.0
buttonPressed = False
handClosed = False

# Constants for Thresholds
X_NEGATIVE_THRESHOLD = 1000
X_POSITIVE_THRESHOLD = 3000
Y_NEGATIVE_THRESHOLD = 1000
Y_POSITIVE_THRESHOLD = 3000

while True:
        # Process Button Debouncing
        joystick.loop()

        # Update values
        # For arm servos (0 to 4095)
        x_val = joystick.read_x()
        y_val = joystick.read_y()

        # Normalize x, y values
        # Check For Thresholds
        # if x_val > X_POSITIVE_THRESHOLD:
        #         x_normalized = 180
        # elif x_val < X_NEGATIVE_THRESHOLD:
        #         x_normalized = 0
        # else:
        #         x_normalized = ((x_val - 1000) / 2000) * 180
        x_normalized = (x_val / 4095) * 180  # Change to value between 0 and 1
        y_normalized = (y_val / 4095) * 180  # Multiply by 180 to get value b/w 0-180 deg

        # Move the arm
        servo1.write(x_normalized)
        servo2.write(y_normalized)

        # For hand servo
        buttonPressed = joystick.is_pressed()

        if buttonPressed:
                handClosed = not handClosed # closes if open and opens if closed

        # if handClosed: # closed
        if buttonPressed and handClosed:
                servo3.write(180)
        # if not handClosed: # open
        elif buttonPressed and not handClosed:
                servo3.write(0)
