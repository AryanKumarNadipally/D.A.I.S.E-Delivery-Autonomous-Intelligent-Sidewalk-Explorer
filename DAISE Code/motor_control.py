import RPi.GPIO as GPIO
import time
import threading

motor_pins = {
    "motor1": {"in1": 17, "in2": 18, "en": 22, "enc_a": 27, "enc_b": 4},  # Back Left
    "motor2": {"in1": 23, "in2": 24, "en": 25, "enc_a": 5, "enc_b": 6},   # Back Right
    "motor3": {"in1": 5, "in2": 6, "en": 13, "enc_a": 12, "enc_b": 16},   # Front Left
    "motor4": {"in1": 19, "in2": 26, "en": 21, "enc_a": 20, "enc_b": 21}  # Front Right
}

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0

    def compute(self, setpoint, measured_value):
        error = setpoint - measured_value
        self.integral += error
        derivative = error - self.previous_error
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.previous_error = error
        return output

# Set GPIO mode
GPIO.setmode(GPIO.BCM)

# Initialize GPIO pins
for motor, pins in motor_pins.items():
    GPIO.setup(pins["in1"], GPIO.OUT)
    GPIO.setup(pins["in2"], GPIO.OUT)
    GPIO.setup(pins["en"], GPIO.OUT)
    GPIO.setup(pins["enc_a"], GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(pins["enc_b"], GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.output(pins["in1"], GPIO.LOW)
    GPIO.output(pins["in2"], GPIO.LOW)
    GPIO.output(pins["en"], GPIO.LOW)

# Initialize PWM on enable pins
pwm = {}
for motor, pins in motor_pins.items():
    pwm[motor] = GPIO.PWM(pins["en"], 100)  # 100 Hz frequency
    pwm[motor].start(0)  # Start with duty cycle of 0

# Motor control functions
def set_motor_speed(motor, speed):
    if motor in motor_pins:
        pwm[motor].ChangeDutyCycle(speed)

def set_motor_direction(motor, direction):
    if motor in motor_pins:
        if direction == "forward":
            GPIO.output(motor_pins[motor]["in1"], GPIO.HIGH)
            GPIO.output(motor_pins[motor]["in2"], GPIO.LOW)
        elif direction == "backward":
            GPIO.output(motor_pins[motor]["in1"], GPIO.LOW)
            GPIO.output(motor_pins[motor]["in2"], GPIO.HIGH)

def move_forward(speed):
    for motor in motor_pins:
        set_motor_direction(motor, "forward")
        set_motor_speed(motor, speed)

def move_backward(speed):
    for motor in motor_pins:
        set_motor_direction(motor, "backward")
        set_motor_speed(motor, speed)

def turn_left(speed):
    set_motor_direction("motor1", "backward")
    set_motor_direction("motor2", "forward")
    set_motor_direction("motor3", "backward")
    set_motor_direction("motor4", "forward")
    set_motor_speed("motor1", speed)
    set_motor_speed("motor2", speed)
    set_motor_speed("motor3", speed)
    set_motor_speed("motor4", speed)

def turn_right(speed):
    set_motor_direction("motor1", "forward")
    set_motor_direction("motor2", "backward")
    set_motor_direction("motor3", "forward")
    set_motor_direction("motor4", "backward")
    set_motor_speed("motor1", speed)
    set_motor_speed("motor2", speed)
    set_motor_speed("motor3", speed)
    set_motor_speed("motor4", speed)

def stop():
    for motor in motor_pins:
        set_motor_speed(motor, 0)

def cleanup():
    for motor in pwm:
        pwm[motor].stop()
    GPIO.cleanup()

# PID Controllers for each motor
pid_controllers = {
    "motor1": PIDController(1.0, 0.1, 0.05),
    "motor2": PIDController(1.0, 0.1, 0.05),
    "motor3": PIDController(1.0, 0.1, 0.05),
    "motor4": PIDController(1.0, 0.1, 0.05)
}

# Placeholder for actual speed from encoder
actual_speeds = {
    "motor1": 0,
    "motor2": 0,
    "motor3": 0,
    "motor4": 0
}

# Encoder pulse counts
encoder_counts = {
    "motor1": 0,
    "motor2": 0,
    "motor3": 0,
    "motor4": 0
}

# Encoder pulse counts per revolution
PULSES_PER_REV = 20  

# Encoder callback functions
def encoder_callback_motor1(channel):
    encoder_counts["motor1"] += 1

def encoder_callback_motor2(channel):
    encoder_counts["motor2"] += 1

def encoder_callback_motor3(channel):
    encoder_counts["motor3"] += 1

def encoder_callback_motor4(channel):
    encoder_counts["motor4"] += 1

# Attach interrupt handlers for encoders
GPIO.add_event_detect(motor_pins["motor1"]["enc_a"], GPIO.BOTH, callback=encoder_callback_motor1)
GPIO.add_event_detect(motor_pins["motor2"]["enc_a"], GPIO.BOTH, callback=encoder_callback_motor2)
GPIO.add_event_detect(motor_pins["motor3"]["enc_a"], GPIO.BOTH, callback=encoder_callback_motor3)
GPIO.add_event_detect(motor_pins["motor4"]["enc_a"], GPIO.BOTH, callback=encoder_callback_motor4)

# Function to update actual speeds from encoders
def update_actual_speeds():
    for motor in encoder_counts:
        actual_speeds[motor] = (encoder_counts[motor] / PULSES_PER_REV) * 60  # RPM

# Control loop for all motors
def control_motors(desired_speeds):
    while True:
        update_actual_speeds()
        for motor in desired_speeds:
            setpoint = desired_speeds[motor]
            measured_value = actual_speeds[motor]
            pid_output = pid_controllers[motor].compute(setpoint, measured_value)
            set_motor_speed(motor, pid_output)
        time.sleep(0.1)

if __name__ == "__main__":
    try:
        move_forward(50)  # Move forward at 50% speed
        time.sleep(5)  # Move forward for 5 seconds
        turn_left(50)  # Turn left at 50% speed
        time.sleep(2)  # Turn left for 2 seconds
        move_backward(50)  # Move backward at 50% speed
        time.sleep(5)  # Move backward for 5 seconds
        stop()  # Stop the robot
    except KeyboardInterrupt:
        pass
    finally:
        cleanup()
