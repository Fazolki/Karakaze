import pigpio
import time
import threading

# GPIO pin assignments - update as per your wiring
MOTOR1_PWM_PIN = 18  # hardware PWM pin
MOTOR1_DIR_PIN = 23

MOTOR2_PWM_PIN = 19  # hardware PWM pin
MOTOR2_DIR_PIN = 24

SERVO1_PIN = 12  # hardware PWM pin for servo 1 (camera tilt)
SERVO2_PIN = 13  # hardware PWM pin for servo 2 (gripper tilt)
SERVO3_PIN = 6   # hardware PWM pin for servo 3 (gripper open/close)

# Constants
SERVO_FREQ = 50  # Hz for servos
PWM_RANGE = 1000000  # pigpio PWM range for hardware PWM

# Initialize pigpio
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("Could not connect to pigpio daemon")
print("[DEBUG] pigpio daemon connected")

# Setup motor pins
# pi.set_mode(MOTOR1_PWM_PIN, pigpio.OUTPUT)
# pi.set_mode(MOTOR1_DIR_PIN, pigpio.OUTPUT)
# pi.set_mode(MOTOR2_PWM_PIN, pigpio.OUTPUT)
# pi.set_mode(MOTOR2_DIR_PIN, pigpio.OUTPUT)
print(f"[DEBUG] Motor pins initialized: PWM {MOTOR1_PWM_PIN}/{MOTOR2_PWM_PIN}, DIR {MOTOR1_DIR_PIN}/{MOTOR2_DIR_PIN}")

# Setup servo pins
pi.set_mode(SERVO1_PIN, pigpio.OUTPUT)
# pi.set_mode(SERVO2_PIN, pigpio.OUTPUT)
# pi.set_mode(SERVO3_PIN, pigpio.OUTPUT)
print(f"[DEBUG] Servo pins initialized: {SERVO1_PIN}, {SERVO2_PIN}, {SERVO3_PIN}")

def angle_to_pulse(angle):
    pulse = int(500 + (angle / 180.0) * 2000)
    print(f"[DEBUG] angle_to_pulse: angle={angle} -> pulse={pulse}")
    return pulse

def set_servo_angle(pin, angle):
    pulse = angle_to_pulse(angle)
    pi.set_servo_pulsewidth(pin, pulse)
    print(f"[DEBUG] set_servo_angle: pin={pin}, angle={angle}, pulse={pulse}")

def stop_servo(pin):
    pi.set_servo_pulsewidth(pin, 0)
    print(f"[DEBUG] stop_servo: pin={pin}")

def set_motor(pin_pwm, pin_dir, speed):
    direction = 1 if speed >= 0 else 0
    duty_cycle = int(abs(speed) * PWM_RANGE)  # scale to pigpio range

    # pi.write(pin_dir, direction)
    # pi.hardware_PWM(pin_pwm, 1000, duty_cycle)  # 1kHz PWM frequency

    print(f"[DEBUG] set_motor: PWM_pin={pin_pwm}, DIR_pin={pin_dir}, speed={speed}, direction={direction}, duty_cycle={duty_cycle}")

def drive_tank(left_speed, right_speed):
    print(f"[DEBUG] drive_tank called with left_speed={left_speed}, right_speed={right_speed}")
    set_motor(MOTOR1_PWM_PIN, MOTOR1_DIR_PIN, left_speed)
    set_motor(MOTOR2_PWM_PIN, MOTOR2_DIR_PIN, right_speed)

def control_camera_tilt(angle):
    print(f"[DEBUG] control_camera_tilt called with angle={angle}")
    set_servo_angle(SERVO1_PIN, angle)

def control_gripper_tilt(angle):
    print(f"[DEBUG] control_gripper_tilt called with angle={angle}")
    set_servo_angle(SERVO2_PIN, angle)

# Gripper angle and threading control
_gripper_angle = 0  # current angle of gripper servo (0=open, 90=closed)
_gripper_lock = threading.Lock()
_gripper_movement_thread = None
_gripper_moving = False

def _gripper_move_step(direction):
    """Increment or decrement gripper angle slowly while moving."""
    global _gripper_angle, _gripper_moving
    step = 1  # degrees per step
    interval = 0.1  # seconds between steps

    print(f"[DEBUG] Gripper move step thread started moving {direction}")

    while _gripper_moving:
        with _gripper_lock:
            if direction == "open":
                _gripper_angle = max(0, _gripper_angle - step)
            elif direction == "close":
                _gripper_angle = min(90, _gripper_angle + step)
            else:
                break

            set_servo_angle(SERVO3_PIN, _gripper_angle)
            print(f"[DEBUG] Gripper angle updated to {_gripper_angle}")

        time.sleep(interval)

    print("[DEBUG] Gripper move step thread exiting")

def start_gripper_move(direction):
    """Start continuous movement of gripper servo in given direction ('open' or 'close')."""
    global _gripper_moving, _gripper_movement_thread
    if _gripper_moving:
        print("[DEBUG] Gripper is already moving")
        return

    _gripper_moving = True
    _gripper_movement_thread = threading.Thread(target=_gripper_move_step, args=(direction,), daemon=True)
    _gripper_movement_thread.start()
    print(f"[DEBUG] Started gripper moving {direction}")

def stop_gripper_move():
    """Stop any ongoing gripper movement."""
    global _gripper_moving, _gripper_movement_thread
    if _gripper_moving:
        _gripper_moving = False
        if _gripper_movement_thread:
            _gripper_movement_thread.join(timeout=1)
        print("[DEBUG] Stopped gripper movement")
    else:
        print("[DEBUG] Gripper was not moving")

def control_gripper(action):
    print(f"[DEBUG] control_gripper called with action={action}")
    if action == "open":
        start_gripper_move("open")
    elif action == "close":
        start_gripper_move("close")
    elif action in ("open_release", "close_release", "stop"):
        stop_gripper_move()
    else:
        stop_servo(SERVO3_PIN)

# Encoder reading placeholders
_encoder_counts = [0, 0]

def encoder_callback(gpio, level, tick):
    _encoder_counts[0] += 1
    print(f"[DEBUG] encoder_callback triggered on GPIO {gpio}, level={level}, tick={tick}")

def get_encoder_counts():
    print(f"[DEBUG] get_encoder_counts called: {_encoder_counts}")
    return _encoder_counts

def play_audio(pcm_data):
    print("[DEBUG] play_audio called - PCM data length:", len(pcm_data) if pcm_data else 0)
    # Stub: actual DAC playback code needed here

def cleanup():
    print("[DEBUG] cleanup called")
    pi.set_servo_pulsewidth(SERVO1_PIN, 0)
    pi.set_servo_pulsewidth(SERVO2_PIN, 0)
    pi.set_servo_pulsewidth(SERVO3_PIN, 0)
    pi.hardware_PWM(MOTOR1_PWM_PIN, 0, 0)
    pi.hardware_PWM(MOTOR2_PWM_PIN, 0, 0)
    pi.stop()
    print("[DEBUG] pigpio stopped and cleaned up")