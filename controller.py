import time
import pigpio  # Required for servo PWM on Raspberry Pi

# Constants
SERVO1_PIN = 12  # GPIO18 (physical pin 12), hardware PWM-capable
SERVO_FREQ = 50  # 50 Hz for servos

# Initialize pigpio
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("Could not connect to pigpio daemon")
print("[DEBUG] pigpio daemon connected")

# Setup servo pin
pi.set_mode(SERVO1_PIN, pigpio.OUTPUT)
print(f"[DEBUG] Servo pin initialized: {SERVO1_PIN}")

def angle_to_pulse(angle):
    # Converts 0-180 degrees to pulse width in microseconds (500–2500us)
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

def control_camera_tilt(angle):
    print(f"[DEBUG] control_camera_tilt called with angle={angle}")
    set_servo_angle(SERVO1_PIN, angle)

def cleanup():
    print("[DEBUG] cleanup called")
    pi.set_servo_pulsewidth(SERVO1_PIN, 0)
    pi.stop()
    print("[DEBUG] pigpio stopped and cleaned up")

# Example usage (safe to remove or modify)
if __name__ == "__main__":
    try:
        control_camera_tilt(90)  # Move to center
        time.sleep(2)
        control_camera_tilt(0)   # Move to one end
        time.sleep(2)
        control_camera_tilt(180) # Move to other end
        time.sleep(2)
    finally:
        cleanup()