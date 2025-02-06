
import time

# ===================================================
# 1. PID Controller Class Definition
# ===================================================
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.previous_error = error
        return output

# ===================================================
# 2. Sensor Conversion & Desired State Functions
# ===================================================
def convert_wheel_rotations_to_speed(rotations):
    # Example conversion (replace with the actual conversion through testing)
    speed = rotations * 1.0  # Dummy conversion factor
    return speed

def convert_displacement_to_angle(displacement):
    calibration_factor = 0.1  # Example calibration factor
    return displacement * calibration_factor

def get_desired_speed():
    # Path planning looking at you here :D
    return 30  # e.g., 30 km/h

def get_desired_steering_angle():
    # Path planning looking at you here :D
    return 0  # e.g., 0 degrees

# ===================================================
# 3. Actuator Command Functions (Stubs)
# ===================================================
def set_throttle(value):
    # Command the throttle actuator (value typically normalized 0.0 to 1.0)
    print(f"[ACTUATOR] Throttle set to: {value:.2f}")

def set_brake(value):
    # Command the brake actuator (value typically normalized 0.0 to 1.0)
    print(f"[ACTUATOR] Brake set to: {value:.2f}")

def set_steering(value):
    # Command the steering actuator (e.g., degrees or a normalized value)
    print(f"[ACTUATOR] Steering set to: {value:.2f}")

def shift_gear(gear):
    # Command the transmission to shift to the given gear
    print(f"[ACTUATOR] Shifting gear to: {gear}")

def set_clutch(depressed):
    # Control the clutch actuator
    if depressed:
        # :(
        print("[ACTUATOR] Clutch depressed")
    else:
        print("[ACTUATOR] Clutch released")

def set_clutch_level(level):
    # Set the clutch engagement level (0.0 = disengaged, 1.0 = fully engaged)
    print(f"[ACTUATOR] Setting clutch level to: {level:.2f}")

def get_normal_throttle_output():
    # Return the throttle output computed by your normal control (PID) logic
    return 0.5  # Dummy value

# ===================================================
# 4. Sensor Read Functions 
# ===================================================
def read_engine_revs():
    # Read and return the engine RPM
    return 1200  # Dummy RPM value

def read_wheel_rotation_sensor():
    # Read and return wheel rotations over the control cycle
    return 10  # Dummy rotation count

def read_steering_displacement_sensor():
    # Read and return the steering rack displacement from its center
    return 5  # Dummy displacement value

# ===================================================
# 5. Logging Function 
# ===================================================
def log_status(speed, rpm, gear, throttle, brake, steering):
    print(f"[STATUS] Speed: {speed:.2f} | RPM: {rpm} | Gear: {gear} | "
          f"Throttle: {throttle:.2f} | Brake: {brake:.2f} | Steering: {steering:.2f}")

# ===================================================
# 6. Constants & Initialization
# ===================================================
# Gear definitions
NEUTRAL_GEAR = 0
MIN_GEAR = 1
MAX_GEAR = 6

# Gear shift thresholds (RPM)
SHIFT_UP_THRESHOLD = 5000    
SHIFT_DOWN_THRESHOLD = 2000  

# Throttle and engine RPM parameters
IDLE_RPM = 800               # Minimum engine RPM (idle)
MIN_SAFE_RPM = 900           # Minimum RPM to safely engage the clutch
IDLE_THROTTLE_LEVEL = 0.2    # Throttle level to keep engine running safely

# Clutch timing and release parameters
CLUTCH_RELEASE_STEPS = 10    
CLUTCH_STEP_DELAY = 0.05     # Delay between each clutch release step (seconds)
CLUTCH_FULL_DELAY = 0.15     # Delay to allow full clutch disengagement/engagement (seconds)

# Control loop time step
dt = 0.05  # 50 milliseconds

# Initialize PID controllers for throttle, brake, and steering
throttle_pid = PIDController(0.8, 0.1, 0.05)
brake_pid    = PIDController(0.9, 0.1, 0.05)
steering_pid = PIDController(1.0, 0.2, 0.1)

# Global variable for current gear
current_gear = NEUTRAL_GEAR

# ===================================================
# 7. Clutch & Gear Shift Coordination Functions
# ===================================================
def perform_gear_shift(target_gear):
    global current_gear
    # Step 1: Disengage the clutch
    set_clutch(depressed=True)
    time.sleep(CLUTCH_FULL_DELAY)
    
    # Step 2: Ensure engine RPM is safe
    while read_engine_revs() < MIN_SAFE_RPM:
        set_throttle(IDLE_THROTTLE_LEVEL)
        time.sleep(0.1)
    
    # Step 3: Shift gear
    shift_gear(target_gear)
    time.sleep(CLUTCH_FULL_DELAY)
    
    # Step 4: Gradually re-engage the clutch
    for step in range(1, CLUTCH_RELEASE_STEPS + 1):
        clutch_level = step / CLUTCH_RELEASE_STEPS
        set_clutch_level(clutch_level)
        current_rpm = read_engine_revs()
        if current_rpm < MIN_SAFE_RPM + 100:
            set_throttle(IDLE_THROTTLE_LEVEL + (0.1 * clutch_level))
        else:
            set_throttle(IDLE_THROTTLE_LEVEL)
        time.sleep(CLUTCH_STEP_DELAY)
    
    # Step 5: Finalize clutch engagement
    set_clutch(depressed=False)
    set_throttle(get_normal_throttle_output())
    current_gear = target_gear

def perform_neutral_to_first_shift():
    global current_gear
    # Step 1: Depress the clutch to disengage power
    set_clutch(depressed=True)
    time.sleep(CLUTCH_FULL_DELAY)
    
    # Step 2: Shift from Neutral to 1st gear
    shift_gear(MIN_GEAR)
    time.sleep(CLUTCH_FULL_DELAY)
    
    # Step 3: Gradually engage the clutch while increasing throttle
    for step in range(1, CLUTCH_RELEASE_STEPS + 1):
        clutch_level = step / CLUTCH_RELEASE_STEPS
        set_clutch_level(clutch_level)
        set_throttle(IDLE_THROTTLE_LEVEL + (0.2 * clutch_level))
        time.sleep(CLUTCH_STEP_DELAY)
    
    # Step 4: Finalize the shift
    set_clutch(depressed=False)
    current_gear = MIN_GEAR

def perform_first_to_neutral_shift():
    global current_gear
    # Step 1: Depress the clutch
    set_clutch(depressed=True)
    time.sleep(CLUTCH_FULL_DELAY)
    
    # Step 2: Shift from 1st to Neutral
    shift_gear(NEUTRAL_GEAR)
    time.sleep(CLUTCH_FULL_DELAY)
    
    # Step 3: Gradually release the clutch
    for step in range(1, CLUTCH_RELEASE_STEPS + 1):
        clutch_level = step / CLUTCH_RELEASE_STEPS
        set_clutch_level(clutch_level)
        time.sleep(CLUTCH_STEP_DELAY)
    
    # Step 4: Finalize the shift
    set_clutch(depressed=False)
    set_throttle(IDLE_THROTTLE_LEVEL)
    current_gear = NEUTRAL_GEAR

# ===================================================
# 8. Main Control Loop
# ===================================================
def main():
    global current_gear
    while True:
        # 8.1 Sensor Data Acquisition
        engine_revs = read_engine_revs()
        wheel_rotations = read_wheel_rotation_sensor()
        steering_displacement = read_steering_displacement_sensor()
        
        current_speed = convert_wheel_rotations_to_speed(wheel_rotations)
        current_steering_angle = convert_displacement_to_angle(steering_displacement)
        
        # 8.2 Obtain Desired States from ADS Module
        desired_speed = get_desired_speed()
        desired_steering_angle = get_desired_steering_angle()
        
        # 8.3 Special Cases: Starting and Stopping
        # If desired speed is zero and we're not in neutral, shift into neutral.
        if desired_speed == 0 and current_gear != NEUTRAL_GEAR:
            perform_first_to_neutral_shift()
        # If desired speed is greater than zero and we're in neutral, start by shifting into 1st.
        elif desired_speed > 0 and current_gear == NEUTRAL_GEAR:
            perform_neutral_to_first_shift()
        
        throttle_output = 0
        brake_output = 0
        steering_output = 0
        
        # 8.4 For speeds > 0 and engaged gears: PID Control
        if current_gear != NEUTRAL_GEAR:
            speed_error = desired_speed - current_speed
            steering_error = desired_steering_angle - current_steering_angle
            
            if speed_error > 0:
                throttle_output = throttle_pid.update(speed_error, dt)
                brake_output = 0
            else:
                throttle_output = 0
                brake_output = brake_pid.update(-speed_error, dt)
            
            steering_output = steering_pid.update(steering_error, dt)
            
            # 8.5 Gear Shifting Logic for Engaged Gears
            if engine_revs > SHIFT_UP_THRESHOLD and current_gear < MAX_GEAR:
                current_gear += 1
                perform_gear_shift(current_gear)
            elif engine_revs < SHIFT_DOWN_THRESHOLD and current_gear > MIN_GEAR:
                current_gear -= 1
                perform_gear_shift(current_gear)
            
            # 8.6 Send Commands to Actuators
            set_throttle(throttle_output)
            set_brake(brake_output)
            set_steering(steering_output)
        
        # 8.7 Log System Status (Optional)
        log_status(current_speed, engine_revs, current_gear, throttle_output,
        brake_output, steering_output)
        
        # 8.8 Wait for Next Control Cycle
        time.sleep(dt)
