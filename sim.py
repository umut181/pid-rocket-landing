import pybullet as p
import pybullet_data
import time
import numpy as np
import matplotlib.pyplot as plt  # Make sure to install this: pip install matplotlib

# --- INITIALIZATION PARAMETERS ---
START_X = 1.0
START_Y = -1.0
START_Z = 4.0
START_ROLL = np.radians(30)
START_PITCH = np.radians(10)

# --- LANDING PARAMETERS ---
GROUND_CONTACT_HEIGHT = 0.723  # touch height (rocket base ~0.5 below COM + 0.223 leg height)
RAMP_START_HEIGHT     = 1    # begin throttle ramp-down at one meter altitude
has_landed = False

# --- PID CONTROLLER CLASS ---
class PIDController:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral_error = 0.0
        self.previous_error = 0.0
        
    def compute(self, target, current):
        error = target - current
        self.integral_error += error * self.dt
        derivative = (error - self.previous_error) / self.dt
        self.previous_error = error
        return (self.kp * error) + (self.ki * self.integral_error) + (self.kd * derivative)

# --- ENVIRONMENT INITIALIZATION ---
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
plane_id = p.loadURDF("plane.urdf")

# --- LANDING PAD MARKER 
pad_visual_id = p.createVisualShape(
    shapeType=p.GEOM_CYLINDER,
    radius=0.3,
    length=0.01,
    rgbaColor=[0.85, 0.05, 0.05, 1.0]
)

pad_id = p.createMultiBody(
    baseMass=0,                    # mass 0 -> static, ignores gravity automatically
    baseCollisionShapeIndex=-1,    # -1 = no collision shape, visual only
    baseVisualShapeIndex=pad_visual_id,
    basePosition=[0, 0, 0.005]     # half the disc's height, so it sits flush on the ground
)

# Spawning rocket 
initial_pos = [START_X,START_Y,START_Z]  
initial_orientation = p.getQuaternionFromEuler([START_ROLL, START_PITCH, 0]) 
rocket_id = p.loadURDF("rocket.urdf", initial_pos, initial_orientation)

dt = 1.0 / 240.0
p.setTimeStep(dt)

# Instantiate the controller
pitch_pid = PIDController(kp=8.0, ki=0.0, kd=2.5, dt=dt)
roll_pid = PIDController(kp=8.0, ki=0.0, kd=2.5, dt=dt)

x_pos_pid = PIDController(kp=0.05, ki=0.0, kd=0.15, dt=dt)
y_pos_pid = PIDController(kp=0.05, ki=0.0, kd=0.15, dt=dt)

TARGET_X = 0.0
TARGET_Y = 0.0
MAX_TARGET_TILT = np.radians(10.0)

camera_lock = True
telemetry_text_id = None

print("Simulation started.")

try:
    while True:
        pos, orient = p.getBasePositionAndOrientation(rocket_id)
        lin_vel, _ = p.getBaseVelocity(rocket_id)

        # --- KEYBOARD CAMERA CONTROL ---
        keys = p.getKeyboardEvents()
        r_key = ord('r') if ord('r') in keys else ord('R')
        if r_key in keys and (keys[r_key] & p.KEY_WAS_TRIGGERED):
            camera_lock = not camera_lock
            print(f"Camera Lock Status: {'ENABLED' if camera_lock else 'DISABLED'}")
            
        # THE MISSING LINK: Actually update the camera position!
        if camera_lock:
            p.resetDebugVisualizerCamera(
                cameraDistance=2.5, 
                cameraYaw=30, 
                cameraPitch=-10, 
                cameraTargetPosition=pos
            )
        
        # --- PID ATTITUDE CONTROL ---
        euler = p.getEulerFromQuaternion(orient)

        current_pitch = euler[1]
        current_roll = euler[0]

        current_x = pos[0]
        current_y = pos[1]

        position_error = TARGET_X - current_x

        raw_target_pitch = x_pos_pid.compute(target=TARGET_X, current=current_x)
        target_pitch = np.clip(raw_target_pitch, -MAX_TARGET_TILT, MAX_TARGET_TILT)

        raw_target_roll = -y_pos_pid.compute(target=TARGET_Y, current=current_y)
        target_roll = np.clip(raw_target_roll, -MAX_TARGET_TILT, MAX_TARGET_TILT)
        
        raw_gimbal_x = -pitch_pid.compute(target=target_pitch, current=current_pitch)
        gimbal_x = np.clip(raw_gimbal_x, -np.radians(15), np.radians(15))

        raw_gimbal_y = roll_pid.compute(target=target_roll, current=current_roll)
        gimbal_y = np.clip(raw_gimbal_y, -np.radians(15), np.radians(15))


        # --- LIVE TELEMETRY / CONTROL PANEL ---
        telemetry_str = (
            f"X: {pos[0]:+.3f}  Y: {pos[1]:+.3f}  Z: {pos[2]:+.3f}\n"
            f"Roll: {np.degrees(euler[0]):+.2f} | Pitch: {np.degrees(euler[1]):+.2f}"
        )

        telemetry_text_id = p.addUserDebugText(
            telemetry_str,
            textPosition=[pos[0] - 0.6, pos[1], pos[2] + 0.9],
            textColorRGB=[1, 1, 1],
            textSize=1.2,
            replaceItemUniqueId=telemetry_text_id if telemetry_text_id is not None else -1
        )

            
        # --- THROTTLE & FORCES ---
        hover_thrust = 0.5 * 9.81
        throttle_modifier = 1.0 - (lin_vel[2] * 0.4) 
        actual_thrust = hover_thrust * np.clip(throttle_modifier, 0.5, 2.0)

        # --- SMOOTH GROUND-PROXIMITY THRUST ROLLOFF ---
        altitude = pos[2]

        if altitude <= GROUND_CONTACT_HEIGHT:

            ramp_factor = 0.0
            if not has_landed:
                has_landed = True
                print(f"\n>>> TOUCHDOWN at t={sim_time:.2f}s | vz={lin_vel[2]:.3f} m/s <<<\n")
        elif altitude < RAMP_START_HEIGHT:
            # normalized 0→1 as altitude goes RAMP_START_HEIGHT → GROUND_CONTACT_HEIGHT
            t = (altitude - GROUND_CONTACT_HEIGHT) / (RAMP_START_HEIGHT - GROUND_CONTACT_HEIGHT)
            ramp_factor = t * t * (3 - 2 * t)  # smoothstep, avoids a kink at the ramp boundary
        else:
            ramp_factor = 1.0

        actual_thrust *= ramp_factor

        local_thrust_vector = [
                actual_thrust * np.sin(gimbal_x), 
                actual_thrust * np.sin(gimbal_y), 
                actual_thrust * np.cos(gimbal_x) * np.cos(gimbal_y) 
            ]
        nozzle_local_pos = [0, 0, -0.5]
        
        p.applyExternalForce(rocket_id, -1, local_thrust_vector, nozzle_local_pos, p.LINK_FRAME)
        
        # Exhaust Debug Line
        nozzle_world_pos, _ = p.multiplyTransforms(pos, orient, nozzle_local_pos, [0,0,0,1])
        rotation_matrix = np.reshape(p.getMatrixFromQuaternion(orient), (3, 3))
        world_thrust_vector = rotation_matrix.dot(local_thrust_vector)
        exhaust_dir = -world_thrust_vector / (np.linalg.norm(world_thrust_vector) + 1e-6)
        p.addUserDebugLine(nozzle_world_pos, [nozzle_world_pos[i] + exhaust_dir[i] * 0.3 for i in range(3)], [1,0.3,0], 3, 0.05)
        
        p.stepSimulation()
        time.sleep(dt)

except (p.error, KeyboardInterrupt):
    print("\nSimulation stopped.")
