"""
material_transport.py
Controller for the Smart Industrial Material Transport Robot.
Webots R2023b or newer, Python 3.

What it does (state machine):
  GO_TO_PICKUP -> PICKING -> GO_TO_DELIVERY -> DROPPING -> AFTER_DELIVERY
  (repeats until TOTAL_DELIVERIES is reached, then DONE)
Obstacle handling (works inside both GO_TO_* states):
  STOP -> TURN -> FORWARD -> back to normal navigation

The gripper is simulated by the Supervisor: during pickup and delivery the
box follows the animated arm end-effector, rather than being carried on top
of the chassis.
"""
import math
from controller import Supervisor

# ----------------------------------------------------------------------
# Settings (change these to customise the demo)
# ----------------------------------------------------------------------
PICKUP_POS = (-1.5, 0.0)       # centre of the green pickup station (x, y)
DELIVERY_POS = (2.0, 0.0)      # centre of the blue delivery station (x, y)
TOTAL_DELIVERIES = 3           # robot stops after this many deliveries

WHEEL_RADIUS = 0.05            # metres (must match the world file)
WHEEL_TRACK = 0.30             # distance between the two wheels (metres)
MAX_WHEEL_SPEED = 10.0         # rad/s (motors allow 12)

CRUISE_SPEED = 0.35            # m/s
MIN_SPEED = 0.06               # m/s, so the robot always reaches its target
ARRIVE_DIST = 0.08             # metres: "arrived" when this close to target
HEADING_GAIN = 2.0
MAX_TURN_RATE = 1.5            # rad/s
MIN_TURN_RATE = 0.3            # rad/s
TURN_START = 0.6               # rad: start turning on the spot above this error
TURN_STOP = 0.1                # rad: stop turning on the spot below this error

FRONT_STOP_DIST = 0.40         # metres, centre sensor
SIDE_STOP_DIST = 0.28          # metres, left / right sensors
AVOID_STOP_TIME = 1.0          # seconds the robot stays stopped
AVOID_TURN_RATE = 1.2          # rad/s
AVOID_TURN_TIMEOUT = 6.0       # seconds, safety limit for turning
AVOID_SPEED = 0.25             # m/s while driving around the obstacle
AVOID_FORWARD_DIST = 0.8       # metres driven after turning away

BOX_REST_Z = 0.08              # box centre height when lying on a station
CARGO_OFFSET_Z = 0.17          # box centre height above the robot origin
ACTION_TIME = 1.0              # seconds per arm reach/lift phase
DELIVERED_WAIT = 2.0           # seconds to show the delivered box

# ----------------------------------------------------------------------
# Devices
# ----------------------------------------------------------------------
robot = Supervisor()
time_step = int(robot.getBasicTimeStep())

left_motor = robot.getDevice("left_motor")
right_motor = robot.getDevice("right_motor")
ds_left = robot.getDevice("ds_left")
ds_center = robot.getDevice("ds_center")
ds_right = robot.getDevice("ds_right")

# Robotic-arm joints (these devices are defined in factory_transport_with_arm.wbt).
arm_base = robot.getDevice("arm_base_motor")
arm_shoulder = robot.getDevice("arm_shoulder_motor")

for name, dev in (("left_motor", left_motor), ("right_motor", right_motor),
                  ("ds_left", ds_left), ("ds_center", ds_center),
                  ("ds_right", ds_right), ("arm_base_motor", arm_base),
                  ("arm_shoulder_motor", arm_shoulder)):
    if dev is None:
        raise RuntimeError("Device '%s' not found. Check the robot in the .wbt file." % name)

left_motor.setPosition(float("inf"))     # velocity control mode
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)
# Position-controlled arm joints. Keep the arm raised while driving.
arm_base.setVelocity(1.0)
arm_shoulder.setVelocity(0.9)
arm_base.setPosition(0.0)
arm_shoulder.setPosition(-0.45)

for sensor in (ds_left, ds_center, ds_right):
    sensor.enable(time_step)

self_node = robot.getSelf()
box_nodes = [robot.getFromDef(name) for name in ("CARGO_BOX", "CARGO_BOX_2", "CARGO_BOX_3")]
if any(node is None for node in box_nodes):
    raise RuntimeError("One or more cargo boxes (CARGO_BOX, CARGO_BOX_2, CARGO_BOX_3) are missing from the world file.")
box_translations = [node.getField("translation") for node in box_nodes]
box_rotations = [node.getField("rotation") for node in box_nodes]
current_box_index = 0
box_translation = box_translations[current_box_index]
box_rotation = box_rotations[current_box_index]
pickup_start = (PICKUP_POS[0], PICKUP_POS[1], BOX_REST_Z)
# Each delivered box gets its own permanent position on the delivery station.
DELIVERY_SLOTS = [(1.78, -0.18), (2.0, 0.0), (2.22, 0.18)]

# ----------------------------------------------------------------------
# Helper functions
# ----------------------------------------------------------------------
def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def get_pose():
    """Return x, y, z and heading (rad) of the robot in world coordinates."""
    p = self_node.getPosition()
    r = self_node.getOrientation()
    heading = math.atan2(r[3], r[0])
    return p[0], p[1], p[2], heading


def read_distance(sensor):
    value = sensor.getValue()
    if value != value:          # NaN guard
        return 1.0
    return value


def drive(v, w):
    """v = forward speed (m/s), w = turn rate (rad/s, positive = left)."""
    left = (v - w * WHEEL_TRACK / 2.0) / WHEEL_RADIUS
    right = (v + w * WHEEL_TRACK / 2.0) / WHEEL_RADIUS
    biggest = max(abs(left), abs(right))
    if biggest > MAX_WHEEL_SPEED:
        left *= MAX_WHEEL_SPEED / biggest
        right *= MAX_WHEEL_SPEED / biggest
    left_motor.setVelocity(left)
    right_motor.setVelocity(right)


def place_box(x, y, z, yaw):
    box_translation.setSFVec3f([x, y, z])
    box_rotation.setSFRotation([0.0, 0.0, 1.0, yaw])


def gripper_pose(x, y, z, heading, shoulder_angle=None):
    """Estimate the end-effector position from the visible arm geometry.

    The Supervisor moves the box to this point to simulate a grasp. The
    arm itself is still animated by its base and shoulder motors.
    """
    if shoulder_angle is None:
        shoulder_angle = -0.45
    # The arm has a short turret, a 0.19 m upper link, and a small gripper.
    horizontal_reach = 0.09 + 0.19 * math.cos(shoulder_angle) + 0.03
    vertical_reach = 0.125 + 0.075 - 0.19 * math.sin(shoulder_angle)
    return (x + math.cos(heading) * horizontal_reach,
            y + math.sin(heading) * horizontal_reach,
            z + vertical_reach)


def approach_target(box_x, box_y, robot_x, robot_y, reach=0.29):
    """Stop the chassis one arm-reach before the target, facing the target."""
    dx, dy = box_x - robot_x, box_y - robot_y
    length = math.hypot(dx, dy)
    if length < 1e-6:
        return box_x - reach, box_y
    ux, uy = dx / length, dy / length
    return box_x - ux * reach, box_y - uy * reach


last_status = ""


def set_status(text):
    """Print status to the console and show it in the 3D window."""
    global last_status
    if text != last_status:
        last_status = text
        print("[STATUS] " + text)
        robot.setLabel(0, "Status: " + text, 0.02, 0.02, 0.06, 0x202020, 0.0, "Arial")


def show_counter(count):
    robot.setLabel(1, "Total deliveries: %d / %d" % (count, TOTAL_DELIVERIES),
                   0.02, 0.08, 0.06, 0x0033AA, 0.0, "Arial")


def show_message(text):
    robot.setLabel(2, text, 0.02, 0.14, 0.06, 0x008800, 0.0, "Arial")


aligning = False   # True while turning on the spot toward a target


def steer_to(tx, ty, x, y, heading):
    """Drive toward (tx, ty). Returns the remaining distance."""
    global aligning
    dx = tx - x
    dy = ty - y
    dist = math.hypot(dx, dy)
    err = wrap_angle(math.atan2(dy, dx) - heading)

    if aligning:
        if abs(err) < TURN_STOP:
            aligning = False
    elif abs(err) > TURN_START:
        aligning = True

    if aligning:
        w = clamp(HEADING_GAIN * err, -MAX_TURN_RATE, MAX_TURN_RATE)
        if abs(w) < MIN_TURN_RATE:
            w = math.copysign(MIN_TURN_RATE, err)
        drive(0.0, w)
    else:
        v = clamp(dist, MIN_SPEED, CRUISE_SPEED)
        w = clamp(HEADING_GAIN * err, -MAX_TURN_RATE, MAX_TURN_RATE)
        if dist < 0.2:
            w *= dist / 0.2
        drive(v, w)
    return dist


# ----------------------------------------------------------------------
# Main program
# ----------------------------------------------------------------------
state = "GO_TO_PICKUP"
avoid_phase = None          # None, "STOP", "TURN" or "FORWARD"
avoid_t0 = 0.0
avoid_start = (0.0, 0.0)
turn_dir = 1.0
action_t0 = 0.0
delivered = 0

show_counter(delivered)
show_message(" ")
set_status("Moving to pickup station")
print("=== Smart Industrial Material Transport Robot started ===")

while robot.step(time_step) != -1:
    now = robot.getTime()
    x, y, z, heading = get_pose()
    dist_l = read_distance(ds_left)
    dist_c = read_distance(ds_center)
    dist_r = read_distance(ds_right)

    if state == "GO_TO_PICKUP" or state == "GO_TO_DELIVERY":
        carrying = (state == "GO_TO_DELIVERY")
        if carrying:
            slot_x, slot_y = DELIVERY_SLOTS[min(delivered, len(DELIVERY_SLOTS) - 1)]
            target = approach_target(slot_x, slot_y, x, y)
            # While travelling, the box stays at the raised arm's end-effector.
            arm_shoulder.setPosition(-0.45)
            arm_base.setPosition(0.0)
            gx, gy, gz = gripper_pose(x, y, z, heading, -0.45)
            place_box(gx, gy, gz, heading)
        else:
            staged = box_translation.getSFVec3f()
            target = approach_target(staged[0], staged[1], x, y)

        if avoid_phase is None:
            dist = math.hypot(target[0] - x, target[1] - y)
            obstacle = (dist_c < FRONT_STOP_DIST or
                        dist_l < SIDE_STOP_DIST or
                        dist_r < SIDE_STOP_DIST)
            if dist < ARRIVE_DIST:
                drive(0.0, 0.0)
                action_t0 = now
                if carrying:
                    state = "DROPPING"
                else:
                    # Start the pickup animation at this box's actual staged position.
                    box_pos = box_translation.getSFVec3f()
                    pickup_start = (box_pos[0], box_pos[1], box_pos[2])
                    state = "PICKING"
            elif obstacle and not aligning:
                drive(0.0, 0.0)
                avoid_phase = "STOP"
                avoid_t0 = now
                print("[SENSOR] Obstacle detected: left=%.2f centre=%.2f right=%.2f m"
                      % (dist_l, dist_c, dist_r))
                set_status("OBSTACLE DETECTED - robot stopped")
            else:
                if carrying:
                    set_status("Carrying box to delivery station")
                else:
                    set_status("Moving to pickup station")
                steer_to(target[0], target[1], x, y, heading)

        elif avoid_phase == "STOP":
            drive(0.0, 0.0)
            if now - avoid_t0 >= AVOID_STOP_TIME:
                turn_dir = 1.0 if dist_l >= dist_r else -1.0
                avoid_phase = "TURN"
                avoid_t0 = now
                set_status("Avoiding obstacle - turning " +
                           ("left" if turn_dir > 0 else "right"))

        elif avoid_phase == "TURN":
            drive(0.0, turn_dir * AVOID_TURN_RATE)
            clear = dist_c > 0.6 and dist_l > 0.45 and dist_r > 0.45
            if clear or now - avoid_t0 > AVOID_TURN_TIMEOUT:
                avoid_phase = "FORWARD"
                avoid_start = (x, y)
                set_status("Avoiding obstacle - driving around it")

        elif avoid_phase == "FORWARD":
            drive(AVOID_SPEED, 0.0)
            travelled = math.hypot(x - avoid_start[0], y - avoid_start[1])
            if dist_c < 0.30 or dist_l < 0.20 or dist_r < 0.20:
                drive(0.0, 0.0)
                avoid_phase = "STOP"
                avoid_t0 = now
                set_status("OBSTACLE DETECTED - robot stopped")
            elif travelled >= AVOID_FORWARD_DIST:
                avoid_phase = None
                aligning = False
                print("[INFO] Obstacle passed, resuming route")

    elif state == "PICKING":
        drive(0.0, 0.0)
        set_status("Arm reaching down and gripping box")
        p = clamp((now - action_t0) / (ACTION_TIME * 2.0), 0.0, 1.0)
        # Phase 1: physically animate the shoulder down to the box. Phase 2:
        # lift the arm and carry the box at the end-effector.
        arm_base.setPosition(0.0)
        if p < 0.5:
            # REACH: move the visible arm down first. The box stays on the
            # floor; it must not move before the gripper reaches it.
            q = p / 0.5
            shoulder = -0.45 + (0.85 + 0.45) * q
            arm_shoulder.setPosition(shoulder)
            bx, by, bz = pickup_start
        else:
            # GRIP + LIFT: only after the reach phase has completed does the
            # Supervisor simulate the fingers holding the box at the gripper.
            q = (p - 0.5) / 0.5
            shoulder = 0.85 + (-0.45 - 0.85) * q
            arm_shoulder.setPosition(shoulder)
            bx, by, bz = gripper_pose(x, y, z, heading, shoulder)
        place_box(bx, by, bz, heading)
        if p >= 1.0:
            print("[INFO] Box picked up")
            show_message(" ")
            state = "GO_TO_DELIVERY"
            aligning = False

    elif state == "DROPPING":
        drive(0.0, 0.0)
        set_status("Arm lowering box onto delivery station")
        p = clamp((now - action_t0) / (ACTION_TIME * 2.0), 0.0, 1.0)
        slot_x, slot_y = DELIVERY_SLOTS[min(delivered, len(DELIVERY_SLOTS) - 1)]
        arm_base.setPosition(0.0)
        # Lower the arm while the box follows the gripper, then release it on
        # the delivery slot. The Supervisor simulates the fingers holding it.
        shoulder = -0.45 + (0.85 + 0.45) * p
        arm_shoulder.setPosition(shoulder)
        gx, gy, gz = gripper_pose(x, y, z, heading, shoulder)
        # Keep the box attached to the gripper as the arm lowers. During the
        # final part of the motion, guide it into the delivery slot, then let
        # it rest on the station when p reaches 1.
        if p < 0.85:
            bx, by, bz = gx, gy, gz
        else:
            q = (p - 0.85) / 0.15
            bx = gx + (slot_x - gx) * q
            by = gy + (slot_y - gy) * q
            bz = gz + (BOX_REST_Z - gz) * q
        place_box(bx, by, bz, heading if p < 1.0 else 0.0)
        if p >= 1.0:
            delivered += 1
            print("[DELIVERY] DELIVERY CONFIRMED - total deliveries: %d" % delivered)
            set_status("DELIVERY CONFIRMED")
            show_counter(delivered)
            show_message("DELIVERY CONFIRMED - box #%d delivered" % delivered)
            state = "AFTER_DELIVERY"
            action_t0 = now

    elif state == "AFTER_DELIVERY":
        drive(0.0, 0.0)
        if now - action_t0 >= DELIVERED_WAIT:
            if delivered >= TOTAL_DELIVERIES:
                state = "DONE"
                set_status("MISSION COMPLETE - %d deliveries" % delivered)
                print("=== Mission complete: %d deliveries ===" % delivered)
            else:
                # Select the next distinct box. Previously delivered boxes stay at delivery.
                current_box_index = delivered
                box_translation = box_translations[current_box_index]
                box_rotation = box_rotations[current_box_index]
                print("[INFO] Moving to pick up box #%d" % (current_box_index + 1))
                state = "GO_TO_PICKUP"
                aligning = False

    else:   # DONE
        drive(0.0, 0.0)
