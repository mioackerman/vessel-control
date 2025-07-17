import threading
import time
import math
import krpc

from pid import PID
from floater.telemetry import TelemetryManager as tm

# ======== 参数配置 ========
FRAME_DT = 0.05  # 20Hz
STANDARD_ALTITUDE = 1000
LANDING_ALTITUDE = 1000
CRITICAL_SPEED = 20.0
CRITICAL_ALTITUDE = 50
CRITICAL_ALTITUDE_PID = 200
LANDING_TOLERANCE = 3  # 启动后会根据包围盒更新
DOB = 0

# PID 控制器初始化
# pid_pitch = PID(Kp=2.0, Ki=0.1, Kd=1.5, output_limits=(-0.3, 0.3))
# pid_roll = PID(Kp=2.0, Ki=0.1, Kd=1.5, output_limits=(-0.3, 0.3))
pid_thrust = PID(Kp=0.15, Ki=0.01, Kd=0.1, output_limits=(0.0, 1.0))

# 状态机阶段
STATE = "LAUNCH"  # 可选: LAUNCH, HOVER, LANDING, EMERGENCY, DONE
landing_lock = True  # ✅ 新增锁逻辑


def pid_thruse_correction():
    alt = tm.get("altitude")
    vs = tm.get("vertical_speed")
    gravity = tm.get("gravity")
    mass = tm.get("mass")
    max_thrust = tm.get("available_thrust")
    thrust_limit_coefficient = 0.25

    if max_thrust != 0:
        thrust_limit_coefficient = 1.5 * mass * gravity / max_thrust
    if alt > CRITICAL_ALTITUDE_PID and -vs > CRITICAL_SPEED:
        thrust_strategy = 'max'
        pid = PID(Kp=0.15, Ki=0.01, Kd=0.1, output_limits=(0.0, 1.0))
    elif alt >= CRITICAL_ALTITUDE_PID:
        thrust_strategy = 'medium'
        pid = PID(Kp=0.01, Ki=0.01, Kd=0.1, output_limits=(0.0, 1.0))
    elif alt <= CRITICAL_ALTITUDE_PID and -vs > CRITICAL_SPEED:
        thrust_strategy = 'medium'
        pid = PID(Kp=0.01, Ki=0.01, Kd=0.1, output_limits=(0.0, 1.0))
    else:
        thrust_strategy = 'min'
        pid = PID(Kp=0.01, Ki=0.01, Kd=0.1, output_limits=(0.0, thrust_limit_coefficient))

    return pid


# ======== 初始化函数 ========
def calculate_landing_tolerance(vessel):
    global LANDING_TOLERANCE
    srf = vessel.surface_reference_frame
    bbox = vessel.bounding_box(srf)
    LANDING_TOLERANCE = abs(bbox[0][1])
    print(f"\r✅ LANDING_TOLERANCE = {LANDING_TOLERANCE:.2f} m")


# ======== 控制逻辑 ========
def update_orientation(vessel, dt):
    return 0


    """姿态控制：平衡 pitch 和 roll"""
    angular_velocity = tm.get("angular_velocity")
    pitch_rate = -angular_velocity[0]
    roll_rate = -angular_velocity[2]

    u_pitch = pid_pitch.update(pitch_rate, dt)
    u_roll = pid_roll.update(roll_rate, dt)

    engines = [e for e in vessel.parts.engines if e.active]
    base_thrust = 0.6
    thrusts = [
        max(0.0, min(1.0, base_thrust + u_pitch - u_roll)),
        max(0.0, min(1.0, base_thrust + u_pitch + u_roll)),
        max(0.0, min(1.0, base_thrust - u_pitch - u_roll)),
        max(0.0, min(1.0, base_thrust - u_pitch + u_roll)),
    ]
    for i, e in enumerate(engines):
        e.thrust_limit = thrusts[i]


def update_thrust(vessel, dt):
    """软着陆 PID 控制"""
    alt = tm.get("altitude")
    vs = tm.get("vertical_speed")

    if alt <= LANDING_TOLERANCE:
        vessel.control.throttle = 0.0
        return

    if alt > 300:
        target_speed = -60
    elif alt > 100:
        target_speed = -30
    elif alt > 20:
        target_speed = -10
    else:
        target_speed = -1

    error = target_speed - vs
    control = pid_thruse_correction().update(error, dt)
    vessel.control.throttle = control


def emergency_check(vessel):
    alt = tm.get("altitude")
    vs = tm.get("vertical_speed")
    if alt < CRITICAL_ALTITUDE and -vs > CRITICAL_SPEED:
        print("\r🚨 Emergency Landing Triggered")
        emergency_rocket(vessel)
        return True
    return False


def emergency_rocket(vessel):
    print(' \r❌emergency stage 4')
    vessel.control.gear = True
    vessel.control.toggle_action_group(2)

    alt = tm.get("altitude")
    vs = tm.get("vertical_speed")
    while alt > CRITICAL_ALTITUDE and -vs > CRITICAL_SPEED:
        vessel.control.throttle = 1
        alt = tm.get("altitude")
        vs = tm.get("vertical_speed")
    vessel.control.throttle = 0.0
    vessel.control.toggle_action_group(1)
    time.sleep(0.5)
    vessel.control.toggle_action_group(3)


# ======== 状态机逻辑 ========
def main_loop(conn, vessel):
    global STATE
    landing_lock = True
    is_landing = False
    landing_init = True
    calculate_landing_tolerance(vessel)

    # 初始化 Telemetry Streams
    tm.init_streams(conn, vessel)

    print("🚀 Main Control Loop Started")
    last_time = time.time()

    STATE = "LAUNCH"
    vessel.control.activate_next_stage()
    vessel.control.gear = True  # ✅ 起飞时收放逻辑初始化
    vessel.control.sas = True
    while True:
        now = time.time()
        dt = now - last_time
        last_time = now

        # 更新状态
        altitude = tm.get("altitude")
        vertical_speed = tm.get("vertical_speed")

        # === 状态机 ===
        if STATE == "LAUNCH":
            vessel.control.throttle = 1.0

            print("\rLaunching", end='', flush=True)

            if altitude > get_landing_altitude() and landing_lock:  # ✅ 超过目标高度解锁
                vessel.control.gear = False
                landing_lock = False
                print("\r 🔓 Landing lock released, gear retracted.")

            if altitude >= STANDARD_ALTITUDE:
                print("\r ✅ Reached target altitude, switching to LANDING")
                vessel.control.throttle = 0.0
                STATE = "LANDING"



        elif STATE == "LANDING":

            if altitude <= LANDING_ALTITUDE and not landing_lock and not is_landing:
                is_landing = True
                vessel.control.gear = True
                print('\r Landing detected')

            if not landing_lock and is_landing:
                if landing_init:
                    landing_init = False
                    vessel.control.toggle_action_group(4)
                    print('\r🔄 Switching to Landing State')

                vessel.control.gear = True
                update_thrust(vessel, dt)

            if altitude <= LANDING_TOLERANCE:
                print("\r ✅ Touchdown detected!")
                STATE = "DONE"

        if emergency_check(vessel):
            STATE = "EMERGENCY"

        # 姿态控制在所有状态运行
        # update_orientation(vessel, dt)

        if STATE == "DONE" or STATE == "EMERGENCY":
            vessel.control.toggle_action_group(4)
            break

        # 控制帧频
        sleep_time = FRAME_DT - (time.time() - now)
        if sleep_time > 0:
            time.sleep(sleep_time)

    print("\r ✅ Control Loop Ended")


def get_landing_altitude():
    if (STANDARD_ALTITUDE / 4) >= 600:
        return STANDARD_ALTITUDE / 4
    else:
        return 600


# ======== 启动入口 ========
if __name__ == '__main__':
    conn = krpc.connect(name='Main Control',
                        address='192.168.2.29',
                        rpc_port=50000,
                        stream_port=50001)
    vessel = conn.space_center.active_vessel
    main_loop(conn, vessel)
