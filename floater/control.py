import math
import time
from pid import PID
from threading import Event

emergency_event = Event()
SAFETY_ALTITUDE = 500
STANDARD_ALTITUDE = 10000
CRITICAL_SPEED = 20.0
CRITICAL_ALTITUDE = 50
LANDING_TOLERANCE = 5


def landing_test_launch(conn, vessel):
    if not emergency_event.is_set():
        flight = vessel.flight
        vref = vessel.orbit.body.reference_frame
        sref = vessel.surface_reference_frame
        alt = flight(sref).surface_altitude

        vessel.control.sas = True
        vessel.control.rcs = False

        def energy_max(speed):
            g = vessel.orbit.body.surface_gravity
            speed_max_reach = speed >= math.sqrt(g * STANDARD_ALTITUDE)

            return speed_max_reach, print('Height predict: %.1f'% (speed**2 / (2*g)))

        vessel.control.activate_next_stage()
        while True:
            alt = vessel.flight(sref).surface_altitude
            speed = flight(vref).vertical_speed

            vessel.control.throttle = 1.0
            if alt >= SAFETY_ALTITUDE and energy_max(speed):
                vessel.control.gear = False
                vessel.control.throttle = 0
                print('landing test launch end normally.')
                break
        print('landing test launch end.')


def rolling_control(conn, vessel):
    if not emergency_event.is_set():
        print(f"🌀 启动 Rolling Control for {vessel.name}")
        engines = [e for e in vessel.parts.engines if e.active]
        if len(engines) == 0:
            vessel.control.activate_next_stage()
            time.sleep(0.5)
            engines = [e for e in vessel.parts.engines if e.active]

        if len(engines) != 4:
            raise Exception(f"❌ Expected 4 engines, got {len(engines)}")

        ref_frame = vessel.surface_reference_frame
        flight = vessel.flight(ref_frame)

        pid_pitch = PID(Kp=2.0, Ki=0.1, Kd=1.5, output_limits=(-0.3, 0.3))
        pid_roll = PID(Kp=2.0, Ki=0.1, Kd=1.5, output_limits=(-0.3, 0.3))
        base_thrust = 0.6

        def clamp(val, lo=0.0, hi=1.0):
            return max(lo, min(hi, val))

        last_time = time.time()

        while True:
            if emergency_event.is_set():
                break
            now = time.time()
            dt = now - last_time
            last_time = now

            angular_velocity = vessel.angular_velocity(ref_frame)
            pitch_rate = -angular_velocity[0]
            roll_rate = -angular_velocity[2]

            u_pitch = pid_pitch.update(pitch_rate, dt)
            u_roll = pid_roll.update(roll_rate, dt)

            thrusts = [
                clamp(base_thrust + u_pitch - u_roll),
                clamp(base_thrust + u_pitch + u_roll),
                clamp(base_thrust - u_pitch - u_roll),
                clamp(base_thrust - u_pitch + u_roll),
            ]

            for i, e in enumerate(engines):
                e.thrust_limit = thrusts[i]

            print(f"Pitch: {pitch_rate:+.2f}  Roll: {roll_rate:+.2f}  Thrusts: {[round(t, 2) for t in thrusts]}")
            time.sleep(0.05)


def stabilization(conn, vessel):
    if not emergency_event.is_set():
        print(f" Altitude Stabilization for {vessel.name}")
        engines = [e for e in vessel.parts.engines if e.active]
        if len(engines) == 0:
            vessel.control.activate_next_stage()

        target_alt = STANDARD_ALTITUDE
        tolerance = 50
        hold_time = 3.0

        flight = vessel.flight(vessel.surface_reference_frame)
        pid = PID(Kp=0.1, Ki=0.003, Kd=0.02)

        vessel.control.sas = True
        vessel.control.rcs = False

        last_time = time.time()
        stable_start = None

        while True:
            if emergency_event.is_set():
                break
            now = time.time()
            dt = now - last_time
            last_time = now

            alt = flight.surface_altitude
            error = target_alt - alt

            control = pid.update(error, dt)
            vessel.control.throttle = max(0.0, min(1.0, control))

            in_range = abs(error) <= tolerance
            if in_range:
                if stable_start is None:
                    stable_start = now
                elif now - stable_start >= hold_time:
                    print('Stabilized 10s, break.')
                    break
            else:
                stable_start = None

            # print(f"[高度] {alt:.2f}m  [误差] {error:.2f}  [推力] {vessel.control.throttle:.2f}  in_range={in_range}")
            time.sleep(0.1)


def landing_monitor(conn, vessel):
    from math import sqrt

    time.sleep(5)  # 等待发射
    landing_lock = True
    is_landing = False
    vessel.control.gear = True
    vessel.control.sas = True

    ref = vessel.orbit.body.reference_frame
    surf = vessel.surface_reference_frame
    gravity = vessel.orbit.body.surface_gravity

    def should_emergency_brake(altitude, vertical_speed):
        v = -vertical_speed  # 向下速度为正
        d_stop = (v ** 2) / (2 * a_net)
        print(f"🔍 当前高度: {altitude:.1f}m, 速度: {vertical_speed:.1f}m/s, 预计减速距离: {d_stop:.1f}m")
        return d_stop + 40 >= altitude and vertical_speed <= 0

    while True:
        mass = vessel.mass
        max_thrust = vessel.available_thrust
        TWR = max_thrust / (mass * gravity)
        # print(f" 推重比: {TWR:.2f}, 减速度: {a_net:.2f} m/s²")
        a_net = max((TWR - 1) * gravity, 0.1)
        altitude = vessel.flight(surf).surface_altitude
        speed = vessel.flight(ref).vertical_speed  ## TODO

        if altitude > SAFETY_ALTITUDE:
            vessel.control.gear = False
            landing_lock = False
            continue
        if altitude < SAFETY_ALTITUDE and landing_lock == False:
            is_landing = True
        vessel.control.gear = True

        if altitude < LANDING_TOLERANCE:
            vessel.control.throttle = 0.0
            print("Touch ground，end monitor")
            break

        # ✅ 紧急判断
        if not landing_lock and is_landing:
            if should_emergency_brake(altitude, speed) and -speed > CRITICAL_SPEED + 100:
                print("🚨 应急着陆触发：速度过快，空间不足")
                emergency_event.set()
                vessel.control.toggle_action_group(2)
                vessel.control.throttle = 0.0
                vessel.control.toggle_action_group(1)
                time.sleep(0.5)
                vessel.control.toggle_action_group(3)
                break
            else:
                print("🟢 进入平稳着陆模式")
                emergency_event.set()  # 通知其他线程停止控制
                gentle_landing_pid_control(conn, vessel)

        print('altitude: %.1f, speed: %.1f' % (altitude, speed))


def gentle_landing_pid_control(conn, vessel, target_speed=-10):
    print("🛬 超高速软着陆控制启动")
    pid = PID(Kp=0.15, Ki=0.01, Kd=0.1, output_limits=(0.0, 1.0))
    ref = vessel.orbit.body.reference_frame

    gravity = vessel.orbit.body.surface_gravity
    mass = vessel.mass
    max_thrust = vessel.available_thrust
    TWR = max_thrust / (mass * gravity)
    a_net = (TWR - 1) * gravity  # 可用减速度

    vessel.control.gear = True
    vessel.control.sas = True

    last_time = time.time()

    while True:

        now = time.time()
        dt = now - last_time
        last_time = now

        flight = vessel.flight(vessel.surface_reference_frame)
        alt = flight.surface_altitude
        vs = vessel.flight(ref).vertical_speed  # vs < 0 表示下落

        if alt > 200:
            pid = PID(Kp=0.15, Ki=0.01, Kd=0.1, output_limits=(0.0, 1.0))
        else:
            pid = PID(Kp=0.03, Ki=0.01, Kd=0.1, output_limits=(0.0, 1.0))


        if alt > 300:
            target_speed = -60
        elif alt > 100:
            target_speed = -30
        else:
            target_speed = -3

        if alt < CRITICAL_ALTITUDE and -vs > CRITICAL_SPEED:
            print("🚨 应急着陆触发：速度过快，空间不足")
            emergency_event.set()
            emergency_rocket(conn, vessel)
            break

        if alt < LANDING_TOLERANCE:
            vessel.control.throttle = 0.0
            print("✅ 已接触地面，结束控制")
            break

        # ========== 🧠 减速距离估算 ==========
        v = abs(vs)  # 取绝对值
        if v > 0.1 and a_net > 0:
            d_stop = (v ** 2) / (2 * a_net)
        else:
            d_stop = 0

        print(f"🔍 高度: {alt:.1f} m 速度: {vs:.1f} m/s 所需减速距离: {d_stop:.1f} m")

        # ========== 🚀 第一阶段：必须硬减速 ==========
        if d_stop + 20 > alt > SAFETY_ALTITUDE:
            vessel.control.throttle = 1.0
            print("⚠️ 强制最大推力刹车中")
        # ========== 🪂 第二阶段：PID微调 ==========
        elif SAFETY_ALTITUDE > alt > LANDING_TOLERANCE:
            error = target_speed - vs
            control = pid.update(error, dt)
            vessel.control.throttle = control
            print(f"[PID] 推力控制: {control:.2f}")
        else:
            vessel.control.throttle = 0.0

        time.sleep(0.001)


def emergency_rocket(conn, vessel):
    print('emergency stage 4')
    vessel.control.gear = True
    vessel.control.toggle_action_group(2)

    ref = vessel.orbit.body.reference_frame
    surf = vessel.surface_reference_frame
    alt = vessel.flight(surf).surface_altitude
    vs = vessel.flight(ref).vertical_speed
    while alt > CRITICAL_ALTITUDE and -vs > CRITICAL_SPEED:
        vessel.control.throttle = 1
        alt = vessel.flight(surf).surface_altitude
        vs = vessel.flight(ref).vertical_speed
    vessel.control.throttle = 0.0
    vessel.control.toggle_action_group(1)
    time.sleep(0.5)
    vessel.control.toggle_action_group(3)
