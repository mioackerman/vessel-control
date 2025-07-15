import math
import time
from pid import PID
from threading import Event

emergency_event = Event()

STANDARD_ALTITUDE = 5000
if (STANDARD_ALTITUDE / 4) >= 600:
    SAFETY_ALTITUDE = STANDARD_ALTITUDE / 4
else:
    SAFETY_ALTITUDE = 600
CRITICAL_SPEED = 20.0
CRITICAL_ALTITUDE = 50
CRITICAL_ALTITUDE_PID = 200
LANDING_TOLERANCE = 5
DOB = 0


def landing_test_launch(conn, vessel):
    # vessel.auto_pilot.engage()
    # vessel.auto_pilot.reference_frame = vessel.surface_reference_frame
    # vessel.auto_pilot.target_direction = (90, 0, 0)  # 表面参考系下，正上方
    # vessel.auto_pilot.target_roll = 0
    if not emergency_event.is_set():
        print("Landing test launched")
        flight = vessel.flight
        vref = vessel.orbit.body.reference_frame
        sref = vessel.surface_reference_frame
        alt = flight(sref).surface_altitude

        vessel.control.sas = True
        vessel.control.rcs = False

        def energy_max(speed):
            g = vessel.orbit.body.surface_gravity
            speed_max = math.sqrt(2 * g * (STANDARD_ALTITUDE - 900))
            print(f'\rHeight predict: {speed ** 2 / (2 * g):.1f} m   ', end='', flush=True)
            return speed >= speed_max

        vessel.control.activate_next_stage()
        while True:
            alt = vessel.flight(sref).surface_altitude
            speed = flight(vref).vertical_speed

            vessel.control.throttle = 1.0
            if alt >= SAFETY_ALTITUDE and energy_max(speed):
                vessel.control.gear = False
                vessel.control.throttle = 0

                g = vessel.orbit.body.surface_gravity
                print(f'\rApex predict: {speed ** 2 / (2 * g) + 900:.1f} m   ')
                print('\rlanding test launch end normally.')
                break

        print('\rlanding test launch end.')


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


def status_monitor(conn, vessel):
    if not emergency_event.is_set():
        print('Status Monitor Activated')
        ref = vessel.orbit.body.reference_frame
        surf = vessel.surface_reference_frame
        gravity = vessel.orbit.body.surface_gravity
        while True:
            if not emergency_event.is_set():
                altitude = vessel.flight(surf).surface_altitude
                speed = vessel.flight(ref).vertical_speed
                print(f"\raltitude: {altitude:.1f}m | speed: {speed:.1f}m/s", end='', flush=True)
                time.sleep(0.2)


def landing_monitor(conn, vessel):
    from math import sqrt, acos, degrees
    import time

    time.sleep(5)  # 等待发射
    print('\rLanding Monitor Activated')
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
        DOB = d_stop
        print(f"🔍 当前高度: {altitude:.1f}m, 速度: {vertical_speed:.1f}m/s, 预计减速距离: {d_stop:.1f}m")
        return d_stop >= altitude and vertical_speed <= 0

    while True:
        mass = vessel.mass
        max_thrust = vessel.available_thrust
        if not mass == 0:
            TWR = max_thrust / (mass * gravity)
        a_net = max((TWR - 1) * gravity, 0.1)
        altitude = vessel.flight(surf).surface_altitude
        speed = vessel.flight(ref).vertical_speed

        if not landing_lock:
            # ✅ 燃料余量检查
            isp = sum(e.specific_impulse for e in vessel.parts.engines if e.active) / max(1, len(vessel.parts.engines))
            g0 = 9.80665
            if isp > 0:
                mass_flow = max_thrust / (isp * g0)  # kg/s
            else:
                print('isp is 0')
            t_burn = abs(speed) / a_net
            fuel_needed = mass_flow * t_burn
            fuel_mass = (vessel.resources.amount("LiquidFuel") + vessel.resources.amount("Oxidizer")) * 5  # 粗略估算

            # ✅ 姿态检查（速度方向 vs 飞船朝向）
            flight_data = vessel.flight(ref)
            vel_dir = flight_data.velocity
            ship_dir = vessel.direction(ref)
            dot = (vel_dir[0] * ship_dir[0] + vel_dir[1] * ship_dir[1] + vel_dir[2] * ship_dir[2]) / (
                    (sum(i * i for i in vel_dir) ** 0.5) * (sum(i * i for i in ship_dir) ** 0.5)
            )
            angle_error = degrees(acos(dot))

        # ✅ 高度控制逻辑
        if altitude > SAFETY_ALTITUDE:
            vessel.control.gear = False
            landing_lock = False

        if altitude <= SAFETY_ALTITUDE and landing_lock == False:
            is_landing = True
            vessel.control.gear = True

        if altitude < LANDING_TOLERANCE:
            vessel.control.throttle = 0.0
            print("\rEnd monitor")
            break

        # ✅ 原有逻辑：应急刹车或平稳模式
        if not landing_lock and is_landing:
            vessel.auto_pilot.engage()
            vessel.auto_pilot.reference_frame = vessel.surface_reference_frame
            vessel.auto_pilot.target_direction = (90, 0, 0)  # 表面参考系下，正上方
            vessel.auto_pilot.target_roll = 0

            # ✅ 紧急条件检查：燃料不足 or 姿态错误
            if fuel_mass < fuel_needed or angle_error < 160:
                if fuel_mass < fuel_needed:
                    print('fule')
                if angle_error < 160:
                    print('angle')
                print("\r❌ 燃料不足或姿态错误，执行紧急火箭！")
                emergency_event.set()
                emergency_rocket(conn, vessel)
                break
            if should_emergency_brake(altitude, speed) and -speed > CRITICAL_SPEED + 100:
                print("\r🚨 应急着陆触发：速度过快，空间不足")
                emergency_event.set()
                emergency_rocket(conn, vessel)
                break
            else:
                print("\r🟢 进入软着陆控制模式")
                emergency_event.set()  # 通知其他线程停止控制
                gentle_landing_pid_control(conn, vessel)


def gentle_landing_pid_control(conn, vessel, target_speed=-10):
    global thrust_limit_coefficient
    print("\r🟢 软着陆控制启动")
    pid = PID(Kp=0.15, Ki=0.01, Kd=0.1, output_limits=(0.0, 1.0))
    ref = vessel.orbit.body.reference_frame

    gravity = vessel.orbit.body.surface_gravity
    mass = vessel.mass
    max_thrust = vessel.available_thrust
    TWR = max_thrust / (mass * gravity)
    a_net = (TWR - 1) * gravity

    vessel.control.gear = True
    vessel.control.sas = True

    last_time = time.time()

    while True:
        thrust_strategy = ''
        now = time.time()
        dt = now - last_time
        last_time = now
        mass = vessel.mass
        max_thrust = vessel.available_thrust
        TWR = max_thrust / (mass * gravity)
        a_net = (TWR - 1) * gravity
        thrust_limit_coefficient = 0.25
        if max_thrust != 0:
            thrust_limit_coefficient = 1.5 * mass * gravity / max_thrust
            print('thrust coefficient: %.3f ' % thrust_limit_coefficient)

        flight = vessel.flight(vessel.surface_reference_frame)
        alt = flight.surface_altitude
        vs = vessel.flight(ref).vertical_speed

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

        if alt > 300:
            target_speed = -60
        elif alt > 100:
            target_speed = -30
        elif alt > 20:
            target_speed = -10
        else:
            target_speed = -1

        if alt < CRITICAL_ALTITUDE and -vs > CRITICAL_SPEED and alt < DOB:
            print("\r❌ Speed over limit. Distance not enough. Emergency Rocket activated.")
            emergency_event.set()
            emergency_rocket(conn, vessel)
            break

        if alt < LANDING_TOLERANCE:
            vessel.control.throttle = 0.0
            print("\rTouched Ground, landing success.")
            break

        # ========== 🧠 减速距离估算 ==========
        v = abs(vs)  # 取绝对值
        if v > 0.1 and a_net > 0:
            d_stop = (v ** 2) / (2 * a_net)
        else:
            d_stop = 0

        if SAFETY_ALTITUDE > alt > LANDING_TOLERANCE:
            error = target_speed - vs
            control = pid.update(error, dt)
            vessel.control.throttle = control

            print(f"\rAltitude: {alt:.1f} m "
                  f"Speed: {vs:.1f} m/s "
                  f"DOB: {d_stop:.1f} m   "
                  f"Thrust Strategy: {thrust_strategy}  "
                  f"Thrust: {control:.3f}   ",
                  end='', flush=True)
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
