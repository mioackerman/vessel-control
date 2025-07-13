import time
from pid import PID
from threading import Event

emergency_event = Event()
SAFETY_ALTITUDE = 500
STANDARD_ALTITUDE = 1000


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
        pid = PID(Kp=0.2, Ki=0.001, Kd=0.01)

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

            print(f"[高度] {alt:.2f}m  [误差] {error:.2f}  [推力] {vessel.control.throttle:.2f}  in_range={in_range}")
            time.sleep(0.1)


def landing_monitor(conn, vessel):
    landing_lock = True

    while True:
        if not emergency_event.is_set():
            altitude = vessel.flight(vessel.surface_reference_frame).surface_altitude
            if altitude > SAFETY_ALTITUDE:
                vessel.control.gear = False
                landing_lock = False
            else:
                vessel.control.gear = True

            if altitude < SAFETY_ALTITUDE and landing_lock is False:
                print("EMERGENCY LANDING")
                emergency_event.set()
                break
            time.sleep(0.1)



