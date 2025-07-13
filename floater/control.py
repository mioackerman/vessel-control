import time
from pid import PID
from threading import Event

emergency_event = Event()
SAFETY_ALTITUDE = 500
STANDARD_ALTITUDE = 1000
CRITICAL_SPEED = 20.0


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

            #print(f"[高度] {alt:.2f}m  [误差] {error:.2f}  [推力] {vessel.control.throttle:.2f}  in_range={in_range}")
            time.sleep(0.1)


def landing_monitor(conn, vessel):
    landing_lock = True

    while True:
        altitude = vessel.flight(vessel.surface_reference_frame).surface_altitude
        speed = vessel.flight(vessel.orbit.body.reference_frame).vertical_speed
        print('altitude: %.1f' % altitude)
        print('velocity: %.1f' % speed)
        if altitude > SAFETY_ALTITUDE:
            vessel.control.gear = False
            landing_lock = False
        else:
            vessel.control.gear = True

        if altitude < SAFETY_ALTITUDE and landing_lock is False:
            print("EMERGENCY LANDING")
            emergency_event.set()
            if altitude < SAFETY_ALTITUDE and -speed > CRITICAL_SPEED:
                vessel.control.toggle_action_group(2)  # parachute eject
                vessel.control.gear = True
                print('emergency stage 1')
                vessel.control.throttle = 1.0
                time.sleep(1.5)
                print('stage 1 end')
            if -speed > CRITICAL_SPEED and altitude < 100:
                vessel.control.toggle_action_group(1)
                vessel.control.throttle = 0.0
                print('emergency stage 2')
            elif -speed <= CRITICAL_SPEED and altitude >= 100:
                print('emergency stage 2.1')
                while altitude > 30:
                    altitude = vessel.flight(vessel.surface_reference_frame).surface_altitude
                    speed = vessel.flight(vessel.orbit.body.reference_frame).vertical_speed
                    vessel.control.throttle = 0.1
                    print('2.1: %.1f' % altitude)
                    if -speed >= CRITICAL_SPEED and altitude <= 30:
                        print('emergency stage 4')
                        vessel.control.throttle = 0.0
                        vessel.control.toggle_action_group(1)
            else:
                while altitude > 30 and -speed > CRITICAL_SPEED:
                    altitude = vessel.flight(vessel.surface_reference_frame).surface_altitude
                    speed = vessel.flight(vessel.orbit.body.reference_frame).vertical_speed
                    vessel.control.throttle = 0.8
                    if -speed >= CRITICAL_SPEED and altitude <= 30:
                        print('emergency stage 4')
                        vessel.control.throttle = 0.0
                        vessel.control.toggle_action_group(1)
                vessel.control.throttle = 0.0
            vessel.control.throttle = 0.0
            print('emergency end')

            break
        time.sleep(0.001)
