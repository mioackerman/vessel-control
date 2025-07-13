from threading import Event
import time

emergency_event = Event()
SAFETY_ALTITUDE = 500
STANDARD_ALTITUDE = 1000
def emergency_landing(conn, vessel):
    altitude = vessel.flight(vessel.surface_reference_frame).surface_altitude
    if altitude < SAFETY_ALTITUDE:
        vessel.control.toggle_action_group(2)
        vessel.control.gear = True
        print('emergency stage 1')
        vessel.control.throttle = 1.0
        time.sleep(1.0)
        vessel.control.throttle = 0.0
        vessel.control.toggle_action_group(1)
        print('emergency stage 2')