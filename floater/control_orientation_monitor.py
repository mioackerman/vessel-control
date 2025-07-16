import math
import threading
import time
import krpc

from floater import telemetry
from floater.control import emergency_event, get_surface_altitude, LANDING_TOLERANCE, LANDING_ALTITUDE

tm = telemetry.TelemetryManager
def control_orientation():
    conn = krpc.connect(name='Orientation Control',
                        address='192.168.2.29',
                        rpc_port=50000,
                        stream_port=50001)
    vessel = conn.space_center.active_vessel

    surf_frame = vessel.surface_reference_frame
    landing_lock = True
    is_landing = False

    print('\rOrientation Control Ready, sleep 5 sec for launch.')

    time.sleep(5)

    print('\rOrientation Control activated')
    while True:
        if tm.get("altitude") > LANDING_ALTITUDE:
            landing_lock = False

        if tm.get("altitude") <= LANDING_ALTITUDE and landing_lock == False:
            is_landing = True

        #print('get: ',tm.get("altitude"))

        if not landing_lock and is_landing:
            print('\rOrientation Control interrupting')
            while True:

                retrograde = tm.get("retrograde_vector")
                print("Retrograde: " , retrograde)
                # ship_dir = tm.get("forward_vector")
                y = (0,0,-1)

                # 计算 retrograde 与 up 的夹角
               
                angle_y = angle_between(retrograde, y)
                
                print(f" Angle to y: {angle_y:.2f}° ")

                vessel.control.sas = True
                if  0 <= angle_y <= 30:
                    # 如果 retrograde 足够接近垂直，则锁定逆行
                    try:
                        vessel.control.sas_mode = conn.space_center.SASMode.retrograde
                        print("✅ Locking Retrograde")
                    except Exception as e:
                        print(f"⚠ Cannot set SAS mode: {e}")
                else:
                    # 否则指向垂直向上
                    vessel.auto_pilot.engage()
                    vessel.auto_pilot.reference_frame = surf_frame
                    vessel.auto_pilot.target_direction = (90, 0, 0)

                    # vessel.control.sas_mode = conn.space_center.SASMode.retrograde
                    print("⚠ Retrograde not safe, pointing up")

                time.sleep(0.2)
                if tm.get("altitude") < LANDING_TOLERANCE:
                    break


        if tm.get("altitude") < LANDING_TOLERANCE:
            print("\rAngle control end")
            break



def angle_between(v1, v2):
    dot = sum(a * b for a, b in zip(v1, v2))
    mag1 = math.sqrt(sum(a * a for a in v1))
    mag2 = math.sqrt(sum(b * b for b in v2))
    return math.degrees(math.acos(dot / (mag1 * mag2)))


