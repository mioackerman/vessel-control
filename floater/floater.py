from floater.control import stabilization, rolling_control, landing_monitor
from floater.emergency import emergency_landing
import threading
import time

def launch(conn, vessel):
    if vessel.name.lower() != 'floater':
        raise Exception('Not floater, choose correct control protocol.')
    else:
        print(' Vessel "floater" activated.')

    landing_thread = threading.Thread(target=landing_monitor, args=(conn, vessel), daemon=False)
    landing_thread.start()

    stabilization(conn, vessel)
    rolling_control(conn, vessel)


