from floater.control import stabilization, rolling_control, landing_monitor,landing_test_launch
from floater.parts import emergency_landing
import threading
import time


def launch(conn, vessel):
    if not vessel.name.lower().startswith('floater'):
        raise Exception('Not floater, choose correct control protocol.')
    else:
        print(' Vessel "floater" activated.')

    landing_thread = threading.Thread(target=landing_monitor, args=(conn, vessel), daemon=False)
    landing_thread.start()

    stabilization(conn, vessel)
    rolling_control(conn, vessel)


def landing(conn, vessel):
    if not vessel.name.lower().startswith('floater'):
        raise Exception('Not floater, choose correct control protocol.')
    else:
        print(' Vessel "floater" activated.')

    landing_thread = threading.Thread(target=landing_monitor, args=(conn, vessel), daemon=False)
    landing_thread.start()


def landing_test(conn, vessel):
    if not vessel.name.lower().startswith('floater'):
        raise Exception('Not floater, choose correct control protocol.')
    else:
        print(' Vessel "floater" activated.')

    landing_thread = threading.Thread(target=landing_monitor, args=(conn, vessel), daemon=False)
    landing_thread.start()

    landing_test_launch(conn, vessel)
