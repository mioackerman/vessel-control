from multiprocessing import Process, Manager

from floater.control import stabilization, rolling_control, landing_monitor, landing_test_launch, status_monitor, \
    LANDING_ALTITUDE
import threading
import time

from floater.control_orientation_monitor import control_orientation
from floater.main_control_loop import main_loop


def init_floater(conn, vessel):
    ui(conn, vessel)


def calculate_landing_tolerance(vessel):
    srf = vessel.surface_reference_frame
    bbox = vessel.bounding_box(srf)
    return abs(bbox[0][1])


def ui(conn, vessel):
    print("1. Floater launch")
    print("2. Float landing")
    print("3. Float Launch & Landing Test")
    print("4. Float launch mode 2")

    choice = input("Enter number: ").strip()

    if not choice:
        print("⚠️ No input received. Exiting.")
        exit(0)

    if choice == "1":
        launch(conn, vessel)
    elif choice == "2":
        landing(conn, vessel)
    elif choice == "3":
        landing_test(conn, vessel)
    elif choice == "4":
        landing_test_2(conn, vessel)
    else:
        print("⚠️ Unknown selection. Exit.")


def launch(conn, vessel):
    if not vessel.name.lower().startswith('floater'):
        raise Exception('Not floater, choose correct control protocol.')
    else:
        print('Vessel "floater" activated.')

    landing_thread = threading.Thread(target=landing_monitor, daemon=False)
    landing_thread.start()

    stabilization(conn, vessel)
    rolling_control(conn, vessel)


def landing(conn, vessel):
    if not vessel.name.lower().startswith('floater'):
        raise Exception('Not floater, choose correct control protocol.')
    else:
        print(' Vessel "floater" activated.')
    calculate_landing_tolerance(vessel)
    landing_thread = threading.Thread(target=landing_monitor, args=(conn, vessel), daemon=False)
    landing_thread.start()


def landing_test(conn, vessel):
    if not vessel.name.lower().startswith('floater'):
        raise Exception('Not floater, choose correct control protocol.')
    else:
        print(' Vessel "floater" activated.')

    calculate_landing_tolerance(vessel)

    landing_thread = threading.Thread(target=landing_monitor, args=(conn, vessel), daemon=False)
    landing_thread.start()

    status_monitor_thread = threading.Thread(target=status_monitor, args=(conn, vessel), daemon=False)
    status_monitor_thread.start()

    # control_thread = threading.Thread(target=control_orientation, daemon=False)
    # control_thread.start()

    manager = Manager()
    shared_data = manager.dict()
    shared_data['LANDING_ALTITUDE'] = LANDING_ALTITUDE

    control_process = Process(target=control_orientation, args=(shared_data,), daemon=False)
    control_process.start()

    landing_test_launch(conn, vessel)

    control_process.join()


def landing_test_2(conn, vessel):
    if not vessel.name.lower().startswith('floater'):
        raise Exception('Not floater, choose correct control protocol.')
    else:
        print(' Vessel "floater" activated.')

    manager = Manager()
    shared_data = manager.dict()
    shared_data['LANDING_ALTITUDE'] = LANDING_ALTITUDE

    control_process = Process(target=control_orientation, args=(shared_data,), daemon=False)
    control_process.start()

    main_loop(conn, vessel)

    control_process.join()
