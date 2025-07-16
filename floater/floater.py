from floater.control import stabilization, rolling_control, landing_monitor, landing_test_launch, status_monitor
import threading
import time
from floater.control_orientation_monitor import control_orientation


def init_floater(conn, vessel):
    ui(conn, vessel)


def ui(conn, vessel):
    print("1. Floater launch")
    print("2. Float landing")
    print("3. Float Launch & Landing Test")

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

    landing_thread = threading.Thread(target=landing_monitor, args=(conn, vessel), daemon=False)
    landing_thread.start()


def landing_test(conn, vessel):
    if not vessel.name.lower().startswith('floater'):
        raise Exception('Not floater, choose correct control protocol.')
    else:
        print(' Vessel "floater" activated.')

    landing_thread = threading.Thread(target=landing_monitor, args=(conn, vessel), daemon=False)
    landing_thread.start()

    status_monitor_thread = threading.Thread(target=status_monitor, daemon=False)
    status_monitor_thread.start()

    control_thread = threading.Thread(target=control_orientation, daemon=False)
    control_thread.start()

    landing_test_launch(conn, vessel)
