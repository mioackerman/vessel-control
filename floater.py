def run(conn, vessel):
    import time

    print("Floater protocol activated.")

    vessel.control.sas = True
    vessel.control.throttle = 1.0
    vessel.control.activate_next_stage()
    print('Stage 1 activated')

    time.sleep(63)

    vessel.control.activate_next_stage()
    print('stage 2 activated')
