import threading

import krpc
from floater import floater
from floater.control_orientation_monitor import control_orientation

try:
    conn = krpc.connect(
        name='edward',
        address='192.168.2.29',
        rpc_port=50000,
        stream_port=50001
    )
    vessel = conn.space_center.active_vessel

    print("✅ Connected to:", vessel.name)
except Exception as e:
    print("❌ Failed to connect:", e)
    exit(1)

print("\nSelect protocol:")
print("1. Floater launch")
print("2. Float landing")
print("3. Float Launch & Landing Test")

choice = input("Enter number: ").strip()

if not choice:
    print("⚠️ No input received. Exiting.")
    exit(0)

if choice == "1":
    floater.launch(conn, vessel)
elif choice == "2":
    floater.landing(conn, vessel)
elif choice == "3":
    floater.landing_test(conn, vessel)
else:
    print("⚠️ Unknown selection. Exit.")
