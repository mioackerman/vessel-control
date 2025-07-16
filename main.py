import threading
import krpc
from floater import floater, telemetry
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
    
telemetry.TelemetryManager.init_streams(conn, vessel)   # initiating telemetry manager


print("\nSelect vessel type:")
print("\n1. Floater series")

choice = input("Enter number: ").strip()

if not choice:
    # print("⚠️ No input received. Exiting.")
    # exit(0)
    floater.init_floater(conn, vessel)

elif choice == "1":
    floater.init_floater(conn, vessel)

else:
    print("⚠️ Unknown selection. Exit.")