import threading

class TelemetryManager:
    _streams = {}
    _lock = threading.Lock()
    _initialized = False
    
    @classmethod
    def init_streams(cls, conn, vessel):
        with cls._lock:
            if cls._initialized:
                return
            
            srf = vessel.surface_reference_frame
            orf = vessel.orbit.body.reference_frame
            flight_srf = vessel.flight(srf)
            flight_orb = vessel.flight(orf)
            
            cls._streams = {
                "altitude": conn.add_stream(getattr, flight_srf, "surface_altitude"),
                "speed": conn.add_stream(getattr, flight_srf, "speed"),
                "vertical_speed": conn.add_stream(getattr, flight_orb, "vertical_speed"),
                "forward_vector": conn.add_stream(vessel.direction, srf),
                "velocity_vector": conn.add_stream(getattr, flight_orb, "velocity"),
                "retrograde_vector": conn.add_stream(getattr, flight_orb, "retrograde"),
                "mass": conn.add_stream(getattr, vessel, "mass"),
                "gravity": conn.add_stream(getattr, vessel.orbit.body, "surface_gravity"),
                "angular_velocity": conn.add_stream(vessel.angular_velocity, srf),
                "available_thrust": conn.add_stream(getattr, vessel, "available_thrust"),
                "liquid_fuel": conn.add_stream(vessel.resources.amount, "LiquidFuel"),
                "oxidizer": conn.add_stream(vessel.resources.amount, "Oxidizer"),
            }
      
    @classmethod                         
    def get(cls, name):
        with cls._lock:
            stream = cls._streams.get(name)
            return stream() if stream else None
    
    @classmethod
    def keys(cls):
        return list(cls._streams.keys())