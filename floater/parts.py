from threading import Event
import time

def check_fuel(conn, vessel):
    active_engines = [e for e in vessel.parts.engine if e.active]
    for engine in active_engines:
        print(f"Active Engine: {engine.part.title}")
        for propellant in engine.propellants:
            available = vessel.resources.amount(propellant.name)
            print(f"  {propellant.name}: {available}")