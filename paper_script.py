import abstracted_api as api
import carla
import time
import math
import random
from typing import Dict, Any, Tuple, Optional, Callable

# --- Global State and Constants ---
SIMULATION_DURATION_SECONDS = 30 
APP_LOGIC_INTERVAL_SECONDS = 5.0
CHARGE_STATION_LOCATION = (10.0, 10.0, 10.0) 

# =======================================================
# === MOCKS & APPLICATION LOGIC (Defined for the Edge Node) ===
# =======================================================

def detector(drone_frame_data: str) -> Tuple[Optional[list], Optional[str]]:
    """MOCK: Simulates a detector returning a normalized bounding box."""
    if 'd1' in drone_frame_data or 'd2' in drone_frame_data:
        # Guarantee success (>0.6 coverage) to trigger battery check and hand-off logic
        return [0.1, 0.1, 0.8, 0.8], f"frame_simulated_{random.randint(0, 100)}" 
    return None, None

def tracker(bbox: list) -> list:
    """MOCK: Simulates object tracking with slight movement."""
    if bbox:
        return [b + random.uniform(-0.01, 0.01) for b in bbox] 
    return []

def recv(drone_name: str) -> Optional[str]:
    """MOCK: Simulates receiving the streamed data from the drone (or connection loss)."""
    if drone_name == "d1":
        return "d1_frame_data" 
    elif drone_name == "d2":
        return "d2_frame_data" 
    return None


def app_func(context: Dict[str, Any]):
    """
    Core application logic to be run on the Edge Node every interval.
    """
    
    # Use user-friendly names for registration/lookup
    D1_NAME = 'd1'
    D2_NAME = 'd2'
    CSL_NAME = 'csl'

    # Retrieve object lookup function from context
    get_object_info = context['get_object_info']
    
    # The application logic should check if D1 is still registered, 
    # as it may have been destroyed by a previous call.
    try:
        d1_info = get_object_info(D1_NAME)
    except ValueError:
        print(f"D1 already destroyed. Application loop stopping.")
        return
    
    print(f"\n--- Edge Application Pipeline Executing ---")
    
    # Receive data from primary drone (d1)
    dl_frame = recv(D1_NAME) 
    detect_bbox, frame_name = detector(dl_frame)
    
    if dl_frame:
        print(f"D1: Processing frame {frame_name}.")
        track_bbox = tracker(detect_bbox)
        
        # Check for FOV coverage (> 0.6 of FOV)
        if (track_bbox[2] - track_bbox[0]) > 0.6 and (track_bbox[3] - track_bbox[1]) > 0.6: 
            
            # Check battery level
            current_battery = api.get_battery_level(D1_NAME)
            # Use api.get_state to fetch battery threshold
            min_bat = api.get_state(D1_NAME).get('battery_minimum_threshold', 5.0) 
            
            if current_battery >= min_bat:
                print(f"D1: Battery OK ({current_battery:.1f}%). Maintaining pursuit.")
                # This block would normally contain api.move logic
                
            else:
                # Low battery: Initiate Hand-off
                print(f"D1: Battery LOW ({current_battery:.1f}%). Initiating hand-off.")
                
                new_drone_name = api.get_nearest_object(D1_NAME, target_type="drone") 
                
                if new_drone_name:
                    d1_pos = api.get_position(D1_NAME)
                    
                    # D2 moves to D1's position
                    print(f"D2: Moving to D1's location for Hand-off.")
                    api.move(new_drone_name, d1_pos[0], d1_pos[1], d1_pos[2], speed=10.0) 
                    
                    nd_frame = recv(D2_NAME) 
                    similarity = random.uniform(0.7, 1.0) 
                    thres = 0.8
                    
                    if nd_frame and similarity >= thres:
                        print(f"D1: Hand-off SUCCESSFUL (Similarity: {similarity:.2f}).")
                        api.return_to_location(D1_NAME, *CHARGE_STATION_LOCATION, speed=5.0)
                        api.unregister(D1_NAME) 
                        print("D1 Mission terminated. D2 is now primary drone.")
                        return
                    
                    else:
                        print("Hand-off failed. D1 attempting recovery.")
                
                else:
                    print("ERROR: No nearby drone for hand-off. D1 initiating emergency return.")
                    api.return_to_location(D1_NAME, *CHARGE_STATION_LOCATION, speed=3.0)
        
        else:
            print("DL: Target out of optimal FOV. Adjusting flight path.")

# ----------------------------------------------------------------------


# =================================================
# === EXPERIMENT RUNNER (Simplified Entry Point) ===
# =================================================

def experiment_runner():
    """Defines the experiment setup and runs the simulation loop."""
    
    try:
       
        if not api.init(host='127.0.0.1', port=2000, seed=42):
            return
            
        api.set_synchronous_mode(fps=20)
        
       
        
        # Vehicle (target for drones)
        target_v1 = api.register_vehicle(name='target_v1', model='vehicle.lincoln.mkz', autopilot=True)
        
        # Drones
        d1 = api.register_drone('d1', 100.0, 10.0, 50.0, 
                                follows=target_v1, 
                                battery_start=70.0, 
                                battery_capacity_wh=60,
                                enable_tracking=True)
        d2 = api.register_drone('d2', 120.0, 30.0, 50.0, 
                                follows=target_v1, 
                                battery_start=95.0, 
                                battery_capacity_wh=60,
                                enable_tracking=True)
        
        # Edge Node and Charging Station
        enl = api.register_edgenode('enl', 10.0, 10.0, 5.0)
        csl = api.register_charge_station('csl', *CHARGE_STATION_LOCATION)
        
        # Link Services
        api.link_drone_to_edge(d1, enl)
        api.link_drone_to_edge(d2, enl)
        api.plug_n_play_model('energy_d1', d1, 'docker.io/energy1') 
        api.setup_stream(d1, "127.0.0.1:9090")
        api.setup_stream(d2, "127.0.0.1:9090")

        # Run the simulation loop
        api.run_experiment(duration_seconds=SIMULATION_DURATION_SECONDS, 
                           app_func=app_func, 
                           app_interval_seconds=APP_LOGIC_INTERVAL_SECONDS)

    except Exception as e:
        print(f"\n!!! FATAL EXPERIMENT ERROR: {e} !!!")
    finally:
        # Cleanup
        api.cleanup()
        print("\nExperiment finished. All resources cleaned up.")


if __name__ == '__main__':
    experiment_runner()