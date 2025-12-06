import uuid
from typing import Optional, Tuple, Union, Dict, Any, List, Callable
import carla
import random
import math
import socket
import time

try:
    import drone_manager as drone_api
except ImportError:
    print("FATAL ERROR: Could not import drone_manager.py (or the file containing low-level CameraManager logic).")
    exit()

class ObjectRegistry:
    """
    Manages registered objects (drones/cameras and vehicles) with a unified interface 
    and handles translation between user-defined names ('d1') and internal IDs ('obj_1').
    """
    def __init__(self):
        self._objects = {}  
        self._name_to_id = {} 
        self._id_counter = 0
        self._world = None
        self._client = None
        self._default_drone_config = {}
    
    def set_world(self, world, client=None):
        """Set the CARLA world instance for spawning objects."""
        self._world = world
        self._client = client
        drone_api._api_world = world
    
    def set_default_drone_config(self, config: Dict[str, Any]):
        """Set default configuration for drone spawning."""
        self._default_drone_config = config
    
    def _generate_id(self) -> str:
        """Generate a unique object ID."""
        self._id_counter += 1
        return f"obj_{self._id_counter}"
    
    def register_object(self, obj, obj_type: str, name: Optional[str] = None) -> str:
        """
        Internal method to register an object, ensuring name uniqueness.
        """
        if name:
            if name in self._name_to_id:
                raise ValueError(f"Object name '{name}' is already registered.")
            
        obj_id = self._generate_id()
        
        if name:
            self._name_to_id[name] = obj_id

        self._objects[obj_id] = {
            "id": obj_id,
            "type": obj_type,
            "name": name or f"{obj_type}_{obj_id}",
            "object": obj
        }
        
        return obj_id
    
    def get_object(self, name_or_id: str) -> Dict[str, Any]:
        """Get registered object info by Name or ID."""
        obj_id = self._name_to_id.get(name_or_id) or name_or_id
        
        if obj_id not in self._objects:
            raise ValueError(f"Object ID/Name '{name_or_id}' not found in registry")
        return self._objects[obj_id]
    
    def get_id_by_name(self, name: str) -> str:
        """Get the unique internal object ID from its user-defined name."""
        obj_id = self._name_to_id.get(name)
        if obj_id is None:
             raise ValueError(f"Object name '{name}' not found in registry.")
        return obj_id
        
    def get_all_objects(self) -> List[Dict[str, Any]]:
        """Get all registered objects."""
        return list(self._objects.values())
    
    def remove_object(self, obj_id: str):
        """Remove object from registry and name map."""
        if obj_id in self._objects:
            name = self._objects[obj_id]['name']
            del self._objects[obj_id]
            if name in self._name_to_id:
                del self._name_to_id[name]


# Global registry instance
_registry = ObjectRegistry()


# =================================================================
## INITIALIZATION, EXECUTION, AND CLEANUP
# =================================================================

def init(host: str = '127.0.0.1', port: int = 2000, seed: Optional[int] = None, **drone_defaults) -> bool:
    """
    Initialize the API, connect to CARLA, and set up default configuration.
    """
    try:
        client = carla.Client(host, port)
        client.set_timeout(10.0)
        world = client.get_world()
        
        if seed is not None:
            traffic_manager = client.get_trafficmanager()
            traffic_manager.set_random_device_seed(seed)
            print(f"Autopilot seed set to: {seed}")
        
        _registry.set_world(world, client)
        
        # Set default drone configuration
        default_config = {
            'simulation_fps': drone_defaults.get('simulation_fps', 20),
            'camera_resolution_x': drone_defaults.get('resolution_x', 1920),
            'camera_resolution_y': drone_defaults.get('resolution_y', 1080),
            'fps': drone_defaults.get('fps', 10),
            'battery_minimum_threshold': drone_defaults.get('battery_threshold', 10.0),
            'output_dir': drone_defaults.get('output_dir', './output'),
        }
        _registry.set_default_drone_config(default_config)
        
        print(f"Connected to CARLA at {host}:{port}")
        return True
        
    except Exception as e:
        print(f"Failed to connect to CARLA: {e}")
        return False


def set_synchronous_mode(fps: int = 20):
    """
    Sets the CARLA server to synchronous mode with a fixed time step.
    Must be called after init().
    """
    if not _registry._world:
        raise RuntimeError("API not initialized. Call init() first.")

    world = _registry._world
    settings = world.get_settings()
    
    # Store original settings for cleanup
    world.__original_sync_mode = settings.synchronous_mode
    world.__original_fixed_delta = settings.fixed_delta_seconds

    settings.synchronous_mode = True 
    settings.fixed_delta_seconds = 1.0 / fps
    world.apply_settings(settings)
    _registry._client.get_trafficmanager().set_synchronous_mode(True)
    
    # Update internal FPS
    _registry._default_drone_config['simulation_fps'] = fps
    
    print(f"SUCCESS: Simulation set to Synchronous Mode at {fps} FPS.")
    
    # Start thread management tools
    drone_api.start_state_saving(world)

    # Initial ticks to process setup commands and unblock threads
    print("Performing initial sync ticks to unblock threads...")
    for _ in range(5):
        world.tick()
        time.sleep(1.0 / fps)


def run_experiment(duration_seconds: int, app_func: Callable[[Dict[str, Any]], None], 
                   app_interval_seconds: float, fps: Optional[int] = None):
    """
    Runs the main synchronous simulation loop and executes the app_func periodically.
    
    Args:
        duration_seconds (int): Total duration of the simulation run.
        app_func (Callable): The application logic function to execute.
        app_interval_seconds (float): The time interval (in simulated seconds) 
                                      between executions of app_func.
        fps (int, optional): The synchronous FPS. Uses the last set FPS if None.
    """
    if not _registry._world:
        raise RuntimeError("API not initialized. Call init() and set_synchronous_mode() first.")
    
    world = _registry._world
    sim_fps = fps or _registry._default_drone_config['simulation_fps']
    total_ticks = sim_fps * duration_seconds
    
    # Calculate the number of ticks between application executions
    app_tick_interval = max(1, round(sim_fps * app_interval_seconds))
    
    print(f"\nSimulation run started. Synchronous loop running for {duration_seconds} seconds.")
    print(f"Application logic will run every {app_interval_seconds} seconds ({app_tick_interval} ticks).")
    
    try:
        # Pass a context dictionary to the app_func containing helper methods
        context = {
            'get_id': _registry.get_id_by_name,
            'get_object_info': _registry.get_object
        }
        
        for i in range(total_ticks):
            # Advance the simulation state
            world.tick()

            # Execute the application logic based on the user-defined interval
            if i % app_tick_interval == 0: 
                app_func(context)

            # Log simulation time every second
            if i % sim_fps == 0:
                print(f"Simulated Time: {i/sim_fps:.0f}s / {duration_seconds}s")

    except Exception as e:
        print(f"An error occurred during the synchronous loop: {e}")


def cleanup():
    """Restores CARLA settings to their original state and destroys all actors."""
    if not _registry._world:
        return

    world = _registry._world
    
    # Stop state saving
    drone_api.stop_state_saving()
    
    # Destroy all actors
    unregister_all()
    
    # Restore original settings
    settings = world.get_settings()
    if hasattr(world, '__original_sync_mode'):
        settings.synchronous_mode = world.__original_sync_mode
        settings.fixed_delta_seconds = world.__original_fixed_delta
        world.apply_settings(settings)
        print("Restored CARLA settings to original (Asynchronous) mode.")

# =================================================================
## HIGH-LEVEL INFRASTRUCTURE REGISTRATION (Simplified Interface)
# =================================================================

def register_drone(name: str, x: float, y: float, z: float, 
                   follows: Optional[str] = None, 
                   battery_start: float = 100.0, 
                   battery_capacity_wh: float = 60,
                   enable_tracking: bool = True,
                   **kwargs) -> str:
    """Register and spawn a new drone with simplified tracking/battery settings."""
    if not _registry._world:
        raise RuntimeError("API not initialized.")
    
    target_actor = None
    if follows:
        try:
            # Resolve the vehicle name to the low-level CARLA actor object
            target_actor = _registry.get_object(follows)['object']
        except ValueError:
            print(f"Warning: Target vehicle '{follows}' not found. Drone starting without tracking.")

    config = _registry._default_drone_config.copy()
    config.update(kwargs) # Allow overriding any default config

    spawn_location = carla.Location(x=x, y=y, z=z)
    transform = carla.Transform(spawn_location)
    
    drone = drone_api.register_drone(
        world=_registry._world,
        transform=transform,
        drone_id=name,
        target_actor=target_actor,
        enable_tracking=enable_tracking,
        **config
    )
    
    drone.battery_level = battery_start
    drone.battery_capacity_wh = battery_capacity_wh
    
    obj_id = _registry.register_object(drone, "drone", name)
    print(f"Spawned and registered drone '{name}' (ID: {obj_id}) at ({x:.2f}, {y:.2f}, {z:.2f})")
    return name 


def register_vehicle(name: str, x: Optional[float] = None, y: Optional[float] = None, 
                     z: Optional[float] = None, model: str = 'vehicle.lincoln.mkz_2020', 
                     autopilot: bool = True, spawn_point_index: Optional[int] = None) -> str:
    """Register and spawn a new vehicle (carla.Vehicle actor)."""
    
    world = _registry._world
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.find(model)
    
    if x is not None and y is not None and z is not None:
        spawn_transform = carla.Transform(carla.Location(x=x, y=y, z=z))
    else:
        spawn_points = world.get_map().get_spawn_points()
        if spawn_point_index is not None:
             spawn_transform = spawn_points[spawn_point_index % len(spawn_points)]
        else:
             spawn_transform = random.choice(spawn_points)
    
    vehicle = world.try_spawn_actor(vehicle_bp, spawn_transform)
    
    if not vehicle:
        raise RuntimeError(f"Failed to spawn vehicle with filter: {model}")
    
    if autopilot and _registry._client:
        traffic_manager = _registry._client.get_trafficmanager()
        vehicle.set_autopilot(True, traffic_manager.get_port())
    
    obj_id = _registry.register_object(vehicle, "vehicle", name)
    print(f"Spawned and registered vehicle '{name}' (ID: {obj_id})")
    return name


def register_edgenode(name: str, x: float, y: float, z: float):
    """Registers a new edge node (for latency calculation)."""
    drone_api.register_edgenode(name, x, y, z)
    _registry.register_object({"location": {'x': x, 'y': y, 'z': z}}, "edgenode", name)
    return name


def register_charge_station(name: str, x: float, y: float, z: float):
    """Registers a conceptual charge station (return-to-home destination)."""
    _registry.register_object({"location": {'x': x, 'y': y, 'z': z}}, "chargestation", name)
    return name

# =================================================================
## LINKING, STREAMING, AND MODEL MANAGEMENT
# =================================================================

def link_drone_to_edge(drone_name: str, node_name: str) -> bool:
    """Links a registered drone to a registered edge node."""
    try:
        return drone_api.link_drone_edgenode(drone_name, node_name)
    except ValueError as e:
        print(f"ERROR: {e}")
        return False

#TODO: Finalize plug_n_play_model.
def plug_n_play_model(model_id: str, infra_name: str, docker_url: str):
    """MOCK: Simulates associating an external model with an infrastructure unit."""
    print(f"MOCK Model PnP: Recording model '{model_id}' from {docker_url} and linking to {infra_name}.")


def setup_stream(drone_name: str, endpoint_url: str) -> bool:
    """Starts streaming a drone's camera feed to a given TCP endpoint."""
    try:
        # The API uses the drone's name string
        return drone_api.stream_drone(drone_name, endpoint_url)
    except ValueError as e:
        print(f"ERROR: {e}")
        return False

# =================================================================
## STATE AND CONTROL FUNCTIONS (Using Names/IDs interchangeably)
# =================================================================

def unregister(name_or_id: str):
    """Unregister and destroy an object from the system."""
    obj_info = _registry.get_object(name_or_id)
    obj = obj_info["object"] # Low-level CameraManager object
    obj_id = obj_info["id"]
    obj_type = obj_info["type"]
    
    if obj_type == "drone":
        # Signal threads to stop immediately
        obj.signal_threads_to_stop()
        
        # Destroy the CARLA actor immediately 
        if obj.actor and obj.actor.is_alive:
            obj.actor.destroy()
        
        # Remove from the high-level registry
        _registry.remove_object(obj_id)
        print(f"Unregistered/Destroyed drone {obj_info['name']} (ID: {obj_id}) (Deferred cleanup).")
    
    elif obj_type == "vehicle":
        if obj.is_alive:
            obj.destroy()
        if _registry._world:
            _registry._world.tick()
        _registry.remove_object(obj_id)
        print(f"Destroyed vehicle {obj_info['name']} (ID: {obj_id})")
        
    else:
        # For passive objects (edgenode, chargestation), simply remove from registry
        _registry.remove_object(obj_id)
        print(f"Unregistered conceptual object {obj_info['name']} (ID: {obj_id})")

def unregister_all():
    """Unregister and destroy all objects, restoring clean slate."""
    print("Unregistering and destroying all objects...")
    
    # Destroy drones (handles threads/cleanup)
    drone_api.destroy_all_drones()
    
    # Destroy remaining CARLA actors (vehicles)
    vehicle_objects = [obj for obj in _registry.get_all_objects() if obj['type'] == 'vehicle']
    for obj_info in vehicle_objects:
        try:
            obj = obj_info['object']
            if obj.is_alive:
                obj.destroy()
        except Exception as e:
            print(f"Error destroying vehicle {obj_info['id']}: {e}")
            
    # Clean registry
    _registry._objects.clear()
    _registry._name_to_id.clear()
    
    if _registry._world:
        _registry._world.tick()
    print("All objects destroyed and unregistered.")


def get_position(name_or_id: str) -> Tuple[float, float, float]:
    """Get the current position of an object."""
    obj_info = _registry.get_object(name_or_id)
    obj = obj_info["object"]
    obj_type = obj_info["type"]
    
    if obj_type == "drone":
        if obj.actor is None or not obj.actor.is_alive:
            raise RuntimeError(f"Drone '{name_or_id}' actor is destroyed or invalid.")
        location = obj.actor.get_location()
    elif obj_type == "vehicle":
        location = obj.get_location()
    elif obj_type == "edgenode" or obj_type == "chargestation":
         loc_dict = obj['location']
         location = carla.Location(x=loc_dict['x'], y=loc_dict['y'], z=loc_dict['z'])
    else:
        raise ValueError(f"Cannot get position for object type: {obj_type}")
    
    return (location.x, location.y, location.z)

def get_state(name_or_id: str) -> Dict[str, Any]:
    """
    Gets the complete state dictionary for a specific object (drone or vehicle).
    """
    obj_info = _registry.get_object(name_or_id)
    obj = obj_info["object"]
    obj_type = obj_info["type"]
    
    if obj_type == "drone":
        return drone_api.get_state(obj.name)
        
    elif obj_type == "vehicle":
        # Construct a state dictionary for the vehicle
        transform = obj.get_transform()
        velocity = obj.get_velocity()
        
        return {
            "name": obj_info["name"],
            "id": name_or_id,
            "type": "vehicle",
            "is_alive": obj.is_alive,
            "transform": {
                "location": {"x": transform.location.x, "y": transform.location.y, "z": transform.location.z},
                "rotation": {"pitch": transform.rotation.pitch, "yaw": transform.rotation.yaw, "roll": transform.rotation.roll}
            },
            "velocity": {"x": velocity.x, "y": velocity.y, "z": velocity.z},
        }
        
    else:
        raise ValueError(f"State retrieval not supported for type: {obj_type}")

def get_battery_level(name_or_id: str) -> float:
    """Get the current battery level of a drone."""
    obj_info = _registry.get_object(name_or_id)
    obj_type = obj_info["type"]
    
    if obj_type != "drone":
        raise ValueError(f"Battery level only available for drones, not {obj_type}")
    
    return obj_info["object"].battery_level


def move(name_or_id: str, x: float, y: float, z: float, speed: Optional[float] = None):
    """Move an object to absolute world coordinates."""
    obj_info = _registry.get_object(name_or_id)
    obj = obj_info["object"]
    obj_type = obj_info["type"]
    
    if obj_type == "drone":
        location = carla.Location(x=x, y=y, z=z)
        drone_api.set_target_location(obj, location)
        
        if speed is not None:
            obj.linear_speed = speed
            
        print(f"Moving {obj_info['name']} to ({x:.2f}, {y:.2f}, {z:.2f})" + 
              (f" at {speed} m/s" if speed else ""))
    
    elif obj_type == "vehicle":
        transform = obj.get_transform()
        transform.location.x = x
        transform.location.y = y
        transform.location.z = z
        obj.set_transform(transform)
        print(f"Moved {obj_info['name']} to ({x:.2f}, {y:.2f}, {z:.2f})")
    
    else:
        raise ValueError(f"Movement not supported for type: {obj_type}")
        
        
def return_to_location(name_or_id: str, x: float, y: float, z: float, speed: Optional[float] = None):
    """Set a target location for a drone to move to, often used for returning home/charging."""
    move(name_or_id, x, y, z, speed=speed if speed is not None else 10.0)
    print(f"Drone {name_or_id} set to return to location ({x}, {y}, {z})")


def get_nearest_object(requester_name_or_id: str, target_type: str) -> Optional[str]:
    """
    Finds the name of the nearest object of a given type to the requester object.
    """
    try:
        requester_pos = get_position(requester_name_or_id)
    except ValueError:
        return None 

    min_distance = float('inf')
    nearest_name = None
    
    for obj_info in _registry.get_all_objects():
        if obj_info['name'] == requester_name_or_id or obj_info['type'] != target_type:
            continue
            
        try:
            target_pos = get_position(obj_info['name'])
            distance = math.sqrt(
                (target_pos[0] - requester_pos[0])**2 + 
                (target_pos[1] - requester_pos[1])**2 + 
                (target_pos[2] - requester_pos[2])**2
            )
            
            if distance < min_distance:
                min_distance = distance
                nearest_name = obj_info['name'] 
                
        except Exception:
            continue

    return nearest_name