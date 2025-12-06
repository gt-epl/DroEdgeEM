# **API Reference**

This API simplifies the management of multi-threaded camera data, distributed edge computing simulation parameters, and object control by using unique string IDs instead of CARLA actor objects.

## **Setup and Initialization**

To use this API, two prerequisites must be met: 
- A running CARLA Server instance   
- [CARLA Python API package](https://carla.readthedocs.io/en/latest/start_quickstart/)

You must also establish a connection to the CARLA server using the `init` function, configuring default parameters like resolution and framerate for newly created drones:  

```python
import abstracted_api as carla_sim

if carla_sim.init(host='127.0.0.1', port=2000, resolution_x=1280, resolution_y=720, fps=15):  
    print("API is ready.")  
else:  
    print("Initialization failed.")
```

## **Object Lifecycle Management**

The API manages all object lifecycle events, from multi-threaded startup to graceful shutdown.

### **Spawning Objects**

#### **Drones**

Register and spawn a drone using `register_drone`:

| Parameter | Type | Description |
| :---- | :---- | :---- |
| `name` | `str` | Unique identifier (e.g., "d1"). |
| `x, y, z` | `float` | Absolute world coordinates. |
| `follows` | `Optional[str]` | Name of vehicle to track. |
| `battery_start` | `float` | Starting battery level (0-100). Default: 100.0 |
| `battery_capacity_wh` | `float` | Battery capacity in Wh. Default: 60 |
| `enable_tracking` | `bool` | Enable automatic tracking. Default: True |
| `**kwargs` | `Dict` | Additional config overrides. |

**Example Usage:**  
```python
car_name = carla_sim.register_vehicle(
    name="target_car",
    x=100, y=50, z=0.5,
    autopilot=True
)

drone_name = carla_sim.register_drone(
    name="d1",
    x=100, y=50, z=30,
    follows="target_car",
    battery_start=80.0
)
```

#### **Vehicles**

Register and spawn a vehicle using `register_vehicle`:

| Parameter | Type | Description |
| :---- | :---- | :---- |
| `name` | `str` | Unique identifier. |
| `x, y, z` | `Optional[float]` | Spawn coordinates. If omitted, uses spawn point. |
| `model` | `str` | Vehicle blueprint ID. Default: 'vehicle.lincoln.mkz_2020' |
| `autopilot` | `bool` | Enable autopilot. Default: True |
| `spawn_point_index` | `Optional[int]` | Index of map spawn point to use. |


#### **Infrastructure**

Register edge nodes and charge stations:

- `register_edgenode(name, x, y, z)`: Registers a new edge node at the specified world coordinates for latency calculation.
- `register_charge_station(name, x, y, z)`: Registers a conceptual charge station (return-to-home destination) at the specified location.

| Parameter | Type | Description |
| :---- | :---- | :---- |
| `name` | `str` | Unique identifier for the infrastructure. |
| `x, y, z` | `float` | Absolute world coordinates. |
### **Destruction and Cleanup**

- `unregister(name_or_id)`: Destroys the specified object actor. For drones, it gracefully signals all worker threads to stop before destruction.
- `unregister_all()`: Destroys all registered drones and vehicles. Must be called before exiting the simulation to ensure clean shutdown.

## **Control, Movement, and Networking**

### **Object Control and Movement**

All drone movement uses asynchronous target setting. The drone smoothly flies toward the target rather than instantly teleporting.

- `move(name_or_id, x, y, z, speed=None)`: Sets the object's absolute target location. For drones, optionally sets movement speed in m/s.
- `return_to_location(name_or_id, x, y, z, speed=None)`: Moves drone to specified location (typically used for returning home/charging). Default speed: 10.0 m/s
- `get_nearest_object(requester_name_or_id, target_type)`: Finds the name of the nearest object of a given type (e.g., "chargestation", "edgenode").


### **Edge Computing and Networking**

These functions simulate the architecture of a distributed system where drones transmit data to nearby edge nodes.

- `link_drone_to_edge(drone_name, node_name)`: Links a drone to an edge node. This link is used to calculate simulated network latency based on physical distance.
- `setup_stream(drone_name, endpoint_url)`: Opens a TCP connection to the specified endpoint (e.g., "127.0.0.1:9999") and begins streaming the drone's compressed camera feed.
- `plug_n_play_model(model_id, infra_name, docker_url)`: **[MOCK]** Simulates associating an external model with an infrastructure unit.


## **State Management and Queries**

The API provides tools to query the current state and performance of all managed objects.

- `get_state(name_or_id)`: Returns a comprehensive dictionary containing location, rotation, velocity (vehicles), battery level, and streaming status.
- `get_battery_level(name_or_id)`: Returns the drone's current battery charge as a percentage (0-100).
- `get_position(name_or_id)`: Returns a tuple `(x, y, z)` of the object's current world coordinates.

