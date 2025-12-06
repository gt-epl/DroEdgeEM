import math
from queue import Queue
import threading
import os
import time
import cv2
import numpy as np
import json
import carla
import socket

_active_cameras = []
_api_world = None
_edge_nodes = {} 

# Initialize saved state file
STATE_FILE = "saved_state.json"
LATEST_STATE_FILE = "latest_saved_state.json"
_state_saving_thread = None
_stop_save_state_event = threading.Event()
_state_file_lock = threading.Lock()

# ==================================
# === CameraManager Class 
# ==================================

# === CAMERA MANAGER CLASS ===
class CameraManager:
    """
    Manages a camera sensor in the CARLA simulator, including image capture, 
    data saving, debug streaming, and autonomous movement logic.
    """
    def __init__(self, world, simulation_fps, camera_resolution_x, camera_resolution_y, transform, output_dir, target_actor, fps, battery_minimum_threshold, name="Camera", battery_drain_function=None, fps_function=None, enable_tracking = False, max_speed = 60):
        """
        Initializes the CameraManager with simulation and camera parameters.

        Args:
            world (carla.World): The CARLA world object.
            simulation_fps (float): The simulation's tick rate.
            camera_resolution_x (int): Horizontal resolution of the camera.
            camera_resolution_y (int): Vertical resolution of the camera.
            transform (carla.Transform): Initial transform for the camera.
            output_dir (str): Directory to save output images.
            target_actor (carla.Actor): The actor to track (e.g., a vehicle).
            fps (int): The desired frames per second for image capture.
            battery_minimum_threshold (float): Battery percentage to trigger shutdown.
            name (str): A descriptive name for the camera.
            battery_drain_function (callable, optional): Custom function for battery drain.
            fps_function (callable, optional): Custom function to update FPS.
            enable_tracking (bool): If True, enables autonomous tracking.
            max_speed (int): The max speed at which the drone can travel
        """
        world_snapshot = world.get_snapshot()
        simulation_time = world_snapshot.timestamp.elapsed_seconds
        self.world = world
        self.transform = transform
        self.output_dir = output_dir
        self.name = name
        self.target_actor = target_actor # Used for tracking vehicle
        self.last_save_time = 0.0 # Used for fps cap
        self.fps = fps
        self.fps_function = fps_function # Logic for updating fps every tick

        self.actor = None
        self.linked_edge_node = None
        self.active = False
        self.simulation_fps = simulation_fps
        self.camera_resolution_x = camera_resolution_x
        self.camera_resolution_y = camera_resolution_y
        self.battery_capacity_wh = 45 # arbitrary default value
        self.battery_level = 100.0
        self.battery_minimum_threshold = battery_minimum_threshold
        self.battery_drain_function = battery_drain_function
        self.startTime = simulation_time
        self.runTime = 0
        self.save_queue = Queue()
        self.processing_queue = Queue()
        self.is_streaming = False      
        self.stream_socket = None      
        self.stream_endpoint = None

        self.show_debug_stream = True  # flag to enable/disable the stream
        self.stream_event = threading.Event()  # New event to signal frame readiness
        self.stream_thread = None

        # -- Controls Targets --
        self.enable_controls = False
        self.enable_track_vehicle = enable_tracking
        self.pitch_target = transform.rotation.pitch
        self.roll_target = transform.rotation.roll
        self.yaw_target = transform.rotation.yaw
        self.x_target = transform.location.x
        self.y_target = transform.location.y
        self.z_target = transform.location.z
        self.linear_speed = 0
        self.max_speed = max_speed

        self.distances_to_others = {}

        # --- Profiling Attributes ---
        self.frame_arrival_deltas = []
        self.processing_times = []
        self.processing_times_real = []
        self.statemanager_times = []
        self.saving_times = []
        self.last_arrival_time = 0.0
        
        self.writer_thread = None
        self.statemanager_thread = None
        self.processing_thread = None
        self.stop_flag = threading.Event()

        self.actor_mutex = threading.Lock()
        # Required to prevent cameras from attempting to stream or transform after they are destroyed 

        self.latest_frame = None
        
        os.makedirs(self.output_dir, exist_ok=True)
        self.spawn()

    def spawn(self):
        """Spawns the camera actor and starts its management threads."""
        blueprint_library = self.world.get_blueprint_library()
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(self.camera_resolution_x))
        camera_bp.set_attribute('image_size_y', str(self.camera_resolution_y))
        camera_bp.set_attribute('sensor_tick', str(0.0))

        if self.show_debug_stream:
            self.stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
            self.stream_thread.start()

        self.actor = self.world.try_spawn_actor(camera_bp, self.transform)
        if not self.actor:
            print(f"ERROR: Failed to spawn {self.name}.")
            return

        print(f"Spawned {self.name} (ID: {self.actor.id})")
        
        
        # Start the dedicated threads for this camera
        self.writer_thread = threading.Thread(target=self._writer_loop)
        self.statemanager_thread = threading.Thread(target=self._statemanager_loop)
        self.processing_thread = threading.Thread(target=self._processing_loop)
        self.writer_thread.start()
        self.statemanager_thread.start()
        self.processing_thread.start()

        # Start listening for image data
        self.actor.listen(self._stream_callback)

    def _stream_loop(self):
        """Dedicated thread to display the live video stream."""
        print(f"[{self.name}] Stream thread started. Press 'q' in the window to stop.")
        while not self.stop_flag.is_set():
            if self.stream_event.wait(timeout=1.0):
                self.stream_event.clear()
                try:
                    frame = self.latest_frame
                    if frame is not None:
                        cv2.imshow(f"{self.name} Debug Stream", frame)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            self.show_debug_stream = False
                            print(f"[{self.name}] Debug stream stopped by user.")
                            cv2.destroyWindow(f"{self.name} Debug Stream")
                            break
                except Exception as e:
                    print(f"[{self.name}] Error in stream thread: {e}")
        
        print(f"[{self.name}] Stream thread stopped.")

    def to_dict(self):
        """Converts the camera's current state to a JSON-serializable dictionary."""
        state = {
            "simulation_fps": self.simulation_fps,
            "camera_resolution_x": self.camera_resolution_x,
            "camera_resolution_y": self.camera_resolution_y,
            "output_dir": self.output_dir,
            "target_actor_id": self.target_actor.id if self.target_actor else None,
            "fps": self.fps,
            "battery_minimum_threshold": self.battery_minimum_threshold,
            "name": self.name,
            "battery_level": round(self.battery_level, 2),
            "runtime": self.runTime,
            "output_dir": self.output_dir,
        }
        if self.actor and self.actor.is_alive:
            try:
                transform = self.actor.get_transform()
                location = transform.location
                rotation = transform.rotation
                state["transform"] = {
                    "location": {"x": location.x, "y": location.y, "z": location.z},
                    "rotation": {"pitch": rotation.pitch, "yaw": rotation.yaw, "roll": rotation.roll}
                }
            except RuntimeError:
                # Catch the "destroyed actor" error just in case, and mark transform as None
                state["transform"] = None
        else:
            state["transform"] = None
            
        return state

    def _writer_loop(self):
        """The 'Consumer' thread that saves images from this camera's queue."""
        print(f"[{self.name}] Writer thread started.")
        while not self.stop_flag.is_set():
            try:
                save_task = self.save_queue.get(timeout=1.0)
                if save_task is None: break

                # Time the saving operation
                save_start_time = time.perf_counter()
                #cv2.imwrite(save_task['frame_path'], save_task['image_array'])
                save_end_time = time.perf_counter()
                self.saving_times.append((save_end_time - save_start_time) * 1000)

                self.save_queue.task_done()
            except Exception:
                continue
        print(f"[{self.name}] Writer thread stopped.")

    def _statemanager_loop(self):
        """A dedicated thread to manage this camera's states."""
        print(f"[{self.name}] Statemanager thread started.")
        while not self.stop_flag.is_set() and self.actor and self.actor.is_alive:
            self.world.wait_for_tick()

            work_start_time = time.perf_counter()

            world_snapshot = self.world.get_snapshot()
            current_time = world_snapshot.timestamp.elapsed_seconds
            self.runTime = current_time - self.startTime
            if self.fps_function:
                self.fps_function(self)

            if self.battery_drain_function:
                self.battery_drain_function(self)
            else: 
                speed = abs(self.linear_speed)
                energy_per_sec = -1 * abs(-0.0516 * speed**4 + 0.4298 * 
                                speed**3 - 1.2804 * speed**2 
                                + 1.5816 * speed - 0.6251)

                # Convert from Watt-hours (Wh) to Joules (J)
                capacity_j = self.battery_capacity_wh * 3600  

                # Energy consumed in this timestep
                energy_used = -1 * energy_per_sec * (1/self.simulation_fps) # convert to energy per tick
                percent_depletion = (energy_used / capacity_j) * 100

                # Update state
                self.battery_level = max(0.0, self.battery_level - percent_depletion)

            if self.battery_level < self.battery_minimum_threshold:
                print(f"--- {self.name} (ID: {self.actor.id}) battery depleted! Shutting down. ---")
                self.stop_flag.set()
                break

            if self.enable_track_vehicle:
                self.follow_target_vehicle_statemanager(0)

            if(self.enable_controls):
                self.move_camera_vector_test()

            current_distances = {}
            if self.actor and self.actor.is_alive:
                my_location = self.actor.get_location()
                # Iterate through the global list of all cameras
                for other_cam in _active_cameras:
                    if other_cam is self:
                        continue # Skip calculating distance to self
                    
                    # Check if the other camera's actor is valid
                    if other_cam.actor and other_cam.actor.is_alive:
                        try:
                            other_location = other_cam.actor.get_location()
                            distance = my_location.distance(other_location)
                            current_distances[other_cam.name] = round(distance, 2)
                        except Exception as e:
                            print(f"[{self.name}] Could not get distance to {other_cam.name}: {e}")
            
            # Update the instance's distance dictionary
            self.distances_to_others = current_distances

            work_end_time = time.perf_counter()
            self.statemanager_times.append((work_end_time - work_start_time) * 1000)

        print(f"[{self.name}] Battery thread stopped.")


    def _processing_loop(self):
        """The 'Processor' thread that converts raw images and handles streaming."""
        print(f"[{self.name}] Processing thread started.")
        while not self.stop_flag.is_set():
            try:
                image = self.processing_queue.get(timeout=1.0)
                if image is None: 
                    break

                # START TIMING
                process_start_time = time.perf_counter()

                # --- Image Conversion ---
                image_array = np.frombuffer(image.raw_data, dtype=np.uint8)
                image_array = image_array.reshape((image.height, image.width, 4))[:, :, :3]
                image_array = np.ascontiguousarray(image_array.copy())

                if self.show_debug_stream:
                    self.latest_frame = image_array
                    self.stream_event.set() 
                
                if self.is_streaming and self.stream_socket:
                    try:
                        # Encode as JPEG for much smaller network footprint
                        result, buffer = cv2.imencode('.jpg', image_array, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                        
                        if result:
                            # Get the byte data and its length
                            data = buffer.tobytes()
                            data_length = len(data)
                            
                            # --- LATENCY EMULATION ---
                            simulated_delay = self._calculate_network_latency(data_length)
                            if simulated_delay > 0:
                                time.sleep(simulated_delay)
                            # -------------------------

                            # Send message with simple framing: 4-byte big-endian length header + data
                            self.stream_socket.sendall(data_length.to_bytes(4, 'big') + data)
                        
                    except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError) as e:
                        print(f"[{self.name}] Stream connection lost: {e}. Stopping stream.")
                        self.is_streaming = False
                        if self.stream_socket:
                            self.stream_socket.close()
                        self.stream_socket = None
                        self.stream_endpoint = None
                    except Exception as e:
                        print(f"[{self.name}] Error during streaming: {e}")
                


                # --- File Saving Logic ---

                # cam_transform = self.actor.get_transform()
                # vehicle_transform = self.target_actor.get_transform()
                # ... (rest of file path generation) ...
                # filepath = os.path.join(self.output_dir, f"frame_{image.frame:06d}_{name}.png")
                # save_task = {
                #     'frame_path': filepath,
                #     'image_array': image_array
                # }
                # self.save_queue.put(save_task)
                
                # --- END TIMING ---
                process_end_time = time.perf_counter()
                self.processing_times_real.append((process_end_time - process_start_time) * 1000)
                
                self.processing_queue.task_done()

            except Exception as e:
                # Catch queue.Empty exception if it times out
                if "Empty" in str(e):
                    continue
                print(f"[{self.name}] Error in processing loop: {e}")
                
        print(f"[{self.name}] Processing thread stopped.")

    def _calculate_network_latency(self, data_size_bytes):
        """
        Calculates transmission latency based on distance to the linked edge node
        using a simplified Shannon-Hartley channel capacity model.
        """
        if not self.linked_edge_node or self.linked_edge_node not in _edge_nodes:
            return 0.0 # No simulated latency if not linked

        # Get positions
        try:
            node_data = _edge_nodes[self.linked_edge_node]
            # Note: We assume register_edgenode packs x,y,z into 'location' dict
            node_loc_dict = node_data['location']
            node_location = carla.Location(x=node_loc_dict['x'], y=node_loc_dict['y'], z=node_loc_dict['z'])
            
            if self.actor and self.actor.is_alive:
                drone_location = self.actor.get_location()
            else:
                return 0.0
            
            # Calculate Distance (in meters)
            distance = drone_location.distance(node_location)
            if distance < 1.0: distance = 1.0 # Clamp to 1m to prevent division by zero/infinite signal
        except Exception as e:
            print(f"[{self.name}] Latency calc error: {e}")
            return 0.0

        # Base Bandwidth
        bandwidth_hz = 20e6  # 20 MHz channel width
        
        # Signal decay model (Inverse Square Law): Signal ~ 1 / d^2
        # Assume at 10 meters, we have a decent SNR of 30dB (1000 linear)
        ref_distance = 10.0
        ref_snr_linear = 1000.0 
        
        # Current SNR = Ref_SNR * (Ref_Dist / Curr_Dist)^2
        current_snr_linear = ref_snr_linear * ((ref_distance / distance) ** 2)
        
        # Shannon-Hartley Theorem: Capacity (bits/s) = B * log2(1 + S/N)
        channel_capacity_bps = bandwidth_hz * math.log2(1 + current_snr_linear)
        
        # Calculate Transmission Time
        data_bits = data_size_bytes * 8
        transmission_latency = data_bits / channel_capacity_bps
        
        # Small base processing latency 
        total_latency = transmission_latency + 0.005
        
        return total_latency
    
    def _stream_callback(self, image):
        """
        This callback is triggered by the CARLA sensor listener. It processes the
        image and hands it off to the writer thread.
        """

        process_start_time = time.perf_counter()

        world_snapshot = self.world.get_snapshot()
        current_time = world_snapshot.timestamp.elapsed_seconds


        current_arrival_time = time.perf_counter()
        if self.last_arrival_time > 0:
            self.frame_arrival_deltas.append((current_arrival_time - self.last_arrival_time) * 1000)
        self.last_arrival_time = current_arrival_time

        # Fps cap logic
        if current_time - self.last_save_time < (1.0 / self.fps):
            return
        
        self.last_save_time = current_time

        self.processing_queue.put(image)


        process_end_time = time.perf_counter()

        self.processing_times.append((process_end_time - process_start_time) * 1000)
    

    def signal_threads_to_stop(self):
        """A non-blocking method that signals all threads to terminate."""
        print(f"[{self.name}] Signaling threads to stop...")
        self.stop_flag.set()

        # Unblock the writer thread's queue.get()
        if self.writer_thread and self.writer_thread.is_alive():
            self.save_queue.put(None)
        
        # Unblock the processing thread's queue.get()
        if self.processing_thread and self.processing_thread.is_alive():
            self.processing_queue.put(None)
        
        # Unblock the stream thread's event.wait()
        if self.show_debug_stream and self.stream_thread and self.stream_thread.is_alive():
            self.stream_event.set()

    def join_threads(self):
        """Waits for all threads to join. Call this *after* a world.tick()."""
        
        if self.writer_thread and self.writer_thread.is_alive():
            self.writer_thread.join()
            print(f"[{self.name}] Writer thread joined.")

        if self.statemanager_thread and self.statemanager_thread.is_alive():
            self.statemanager_thread.join()
            print(f"[{self.name}] Statemanager thread joined.")

        if self.processing_thread and self.processing_thread.is_alive():
            self.processing_thread.join()
            print(f"[{self.name}] Processing thread joined.")
            
        if self.show_debug_stream and self.stream_thread and self.stream_thread.is_alive():
            self.stream_thread.join()
            print(f"[{self.name}] Stream thread joined.")
        
        print(f"[{self.name}] All threads have been joined.")

    def destroy_actor(self):
        """Destroys the CARLA actor. Call this *after* all threads are joined."""
        with self.actor_mutex:
            if self.actor and self.actor.is_alive:
                self.actor.stop()
                self.actor.destroy()
                print(f"Destroyed actor: {self.name} (ID: {self.actor.id})")
            self.actor = None
            print(f"[{self.name}] Actor reference cleared.")


    

    def get_profiling_summary(self):
        return {
            "arrival_deltas_realtime": (self.frame_arrival_deltas),
            "processing_realtime": (self.processing_times),
            "processing_realtime_parallel": (self.processing_times_real), # 
            "saving_realtime": (self.saving_times),
            "statemanager_realtime": (self.statemanager_times), # 
            "mean_arrival_delta_realtime": np.mean(self.frame_arrival_deltas) if self.frame_arrival_deltas else 0,
            "mean_processing_realtime": np.mean(self.processing_times) if self.processing_times else 0,
            "mean_processing_realtime_parallel": np.mean(self.processing_times_real) if self.processing_times_real else 0, # 
            "mean_saving_realtime": np.mean(self.saving_times) if self.saving_times else 0,
            "mean_statemanager_realtime": np.mean(self.statemanager_times) if self.statemanager_times else 0 #
        }
    
    def get_shortest_angle_diff(self, current_angle, target_angle):
        """
        Calculates the shortest angular distance between two angles in degrees.
        """
        diff = target_angle - current_angle
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        return diff

    def move_camera_vector_test(self):
        """
        Moves the camera towards its target position and rotation using a vector-based approach,
        clamping the movement speed and rotation rate per simulation tick.
        """
        # Acquire the lock using a 'with' statement for safe management
        with self.actor_mutex:
            
            # Check if the actor is still valid inside the protected section
            if not (self.actor and self.actor.is_alive):
                print(f"ERROR: {self.name} was destroyed while trying to move.")
                return

            current_transform = self.actor.get_transform()
            
            # Calculate max step size per tick
            max_step = self.linear_speed / self.simulation_fps
            
            # Calculate deltas
            delta_x = self.x_target - current_transform.location.x
            delta_y = self.y_target - current_transform.location.y
            delta_z = self.z_target - current_transform.location.z
            
            # Clamp each delta to max_step (preserving sign/direction)
            incr_x = max(min(delta_x, max_step), -max_step)
            incr_y = max(min(delta_y, max_step), -max_step)
            incr_z = max(min(delta_z, max_step), -max_step)
            
            # Check if movement is complete
            if incr_x == incr_y == incr_z == 0:
                self.linear_speed = 0
            incr_yaw = self.get_shortest_angle_diff(current_transform.rotation.yaw, self.yaw_target)
            incr_pitch = self.get_shortest_angle_diff(current_transform.rotation.pitch, self.pitch_target)
            incr_roll = self.get_shortest_angle_diff(current_transform.rotation.roll, self.roll_target)
            
            # Update the transform with a small step towards the target
            delta_transform = carla.Transform(
                carla.Location(x=incr_x, y=incr_y, z=incr_z),
                carla.Rotation(pitch=incr_pitch, yaw=incr_yaw, roll=incr_roll)
            )
            
            # Apply the new transform
            maxStepSize = (1/self.simulation_fps) * (self.max_speed/3.6)
            maxRotationSize = 2
            move_vector = carla.Vector3D(incr_x, incr_y, incr_z)

            # Calculate the current magnitude (total distance) of the movement
            current_magnitude = move_vector.length()

            # Check if the current magnitude exceeds the maxStepSize
            if current_magnitude > maxStepSize:
                # Scale the vector down to the maxStepSize
                # This preserves the direction while capping the speed
                scale_factor = maxStepSize / current_magnitude
                incr_x *= scale_factor
                incr_y *= scale_factor
                incr_z *= scale_factor

            if incr_yaw > maxRotationSize:
                incr_yaw = maxRotationSize
            elif incr_yaw < -maxRotationSize:
                incr_yaw = -maxRotationSize
            if incr_pitch > maxRotationSize:
                incr_pitch = maxRotationSize
            elif incr_pitch < -maxRotationSize:
                incr_pitch = -maxRotationSize
            if incr_roll > maxRotationSize:
                incr_roll = maxRotationSize
            elif incr_roll < -maxRotationSize:
                incr_roll = -maxRotationSize
            
            current_transform.location.x += incr_x
            current_transform.location.y += incr_y
            current_transform.location.z += incr_z
            current_transform.rotation.yaw += incr_yaw
            current_transform.rotation.pitch += incr_pitch
            current_transform.rotation.roll += incr_roll
            self.actor.set_transform(current_transform)

        # The lock is automatically released here

    """
    def follow_target_vehicle_statemanager(self, angle_allowed):
        
        Manages the camera's position and orientation to follow the target vehicle.

        This function sets the camera's rotation to always face the vehicle's
        current location, and its position to track a point slightly behind the vehicle.

        Args:
            angle_allowed (float): The maximum angle (in degrees) from directly below 
                                   the camera that the vehicle can be in before 
                                   the camera starts to move.
       

        if not self.actor:
            print(f"ERROR: {self.name} could not be follow a vehicle because it does not exist")
            return
        if not self.target_actor:
            print(f"ERROR: {self.name} could not follow a vehicle because its target vehicle does not exist")

        import math
        try:
            if not self.actor_mutex.acquire(timeout=5):
                print(f"ERROR: Mutex acquisition timed out for {self.name}.")
                return
            
            if not (not self.stop_flag.is_set() and self.actor and self.actor.is_alive):
                self.actor_mutex.release()
            
            camera_location = self.actor.get_location()
            vehicle_transform = self.target_actor.get_transform()
            
            self.actor_mutex.release()

            offset_distance = 20

            # The vehicle's exact location for camera rotation
            vehicle_location = vehicle_transform.location

            # Calculate the offset location for the camera's movement target
            forward_vector = vehicle_transform.get_forward_vector()
            offset_location = vehicle_location - offset_distance * forward_vector

            # Calculate yaw and pitch to face the vehicle's exact location
            direction_vector = vehicle_location - camera_location
            horizontal_distance = math.sqrt(direction_vector.x**2 + direction_vector.y**2)
            
            new_yaw = math.degrees(math.atan2(direction_vector.y, direction_vector.x))
            new_pitch = math.degrees(math.atan2(direction_vector.z, horizontal_distance))

            self.yaw_target = new_yaw
            self.pitch_target = new_pitch
            self.roll_target = 0.0

            # Now, use the offset location for the movement calculations
            x_dist = offset_location.x - camera_location.x
            y_dist = offset_location.y - camera_location.y
            z_dist = offset_location.z - camera_location.z
        

            max_dist = abs(z_dist) / (math.cos(math.radians(angle_allowed))) 
            if camera_location.distance(offset_location) >= max_dist:
                
               
                incr_x = x_dist
                incr_y = y_dist
                
                veh_vel = self.target_actor.get_velocity()
                self.linear_speed = math.sqrt(veh_vel.x**2 + veh_vel.y**2 + veh_vel.z**2)
                
                if not self.actor_mutex.acquire(timeout=5):
                    print(f"ERROR: Mutex acquisition timed out for {self.name}.")
                    return
            

                if not (not self.stop_flag.is_set() and self.actor and self.actor.is_alive):
                    self.actor_mutex.release()
                    

                transform = self.actor.get_transform() 
                self.actor_mutex.release()

                self.x_target = incr_x + transform.location.x
                self.y_target = incr_y + transform.location.y
                self.enable_controls = True

        except Exception as e:
            print(f"ERROR: {self.name} had an error while following: {e}")
        finally:
            if self.actor_mutex.locked():
                self.actor_mutex.release()
    """
    def follow_target_vehicle_statemanager(self, angle_allowed):
        """
        Manages the camera's position and orientation to follow the target vehicle.

        This function sets the camera's rotation to always face the vehicle's
        current location, and its position to track a point slightly behind the vehicle.

        Args:
            angle_allowed (float): The maximum angle (in degrees) from directly below 
                                   the camera that the vehicle can be in before 
                                   the camera starts to move.
        """
        if not self.actor:
            print(f"ERROR: {self.name} could not be follow a vehicle because it does not exist")
            return
        if not self.target_actor:
            print(f"ERROR: {self.name} could not follow a vehicle because its target vehicle does not exist")
            return

        import math
        
        # Acquire lock to safely read actor states.
        with self.actor_mutex:
            if not (self.actor and self.actor.is_alive):
                return
            
            camera_location = self.actor.get_location()
            vehicle_transform = self.target_actor.get_transform()
        
        offset_distance = 20
        vehicle_location = vehicle_transform.location
        forward_vector = vehicle_transform.get_forward_vector()
        offset_location = vehicle_location - offset_distance * forward_vector

        direction_vector = vehicle_location - camera_location
        horizontal_distance = math.sqrt(direction_vector.x**2 + direction_vector.y**2)
        
        new_yaw = math.degrees(math.atan2(direction_vector.y, direction_vector.x))
        new_pitch = math.degrees(math.atan2(direction_vector.z, horizontal_distance))

        self.yaw_target = new_yaw
        self.pitch_target = new_pitch
        self.roll_target = 0.0

        x_dist = offset_location.x - camera_location.x
        y_dist = offset_location.y - camera_location.y
        z_dist = offset_location.z - camera_location.z
        
        # Movement logic check
        try:
            max_dist = abs(z_dist) / (math.cos(math.radians(angle_allowed))) 
            if camera_location.distance(offset_location) >= max_dist:
                veh_vel = self.target_actor.get_velocity()
                veh_speed = math.sqrt(veh_vel.x**2 + veh_vel.y**2 + veh_vel.z**2)
                self.linear_speed = veh_speed * 1.2 
                
                # Apply new target position
                self.x_target = x_dist + camera_location.x
                self.y_target = y_dist + camera_location.y
                self.enable_controls = True
        
        except Exception as e:
            print(f"ERROR: {self.name} had an error while following: {e}")
    

# ==================================
# === Private Functions ===
# ==================================
def _find_drone_by_id(drone_id):
    """Helper to find a drone by its name (which we treat as its ID)."""
    # We can search by name (drone_id) or actor.id
    for drone in _active_cameras:
        if drone.name == drone_id:
            return drone
        if drone.actor and drone.actor.id == drone_id:
            return drone
    return None

# ==================================
# === Public API Functions ===
# ==================================


def register_edgenode(node_id, x, y, z, extra_info=None):
    """
    Registers a new edge node with the system.
    
    Args:
        node_id (str): A unique name for the node (e.g., "edge_server_1").
        x (float): X coordinate in the world.
        y (float): Y coordinate in the world.
        z (float): Z coordinate in the world.
        extra_info (dict, optional): Additional info about the node (e.g., {'ip': '192.168.1.100', 'port': 8080}).
    """
    if node_id in _edge_nodes:
        print(f"Warning: Edge node '{node_id}' is already registered. Overwriting.")
    
    # Construct the internal data structure
    info = extra_info if extra_info else {}
    info['location'] = {'x': x, 'y': y, 'z': z}

    _edge_nodes[node_id] = info
    print(f"Edge node '{node_id}' registered at ({x}, {y}, {z}) with info: {info}")

#link_drone_edgenode()
def link_drone_edgenode(drone_id, node_id):
    """
    Links a drone to a registered edge node by setting its attribute.
    
    Args:
        drone_id (str or int): The ID/name of the drone to link.
        node_id (str): The ID of the edge node to link to.
    """
    drone = _find_drone_by_id(drone_id)
    if not drone:
        print(f"ERROR: Cannot link. Drone '{drone_id}' not found.")
        return False

    if node_id not in _edge_nodes:
        print(f"ERROR: Cannot link. Edge node '{node_id}' not found.")
        return False
        
    # Set the attribute directly on the drone object
    drone.linked_edge_node = node_id
    print(f"Successfully linked drone '{drone.name}' to edge node '{node_id}'.")
    return True

#destroy_drone(id)
def destroy_drone(camera_manager):
    """
    Handles the complete four-phase shutdown for a single camera,
    including the necessary world ticks.
    """
    global _api_world
    if camera_manager not in _active_cameras:
        print(f"Warning: Camera {camera_manager.name} not in active list. Canceled destroy.")
        return
    
    if not _api_world:
        print(f"ERROR: Cannot destroy {camera_manager.name}. World object not registered in API.")
        return

    print(f"--- Cleanup Phase 1: Signaling {camera_manager.name} ---")
    camera_manager.signal_threads_to_stop()

    print(f"--- Cleanup Phase 2: Ticking world ---")
    _api_world.tick()

    print(f"--- Cleanup Phase 3: Joining {camera_manager.name} ---")
    camera_manager.join_threads()
    
    print(f"--- Cleanup Phase 4: Destroying {camera_manager.name} actor ---")
    camera_manager.destroy_actor()

    _api_world.tick() # Final tick to process actor destruction
    _active_cameras.remove(camera_manager)
    print(f"Camera {camera_manager.name} destroyed and removed.")

def destroy_all_drones():
    """
    Handles the complete four-phase shutdown for all active cameras,
    including the necessary world ticks.
    """
    global _api_world
    if not _api_world:
        print("ERROR: Cannot destroy cameras. World object not registered in API.")
        return
    
    if not _active_cameras:
        print("No active cameras to destroy.")
        return

    print("--- Cleanup Phase 1: Signaling all threads ---")
    for camera in _active_cameras:
        camera.signal_threads_to_stop()

    print("--- Cleanup Phase 2: Ticking world to allow graceful thread exit ---")
    _api_world.tick()

    print("--- Cleanup Phase 3: Joining all threads ---")
    for camera in list(_active_cameras): 
        camera.join_threads()
    
    print("--- Cleanup Phase 4: Destroying all actors ---")
    for camera in list(_active_cameras):
        camera.destroy_actor()
        _active_cameras.remove(camera) # Remove from list

    _api_world.tick() # Final tick to process actor destructions
    print("All cameras destroyed and removed.")

#get_state(id)
def get_state(drone_id):
    """
    Gets the complete state dictionary for a specific drone.
    
    Args:
        drone_id (str or int): The name (e.g., "cam1") or actor ID of the drone.
    
    Returns:
        dict: A dictionary with the drone's state, or None if not found.
    """
    drone = _find_drone_by_id(drone_id)
    if not drone:
        print(f"ERROR: Could not find drone with ID '{drone_id}'")
        return None
    
    if not drone.actor or not drone.actor.is_alive:
        print(f"Warning: Drone '{drone_id}' actor is not valid.")
        return None

    # Get the base state from the drone's to_dict() method
    state = drone.to_dict()
    
    state['is_alive'] = drone.actor.is_alive
    state['distances_to_others'] = drone.distances_to_others
    state['is_streaming'] = drone.is_streaming
    state['stream_endpoint'] = drone.stream_endpoint
    state['linked_edge_node'] = drone.linked_edge_node

    return state

def stream_drone(drone_id, endpoint_url):
    """
    Starts streaming a drone's camera feed to a given TCP endpoint.
    
    The endpoint must be a simple TCP server listening for connections.
    The stream sends frames as: [4-byte length header][JPEG data]
    
    Args:
        drone_id (str or int): The ID/name of the drone.
        endpoint_url (str): The endpoint to stream to, e.g., "127.0.0.1:9999"
    """
    drone = _find_drone_by_id(drone_id)
    if not drone:
        print(f"ERROR: Cannot stream. Drone '{drone_id}' not found.")
        return False
        
    if drone.is_streaming:
        print(f"Warning: Drone '{drone_id}' is already streaming to {drone.stream_endpoint}")
        return True

    try:
        host, port = endpoint_url.split(':')
        port = int(port)
        
        # Create a new socket and connect
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print(f"[{drone.name}] Connecting to stream endpoint {host}:{port}...")
        s.connect((host, port))
        
        # Assign the socket and update state
        drone.stream_socket = s
        drone.stream_endpoint = endpoint_url
        drone.is_streaming = True
        print(f"Drone '{drone_id}' is now streaming to {endpoint_url}")
        return True
        
    except Exception as e:
        print(f"ERROR: Could not connect to endpoint {endpoint_url}: {e}")
        if 's' in locals():
            s.close()
        return False

def stop_stream_drone(drone_id):
    """Stops an active stream for a specific drone."""
    drone = _find_drone_by_id(drone_id)
    if not drone:
        print(f"ERROR: Cannot stop stream. Drone '{drone_id}' not found.")
        return
        
    if not drone.is_streaming:
        print(f"Warning: Drone '{drone_id}' is not currently streaming.")
        return

    print(f"[{drone.name}] Stopping stream to {drone.stream_endpoint}...")
    drone.is_streaming = False
    if drone.stream_socket:
        drone.stream_socket.close()
    drone.stream_socket = None
    drone.stream_endpoint = None
    print(f"Drone '{drone_id}' stream stopped.")

#register_drone()
def register_drone(world, simulation_fps, camera_resolution_x, camera_resolution_y, transform, output_dir, target_actor, fps, battery_minimum_threshold, drone_id="Drone", battery_drain_function=None, fps_function=None, enable_tracking=False):
    """
    Creates and manages a new CameraManager instance.

    This function should be called by external scripts to spawn a new camera.
    It returns the new CameraManager object for direct access if needed.

    Args:
        world (carla.World): The CARLA world object.
        simulation_fps (float): The simulation's tick rate.
        camera_resolution_x (int): The horizontal resolution of the camera.
        camera_resolution_y (int): The vertical resolution of the camera.
        transform (carla.Transform): The initial transform for the camera.
        output_dir (str): The directory to save the output images.
        target_actor (carla.Actor): The actor to track (e.g., the vehicle).
        fps (int): The desired frames per second for image capture.
        battery_minimum_threshold (double): The battery threshold at which the camera is taken down.
        name (str): A descriptive name for the camera.
        battery_drain_function (function): An optional function to define battery drain behavior.
        enable tracking (bool, optional): If True, enables autonomous tracking.
    
    Returns:
        CameraManager: The newly created camera manager object.
    """

    global _api_world
    if _api_world is None:
        print(f"API registering world object from {drone_id}'s register command.")
        _api_world = world

    new_camera = CameraManager(
        world=world,
        simulation_fps=simulation_fps,
        camera_resolution_x=camera_resolution_x,
        camera_resolution_y=camera_resolution_y,
        transform=transform,
        output_dir=output_dir,
        target_actor=target_actor,
        fps=fps,
        battery_minimum_threshold=battery_minimum_threshold,
        battery_drain_function=battery_drain_function,
        fps_function=fps_function,
        name=drone_id,
        enable_tracking=enable_tracking
    )
    _active_cameras.append(new_camera)
    #new_camera.rotate_camera_yaw(180, 300)
    return new_camera

#--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


#--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

def get_active_drones():
    """
    Returns a list of all currently active CameraManager instances.

    """
    return _active_cameras

#--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------



#--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


#--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

def get_drone_analytics(_cameras):
    """
    Return all given CameraManager analytics.

    Args:
        _cameras ([CameraManager]): A list of the instances to get analytics of.

    Returns:
         [dict]: A list of dictionaries, where each dictionary contains the camera's
                 ID and its profiling summary. 
                {
                    "camera_id": camera.actor.id,
                    "camera_analytics": {
                                            "arrival_deltas_realtime",
                                            "processing_realtime",
                                            "saving_realtime",
                                            "mean_arrival_delta_realtime",
                                            "mean_processing_realtime",
                                            "mean_saving_realtime",
                                        }
                }
    """
    print("Giving camera analytics...")
    analytics = []
    for camera in _cameras:
        if camera.actor: 
            analytics.append({
                "camera_id": camera.actor.id,
                "camera_analytics": camera.get_profiling_summary()
            })
    
    
    print("All specified cameras analytics given.")
    return analytics

# Controls --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

def enable_target_following(camera_manager, enable=True):
    """
    Enables or disables the camera's ability to autonomously follow its target actor.

    Args:
        camera_manager (CameraManager): The camera instance to control.
        enable (bool): True to enable, False to disable.
    """
    camera_manager.enable_track_vehicle = enable

def enable_camera_controls(camera_manager, enable=True):
    """
    Enables or disables the camera's movement and rotation control logic.

    Args:
        camera_manager (CameraManager): The camera instance to control.
        enable (bool): True to enable, False to disable.
    """
    camera_manager.enable_controls = enable

def set_target_location(camera_manager, location):
    """
    Sets the camera's target location. This will trigger camera movement and enable controls.

    Args:
        camera_manager (CameraManager): The camera instance to control.
        location (carla.Location): The new target location.
    """
    camera_manager.x_target = location.x
    camera_manager.y_target = location.y
    camera_manager.z_target = location.z
    camera_manager.enable_controls = True
    camera_manager.enable_track_vehicle = False # Disable tracking if manual target is set

def set_target_rotation(camera_manager, rotation):
    """
    Sets the camera's target rotation. This will trigger camera rotation and enable controls.

    Args:
        camera_manager (CameraManager): The camera instance to control.
        rotation (carla.Rotation): The new target rotation.
    """
    camera_manager.yaw_target = rotation.yaw
    camera_manager.pitch_target = rotation.pitch
    camera_manager.roll_target = rotation.roll
    camera_manager.enable_controls = True
    camera_manager.enable_track_vehicle = False # Disable tracking if manual target is set

def add_target_location(camera_manager, location_delta):
    """
    Adds a delta to the camera's current target location.

    Args:
        camera_manager (CameraManager): The camera instance to control.
        location_delta (carla.Location): The location increment to add.
    """
    with camera_manager.actor_mutex:
        if camera_manager.actor and camera_manager.actor.is_alive:
            current_transform = camera_manager.actor.get_transform()
            camera_manager.x_target = current_transform.location.x + location_delta.x
            camera_manager.y_target = current_transform.location.y + location_delta.y
            camera_manager.z_target = current_transform.location.z + location_delta.z
            camera_manager.enable_controls = True
            camera_manager.enable_track_vehicle = False

def add_target_rotation(camera_manager, rotation_delta):
    """
    Adds a delta to the camera's current target rotation.

    Args:
        camera_manager (CameraManager): The camera instance to control.
        rotation_delta (carla.Rotation): The rotation increment to add.
    """
    with camera_manager.actor_mutex:
        if camera_manager.actor and camera_manager.actor.is_alive:
            current_transform = camera_manager.actor.get_transform()
            camera_manager.yaw_target = camera_manager.get_shortest_angle_diff(current_transform.rotation.yaw, current_transform.rotation.yaw + rotation_delta.yaw)
            camera_manager.pitch_target = camera_manager.get_shortest_angle_diff(current_transform.rotation.pitch, current_transform.rotation.pitch + rotation_delta.pitch)
            camera_manager.roll_target = camera_manager.get_shortest_angle_diff(current_transform.rotation.roll, current_transform.rotation.roll + rotation_delta.roll)
            camera_manager.enable_controls = True
            camera_manager.enable_track_vehicle = False

def move_linear(camera_manager, x, y, z, speed=None):
    """
    Sets the camera's target location relative to its current position and orientation.
    e.g., location(1, 0, 0) would target 1 meter forward from the current camera location.
    
    Args:
        camera_manager (CameraManager): The camera instance to control.
        location (carla.Location): The relative target location.
    """
    with camera_manager.actor_mutex:
        if camera_manager.actor and camera_manager.actor.is_alive:
            current_transform = camera_manager.actor.get_transform()
            # Apply the transform to the relative location
            absolute_target_transform = current_transform.transform(carla.Transform(carla.location(x, y, z)))
            camera_manager.x_target = absolute_target_transform.location.x
            camera_manager.y_target = absolute_target_transform.location.y
            camera_manager.z_target = absolute_target_transform.location.z
            if speed: 
                camera_manager.linear_speed = speed # m/s
            camera_manager.enable_controls = True
            camera_manager.enable_track_vehicle = False

def move_rotation(camera_manager, roll, pitch, yaw):
    """
    Sets the camera's target rotation relative to its current orientation.
    
    Args:
        camera_manager (CameraManager): The camera instance to control.
        rotation (carla.Rotation): The relative target rotation.
    """
    with camera_manager.actor_mutex:
        if camera_manager.actor and camera_manager.actor.is_alive:
            current_rotation = camera_manager.actor.get_transform().rotation
            camera_manager.yaw_target = camera_manager.get_shortest_angle_diff(current_rotation.yaw, current_rotation.yaw + yaw)
            camera_manager.pitch_target = camera_manager.get_shortest_angle_diff(current_rotation.pitch, current_rotation.pitch + pitch)
            camera_manager.roll_target = camera_manager.get_shortest_angle_diff(current_rotation.roll, current_rotation.roll + roll)
            camera_manager.enable_controls = True
            camera_manager.enable_track_vehicle = False

#--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
# Note: battery_drain_function and fps_function are not saved to JSON right now. Will need to add them once I implement the external function calls.
# Need to update these two functions after refactor. They are currently not functioning
def load_latest_state(world, simulation_fps, target_actor):
    """
    Initializes cameras from the latest saved state file.

    This function reads the `LATEST_SAVED_FILE` file and spawns new
    CameraManager instances with the saved parameters.

    Args:
        world (carla.World): The current CARLA world instance.
        simulation_fps (float): The simulation's tick rate.
        target_actor (carla.Actor): The actor to track, which must be
                                    available in the current simulation.
    """
    global _active_cameras
    
    # Destroy any existing cameras to avoid duplicates.
    

    # Read file
    if not os.path.exists(LATEST_STATE_FILE):
        print(f"No latest state file found at '{LATEST_STATE_FILE}'. Skipping load.")
        return

    try:
        with open(LATEST_STATE_FILE, 'r') as f:
            state_data = json.load(f)
    except (IOError, json.JSONDecodeError) as e:
        print(f"Error reading state file '{LATEST_STATE_FILE}': {e}. Skipping load.")
        return

    cameras_to_load = state_data.get("cameras", [])
    if not cameras_to_load:
        print("Latest state file is empty or has no camera data. Skipping load.")
        return
    
    print(f"Loading state for {len(cameras_to_load)} cameras from {LATEST_STATE_FILE}...")
    
    for cam_state in cameras_to_load:
        try:
            # Reconstruct the carla.Transform object from the dictionary
            location_dict = cam_state["transform"]["location"]
            rotation_dict = cam_state["transform"]["rotation"]
            
            transform = carla.Transform(
                carla.Location(x=location_dict["x"], y=location_dict["y"], z=location_dict["z"]),
                carla.Rotation(pitch=rotation_dict["pitch"], yaw=rotation_dict["yaw"], roll=rotation_dict["roll"])
            )

            # Spawn a new camera using the reconstructed parameters
           
            
        except KeyError as e:
            print(f"Skipping camera due to missing key in state data: {e}. Camera state: {cam_state}")
        except Exception as e:
            print(f"An unexpected error occurred while loading a camera: {e}")
            
    print("Finished loading camera state.")


# State Manager -----------------------------------------------------------------------------------------------------------------------------------

def _save_state_loop(world):
    """The target function for the state-saving thread."""
    print("State saving thread started. Will save state every 10 seconds.")
    while not _stop_save_state_event.is_set():
        _stop_save_state_event.wait(10.0)
        if _stop_save_state_event.is_set():
            break

        world_snapshot = world.get_snapshot()
        entry_key = f"{time.time()}"
        
        # Gather all current camera states
        current_camera_states = [cam.to_dict() for cam in _active_cameras if cam.actor]

        if not current_camera_states:
            continue

        entry_data = {
            "system_timestamp": time.time(),
            "performance_counter": time.perf_counter(),
            "simulation_time": world_snapshot.timestamp.elapsed_seconds,
            "cameras": current_camera_states
        }
        
        # Read, Update, and Write to STATE_FILE 
        with _state_file_lock:
            # --- Load Historical Data ---
            data = {}
            try:
                with open(STATE_FILE, 'r') as f:
                    file_content = f.read().strip()
                    # Only attempt to load if the file is not empty
                    if file_content:
                        data = json.loads(file_content)
                
            except (IOError, json.JSONDecodeError) as e:
                # Log the error but continue by using the empty dict data={}
                print(f"Warning: Failed to load historical data from {STATE_FILE}. Starting new history.")
                
            try:
                # Update and Write
                data[entry_key] = entry_data
                with open(STATE_FILE, 'w') as f:
                    json.dump(data, f, indent=4)
                print(f"Saved state for {len(current_camera_states)} cameras to {STATE_FILE}")
                
            except Exception as e:
                print(f"Error saving historical state to {STATE_FILE}: {e}")

            # Write LATEST_STATE_FILE
            try:
                with open(LATEST_STATE_FILE, 'w') as f:
                    json.dump(entry_data, f, indent=4)
                print(f"Saved latest state for {len(current_camera_states)} cameras to {LATEST_STATE_FILE}")

            except Exception as e:
                print(f"Error saving latest state to {LATEST_STATE_FILE}: {e}")

    print("State saving thread stopped.")


def start_state_saving(world):
    """Initializes the state file and starts the background saving thread."""
    global _state_saving_thread
    
    if not os.path.exists(STATE_FILE):
        print(f"'{STATE_FILE}' not found. Creating a new one.")
        with open(STATE_FILE, 'w') as f:
            json.dump({}, f) # Create an empty JSON object

    if not os.path.exists(LATEST_STATE_FILE):
        print(f"'{LATEST_STATE_FILE}' not found. Creating a new one.")
        with open(LATEST_STATE_FILE, 'w') as f:
            json.dump({}, f) # Create an empty JSON object

    # Start the state saving thread
    if _state_saving_thread is None or not _state_saving_thread.is_alive():
        _stop_save_state_event.clear()
        _state_saving_thread = threading.Thread(target=_save_state_loop, args=(world,))
        _state_saving_thread.start()
    else:
        print("Warning: State saving thread is already running.")

def stop_state_saving():
    """Signals the state-saving thread to stop and waits for it to exit."""
    global _state_saving_thread
    if _state_saving_thread and _state_saving_thread.is_alive():
        print("Signaling state saving thread to stop...")
        _stop_save_state_event.set()
        _state_saving_thread.join()
        _state_saving_thread = None





