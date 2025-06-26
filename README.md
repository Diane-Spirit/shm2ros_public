# shm2ros

This repository implements a ROS2 relay node that reads velocity data from shared memory and publishes it to ROS2 topics. It is designed to bridge data from a shared memory interface (e.g., from a low-level controller or simulator) to the ROS2 ecosystem, enabling integration with other ROS2 nodes.

## Data Structure

The shared memory segment is expected to contain the following structure:

- **Counter**: int32 (4 bytes)
- **Linear velocity X**: float64 (8 bytes)
- **Linear velocity Y**: float64 (8 bytes)
- **Linear velocity Z**: float64 (8 bytes)
- **Angular velocity X**: float64 (8 bytes)
- **Angular velocity Y**: float64 (8 bytes)
- **Angular velocity Z**: float64 (8 bytes)

**NOTE:** Linear velocities Y and Z and angular velocities X and Y are expected to be 0 since the system is controlling a differential drive.

---

## Project Structure

- `ros2_ws/src/shm2ros/shm2ros/`  
  Main Python package containing:
  - `shm_api.py`: Shared memory interface implementation
  - `shm2ros.py`: Main ROS2 node logic
- `ros2_ws/src/shm2ros/config/shm2ros_params.yaml`  
  Node parameters (e.g., shared memory key, topic names)
- `ros2_ws/src/shm2ros/launch/shm2ros.launch.py`  
  Launch file for starting the node
- `docker_ws/`  
  Dockerfiles and build scripts for containerized deployment

---

## Setup and Usage

### 1. Build the ROS2 Workspace

```bash
cd ros2_ws
colcon build
source install/local_setup.bash
```

### 2. Configure Parameters (Optional)

Edit `ros2_ws/src/shm2ros/config/shm2ros_params.yaml` to set shared memory keys, topic names, or other parameters as needed.

### 3. Docker Deployment

To build and run the node in a Docker container, use the scripts in `docker_ws/`:

```bash
./docker_ws/build_shm2ros.sh
./docker_run.sh
```

### 4. Run the Node

From the container, you can launch the node using the provided launch file:

```bash
ros2 launch shm2ros shm2ros.launch.py
```

Or run directly (for deployment):

```bash
./docker_run_deploy.sh
```

---

## Additional Notes

- Make sure the shared memory segment is being written to by the expected producer process.
- The node expects the shared memory to be formatted as described above.
- For more details, see the code in `shm2ros/shm_api.py` and `shm2ros/shm2ros.py`.

---

## References
- [ROS2 Documentation](https://docs.ros.org/en/foxy/index.html)
- [Python multiprocessing.shared_memory](https://docs.python.org/3/library/multiprocessing.shared_memory.html)