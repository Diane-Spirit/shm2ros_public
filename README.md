# **DIANE Project: shm2ros**  
*ROS2 Relay Node for Shared Memory Data Integration*  

## Overview  
This repository implements a **ROS2 relay node** that reads velocity data from **shared memory** and publishes it to ROS2 topics. It bridges data from low-level controllers or simulators to the ROS2 ecosystem, enabling seamless integration with other ROS2 nodes.

---

## Key Features  
- **Shared Memory Interface**: Reads structured data from a shared memory segment.  
- **ROS2 Compatibility**: Publishes data to ROS2 topics for downstream processing.  
- **Containerization**: Supports Docker deployment for scalable and portable execution.  

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

### 1. **Build the ROS2 Workspace**  
```bash
cd ros2_ws
colcon build
source install/local_setup.bash
```

### 2. **Configure Parameters**  (optional) 
Edit `shm2ros_params.yaml` to set:  
- Shared memory key  
- ROS2 topic names  
- Other runtime parameters  

### 3. **Docker Deployment**  
Build and run the node in a container:  
```bash
cd docker_ws
./build_shm2ros.sh
./docker_run.sh
```

### 4. **Run the Node**  
From the container:  
```bash
ros2 launch shm2ros shm2ros.launch.py
```
Or for deployment:  
```bash
./docker_run_deploy.sh
```

---

## Data Structure  

The shared memory segment must contain the following data (in order):  
- **Counter**: `int32` (4 bytes)  
- **Linear velocity X**: `float64` (8 bytes)  
- **Linear velocity Y**: `float64` (8 bytes)  
- **Linear velocity Z**: `float64` (8 bytes)  
- **Angular velocity X**: `float64` (8 bytes)  
- **Angular velocity Y**: `float64` (8 bytes)  
- **Angular velocity Z**: `float64` (8 bytes)  

> **Note**: Y, Z linear velocities and X, Y angular velocities are typically zero for differential drive systems.  

---

## Additional Notes  
- Ensure the shared memory segment is populated by the producer process.  
- For debugging, inspect `shm2ros/shm_api.py` and `shm2ros/shm2ros.py`.

---

## Contributions  
- **Reporting Issues**: Use the GitHub Issues tracker.  
- **Code Contributions**: Fork the repository, implement changes, and submit a pull request.  

---

## References
- [ROS2 Documentation](https://docs.ros.org/en/foxy/index.html)
- [Python multiprocessing.shared_memory](https://docs.python.org/3/library/multiprocessing.shared_memory.html)

---

## Cite us
If you use this code in your work, please cite the DIANE project as follows:

```
@inproceedings{10.1145/3712676.3719263,
  author = {Barone, Nunzio and Brescia, Walter and Santangelo, Gabriele and Maggio, Antonio Pio and Cisternino, Ivan and De Cicco, Luca and Mascolo, Saverio},
  title = {Real-time Point Cloud Transmission for Immersive Teleoperation of Autonomous Mobile Robots},
  year = {2025},
  isbn = {9798400714672},
  publisher = {Association for Computing Machinery},
  address = {New York, NY, USA},
  url = {https://doi.org/10.1145/3712676.3719263},
  doi = {10.1145/3712676.3719263},
  booktitle = {Proceedings of the 16th ACM Multimedia Systems Conference},
  pages = {311–316},
  numpages = {6},
  keywords = {teleoperation, mobile robots, VR, point clouds, WebRTC},
  location = {Stellenbosch, South Africa},
  series = {MMSys '25}
}
```

or: 

DIANE: Distributed Immersive Platform for Robot Teleoperation. https://github.com/Diane-Spirit