# Autonomous AprilTag Navigation with PID Control and Future TEB Integration

This package allows a robot (TurtleBot3 Burger) to autonomously detect AprilTags and move towards them using PID control. The system is designed to integrate with the TEB (Timed Elastic Band) planner for future enhancements in autonomous navigation. Currently, there is a quaternion error in the TEB planner due to a transformation issue, which will be corrected in future work.
## DEMO VIDEO
  [![Watch the demo](https://path/to/your/image.png)](https://drive.google.com/file/d/1XolpNXyXHvBhDsE9xbqjaliXJMMERF_x/view?usp=drive_link)
## Features
- **AprilTag Detection:** Detects AprilTags in the environment and calculates the robot's pose with respect to the AprilTag.
- **PID Control:** Uses a PID controller to move the robot towards the detected AprilTag.
- **Custom World Setup:** Includes a custom world with a TurtleBot3 Burger and AprilTags.
- **Task Allocation:** Task allocation is based on user input, which is handled by the `task_allocator.py` script.
- **Future Work:** The package will integrate with the TEB planner for more sophisticated autonomous navigation. Currently, quaternion errors in the TEB planner will be corrected in future releases.

## Package Structure
- `custom_world`: Contains the world definition and launch files.
  - **`custom_world.launch`**: Launches the world with a TurtleBot3 Burger and AprilTags.
  - **`task_allocator.py`**: A Python script that allocates tasks to the robot based on user input.
  
- `slam_gmapping`: Installed on the system for SLAM-based mapping (GMapping).
- `april_tags_ros`: Used for AprilTag detection and providing the pose of the AprilTag relative to the camera.
- `teb_local_planner`: Currently in development, this package will eventually be used for path planning with TEB.

## How to Use

1. **Launch the World:**
   - First, launch the world with the following command:
     ```bash
     roslaunch custom_world custom_world.launch
     ```

2. **Allocate Tasks:**
   - After launching the world, run the task allocation script to provide user input and assign tasks to the robot:
     ```bash
     rosrun custom_world task_allocator.py
     ```

3. **Detect AprilTags:**
   - The robot will detect AprilTags using the `april_tags_ros` package and calculate its pose relative to the AprilTag.

4. **PID Control:**
   - The robot will autonomously move towards the detected AprilTag using PID control.

## Future Work
- **TEB Planner Integration:** The package will be integrated with the TEB planner for better autonomous navigation. The current quaternion error caused by transformation issues will be corrected in future updates.
- **Improved Path Planning:** Future improvements will include refining the path planning and navigation using TEB's full capabilities.

## Dependencies
- **slam_gmapping**: For SLAM-based mapping (GMapping).
- **april_tags_ros**: For AprilTag detection.
- **teb_local_planner**: For future TEB-based navigation (currently under development).

## Installation

### Clone the Repository inside src folder
```bash
git clone https://github.com/your-username/your-repository-name.git
cd your-repository-name
 ```
### copy the models to  gazebo models
- Navigate to the Custom_world/models enter this cmd.
  ```bash
     cp -r *cw .gazebo/models
  ```

