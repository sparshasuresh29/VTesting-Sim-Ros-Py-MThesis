# CARLA ROS2 Autonomous Overtaking System

This guide provides comprehensive instructions for setting up and running an autonomous overtaking system in CARLA using ROS2 Humble. The system uses radar/lidar sensors to detect obstacles and make intelligent overtaking decisions.

## System Architecture

The system consists of the following components:

1. **CARLA Simulator** - The 3D simulation environment
2. **ScenarioRunner** - CARLA's scenario execution engine
3. **ROS2 Bridge Agent** - Connects ScenarioRunner with ROS2
4. **ROS2 Decision Node** - Makes autonomous overtaking decisions

The workflow is as follows:
1. The Bridge Agent attaches a radar or lidar sensor to the ego vehicle
2. Sensor data is published to ROS2 topics
3. The Decision Node processes this data and makes overtaking decisions
4. Decisions are sent back to the Bridge Agent, which controls the vehicle

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- CARLA 0.9.13 or newer
- CARLA ScenarioRunner
- Python 3.8+

## Installation

### 1. Set up the ROS2 workspace

```bash
# Create a ROS2 workspace
mkdir -p ~/carla_ros2_ws/src
cd ~/carla_ros2_ws/src

# Create package structure
mkdir -p carla_ros2_bridge/carla_ros2_bridge
mkdir -p carla_ros2_bridge/launch
mkdir -p carla_ros2_bridge/scripts
mkdir -p carla_ros2_bridge/resource
touch carla_ros2_bridge/resource/carla_ros2_bridge
```

### 2. Copy the source files

```bash
# Navigate to the scripts directory
cd ~/carla_ros2_ws/src/carla_ros2_bridge/scripts

# Create the agent script
# Copy the ros2_bridge_agent.py content here
nano ros2_bridge_agent.py

# Create the decision maker script
# Copy the overtaking_decision_maker.py content here
nano overtaking_decision_maker.py

# Make them executable
chmod +x ros2_bridge_agent.py
chmod +x overtaking_decision_maker.py

# Navigate to the launch directory
cd ~/carla_ros2_ws/src/carla_ros2_bridge/launch

# Create the launch file
# Copy the overtaking_scenario.launch.py content here
nano overtaking_scenario.launch.py

# Copy remaining files (CMakeLists.txt, package.xml, etc.)
cd ~/carla_ros2_ws/src/carla_ros2_bridge

# Copy the package.xml content here
nano package.xml

# Copy the CMakeLists.txt content here
nano CMakeLists.txt

# Copy the setup.py content here
nano setup.py
```

### 3. Build the ROS2 package

```bash
cd ~/carla_ros2_ws
colcon build --packages-select carla_ros2_bridge
source install/setup.bash
```

## Setting up ScenarioRunner Integration

### 1. Create a symbolic link to the agent

```bash
# Set the environment variable for ScenarioRunner
export SCENARIO_RUNNER_PATH=/path/to/scenario_runner

# Create a symbolic link to the agent in the ScenarioRunner agents directory
ln -s ~/carla_ros2_ws/src/carla_ros2_bridge/scripts/ros2_bridge_agent.py $SCENARIO_RUNNER_PATH/srunner/autoagents/ros2_bridge_agent.py
```

### 2. Copy the modified OpenSCENARIO file

```bash
# Copy the modified OpenSCENARIO file to the ScenarioRunner scenarios directory
cp ~/carla_ros2_ws/src/carla_ros2_bridge/Overtaking.xosc $SCENARIO_RUNNER_PATH/srunner/examples/
```

## Running the System

### 1. Start CARLA Server

```bash
cd /path/to/carla
./CarlaUE4.sh -quality-level=Low
```

### 2. Launch the ROS2 Decision Node

```bash
# In a new terminal
source ~/carla_ros2_ws/install/setup.bash
ros2 launch carla_ros2_bridge overtaking_scenario.launch.py distance_threshold:=25.0 min_overtaking_time:=15.0
```

### 3. Run ScenarioRunner with the OpenSCENARIO file

```bash
# In a new terminal
cd $SCENARIO_RUNNER_PATH
python scenario_runner.py --openscenario srunner/examples/Overtaking.xosc --agent ros2_bridge_agent.py
```

### 4. (Optional) Set up Spectator View

```bash
# In a new terminal
cd /path/to/your/scripts
python3 spectator.py
```

## System Components in Detail

### Bridge Agent

The Bridge Agent serves as an interface between CARLA/ScenarioRunner and ROS2:

- It inherits from ScenarioRunner's `AutonomousAgent` class
- It attaches a radar or lidar sensor to the ego vehicle
- It processes sensor data and publishes it to ROS2 topics
- It subscribes to decision commands and controls the vehicle

### Decision Node

The Decision Node implements the overtaking logic:

- It processes sensor data to detect obstacles
- It maintains a state machine with three states: NORMAL, OVERTAKING_LEFT, RETURNING_RIGHT
- It publishes decision commands to the Bridge Agent

### Overtaking State Machine

1. **NORMAL** state:
   - Monitor for obstacles ahead
   - If an obstacle is detected within the distance threshold (default: 25m), transition to OVERTAKING_LEFT

2. **OVERTAKING_LEFT** state:
   - Execute left lane change
   - Stay in the overtaking lane for a minimum time (default: 15s)
   - Transition to RETURNING_RIGHT after the minimum time

3. **RETURNING_RIGHT** state:
   - Execute right lane change to return to the original lane
   - Stay in the original lane for a minimum time
   - Transition back to NORMAL state

## ROS2 Topics

### Published by Bridge Agent
- `/carla/ego_vehicle/status` (String): Vehicle status information
- `/carla/ego_vehicle/radar` or `/carla/ego_vehicle/lidar` (Range): Sensor data
- `/carla/ego_vehicle/speed` (Float32): Current vehicle speed

### Published by Decision Node
- `/carla/ego_vehicle/decision` (String): Overtaking decisions ("LANE_CHANGE_LEFT", "LANE_CHANGE_RIGHT", "MAINTAIN_LANE")

## Customization Options

### Sensor Type

You can choose between radar and lidar by modifying the `self.sensor_type` variable in `ros2_bridge_agent.py`.

### Decision Parameters

You can adjust the following parameters via the launch file:

- `distance_threshold`: Distance at which to trigger overtaking (default: 25.0m)
- `min_overtaking_time`: Minimum time to spend in overtaking lane (default: 15.0s)
- `safe_return_distance`: Safe distance to return to original lane (default: 35.0m)
- `min_overtaking_speed`: Minimum speed required to initiate overtaking (default: 35.0 km/h)

## Troubleshooting

### Common Issues

1. **Agent not found in ScenarioRunner**:
   - Ensure the symbolic link is correctly set up
   - Check that the `SCENARIO_RUNNER_PATH` environment variable is correctly set

2. **ROS2 nodes not communicating**:
   - Check that both the bridge agent and decision node are running
   - Use `ros2 topic list` and `ros2 topic echo` to debug communication

3. **Vehicle not responding to commands**:
   - Ensure the ego vehicle is correctly identified in the scenario
   - Check that the ScenarioRunner is properly initialized with the external agent

4. **Sensor not detecting obstacles**:
   - Verify sensor positioning and parameters
   - Check that the sensor data is being published to the correct topic

### Logs and Debugging

- Bridge Agent logs can be found in the ScenarioRunner terminal
- Decision Node logs can be found in the ROS2 launch terminal
- Use `ros2 topic echo` to monitor the data flow between nodes

## Advanced Topics

### Multi-Lane Highways

The current implementation works well for two-lane roads. For multi-lane highways, the logic would need to be extended to:
1. Identify the optimal lane for overtaking
2. Handle multiple vehicle obstacles
3. Implement more sophisticated decision-making

### Integration with Planning Algorithms

For more complex scenarios, consider integrating with global path planning algorithms:
1. Use ROS2's Navigation2 stack for global planning
2. Implement behavior trees for more complex decision-making
3. Add machine learning components for obstacle prediction

## Conclusion

This ROS2-based autonomous overtaking system provides a flexible framework for implementing and testing autonomous driving capabilities in CARLA. By separating the sensing, decision-making, and control components into distinct ROS2 nodes, the system is modular and can be extended for more complex scenarios.
