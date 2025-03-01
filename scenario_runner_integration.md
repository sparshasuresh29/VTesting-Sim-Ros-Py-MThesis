# Integrating ROS2 Nodes with CARLA ScenarioRunner

When working with ScenarioRunner, we need to modify our approach to ensure proper integration. ScenarioRunner handles the execution of OpenSCENARIO files and manages the simulation lifecycle, so our ROS2 nodes need to work alongside it rather than controlling the simulation directly.

## Modified Approach

### 1. ROS2 Bridge Node as an External Agent

The CARLA ScenarioRunner supports external agents through its API. We'll modify our ROS2 Bridge node to act as an external agent that communicates with ScenarioRunner.

### 2. Directory Structure

```
carla_ros2_bridge/
├── carla_ros2_bridge/
│   ├── __init__.py
│   ├── ros2_bridge_agent.py  (modified to work with ScenarioRunner)
│   └── overtaking_decision_maker.py
├── launch/
│   └── overtaking_scenario.launch.py
├── CMakeLists.txt
├── package.xml
└── setup.py
```

### 3. ScenarioRunner Integration

The `ros2_bridge_agent.py` will be designed to:
1. Register as an agent with ScenarioRunner
2. Publish sensor data to ROS2 topics
3. Subscribe to decision commands
4. Apply those decisions to the ego vehicle through ScenarioRunner's API

## Steps to Run with ScenarioRunner

1. Start CARLA server:
   ```bash
   ./CarlaUE4.sh
   ```

2. Launch the ROS2 decision node:
   ```bash
   ros2 launch carla_ros2_bridge overtaking_scenario.launch.py
   ```

3. Run ScenarioRunner with the ROS2 agent:
   ```bash
   cd /path/to/scenario_runner
   python scenario_runner.py --openscenario Overtaking.xosc --agent /path/to/ros2_bridge_agent.py
   ```

## Required Modifications to Ego_test.py

The `Ego_test.py` script will need to be modified to integrate with both ScenarioRunner and ROS2. Here's how to adapt it:

1. Turn it into a proper ScenarioRunner agent by implementing the required agent interface
2. Add ROS2 publishing and subscription capabilities
3. Route the sensor data through ROS2 topics
4. Implement the decision execution based on commands from the decision node

The main function will be replaced with the agent interface methods that ScenarioRunner expects.
