#!/usr/bin/env python3

"""
ROS2 Bridge Agent for CARLA ScenarioRunner
This agent connects ScenarioRunner with ROS2 for autonomous overtaking.
"""

import os
import sys
import time
import math
import threading

import carla
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import String, Float32

# Import ScenarioRunner modules (adjust the path as needed)
scenario_runner_path = os.environ.get('SCENARIO_RUNNER_PATH', '/path/to/scenario_runner')
if scenario_runner_path not in sys.path:
    sys.path.append(scenario_runner_path)

# Now we can import from ScenarioRunner
from srunner.autoagents.autonomous_agent import AutonomousAgent, Track
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider


class ROS2BridgeAgent(AutonomousAgent):
    """
    ROS2BridgeAgent is an autonomous agent implementation that bridges 
    CARLA/ScenarioRunner with ROS2.
    """

    def setup(self, path_to_conf_file):
        """
        Initialize the agent, setup ROS2 node, publishers and subscribers
        """
        # Initialize ROS2
        rclpy.init(args=None)
        
        # Create the ROS2 node
        self.node = rclpy.create_node('ros2_bridge_agent')
        self.node.get_logger().info('ROS2 Bridge Agent started')
        
        # Define the sensor we'll use (radar or lidar)
        self.sensor_type = 'radar'  # Default - can be changed to 'lidar'
        
        # Publishers
        self.vehicle_status_pub = self.node.create_publisher(
            String, '/carla/ego_vehicle/status', 10)
        self.sensor_data_pub = self.node.create_publisher(
            Range, '/carla/ego_vehicle/radar', 10)  # We'll use Range for both radar/lidar for simplicity
        self.velocity_pub = self.node.create_publisher(
            Float32, '/carla/ego_vehicle/speed', 10)
        
        # Subscribers
        self.decision_sub = self.node.create_subscription(
            String,
            '/carla/ego_vehicle/decision',
            self.decision_callback,
            10)
        
        # Control state variables
        self.current_control = carla.VehicleControl()
        self.current_control.steer = 0.0
        self.current_control.throttle = 0.0
        self.current_control.brake = 0.0
        
        # Current state for overtaking
        self.current_state = "NORMAL"  # NORMAL, OVERTAKING_LEFT, RETURNING_RIGHT
        self.overtaking_start_time = None
        self.lane_return_start_time = None
        self.min_overtaking_time = 15.0  # seconds
        self.min_return_time = 15.0  # seconds
        
        # Lane information
        self.original_lane_id = None
        self.current_lane_id = None
        
        # Sensor data
        self.front_distance = float('inf')
        self.closest_adversary_distance = float('inf')
        self.closest_adversary_behind_distance = float('inf')
        
        # Start the ROS2 spinner in a separate thread
        self.spin_thread = threading.Thread(target=self.spin_ros2, daemon=True)
        self.spin_thread.start()
        
        # Waypoint follower for path generation
        self.waypoint_follower = None
        
        self.node.get_logger().info('ROS2 Bridge Agent setup completed')
    
    def sensors(self):
        """
        Define the sensors that will be used by the agent
        """
        sensors = []
        
        # Add the appropriate sensor (radar or lidar)
        if self.sensor_type == 'radar':
            sensors.append({
                'type': 'sensor.other.radar',
                'id': 'radar',
                'x': 2.0, 'y': 0.0, 'z': 1.0,
                'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                'horizontal_fov': 60.0,
                'vertical_fov': 30.0,
                'range': 100.0
            })
        else:  # lidar
            sensors.append({
                'type': 'sensor.lidar.ray_cast',
                'id': 'lidar',
                'x': 0.0, 'y': 0.0, 'z': 1.8,
                'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                'range': 85.0,
                'rotation_frequency': 20,
                'channels': 32,
                'points_per_second': 90000,
                'upper_fov': 15.0,
                'lower_fov': -15.0
            })
        
        # Add collision sensor for safety
        sensors.append({
            'type': 'sensor.other.collision',
            'id': 'collision',
            'x': 0.0, 'y': 0.0, 'z': 0.0
        })
        
        # Add lane invasion sensor to detect lane changes
        sensors.append({
            'type': 'sensor.other.lane_invasion',
            'id': 'lane_invasion',
            'x': 0.0, 'y': 0.0, 'z': 0.0
        })
        
        return sensors
    
    def spin_ros2(self):
        """
        ROS2 spinner function to be run in a separate thread
        """
        try:
            while rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.01)
                time.sleep(0.01)
        except Exception as e:
            self.node.get_logger().error(f"ROS2 spinner error: {e}")
    
    def initialize_vehicle_and_sensors(self, vehicle):
        """
        Initialize agent-specific variables when a new episode starts
        """
        self.vehicle = vehicle
        
        # Get initial lane information
        world = CarlaDataProvider.get_world()
        map = world.get_map()
        vehicle_loc = vehicle.get_location()
        current_waypoint = map.get_waypoint(vehicle_loc)
        self.current_lane_id = current_waypoint.lane_id
        self.original_lane_id = current_waypoint.lane_id
        
        self.node.get_logger().info(f"Agent initialized with vehicle {vehicle.id}. Original lane ID: {self.original_lane_id}")
        
        # Initialize the waypoint follower helper class
        self.waypoint_follower = WaypointFollower(world, vehicle)
        self.waypoint_follower.generate_path(distance=500)
    
    def decision_callback(self, msg):
        """
        Process decisions from the decision node
        """
        decision = msg.data
        
        self.node.get_logger().info(f"Received decision: {decision}")
        
        if decision == "LANE_CHANGE_LEFT":
            self.current_state = "OVERTAKING_LEFT"
            self.overtaking_start_time = time.time()
            
            # Update waypoint path for left lane change
            if self.waypoint_follower:
                self.waypoint_follower.generate_path(distance=500, lane_change='left')
        
        elif decision == "LANE_CHANGE_RIGHT":
            self.current_state = "RETURNING_RIGHT"
            self.lane_return_start_time = time.time()
            
            # Update waypoint path for right lane change
            if self.waypoint_follower:
                self.waypoint_follower.generate_path(distance=500, lane_change='right')
        
        elif decision == "MAINTAIN_LANE":
            # If we've completed the return to the original lane
            if self.current_state == "RETURNING_RIGHT" and time.time() - self.lane_return_start_time >= self.min_return_time:
                self.current_state = "NORMAL"
    
    def process_sensor_data(self, data):
        """
        Process sensor data and publish to ROS2 topics
        """
        sensor_id = data[0]
        sensor_data = data[1]
        
        if sensor_id == 'radar':
            self.process_radar_data(sensor_data)
        elif sensor_id == 'lidar':
            self.process_lidar_data(sensor_data)
        elif sensor_id == 'lane_invasion':
            self.process_lane_invasion(sensor_data)
        
        # Publish vehicle status
        self.publish_vehicle_status()
    
    def process_radar_data(self, radar_data):
        """
        Process radar data and publish to ROS2
        """
        # Find closest detection point in front
        closest_distance = float('inf')
        
        for detection in radar_data:
            # Only consider detections in front (+/- 30 degrees)
            azimuth = math.degrees(detection.azimuth)
            if abs(azimuth) < 30:
                distance = detection.depth
                if distance < closest_distance:
                    closest_distance = distance
        
        self.front_distance = closest_distance if closest_distance < float('inf') else 100.0
        
        # Create and publish Range message
        range_msg = Range()
        range_msg.header.stamp = self.node.get_clock().now().to_msg()
        range_msg.header.frame_id = "ego_vehicle_radar"
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = math.radians(60)
        range_msg.min_range = 0.0
        range_msg.max_range = 100.0
        range_msg.range = self.front_distance
        
        self.sensor_data_pub.publish(range_msg)
    
    def process_lidar_data(self, lidar_data):
        """
        Process lidar data and publish to ROS2
        """
        # This is a simplified implementation - in a real application,
        # you would process the point cloud more thoroughly
        
        # For now, we'll use this as a proxy for radar data by finding the closest point
        import numpy as np
        
        # Get point cloud data
        data = np.copy(np.frombuffer(lidar_data.raw_data, dtype=np.dtype('f4')))
        data = np.reshape(data, (int(data.shape[0] / 4), 4))
        
        # Filter points in front of the vehicle
        front_points = data[np.abs(np.arctan2(data[:, 1], data[:, 0])) < np.radians(30)]
        
        closest_distance = float('inf')
        if len(front_points) > 0:
            # Calculate distances for each point
            distances = np.sqrt(front_points[:, 0]**2 + front_points[:, 1]**2 + front_points[:, 2]**2)
            closest_distance = np.min(distances)
        
        self.front_distance = closest_distance if closest_distance < float('inf') else 100.0
        
        # Create and publish Range message
        range_msg = Range()
        range_msg.header.stamp = self.node.get_clock().now().to_msg()
        range_msg.header.frame_id = "ego_vehicle_lidar"
        range_msg.radiation_type = Range.INFRARED
        range_msg.field_of_view = math.radians(60)
        range_msg.min_range = 0.0
        range_msg.max_range = 85.0
        range_msg.range = self.front_distance
        
        self.sensor_data_pub.publish(range_msg)
    
    def process_lane_invasion(self, lane_invasion_data):
        """
        Process lane invasion events to update lane information
        """
        # Update current lane ID
        if self.vehicle:
            world = CarlaDataProvider.get_world()
            map = world.get_map()
            vehicle_loc = self.vehicle.get_location()
            current_waypoint = map.get_waypoint(vehicle_loc)
            self.current_lane_id = current_waypoint.lane_id
    
    def publish_vehicle_status(self):
        """
        Publish vehicle status information
        """
        if not self.vehicle:
            return
        
        # Get vehicle transform and velocity
        transform = self.vehicle.get_transform()
        velocity = self.vehicle.get_velocity()
        speed_kmh = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        
        # Publish status
        status_msg = String()
        status_msg.data = (
            f"Position: ({transform.location.x:.2f}, {transform.location.y:.2f}, {transform.location.z:.2f}), "
            f"Heading: {transform.rotation.yaw:.2f}, Speed: {speed_kmh:.2f} km/h, "
            f"State: {self.current_state}, Lane ID: {self.current_lane_id}"
        )
        self.vehicle_status_pub.publish(status_msg)
        
        # Publish speed
        speed_msg = Float32()
        speed_msg.data = speed_kmh
        self.velocity_pub.publish(speed_msg)
    
    def run_step(self, input_data, timestamp):
        """
        Execute one step of the agent's behavior.
        This is the main function called by ScenarioRunner in each simulation step.
        """
        # Process all sensor inputs
        for key, val in input_data.items():
            sensor_id = key.split('/')[0]
            self.process_sensor_data((sensor_id, val))
        
        # If waypoint follower is not initialized, initialize it
        if not self.vehicle and CarlaDataProvider.get_hero_actor():
            self.initialize_vehicle_and_sensors(CarlaDataProvider.get_hero_actor())
        
        # Calculate steering based on waypoint following
        if self.waypoint_follower:
            steering = self.waypoint_follower.calculate_steering()
            self.current_control.steer = steering
        
        # Speed control - adjusted for overtaking scenario
        current_speed = 0.0
        if self.vehicle:
            velocity = self.vehicle.get_velocity()
            current_speed = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        
        # Target speed depends on state
        if self.current_state == "OVERTAKING_LEFT":
            target_speed = 45.0  # Higher speed for overtaking
        else:
            target_speed = 35.0  # Normal speed
        
        # Apply throttle or brake based on speed difference
        speed_error = target_speed - current_speed
        if speed_error > 0:
            self.current_control.throttle = min(0.7, max(0.05, speed_error / 30.0))
            self.current_control.brake = 0.0
        else:
            self.current_control.throttle = 0.0
            self.current_control.brake = min(0.5, max(0.05, -speed_error / 30.0))
        
        # Log the control actions (less frequently to avoid spam)
        if int(timestamp * 10) % 50 == 0:  # Log every ~5 seconds
            self.node.get_logger().info(
                f"Control: throttle={self.current_control.throttle:.2f}, "
                f"brake={self.current_control.brake:.2f}, "
                f"steer={self.current_control.steer:.2f}, "
                f"Speed={current_speed:.1f} km/h, Target={target_speed:.1f} km/h, "
                f"Front Distance={self.front_distance:.1f}m, State={self.current_state}"
            )
        
        return self.current_control


class WaypointFollower:
    """
    Helper class for following waypoints and calculating steering
    This is adapted from the Ego_test.py script
    """
    
    def __init__(self, world, vehicle):
        self.world = world
        self.vehicle = vehicle
        self.map = world.get_map()
        self.waypoint_separation = 2.0  # meters
        self.waypoints = []
        self.current_waypoint_index = 0
        self.lookahead_distance = 5.0  # meters
        self.previous_steering = 0.0  # For smoothing steering
    
    def generate_path(self, distance=500, lane_change=None):
        """Generate a path of waypoints for the vehicle to follow"""
        vehicle_loc = self.vehicle.get_location()
        current_waypoint = self.map.get_waypoint(vehicle_loc)
        
        if lane_change == 'left' and current_waypoint.get_left_lane():
            current_waypoint = current_waypoint.get_left_lane()
        elif lane_change == 'right' and current_waypoint.get_right_lane():
            current_waypoint = current_waypoint.get_right_lane()
            
        self.waypoints = [current_waypoint]
        distance_traveled = 0
        
        while distance_traveled < distance:
            next_waypoints = current_waypoint.next(self.waypoint_separation)
            if not next_waypoints:
                break
            current_waypoint = next_waypoints[0]
            self.waypoints.append(current_waypoint)
            distance_traveled += self.waypoint_separation
            
        self.current_waypoint_index = 0
        return self.waypoints
    
    def update_waypoints(self):
        """Update the current waypoint index based on vehicle position"""
        if not self.waypoints:
            return
            
        vehicle_loc = self.vehicle.get_location()
        min_distance = float('inf')
        closest_index = self.current_waypoint_index
        
        for i in range(self.current_waypoint_index, len(self.waypoints)):
            waypoint = self.waypoints[i]
            distance = vehicle_loc.distance(waypoint.transform.location)
            if distance < min_distance:
                min_distance = distance
                closest_index = i
                
        self.current_waypoint_index = max(closest_index, self.current_waypoint_index)  # Prevent going backward
        
        if self.current_waypoint_index >= len(self.waypoints) - 20:  # Replenish waypoints if running low
            self.generate_path(distance=500)
    
    def get_target_waypoint(self):
        """Get the target waypoint for steering calculations"""
        self.update_waypoints()
        
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
            return None
            
        vehicle_loc = self.vehicle.get_location()
        cumulative_distance = 0
        target_index = self.current_waypoint_index
        
        for i in range(self.current_waypoint_index + 1, len(self.waypoints)):
            waypoint = self.waypoints[i]
            prev_waypoint = self.waypoints[i-1]
            segment_distance = waypoint.transform.location.distance(prev_waypoint.transform.location)
            cumulative_distance += segment_distance
            
            if cumulative_distance >= self.lookahead_distance:
                target_index = i
                break
                
        if target_index >= len(self.waypoints):
            target_index = len(self.waypoints) - 1
            
        return self.waypoints[target_index]
    
    def calculate_steering(self):
        """Calculate steering angle to follow waypoints"""
        target_waypoint = self.get_target_waypoint()
        
        if not target_waypoint:
            return 0.0
            
        vehicle_transform = self.vehicle.get_transform()
        target_loc = target_waypoint.transform.location
        vehicle_loc = vehicle_transform.location
        
        direction_vector = target_loc - vehicle_loc
        vehicle_forward = vehicle_transform.get_forward_vector()
        vehicle_right = vehicle_transform.get_right_vector()
        
        forward_dot = direction_vector.x * vehicle_forward.x + direction_vector.y * vehicle_forward.y
        right_dot = direction_vector.x * vehicle_right.x + direction_vector.y * vehicle_right.y
        
        steering_angle = math.atan2(right_dot, forward_dot)
        smoothed_steering = 0.7 * self.previous_steering + 0.3 * steering_angle
        self.previous_steering = smoothed_steering
        
        return max(-1.0, min(1.0, smoothed_steering))


# ==============================================================================
# -- Main --------------------------------------------------------------------
# ==============================================================================

def main():
    """
    Main function to create and run the ROS2 bridge agent with ScenarioRunner.
    """
    agent = ROS2BridgeAgent()
    
    try:
        # Register with Track enumeration
        agent.setup('', Track.SCENARIO)  # No config file needed
        
        # The agent will be controlled by ScenarioRunner
        print("ROS2 Bridge Agent created successfully")
        
    except KeyboardInterrupt:
        print("User interrupted execution")
    except Exception as e:
        print(f"Error: {e}")
    
    return agent


if __name__ == '__main__':
    # When called from ScenarioRunner directly, return the agent
    main()
