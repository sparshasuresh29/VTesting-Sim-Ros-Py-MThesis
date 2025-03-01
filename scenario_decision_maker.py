#!/usr/bin/env python3

"""
Decision Making Node for CARLA ScenarioRunner with ROS2
This node subscribes to sensor data and vehicle status and publishes overtaking decisions.
"""

import time
import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Range
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist


class OvertakingDecisionMaker(Node):
    """
    ROS2 node for making overtaking decisions based on sensor data from CARLA
    """
    
    def __init__(self):
        super().__init__('overtaking_decision_maker')
        
        # Parameters
        self.declare_parameter('distance_threshold', 25.0)  # meters for overtaking
        self.declare_parameter('safe_return_distance', 35.0)  # meters for returning to lane
        self.declare_parameter('min_overtaking_speed', 35.0)  # km/h
        self.declare_parameter('min_overtaking_time', 15.0)  # seconds in overtaking lane
        
        # Get parameters
        self.distance_threshold = self.get_parameter('distance_threshold').value
        self.safe_return_distance = self.get_parameter('safe_return_distance').value
        self.min_overtaking_speed = self.get_parameter('min_overtaking_speed').value
        self.min_overtaking_time = self.get_parameter('min_overtaking_time').value
        
        # State variables
        self.current_state = "NORMAL"  # NORMAL, OVERTAKING_LEFT, RETURNING_RIGHT
        self.overtaking_start_time = None
        self.lane_return_start_time = None
        self.current_speed = 0.0
        self.front_distance = float('inf')
        self.current_lane_id = None
        self.original_lane_id = None
        self.state_changed = False
        self.last_decision_time = 0.0
        self.last_decision = "MAINTAIN_LANE"
        
        # Subscribers
        self.sensor_sub = self.create_subscription(
            Range,
            '/carla/ego_vehicle/radar',  # Will work with either radar or lidar data
            self.sensor_callback,
            10
        )
        
        self.speed_sub = self.create_subscription(
            Float32,
            '/carla/ego_vehicle/speed',
            self.speed_callback,
            10
        )
        
        self.status_sub = self.create_subscription(
            String,
            '/carla/ego_vehicle/status',
            self.status_callback,
            10
        )
        
        # Publishers
        self.decision_pub = self.create_publisher(
            String, 
            '/carla/ego_vehicle/decision', 
            10
        )
        
        # Decision timer
        self.decision_timer = self.create_timer(0.1, self.make_decision)
        
        self.get_logger().info('Overtaking Decision Maker started with:')
        self.get_logger().info(f'- Distance threshold: {self.distance_threshold}m')
        self.get_logger().info(f'- Safe return distance: {self.safe_return_distance}m')
        self.get_logger().info(f'- Min overtaking speed: {self.min_overtaking_speed}km/h')
    
    def sensor_callback(self, msg):
        """Process sensor data from radar/lidar"""
        self.front_distance = msg.range
    
    def speed_callback(self, msg):
        """Update current vehicle speed"""
        self.current_speed = msg.data
    
    def status_callback(self, msg):
        """Process vehicle status information"""
        # Parse lane ID and other info from status message
        status = msg.data
        
        # Attempt to extract lane ID if it's in the message
        if "Lane ID:" in status:
            try:
                lane_id_part = status.split("Lane ID:")[1].strip()
                lane_id = int(lane_id_part.split(",")[0] if "," in lane_id_part else lane_id_part)
                
                # Update lane info
                self.current_lane_id = lane_id
                
                # Initialize original lane ID if not set
                if self.original_lane_id is None:
                    self.original_lane_id = lane_id
                    self.get_logger().info(f"Original lane ID set to {self.original_lane_id}")
            except Exception as e:
                self.get_logger().warn(f"Could not parse lane ID from status: {e}")
    
    def make_decision(self):
        """State machine for overtaking decisions"""
        current_time = time.time()
        
        # Limit decision rate to avoid flooding
        if current_time - self.last_decision_time < 0.5 and not self.state_changed:
            return
        
        # Reset state change flag
        self.state_changed = False
        self.last_decision_time = current_time
        decision = "MAINTAIN_LANE"  # Default decision
        
        # State machine
        if self.current_state == "NORMAL":
            # Check if we need to overtake
            if self.front_distance <= self.distance_threshold and self.current_speed >= self.min_overtaking_speed:
                self.get_logger().info(f"Obstacle detected at {self.front_distance:.2f}m. Initiating overtaking maneuver.")
                self.current_state = "OVERTAKING_LEFT"
                self.overtaking_start_time = current_time
                decision = "LANE_CHANGE_LEFT"
                self.state_changed = True
        
        elif self.current_state == "OVERTAKING_LEFT":
            # Check if we've been in the overtaking state long enough
            if current_time - self.overtaking_start_time >= self.min_overtaking_time:
                self.get_logger().info(f"Overtaking time completed ({self.min_overtaking_time}s). Initiating return to original lane.")
                self.current_state = "RETURNING_RIGHT"
                self.lane_return_start_time = current_time
                decision = "LANE_CHANGE_RIGHT"
                self.state_changed = True
            # If still in overtaking state but not long enough, maintain lane
            else:
                decision = "MAINTAIN_LANE"
        
        elif self.current_state == "RETURNING_RIGHT":
            # Check if we've been in the returning state long enough and have returned to original lane
            if (current_time - self.lane_return_start_time >= self.min_overtaking_time and 
                self.current_lane_id == self.original_lane_id):
                self.get_logger().info(f"Return to original lane completed. Resuming normal driving.")
                self.current_state = "NORMAL"
                decision = "MAINTAIN_LANE"
                self.state_changed = True
            # If still returning but not long enough, maintain current trajectory
            else:
                decision = "MAINTAIN_LANE"
        
        # Log decisions but avoid spamming the log
        if decision != self.last_decision or self.state_changed:
            self.get_logger().info(f"Decision: {decision}, State: {self.current_state}, " +
                                 f"Front Distance: {self.front_distance:.2f}m, Speed: {self.current_speed:.2f}km/h, " +
                                 f"Lane ID: {self.current_lane_id} (Original: {self.original_lane_id})")
            self.last_decision = decision
        
        # Publish the decision
        msg = String()
        msg.data = decision
        self.decision_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    decision_maker = OvertakingDecisionMaker()
    
    try:
        rclpy.spin(decision_maker)
    except KeyboardInterrupt:
        pass
    finally:
        decision_maker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()