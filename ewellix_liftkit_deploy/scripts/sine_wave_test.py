#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayLayout
from sensor_msgs.msg import JointState
import math
import time

class SineWaveCommandPublisher(Node):
    def __init__(self):
        super().__init__('sine_wave_command_publisher')
        
        # Publisher for position commands
        self.cmd_publisher = self.create_publisher(
            Float64MultiArray,
            '/lift_position_controller/commands',
            10
        )
        
        # Subscriber for actual joint state feedback
        self.state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Timer to publish commands at 45Hz
        self.timer = self.create_timer(1/45, self.publish_command)
        
        # Sine wave parameters
        self.center = 0.300      # Center position (meters)
        self.amplitude = 0.150   # Amplitude above/below center (meters)
        self.period = 40.0        # Full cycle time (seconds)
        self.start_time = time.time()
        
        # Initialization phase
        self.init_duration = 10.0  # Hold at center for 5 seconds
        self.init_complete = False
        self.sine_start_time = None
        
        # Latest feedback
        self.latest_position = 0.0
        self.latest_velocity = 0.0
        
        self.get_logger().info('Sine Wave Publisher Started')
        self.get_logger().info(f'  Phase 1: Hold at center ({self.center}m) for {self.init_duration}s')
        self.get_logger().info(f'  Phase 2: Sine wave - Center: {self.center}m, Amplitude: {self.amplitude}m, Period: {self.period}s')
    
    def publish_command(self):
        """Publish position command based on current phase"""
        elapsed = time.time() - self.start_time
        
        # Phase 1: Initialize at center position for 5 seconds
        if not self.init_complete:
            if elapsed < self.init_duration:
                position = self.center
                phase_info = "INIT"
            else:
                # Transition to sine wave phase
                self.init_complete = True
                self.sine_start_time = time.time()
                self.get_logger().info(f'✓ Initialization complete. Starting sine wave motion.')
                position = self.center
                phase_info = "INIT→SINE"
        
        # Phase 2: Continuous sine wave
        if self.init_complete:
            sine_elapsed = time.time() - self.sine_start_time
            
            # Calculate sine wave position
            # position = center + amplitude * sin(2π*t/period)
            angle = 2.0 * math.pi * sine_elapsed / self.period
            position = self.center + self.amplitude * math.sin(angle)
            phase_info = "SINE"
        
        # Clamp to safe range
        position = max(0.048, min(0.540, position))
        
        # Create message with proper layout
        msg = Float64MultiArray()
        msg.layout = MultiArrayLayout(dim=[], data_offset=0)
        msg.data = [position]
        
        # Publish
        self.cmd_publisher.publish(msg)
        
        # Log with comparison to actual
        error = position - self.latest_position
        self.get_logger().info(
            f'[{phase_info}] t={elapsed:6.2f}s | Cmd: {position:.4f}m | Actual: {self.latest_position:.4f}m | '
            f'Error: {error:+.4f}m | Vel: {self.latest_velocity:.4f}m/s'
        )
    
    def joint_state_callback(self, msg):
        """Store latest joint state feedback"""
        if len(msg.position) > 0:
            self.latest_position = msg.position[0]
        if len(msg.velocity) > 0:
            self.latest_velocity = msg.velocity[0]

def main(args=None):
    rclpy.init(args=args)
    node = SineWaveCommandPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutdown requested")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()