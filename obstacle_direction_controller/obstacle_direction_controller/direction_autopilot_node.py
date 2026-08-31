import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from obstacle_direction_interfaces.srv import SetDirection


# Autonomous state machine states
STATE_FORWARD = "FORWARD"
STATE_REVERSE = "REVERSE"
STATE_TURN = "TURN"

VALID_DIRECTIONS = ("forward", "reverse", "left", "right")


class DirectionAutopilotNode(Node):

    def __init__(self):
        super().__init__('direction_autopilot_node')

        # --- Tunable parameters ---
        self.declare_parameter('obstacle_distance', 0.5)     # meters
        self.declare_parameter('linear_speed', 0.15)          # m/s
        self.declare_parameter('angular_speed', 0.5)           # rad/s
        self.declare_parameter('reverse_ticks', 10)            # control loop ticks
        self.declare_parameter('turn_ticks', 15)               # control loop ticks (safety ceiling)
        self.declare_parameter('override_ticks', 30)           # control loop ticks
        self.declare_parameter('turn_safety', 0.40)            # meters, min clearance to turn into
        self.declare_parameter('free_forward_distance', 1.00)  # meters, front clearance to exit TURN

        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.reverse_ticks_total = self.get_parameter('reverse_ticks').value
        self.turn_ticks_total = self.get_parameter('turn_ticks').value
        self.override_ticks_total = self.get_parameter('override_ticks').value
        self.turn_safety = self.get_parameter('turn_safety').value
        self.free_forward_distance = self.get_parameter('free_forward_distance').value

        # --- Autonomous state ---
        self.state = STATE_FORWARD
        self.turn_direction = None
        self.state_ticks_remaining = 0

        # --- Manual override state ---
        self.override_active = False
        self.override_direction = None
        self.override_ticks_remaining = 0

        # --- Latest LiDAR sector distances ---
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')

        # Subscriber: LiDAR
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        # Publisher: robot velocity
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Service: manual override
        self.set_direction_service = self.create_service(
            SetDirection,
            '/set_direction',
            self.set_direction_callback
        )

        # Control loop timer (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('direction_autopilot_node started. State: FORWARD')

    # ------------------------------------------------------------------
    # LiDAR processing
    # ------------------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        self.front_distance = self.get_sector_min(msg, -30.0, 30.0)
        self.left_distance = self.get_sector_min(msg, 30.0, 90.0)
        self.right_distance = self.get_sector_min(msg, -90.0, -30.0)

    def get_sector_min(self, msg: LaserScan, angle_start_deg: float, angle_end_deg: float) -> float:
        """Return the minimum valid range reading within an angle sector (degrees)."""
        angle_start = math.radians(angle_start_deg)
        angle_end = math.radians(angle_end_deg)

        start_index = int((angle_start - msg.angle_min) / msg.angle_increment)
        end_index = int((angle_end - msg.angle_min) / msg.angle_increment)

        start_index = max(0, min(start_index, len(msg.ranges) - 1))
        end_index = max(0, min(end_index, len(msg.ranges) - 1))

        if start_index > end_index:
            start_index, end_index = end_index, start_index

        sector = msg.ranges[start_index:end_index + 1]
        valid = [r for r in sector if msg.range_min < r < msg.range_max and not math.isinf(r) and not math.isnan(r)]

        return min(valid) if valid else float('inf')

    # ------------------------------------------------------------------
    # Service: manual override
    # ------------------------------------------------------------------
    def set_direction_callback(self, request, response):
        direction = request.direction.strip().lower()

        if direction not in VALID_DIRECTIONS:
            response.success = False
            response.message = f"Invalid direction '{request.direction}'. Must be one of {VALID_DIRECTIONS}."
            self.get_logger().warn(response.message)
            return response

        self.override_active = True
        self.override_direction = direction
        self.override_ticks_remaining = self.override_ticks_total

        response.success = True
        response.message = f"Override accepted: moving '{direction}' for {self.override_ticks_total} ticks."
        self.get_logger().info(f"MANUAL OVERRIDE -> {direction}")
        return response

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    def control_loop(self):
        if self.override_active:
            twist = self.twist_for_direction(self.override_direction)
            self.override_ticks_remaining -= 1

            if self.override_ticks_remaining <= 0:
                self.override_active = False
                self.override_direction = None
                self.get_logger().info('Override finished. Returning to autonomous mode.')
        else:
            twist = self.run_autonomous_step()

        self.cmd_vel_publisher.publish(twist)

    def twist_for_direction(self, direction: str) -> Twist:
        twist = Twist()
        if direction == 'forward':
            twist.linear.x = self.linear_speed
        elif direction == 'reverse':
            twist.linear.x = -self.linear_speed
        elif direction == 'left':
            twist.angular.z = self.angular_speed
        elif direction == 'right':
            twist.angular.z = -self.angular_speed
        return twist

    # ------------------------------------------------------------------
    # Autonomous obstacle avoidance: forward -> turn -> reverse
    # ------------------------------------------------------------------
    def run_autonomous_step(self) -> Twist:
        twist = Twist()

        can_turn_left = self.left_distance > self.turn_safety
        can_turn_right = self.right_distance > self.turn_safety

        # ==============================================================
        # FORWARD
        # ==============================================================
        if self.state == STATE_FORWARD:

            if self.front_distance < self.obstacle_distance:
                self.state = STATE_REVERSE
                self.state_ticks_remaining = self.reverse_ticks_total

                self.get_logger().warn(
                    f'OBSTACLE: Front {self.front_distance:.2f} m '
                    f'<= {self.obstacle_distance:.2f} m'
                )
                self.get_logger().info('State: FORWARD -> REVERSE')

                twist.linear.x = -self.linear_speed
            else:
                twist.linear.x = self.linear_speed

        # ==============================================================
        # REVERSE
        # ==============================================================
        elif self.state == STATE_REVERSE:

            twist.linear.x = -self.linear_speed
            self.state_ticks_remaining -= 1

            if self.state_ticks_remaining <= 0:

                if can_turn_left and can_turn_right:
                    self.turn_direction = (
                        'left' if self.left_distance >= self.right_distance else 'right'
                    )
                    self.state = STATE_TURN
                    self.state_ticks_remaining = self.turn_ticks_total
                    self.get_logger().info(f'State: REVERSE -> TURN ({self.turn_direction})')

                elif can_turn_left:
                    self.turn_direction = 'left'
                    self.state = STATE_TURN
                    self.state_ticks_remaining = self.turn_ticks_total
                    self.get_logger().info('State: REVERSE -> TURN (left)')

                elif can_turn_right:
                    self.turn_direction = 'right'
                    self.state = STATE_TURN
                    self.state_ticks_remaining = self.turn_ticks_total
                    self.get_logger().info('State: REVERSE -> TURN (right)')

                else:
                    # Both sides blocked. Stay in reverse for another period.
                    self.state_ticks_remaining = self.reverse_ticks_total
                    self.get_logger().error(
                        f'TRAPPED: Left {self.left_distance:.2f} m | '
                        f'Right {self.right_distance:.2f} m'
                    )

        # ==============================================================
        # TURN
        # ==============================================================
        elif self.state == STATE_TURN:

            # Re-check whether the current turning direction is still safe
            if self.turn_direction == 'left':
                if not can_turn_left and can_turn_right:
                    self.turn_direction = 'right'
                    self.get_logger().warn('LEFT blocked -> changing turn direction to RIGHT')
                elif not can_turn_left and not can_turn_right:
                    self.state = STATE_REVERSE
                    self.state_ticks_remaining = self.reverse_ticks_total
                    self.get_logger().error('Both directions blocked -> REVERSE')
                    twist.linear.x = -self.linear_speed
                    return twist

            elif self.turn_direction == 'right':
                if not can_turn_right and can_turn_left:
                    self.turn_direction = 'left'
                    self.get_logger().warn('RIGHT blocked -> changing turn direction to LEFT')
                elif not can_turn_right and not can_turn_left:
                    self.state = STATE_REVERSE
                    self.state_ticks_remaining = self.reverse_ticks_total
                    self.get_logger().error('Both directions blocked -> REVERSE')
                    twist.linear.x = -self.linear_speed
                    return twist

            # Tick ceiling: guarantees TURN always ends even if front never clears
            self.state_ticks_remaining -= 1

            front_clear = self.front_distance > self.free_forward_distance
            ticks_expired = self.state_ticks_remaining <= 0

            if front_clear or ticks_expired:
                self.state = STATE_FORWARD
                self.turn_direction = None
                self.state_ticks_remaining = 0
                twist.linear.x = self.linear_speed

                if front_clear:
                    self.get_logger().info(f'PATH CLEAR: Front {self.front_distance:.2f} m')
                else:
                    self.get_logger().info('TURN ticks expired -> forcing FORWARD')

                self.get_logger().info('State: TURN -> FORWARD')
            else:
                twist.angular.z = (
                    self.angular_speed if self.turn_direction == 'left' else -self.angular_speed
                )
                self.get_logger().info(
                    f'TURNING {self.turn_direction.upper()} | '
                    f'Front: {self.front_distance:.2f} m | '
                    f'Left: {self.left_distance:.2f} m | '
                    f'Right: {self.right_distance:.2f} m',
                    throttle_duration_sec=0.5
                )

        return twist


def main(args=None):
    rclpy.init(args=args)
    node = DirectionAutopilotNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()