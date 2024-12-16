import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, GlobalPositionTarget
from mavros_msgs.srv import CommandBool, SetMode
import math

current_state = State()
current_gps = GlobalPositionTarget()

# Target coordinates (latitude, longitude, altitude)
target_lat = 47.3700001  # example latitude
target_lon = 8.555428  # example longitude
target_alt = 2.0  # example altitude in meters
max_speed = 20.0  # starting speed in meters per second

class PositionControlNode(Node):
    def __init__(self):
        super().__init__("position_control_node")

        # Subscriber to MAVROS state and global position
        self.state_sub = self.create_subscription(State, "mavros/state", self.state_cb, 10)
        self.gps_sub = self.create_subscription(GlobalPositionTarget, "mavros/global_position/global", self.gps_cb, 10)

        # Publisher for position setpoint
        self.local_pos_pub = self.create_publisher(PoseStamped, "mavros/setpoint_position/local", 10)

        # Service clients for arming and mode change
        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")

        # Wait for services to be available
        self.wait_for_services()

        # Timer to control setpoint publishing rate
        self.rate = self.create_timer(0.05, self.run_loop)

        # Pose initialization (Position control)
        self.pose = PoseStamped()
        self.pose.pose.position.x = 0.0
        self.pose.pose.position.y = 0.0
        self.pose.pose.position.z = 10.0

        self.last_req = self.get_clock().now()

    def wait_for_services(self):
        # Wait for arming and set_mode services
        while not self.arming_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("Arming service not available, waiting...")

        while not self.set_mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("SetMode service not available, waiting...")

    def state_cb(self, msg):
        global current_state
        current_state = msg

    def gps_cb(self, msg):
        global current_gps
        current_gps = msg
        self.get_logger().info(f"Current position - Lat: {current_gps.latitude}, Lon: {current_gps.longitude}, Alt: {current_gps.altitude}")

    def lat_lon_to_local(self, lat, lon, alt, current_lat, current_lon, current_alt):
        # Assuming simple ECEF to NED conversion (you may need a more accurate conversion method)
        lat_diff = lat - current_lat
        lon_diff = lon - current_lon
        alt_diff = alt - current_alt

        # Conversion to meters (this is approximate, you should implement more precise conversion)
        x = lat_diff * 1e5  # Approximate conversion factor
        y = lon_diff * 1e5  # Approximate conversion factor
        z = alt_diff  # Altitude difference

        return x, y, z

    def calculate_velocity(self, target_x, target_y, target_z, current_x, current_y, current_z):
        # Calculate vector towards the target position
        dx = target_x - current_x
        dy = target_y - current_y
        dz = target_z - current_z

        # Calculate distance to the target
        distance = math.sqrt(dx**2 + dy**2 + dz**2)

        if distance > 0:
            speed = max_speed  
            vx = (dx / distance) * speed
            vy = (dy / distance) * speed
            vz = (dz / distance) * speed
        else:
            vx, vy, vz = 0.0, 0.0, 0.0

        return vx, vy, vz

    def run_loop(self):
        if not current_state.connected:
            return
        if self.get_clock().now() - self.last_req < rclpy.duration.Duration(seconds=5.0):
            self.local_pos_pub.publish(self.pose)
            return

        if current_state.mode != "OFFBOARD":
            offb_set_mode = SetMode.Request()
            offb_set_mode.custom_mode = "OFFBOARD"

            if self.set_mode_client.call(offb_set_mode).mode_sent:
                self.get_logger().info("OFFBOARD enabled")
            self.last_req = self.get_clock().now()

        if not current_state.armed:
            arm_cmd = CommandBool.Request()
            arm_cmd.value = True

            if self.arming_client.call(arm_cmd).success:
                self.get_logger().info("Vehicle armed")
            self.last_req = self.get_clock().now()

        # Convert the target lat/lon to local coordinates
        target_x, target_y, target_z = self.lat_lon_to_local(
            target_lat, target_lon, target_alt,
            current_gps.latitude, current_gps.longitude, current_gps.altitude
        )

        # Calculate velocity towards the target
        vx, vy, vz = self.calculate_velocity(target_x, target_y, target_z, 
                                              self.pose.pose.position.x, 
                                              self.pose.pose.position.y, 
                                              self.pose.pose.position.z)

        # Update pose with the new position (speed-controlled)
        self.pose.pose.position.x += vx * 0.05  # Move in small steps based on velocity and time
        self.pose.pose.position.y += vy * 0.05
        self.pose.pose.position.z += vz * 0.05

        # Publish position setpoint
        self.local_pos_pub.publish(self.pose)

def main(args=None):
    rclpy.init(args=args)

    # Create and spin the node
    position_control_node = PositionControlNode()
    try:
        rclpy.spin(position_control_node)
    except KeyboardInterrupt:
        position_control_node.get_logger().info('Shutting down node...')
    finally:
        position_control_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
