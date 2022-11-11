# python imports
from math import degrees, isinf, radians
from array import array

# ros2 imports
from geometry_msgs.msg import Point, Quaternion, Twist
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan

# personal imports
from sgaar.DetectedObject import DetectedObject
from sgaar.Logger import Logger
from sgaar.Point import Point as support_module_Point

class Turtle(Node):
    def __init__(self, namespace='', name='Turtle') -> None:
        super().__init__(name)

        self.namespace = namespace
        self.name = name

        # # # cmd_vel # # # 
        self.cmd_vel_publisher: Publisher = self.create_publisher(
            Twist, f"{namespace}/cmd_vel", 10)

        self.twist: Twist = Twist()

        # # # odom # # # 
        self.odom_subscriber: Subscription = self.create_subscription(
            Odometry, f"{namespace}/odom", self.__odom_callback, 10)

        self.check_topic_available(self.odom_subscriber)

        self.position: Point = Point()
        self.orientation: Quaternion = Quaternion()
        self.roll: float = 0.0
        self.pitch: float = 0.0
        self.yaw: float = 0.0
        self.odom_dt = 0.0

        # # # clock # # # 
        self.clock_subscriber: Subscription = self.create_subscription(
            Clock, f"{namespace}/clock", self.__clock_callback, 10)

        self.check_topic_available(self.clock_subscriber)

        self.previous_wall_time: float = 0.0
        self.current_wall_time: float = self.get_clock().now().nanoseconds / 1e9
        self.sim_current_time: float = 0.0
        self.sim_start_time: float = None
        self.sim_elapsed_time: float = 0.0

        # # # lidar # # # 
        self.lidar_subscriber: Subscription = self.create_subscription(
            LaserScan, f"{namespace}/scan", self.__lidar_callback, 10)

        self.check_topic_available(self.lidar_subscriber)

        self.detected_objects: list[DetectedObject] = []
        self.lidar_dt = 0.0

        # # # occupancy grid # # # 
        self.occupancy_grid_subscriber: Subscription = self.create_subscription(
            OccupancyGrid, f"{namespace}/map", self.__occupancy_grid_callback, 10)

        self.check_topic_available(self.occupancy_grid_subscriber)

        self.map_meta_data: MapMetaData = MapMetaData()
        self.map: array[int] = array('i')
        self.occupancy_grid_dt = 0.0

        self.last_callback = FileNotFoundError

        # # # loggers # # # 
        self.command_logger = Logger(
            headers=["time", "linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z"], 
            filename=f"{name}_command_log.csv")
        self.heading_logger = Logger(
            headers=["time", "desired_heading", "actual_heading"], 
            filename=f"{name}_heading_log.csv")
        self.pose_logger = Logger(
            headers=["time", "position_x", "position_y", "position_z", "roll", "pitch", "yaw"], 
            filename=f"{name}_pose_log.csv")
        self.lidar_logger = Logger(
            headers=["time", "num_objects"],
            filename=f"{name}_lidar_log.csv")

    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
    # Subscribers
    def check_topic_available(self, subscriber: Subscription) -> None:
        for topic in self.get_topic_names_and_types():
            if topic[0] == subscriber.topic_name:
                return
        raise Exception(f"Invalid subscriber topic: {subscriber.topic_name}")

    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
    # Callbacks
    def __odom_callback(self, msg: Odometry) -> None:
        self.last_callback = self.__odom_callback

        self.position = Point(
            x=msg.pose.pose.position.x,
            y=msg.pose.pose.position.y
        )
        self.orientation = msg.pose.pose.orientation
        self.set_time()
        self.odom_dt = self.current_wall_time - self.previous_wall_time

        self.pose_logger.log([
            self.get_clock().now().nanoseconds / 1e9, 
            self.position.x, 
            self.position.y, 
            self.position.z, 
            degrees(self.roll),
            degrees(self.pitch),
            degrees(self.yaw)
        ])

    def __clock_callback(self, msg: Clock) -> None:
        self.last_callback = self.__clock_callback

        self.current_sim_time = msg.clock.sec + msg.clock.nanosec / 1e9
        self.sim_start_time = self.sim_start_time if self.sim_start_time else self.current_sim_time
        self.sim_elapsed_time = self.current_sim_time - self.sim_start_time

    def __lidar_callback(self, msg: LaserScan) -> None:
        # We can't use the lidar if we don't have odom data, for we need 
        # position and current yaw to convert an angle and distance to a point.
        if self.odom_dt == 0.0:
            return

        self.last_callback = self.__lidar_callback

        self.set_time()
        self.lidar_dt = self.current_wall_time - self.previous_wall_time

        self.detected_objects: list[DetectedObject] = []
        for i, distance in enumerate(msg.ranges):
            if not isinf(distance):
                angle: float = radians(float(i if i < 180 else i - 360))
                detected_object: DetectedObject = DetectedObject(
                    distance=distance, 
                    angle=angle, 
                    current_position=support_module_Point(
                        x=self.position.x, 
                        y=self.position.y, 
                        z=0), 
                    current_yaw=self.yaw)
                self.detected_objects.append(detected_object)

        self.lidar_logger.log([
            self.get_clock().now().nanoseconds / 1e9,
            len(self.detected_objects)
        ])

    def __occupancy_grid_callback(self, msg: OccupancyGrid) -> None:
        self.last_callback = self.__occupancy_grid_callback

        self.set_time()
        self.occupancy_grid_dt = self.current_wall_time - self.previous_wall_time
        
        self.map_meta_data = msg.info
        self.map = msg.data

    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
    # Publishers
    def move(self):
        self.cmd_vel_publisher.publish(self.twist)

        self.command_logger.log([
            self.get_clock().now().nanoseconds / 1e9, 
            self.twist.linear.x, 
            self.twist.linear.y, 
            self.twist.linear.z, 
            self.twist.angular.x, 
            self.twist.angular.y, 
            self.twist.angular.z
        ])

    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
    # Support

    def set_time(self) -> None:
        self.previous_wall_time = self.current_wall_time
        self.current_wall_time = self.get_clock().now().nanoseconds / 1e9


    def close_logs(self) -> None:
        self.command_logger.close()
        self.heading_logger.close()
        self.pose_logger.close()
        self.lidar_logger.close()

    def dump_point_cloud(self, filename: str) -> None:
        print(f"Dumping point cloud to {filename}")
        with open(filename, 'w') as f:
            f.writelines([
                "x,y,z\n",
                "\n".join([f"{obj.x},{obj.y},{obj.z}" for obj in self.detected_objects])
            ])