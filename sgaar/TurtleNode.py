# python imports
from array import array
from math import degrees, isinf, radians
import numpy as np

# ros2 imports
from geometry_msgs.msg import Point, PolygonStamped, PoseWithCovarianceStamped, Quaternion, Twist
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan

# personal imports
# from sgaar.DetectedObject import detect_objects
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

        self.odom_position: Point = Point()
        self.odom_orientation: Quaternion = Quaternion()
        self.odom_roll: float = 0.0
        self.odom_pitch: float = 0.0
        self.odom_yaw: float = 0.0
        self.odom_timestamp: float = 0.0
        self.odom_dt = 0.0

        self.check_topic_available(self.odom_subscriber)

        # # # amcl_pose # # #
        self.amcl_pose_subscriber: Subscription = self.create_subscription(
            PoseWithCovarianceStamped, f"/amcl_pose", self.__amcl_pose_callback, 10)

        self.amcl_position: Point = Point()
        self.amcl_orientation: Quaternion = Quaternion()
        self.amcl_roll: float = 0.0
        self.amcl_pitch: float = 0.0
        self.amcl_yaw: float = 0.0
        self.amcl_timestamp: float = 0.0
        self.amcl_dt = 0.0

        self.check_topic_available(self.amcl_pose_subscriber)

        # # # clock # # # 
        self.clock_subscriber: Subscription = self.create_subscription(
            Clock, f"{namespace}/clock", self.__clock_callback, 10)

        self.previous_wall_time: float = 0.0
        self.current_wall_time: float = self.get_clock().now().nanoseconds / 1e9
        self.sim_current_time: float = 0.0
        self.sim_start_time: float = None
        self.sim_elapsed_time: float = 0.0

        self.check_topic_available(self.clock_subscriber)

        # # # lidar # # # 
        self.lidar_subscriber: Subscription = self.create_subscription(
            LaserScan, f"{namespace}/scan", self.__lidar_callback, 10)

        # self.detected_objects: list[DetectedObject] = []
        self.lidar_timestamp: float = 0.0
        self.lidar_dt = 0.0

        self.check_topic_available(self.lidar_subscriber)

        # # # occupancy grid # # # 
        self.global_costmap_subscriber: Subscription = self.create_subscription(
            OccupancyGrid, f"/global_costmap/costmap", self.__global_costmap_callback, 10)

        self.map_meta_data: MapMetaData = MapMetaData()
        self.map_image: np.ndarray[np.ndarray[int]] = np.ndarray(shape=(0, 0), dtype=int)
        self.map_origin: Point = Point()
        self.local_costmap_footprint: PolygonStamped = PolygonStamped()
        self.occupancy_grid_timestamp: float = 0.0
        self.occupancy_grid_dt = 0.0

        self.check_topic_available(self.global_costmap_subscriber)

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

        self.odom_position = Point(
            x=msg.pose.pose.position.x,
            y=msg.pose.pose.position.y
        )
        self.orientation = msg.pose.pose.orientation
        self.set_time()
        self.odom_dt = self.current_wall_time - self.odom_timestamp
        self.odom_timestamp = self.current_wall_time

        self.pose_logger.log([
            self.get_clock().now().nanoseconds / 1e9, 
            self.odom_position.x, 
            self.odom_position.y, 
            self.odom_position.z, 
            degrees(self.odom_roll),
            degrees(self.odom_pitch),
            degrees(self.odom_yaw)
        ])
    
    def __amcl_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self.last_callback = self.__amcl_pose_callback            
    
        # rotate point 90 degrees clockwise
        self.amcl_position = Point(
            x=self.map_origin.x - msg.pose.pose.position.x,
            y=self.map_origin.y - msg.pose.pose.position.y,
            z=msg.pose.pose.position.z
        )
        # self.amcl_position = Point(
        #     x=msg.pose.pose.position.x,
        #     y=msg.pose.pose.position.y
        # )
        print(self.amcl_position)
        self.amcl_orientation = msg.pose.pose.orientation
        self.set_time()
        self.amcl_dt = self.current_wall_time - self.amcl_timestamp
        self.amcl_timestamp = self.current_wall_time

        self.pose_logger.log([
            self.get_clock().now().nanoseconds / 1e9, 
            self.amcl_position.x, 
            self.amcl_position.y, 
            self.amcl_position.z, 
            degrees(self.amcl_roll),
            degrees(self.amcl_pitch),
            degrees(self.amcl_yaw)
        ])

        return None

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
        self.lidar_dt = self.current_wall_time - self.lidar_timestamp
        self.lidar_timestamp = self.current_wall_time

        # self.detected_objects: list[DetectedObject] = []
        for i, distance in enumerate(msg.ranges):
            if not isinf(distance):
                angle: float = radians(float(i if i < 180 else i - 360))
                # detected_object: DetectedObject = DetectedObject(
                #     distance=distance, 
                #     angle=angle, 
                #     current_position=support_module_Point(
                #         x=self.position.x, 
                #         y=self.position.y, 
                #         z=0), 
                #     current_yaw=self.yaw)
                # self.detected_objects.append(detected_object)

        # self.lidar_logger.log([
        #     self.get_clock().now().nanoseconds / 1e9,
        #     len(self.detected_objects)
        # ])

    def __global_costmap_callback(self, msg: OccupancyGrid) -> None:
        self.last_callback = self.__global_costmap_callback

        self.set_time()
        self.occupancy_grid_dt = self.current_wall_time - self.occupancy_grid_timestamp
        self.occupancy_grid_timestamp = self.current_wall_time

        self.map_origin = msg.info.origin.position
        
        self.map_meta_data = msg.info
        self.map_image = (np.array(msg.data)
            .reshape(self.map_meta_data.height, self.map_meta_data.width))
        
        return None


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