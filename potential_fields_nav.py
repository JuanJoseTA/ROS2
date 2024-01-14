import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped, PoseArray
import numpy as np


class PotentialFieldsNav(Node):
    def __init__(self):
        super().__init__("potential_fields_nav")

        self.laser_sub = self.create_subscription(
            LaserScan, "/laser1", self.laser_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pose_sub = self.create_subscription(
            PoseArray, "/particlecloud", self.pose_callback, 10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_callback, 10
        )
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

        # Estado variables
        self.obstacle_force = np.array([0.0, 0.0])
        
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = "map"
        self.current_pose.pose.position.x = 0.0
        self.current_pose.pose.position.y = 0.0
        self.current_pose.pose.position.z = 0.0
        self.current_pose.pose.orientation.w = 1.0  # Orientación neutra
        self.goal_pose = None
        self.goal_force = np.array([0.0, 0.0])

        # Establecer el punto objetivo
        self.set_initial_goal()

    def set_initial_goal(self):
        # Crea un mensaje PoseStamped para el objetivo
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = 19.0
        goal_msg.pose.position.y = 0.0
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0  # Orientación neutra

        # Publica el objetivo
        self.goal_pub.publish(goal_msg)
        
        # Inicializa la pose actual en (0, 0, 0, 0)
        self.current_pose.pose.position.x = 0.0
        self.current_pose.pose.position.y = 0.0
        self.current_pose.pose.position.z = 0.0
        self.current_pose.pose.orientation.w = 1.0

    def laser_callback(self, msg):
        #print("Laser callback triggered")
        self.obstacle_force = self.calculate_obstacle_avoidance_force(msg)
        # Actualizar la fuerza de objetivo basado en la pose actual y el objetivo
        if self.goal_pose is not None and self.current_pose is not None:
            self.goal_force = self.calculate_goal_force(
                self.current_pose, self.goal_pose
            )
        self.update_cmd_vel()

    def goal_callback(self, msg):
        print("Goal callback triggered")
        print(msg)
        self.goal_pose = msg.pose

    def update_cmd_vel(self):
        if self.has_reached_goal():
            self.stop_robot()
            return

        if self.goal_force is not None and self.obstacle_force is not None:
            result_force = self.combine_forces(self.goal_force, self.obstacle_force)
            cmd_msg = self.convert_force_to_twist(result_force)
            #print("Publishing cmd_vel:", cmd_msg)
            self.cmd_pub.publish(cmd_msg)
    
    def has_reached_goal(self, tolerance=1):
        if self.current_pose is None or self.goal_pose is None:
            return False

        current_position = self.current_pose.pose.position
        goal_position = self.goal_pose.position

        distance = np.sqrt(
            (goal_position.x - current_position.x) ** 2 +
            (goal_position.y - current_position.y) ** 2
        )
        #print(distance)
        return distance < tolerance
    
    def stop_robot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_pub.publish(stop_msg)

    def pose_callback(self, msg):
        #print("Pose callback triggered")
        if msg.poses:
            # Asumiendo que tomamos la primera pose del array como la pose actual
            self.current_pose.pose = msg.poses[0]
            #print("Current Pose:", self.current_pose.pose)

    def calculate_obstacle_avoidance_force(self, laser_msg):
        radius_of_influence = 1.5  # Define el radio de influencia
        k_obstacles = 0.8  # Define la constante de repulsión

        # Convertir lecturas láser a coordenadas de obstáculos
        obstacles = [
            (distance * np.cos(laser_msg.angle_min + i * laser_msg.angle_increment),
             distance * np.sin(laser_msg.angle_min + i * laser_msg.angle_increment))
            for i, distance in enumerate(laser_msg.ranges)
            if distance < laser_msg.range_max
        ]

    # Posición actual del robot
        x_robot = np.array([self.current_pose.pose.position.x, self.current_pose.pose.position.y])

    # Calcular la fuerza de repulsión
        return self.repulsive_force(x_robot, np.array(obstacles).T, radius_of_influence, k_obstacles)
        
    def repulsive_force(self, x_robot, map_obstacles, radius_of_influence, k_obstacles):
        p_to_object = x_robot[:, None] - map_obstacles
        d = np.linalg.norm(p_to_object, axis=0)
        influential = d <= radius_of_influence

        if np.any(influential):
            p_to_object = p_to_object[:, influential]
            d = d[influential]
            f_rep = k_obstacles * np.sum(
                ((1 / d) - (1 / radius_of_influence)) * (1 / d**2) * (p_to_object / d), 
                axis=1, keepdims=True
            )
        else:
            f_rep = np.array([[0], [0]])

        return f_rep.ravel()


    def combine_forces(self, goal_force, obstacle_force):
        alpha = 0.5  # Peso hacia el objetivo
        beta = 0.5   # Peso de evitación    

    # Combinar fuerzas lineales y angulares
        combined_linear_force = alpha * goal_force[0] + beta * obstacle_force[0]
        combined_angular_direction = np.arctan2(
            alpha * np.sin(goal_force[1]) + beta * np.sin(obstacle_force[1]),
            alpha * np.cos(goal_force[1]) + beta * np.cos(obstacle_force[1])
        )

        return np.array([combined_linear_force, combined_angular_direction])


    def convert_force_to_twist(self, force_vector):
        max_linear_speed = 1.0
        max_angular_speed = 1.0

        linear_speed = np.clip(force_vector[0], -max_linear_speed, max_linear_speed)
        angular_speed = np.clip(force_vector[1], -max_angular_speed, max_angular_speed)
        cmd_msg = Twist()
        cmd_msg.linear.x = linear_speed
        cmd_msg.angular.z = angular_speed - self.current_pose.pose.orientation.z  # Ajuste basado en la orientación actual
        return cmd_msg


    def calculate_goal_force(self, current_pose, goal_pose):
        if current_pose is None or goal_pose is None:
            return np.array([0.0, 0.0])

        attractive_force_constant = 0.1
        current_position = current_pose.pose.position
        goal_position = goal_pose.position

        vector_to_goal = np.array([goal_position.x - current_position.x, goal_position.y - current_position.y])
        distance_to_goal = np.linalg.norm(vector_to_goal)

        if distance_to_goal == 0:
            return np.array([0.0, 0.0])

        linear_force = attractive_force_constant * distance_to_goal
        direction_to_goal = np.arctan2(vector_to_goal[1], vector_to_goal[0])

        return np.array([linear_force, direction_to_goal])






def main(args=None):
    rclpy.init(args=args)
    potential_fields_nav_node = PotentialFieldsNav()
    rclpy.spin(potential_fields_nav_node)
    potential_fields_nav_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()