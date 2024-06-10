import copy
import math
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist 
from turtlesim.msg import Pose
import time

from robot_cleaner.RobotPose import RobotPose

class RoboCleaner(Node):

    LINEAR_SPEED = 0.0
    ANGULAR_SPEED = 0.3

    def __init__(self):
        super().__init__('robocleaner')
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        timer_period = 0.01
        time.sleep(1.0)
        self.pose = RobotPose()
        rclpy.spin_once(self)
        
    def pose_callback(self, msg):
        self.pose.x = msg.x
        self.pose.y  = msg.y
        self.pose.theta  = msg.theta
        #print (self.pose)

    def move (self, linear_speed, distance, is_forward):
        print('Start Moving the Robot ...')
        rclpy.spin_once(self)
        twist_msg=Twist()
        
        if (linear_speed>1.0) :
            print('[ERROR]: The speed must be lower than 1.0!')
            return -1
                
        twist_msg.linear.x = abs(linear_speed) * (1 if is_forward else -1)

        start_pose = copy.copy(self.pose)
        
        rclpy.spin_once(self)

        while self.get_distance (start_pose, self.pose)<distance:
            rclpy.spin_once(self)
            self.velocity_publisher.publish(twist_msg)
            rclpy.spin_once(self)
            time.sleep(0.05)

        twist_msg.linear.x = 0.0
        self.velocity_publisher.publish(twist_msg)
        print ('start_pose', start_pose, 'self.pose', self.pose, 'distance: ', self.get_distance (start_pose, self.pose))
        print('The Robot has stopped...')

        return 0
    
    def get_distance(self,start, destination):
        return math.sqrt(((destination.x-start.x)**2 + (destination.y-start.y)**2))

    def rotate (self, angular_speed_degree, desired_relative_angle_degree, clockwise):
        print('Start Rotating the Robot ...')
        rclpy.spin_once(self)
        twist_msg=Twist()
        angular_speed_degree=abs(angular_speed_degree) #make sure it is a positive relative angle
        if (angular_speed_degree>30) :
            print (angular_speed_degree)
            print('[ERROR]: The rotation speed must be lower than 0.5!')
            return -1
        
        angular_speed_radians = math.radians(angular_speed_degree)
        twist_msg.angular.z = -abs(angular_speed_radians) if clockwise else abs(angular_speed_radians)
        twist_msg.angular.z = abs(angular_speed_radians) * (-1 if clockwise else 1)

        start_pose = copy.copy(self.pose)
        
        rclpy.spin_once(self)

        rotated_related_angle_degree=0.0

        while rotated_related_angle_degree<desired_relative_angle_degree:
            rclpy.spin_once(self)
            self.velocity_publisher.publish(twist_msg)
            #print ('rotated_related_angle_degree', rotated_related_angle_degree, 'desired_relative_angle_degree', desired_relative_angle_degree)
            rotated_related_angle_degree = math.degrees(abs(start_pose.theta - self.pose.theta))
            rclpy.spin_once(self)
            
            rclpy.spin_once(self)
            time.sleep(0.01)
        print ('rotated_related_angle_degree', rotated_related_angle_degree, 'desired_relative_angle_degree', desired_relative_angle_degree)
        twist_msg.angular.z = 0.0
        self.velocity_publisher.publish(twist_msg)
        print('The Robot has stopped...')

        return 0

    def go_to_goal(self, goal: RobotPose, distance_error_tolerance=0.5):
        twist_msg = Twist()

        while (True):
            
            p_gain_linear = 1.0 
            distance = abs(math.sqrt(((goal.get_x()-self.pose.get_x()) ** 2) + ((goal.get_y()-self.pose.get_y()) ** 2)))
            linear_speed = distance * p_gain_linear

            p_gain_angular = 2.0
            desired_angle_goal = math.atan2(goal.get_y()-self.pose.get_y(), goal.get_x()-self.pose.get_x())
            angular_speed = (desired_angle_goal-self.pose.get_theta())*p_gain_angular

            twist_msg.linear.x = linear_speed
            twist_msg.angular.z = angular_speed
            
            #-----____PUBLISHING----------------------
            rclpy.spin_once(self)
            self.velocity_publisher.publish(twist_msg)
            time.sleep(0.1)
            #---------------------------------------------

            if (distance <distance_error_tolerance):
                print('Goal reached. Mission completed.')
                rclpy.spin_once(self)
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                #-----____PUBLISHING TO STOP----------------------
                
                self.velocity_publisher.publish(twist_msg)
                time.sleep(0.1)
                #---------------------------------------------
                break
        print('distance = ', distance)
        print ('angle error = ', (desired_angle_goal-self.pose.get_theta()))
        return 0

    def menu (self):
        print('|----------------------------------|')
        print('| 1: Move')
       # print('| 2: Rotate')
       # print('| 3: Go to Goal')
        print('| Q: Quit')
        print('|----------------------------------|')
        choice = input("Enter your choice: ")
        return choice

    def run_app(self):
            rclpy.spin_once(self)
            while True:
                choice = self.menu()
                if choice == '1':
                    dist = input("distance: ")
                    direction = int(input("is_forward: 0 or 1: "))
                    speed = float(input("speed: "))
                    if int(direction) == 1:
                        direction = True
                    else: direction = False
                    print("Moving...")
                    self.move(linear_speed=speed, distance=float(dist), is_forward=direction)
                elif choice == '2':
                    print("Rotating...")
                    angle = input("angle: ")
                    clock = input("clockwise: 0 or 1: ")
                    if int(clock) == 1:
                        clock = True
                    else: clock = False 
                    self.rotate(15, float(angle), clock)
                elif choice == '3':
                    print("Go to Goal ...")
                    x = float(input("x: "))
                    y = float(input("y: "))
                    theta = float(input("theta: "))
                    self.go_to_goal(RobotPose(x, y, theta), distance_error_tolerance=0.5)
                elif choice == 'q':
                    print("Quit.....")
                    return 0
                else:
                    print("Invalid choice. Try again.")

def main (args=None):
        rclpy.init(args=args)
        robot_cleaner = RoboCleaner()

        robot_cleaner.run_app()    

        robot_cleaner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()