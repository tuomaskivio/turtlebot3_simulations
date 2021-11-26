#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Log
from turtlesim.msg import Pose
from gazebo_msgs.msg import ModelStates
from math import pow, atan2, sqrt, pi, floor
import sys
import time

def parse_path(file_location):
    f = open(file_location, "r")
    path = f.read()
    path = path.split("---\n")
    path.pop()
    for (i, pose) in enumerate(path):
        pose = pose.split('\n')
        pose = {'x': float(pose[0].split(': ')[1]),
                'y': float(pose[1].split(': ')[1]),
                'theta': float(pose[2].split(': ')[1])}
        path[i] = pose
    return path

def parse_path_gazebo(file_location):
    f = open(file_location, "r")
    path = f.read()
    path = path.split("<model name='beer")
    path.pop(0)
    for (i, pose) in enumerate(path):
        pose = pose.split('<pose>')[1]
        pose = pose.split(' ')
        pose = {'x': float(pose[0]),
                'y': float(pose[1])}
        path[i] = pose
    return path


class TurtleBot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('turtlebot_controller', anonymous=True)

        # Publisher which will publish to the topic '/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/cmd_vel',
                                                  Twist, queue_size=10)
                                                  
        # rosout publisher
        self.log = rospy.Publisher('/rosout', Log, queue_size=10)
        
        self.error_publisher = rospy.Publisher('/turtlebot_controller/error', Pose, queue_size=10)


        # A subscriber to the topic '/gazebo/model_states'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/gazebo/model_states',
                                                ModelStates, self.update_pose)

        self.pose = Pose()
        self.rate = rospy.Rate(10)
        

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        pose = data.pose[2]
        self.pose.x = round(pose.position.x, 4)
        self.pose.y = round(pose.position.y, 4)
        theta = atan2(pose.orientation.z, pose.orientation.w)*2
        self.pose.theta = (theta + pi) % (2*pi) - pi

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        vel = constant * self.euclidean_distance(goal_pose)
        return max(-0.5,min(0.5,vel))

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=2.5, target_reached=False):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        if (not target_reached):
            steering_angle = self.steering_angle(goal_pose)
            diff = steering_angle - self.pose.theta
            diff = (diff + pi) % (2*pi) - pi
            return max(-1.86,min(1.86,constant * diff))
        else:
            return constant*0.5 * self.angle_diff(goal_pose)
            
    def angle_diff(self,goal_pose):
        return (goal_pose.theta - self.pose.theta) % (2*pi)
        
    def publish_log(self, message):
        log = Log()
        log.msg = message
        current_time = time.time()
        s = floor(current_time)
        ns = floor((current_time - s) * 1000000)
        log.header.stamp.secs = s
        log.header.stamp.nsecs = ns
        log.name = rospy.get_name()
        self.log.publish(log)
    

    def move2goal(self,x,y,theta="Not used",distance_tolerance=0.1,angle_tolerance=0.1, constant_vel=0, stop_after=True):
        """Moves the turtle to the goal."""
        goal_pose = Pose()

        # Get the input from the user.
        goal_pose.x = x
        goal_pose.y = y
        if (theta != "Not used"):
          goal_pose.theta = theta
          
        # New goal
        if (theta != "Not used"):
            self.publish_log(f"New goal: (x={x}, y={y}, theta={theta})")
        else:
            self.publish_log(f"New goal: (x={x}, y={y})")

        vel_msg = Twist()
        while self.pose.x == 0:
            pass
            
        while self.euclidean_distance(goal_pose) >= distance_tolerance:

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            if (constant_vel != 0):
                vel_msg.linear.x = constant_vel
            else:
                vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()
            
        if stop_after:
          # Stopping our robot after the movement is over.
          vel_msg.linear.x = 0
          vel_msg.angular.z = 0
          self.velocity_publisher.publish(vel_msg)
        

        while theta != "Not used" and abs(self.angle_diff(goal_pose)) >= angle_tolerance:

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            
            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose, target_reached=True)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        if stop_after:
          # Stopping our robot after the movement is over.
          vel_msg.linear.x = 0
          vel_msg.angular.z = 0
          self.velocity_publisher.publish(vel_msg)
        
        # Goal reached
        self.publish_log(f"Goal reached: (x={self.pose.x}, y={self.pose.y}, theta={self.pose.theta})")
        error = Pose()
        error.x = x - self.pose.x
        error.y = y - self.pose.y
        self.error_publisher.publish(error)

        # If we press control + C, the node will stop.
        # rospy.spin()

if __name__ == '__main__':
    if (len(sys.argv) == 2):
        path = parse_path_gazebo(sys.argv[1])
        try:
            x = TurtleBot()
            for pose in path:
                if pose == path[-1]:
                    x.move2goal(pose['x'], pose['y'], constant_vel=0.5, stop_after=True)
                else:
                    x.move2goal(pose['x'], pose['y'], constant_vel=0.5, stop_after=False)
        except rospy.ROSInterruptException:
            pass
    else:    
        goal_x = float(sys.argv[1])
        goal_y = float(sys.argv[2])
        if len(sys.argv) > 3:
            goal_theta = float(sys.argv[3])
        else:
            goal_theta = "Not used"
        try:
            x = TurtleBot()
            x.move2goal(goal_x, goal_y, goal_theta)
        except rospy.ROSInterruptException:
            pass