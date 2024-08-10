# week-2
1)MOVING TUTLE BOT WITH CONSTANT X VELOCITY.
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def move_constant_velocity():
    # Initialize the ROS node
    rospy.init_node('move_constant_velocity', anonymous=True)
    # Create a publisher to publish velocity commands
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    # Set the rate at which to publish the commands
    rate = rospy.Rate(10)  # 10 Hz

    # Create a Twist message
    vel_msg = Twist()
    vel_msg.linear.x = 0.5  # Set constant velocity on the x-axis
    vel_msg.angular.z = 0.0  # No rotation

    # Loop to keep publishing the velocity
    while not rospy.is_shutdown():
        velocity_publisher.publish(vel_msg)
        rate.sleep()
if __name__ == '__main__':
    try:
        move_constant_velocity()
    except rospy.ROSInterruptException:
        pass

  2)MOVING TURTLE BOT TO GOAL POINT
  #!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

# Goal point coordinates (you can set this based on your environment)
goal_x = 2.0
goal_y = 0.0
tolerance = 0.1

def get_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def odometry_callback(data):
    global goal_x, goal_y, tolerance

    # Extract position and orientation from the Odometry message
    position = data.pose.pose.position
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    _, _, yaw = euler_from_quaternion(orientation_list)

    distance_to_goal = get_distance(position.x, position.y, goal_x, goal_y)

    # Create a Twist message
    vel_msg = Twist()

    if distance_to_goal > tolerance:
        vel_msg.linear.x = 0.5  # Move forward
    else:
        vel_msg.linear.x = 0.0  # Stop the robot

    velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('move_to_goal', anonymous=True)
        velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, odometry_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
3)OBSTOICLW AVOIDANCE
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def scan_callback(data):
    # Define the safe distance from the obstacle
    min_safe_distance = 0.1

    # Check the minimum distance in front of the robot
    min_distance = min(min(data.ranges[0:15]), min(data.ranges[-15:]))

    # Create a Twist message
    vel_msg = Twist()

    if min_distance < min_safe_distance:
        vel_msg.linear.x = 0.0  # Stop the robot
    else:
        vel_msg.linear.x = 0.2  # Move forward

    velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('obstacle_avoidance', anonymous=True)
        velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, scan_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

4)CUSTOM KEYBOARD CONTROL
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys
import termios
import tty

def getKey():
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('keyboard_control')
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    try:
        while not rospy.is_shutdown():
            key = getKey()

            if key == 'w':  # Move forward
                vel_msg.linear.x = 0.5
                vel_msg.angular.z = 0.0
            elif key == 's':  # Move backward
                vel_msg.linear.x = -0.5
                vel_msg.angular.z = 0.0
            elif key == 'a':  # Turn left
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.5
            elif key == 'd':  # Turn right
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = -0.5
            elif key == 'x':  # Stop
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.0

            velocity_publisher.publish(vel_msg)

            if key == '\x03':  # Exit on Ctrl+C
                break

    except Exception as e:
        print(e)

    finally:
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        velocity_publisher.publish(vel_msg)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

