#!/usr/bin/env python

from math import atan2

# import library ros
import rospy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion


class GoalPoint:
    first = 0
    second = 1
    third = 2
    fourth = 3

    def __init__(self, x_init, y_init, z_init):
        self.status = GoalPoint.first
        self.x = x_init
        self.y = y_init
        self.z = z_init
        pass

    def set_status(self, point):
        if GoalPoint.first <= point <= GoalPoint.fourth:
            self.status = point
        else:
            self.status = 0

    def get_status(self):
        return self.status

    def get_point_by_status(self):
        if self.status == GoalPoint.first:
            return GoalPoint(-5.0, -5.0, 0.0)
        elif self.status == GoalPoint.second:
            return GoalPoint(5.0, -5.0, 0.0)
        elif self.status == GoalPoint.third:
            return GoalPoint(5.0, 5.0, 0.0)
        elif self.status == GoalPoint.fourth:
            return GoalPoint(-5.0, 5.0, 0.0)


class AutonomousFlight2:
    def __init__(self):
        self.sub_x = 0.0
        self.sub_y = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.on_point_counter = 0
        self.goal_point = GoalPoint(-2.0, -2.0, 0.0)
        self.rate = rospy.Rate(10)
        self.pub_take_off = rospy.Publisher("ardrone_2/takeoff", Empty, queue_size=10)
        self.pub_land = rospy.Publisher("ardrone_2/land", Empty, queue_size=10)
        self.pub_command = rospy.Publisher('ardrone_2/cmd_vel', Twist, queue_size=10)
        self.sub_command = rospy.Subscriber('ardrone_2/ground_truth/state', Odometry, self.own_odom_callback)
        rospy.Subscriber('ardrone_1/ground_truth/state', Odometry, self.odom_callback)
        self.command = Twist()
        rospy.on_shutdown(self.send_land)

    def send_take_off(self):
        self.pub_take_off.publish(Empty())
        self.rate.sleep()

    def send_land(self):
        self.pub_land.publish(Empty())

    def own_odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def odom_callback(self, msg):
        self.sub_x = msg.pose.pose.position.x
        self.sub_y = msg.pose.pose.position.y

    def set_command(self, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
        self.command.linear.x = linear_x
        self.command.linear.y = linear_y
        self.command.linear.z = linear_z
        self.command.angular.x = angular_x
        self.command.angular.y = angular_y
        self.command.angular.z = angular_z
        self.pub_command.publish(self.command)
        # self.rate.sleep()

    def is_on_point(self):
        inc_x = self.goal_point.get_point_by_status().x - self.x
        inc_y = self.goal_point.get_point_by_status().y - self.y
        if abs(inc_x) <= 1 and abs(inc_y) <= 1:
            return True
        else:
            return False

    def go_to_goal(self):
        if abs(self.sub_x - GoalPoint(-5.0, -5.0, 0.0).x) <= 1 \
                and abs(self.sub_y - GoalPoint(-5.0, -5.0, 0.0).y) <= 1:
            self.goal_point = GoalPoint(-2.0, -2.0, 0.0)

        elif abs(self.sub_x - GoalPoint(5.0, -5.0, 0.0).x) <= 1 \
                and abs(self.sub_y - GoalPoint(5.0, -5.0, 0.0).y) <= 1:
            self.goal_point = GoalPoint(2.0, -2.0, 0.0)

        elif abs(self.sub_x - GoalPoint(5.0, 5.0, 0.0).x) <= 1 \
                and abs(self.sub_y - GoalPoint(5.0, 5.0, 0.0).y) <= 1:
            self.goal_point = GoalPoint(2.0, 2.0, 0.0)

        elif abs(self.sub_x - GoalPoint(-5.0, 5.0, 0.0).x) <= 1 \
                and abs(self.sub_y - GoalPoint(-5.0, 5.0, 0.0).y) <= 1:
            self.goal_point = GoalPoint(-2.0, 2.0, 0.0)

        inc_x = self.goal_point.x - self.x
        inc_y = self.goal_point.y - self.y

        angle_to_goal = atan2(inc_y, inc_x)
        angle_mod = abs(angle_to_goal - self.theta)

        print "drone sub pos: ", abs(inc_x), ' ', abs(inc_y), " angel: ", angle_mod

        if abs(inc_x) <= 1 and abs(inc_y) <= 1:
            is_on_point = True
        else:
            is_on_point = False

        if angle_mod < 0.1 and not is_on_point:
            self.set_command(0.5, 0.0, 0.0, 0.0, 0.0, 0.0)
        elif angle_mod > 0.1 and not is_on_point:
            self.set_command(0.0, 0.0, 0.0, 0.0, 0.0, 0.3)
        elif is_on_point:
            self.set_command(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


class AutonomousFlight1:
    def __init__(self, drone_number, first_goal_point_x, first_goal_point_y):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.goal_point = GoalPoint(first_goal_point_x, first_goal_point_y, 0.0)
        self.on_point_counter = 0
        self.on_start_point = True
        self.rate = rospy.Rate(10)
        self.pub_take_off = rospy.Publisher("ardrone_" + drone_number + "/takeoff", Empty, queue_size=10)
        self.pub_land = rospy.Publisher("ardrone_" + drone_number + "/land", Empty, queue_size=10)
        self.pub_command = rospy.Publisher("ardrone_" + drone_number + "/cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("ardrone_" + drone_number + "/ground_truth/state", Odometry, self.odom_callback)
        self.command = Twist()
        rospy.on_shutdown(self.send_land)

    def send_take_off(self):
        self.pub_take_off.publish(Empty())
        self.rate.sleep()

    def send_land(self):
        self.pub_land.publish(Empty())

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def set_command(self, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
        self.command.linear.x = linear_x
        self.command.linear.y = linear_y
        self.command.linear.z = linear_z
        self.command.angular.x = angular_x
        self.command.angular.y = angular_y
        self.command.angular.z = angular_z
        self.pub_command.publish(self.command)
        self.rate.sleep()

    def is_on_point(self):
        inc_x = self.goal_point.get_point_by_status().x - self.x
        inc_y = self.goal_point.get_point_by_status().y - self.y
        if abs(inc_x) <= 1 and abs(inc_y) <= 1:
            return True
        else:
            return False

    def go_to_goal(self):
        goal_point = self.goal_point.get_point_by_status()

        inc_x = goal_point.x - self.x
        inc_y = goal_point.y - self.y

        angle_to_goal = atan2(inc_y, inc_x)

        angle_mod = abs(angle_to_goal - self.theta)

        # print "drone1 pos: ", abs(inc_x), ' ', abs(inc_y), " angel: ", angle_mod

        if self.on_start_point:
            self.command.linear.x = 0.9
            self.command.angular.z = 0.0
            self.pub_command.publish(self.command)
            if not self.is_on_point():
                self.on_start_point = False
                return True
            rospy.Rate(1).sleep()
            return False

        if angle_mod < 0.1 and not self.is_on_point():
            self.command.linear.x = 0.9
            self.command.angular.z = 0.0
            self.pub_command.publish(self.command)
            self.on_point_counter = 0
            return False

        elif angle_mod > 0.1 and self.is_on_point():
            self.command.linear.x = 0.0
            self.command.angular.z = 0.4
            self.pub_command.publish(self.command)
            self.on_point_counter += 1
            if self.on_point_counter == 1:
                return True
            return False

        elif angle_mod < 0.1 and self.is_on_point():
            self.command.linear.x = 0.9
            self.command.angular.z = 0.0
            self.pub_command.publish(self.command)
            self.on_point_counter += 1
            if self.on_point_counter == 1:
                return True
            return False

        elif angle_mod > 0.1 and not self.is_on_point():
            self.command.linear.x = 0.0
            self.command.angular.z = 0.4
            self.pub_command.publish(self.command)
            self.on_point_counter = 0
            return False


if __name__ == '__main__':
    try:
        rospy.init_node('forward', anonymous=False)
        uav1 = AutonomousFlight1('1', -5.0, -5.0)
        uav2 = AutonomousFlight2()

        while not rospy.is_shutdown():
            uav1.send_take_off()
            uav2.send_take_off()

            if uav1.goal_point.get_status() == GoalPoint.first:
                if uav1.go_to_goal():
                    uav1.goal_point.set_status(GoalPoint.second)

            elif uav1.goal_point.get_status() == GoalPoint.second:
                if uav1.go_to_goal():
                    uav1.goal_point.set_status(GoalPoint.third)

            elif uav1.goal_point.get_status() == GoalPoint.third:
                if uav1.go_to_goal():
                    uav1.goal_point.set_status(GoalPoint.fourth)

            elif uav1.goal_point.get_status() == GoalPoint.fourth:
                if uav1.go_to_goal():
                    uav1.goal_point.set_status(GoalPoint.first)

            uav2.go_to_goal()

    except rospy.ROSInterruptException:
        pass
