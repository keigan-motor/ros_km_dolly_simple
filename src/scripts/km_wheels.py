#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys,time
from math import sqrt, cos, sin
import rospy
import tf,tf2_ros
import numpy as np
from time import sleep
from geometry_msgs.msg import Twist, Pose, TransformStamped,Quaternion
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from std_msgs.msg import *
from nav_msgs.msg import Odometry

import rosparam
params = rosparam.get_param("/km_dolly_wheels")
if params['connect_mode']=='ble':
    from pykeigan import blecontroller
    SLEEP_TIME = 0.01
else:
    from pykeigan import usbcontroller
    SLEEP_TIME = 0.1
import atexit

D_RIGHT=0.0597    #m, right wheel radius
D_LEFT=0.0597     #m, left wheel radius
TREAD=0.306 #0.175       #m, length between wheels
LINEAR_RATIO = 0.25 # Tuning parameter of dolly linear speed
ANGULAR_RATIO = 2.5 #Tuning parameter of dolly angular speed
GYRO_OFFSET = np.array([-0.02053863,0.0176115,0.03733194])

def calc_input(omega_r,omega_l):
    """
    Parameters
    ----------
    omega_r, omega_l : float
        angular velocities of the wheels

    Returns
    ----------
    v : float
        Linear velocity of the dolly
    yawrate : float
        Yaw anguler velocity of the dolly
    """
    vr=D_RIGHT*omega_r
    vl=D_LEFT*omega_l
    v=(vr+vl)/2.0
    yawrate=(vr-vl)/(TREAD/2.0)
    u = np.array([v,yawrate])
    return u

def MotionModel(x,u,dt):
    """
    Parameters
    ----------
    x : array
        Dolly pose at the last timestamp. Location and orientation (loc_x,loc_y,theta) of the dolly.
    u : array
        Dolly velocity. Linear and yaw angular velocity of the dolly.
    dt : float
        The difference between the current and last timestamp.

    Returns
    ----------
    x : array
        Estimated dolly pose at the current timestamp. Location and orientation (loc_x,loc_y,theta) of the dolly.
    """
    loc_x = x[0]
    loc_y = x[1]
    theta = x[2]
    v = u[0]
    omega = u[1]
    loc_x = loc_x + np.cos(theta+np.pi/2)*v*dt
    loc_y = loc_y + np.sin(theta+np.pi/2)*v*dt
    theta = theta + omega*dt
    theta = PItoPI(theta)
    x=np.array([loc_x,loc_y,theta])
    return x

def PItoPI(angle):
    while angle>=np.pi:
        angle=angle-2*np.pi
    while angle<=-np.pi:
        angle=angle+2*np.pi
    return angle

class KMWheels:
    def __init__(self):
        '''
        right wheel: front:-x,left:z,top:-y left-handed cordinate
        '''
        if params['connect_mode'] == 'ble':
            self.right_w_dev=blecontroller.BLEController(params['right_w_addr'])
        else:
            self.right_w_dev=usbcontroller.USBController(params['right_w_addr'])
        print "Right Wheel Connected"
        rospy.loginfo("Connected to Right Wheel")
        self.right_w_dev.enable_action()
        self.right_w_dev.enable_continual_imu_measurement()
        self.right_w_dev.set_max_torque(1.0)
        self.right_w_dev.set_led(1, 0, 200, 0)
        sleep(1.0)
        self.last_right_pos=self.right_w_dev.read_motor_measurement()['position']
        '''
        left wheel: front:x,left:-z,top:-y left-handed cordinate
        '''
        if params['connect_mode'] == 'ble':
            self.left_w_dev=blecontroller.BLEController(params['left_w_addr'])
        else:
            self.left_w_dev=usbcontroller.USBController(params['left_w_addr'])
        print "Left Wheel Connected"
        rospy.loginfo("Connected to Left Wheel")
        self.left_w_dev.enable_action()
        self.left_w_dev.enable_continual_imu_measurement()
        self.left_w_dev.set_max_torque(1.0)
        self.left_w_dev.set_led(1, 0, 200, 0)

        sleep(1.0)
        self.last_left_pos=self.left_w_dev.read_motor_measurement()['position']
        sleep(0.1)

        self.imu = Imu()
        self.imu.header.stamp = rospy.Time.now()
        self.imu.header.frame_id = "imu_link"
        self.pub_imu = rospy.Publisher('imu', Imu, queue_size=10)
        self.imu_count = 0
        self.imu_val = np.array([0.0,0.0,0.0])

        self.odo = Odometry()
        self.odo.header.frame_id='odom'
        self.odo.child_frame_id='base_link'
        self.odocount = 0
        self.pub_odo = rospy.Publisher('/odom', Odometry,queue_size=10)
        self.x = np.array([0.0,0.0,0.0])
        self.current_time = time.time()
        self.last_time = time.time()

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.odom_trans = TransformStamped()
        self.odom_trans.header.stamp = rospy.Time.now()
        self.odom_trans.header.frame_id = "odom"
        self.odom_trans.child_frame_id = "base_link"

        self.right_velocity = 0
        self.left_velocity = 0

        self.right_angle_trans = TransformStamped()
        self.right_angle_trans.header.stamp = rospy.Time.now()
        self.right_angle_trans.header.frame_id = "right_km1"
        self.right_angle_trans.child_frame_id = "right_wheel"

        self.left_angle_trans = TransformStamped()
        self.left_angle_trans.header.stamp = rospy.Time.now()
        self.left_angle_trans.child_frame_id = "left_wheel"
        self.left_angle_trans.header.frame_id = "left_km1"

        rospy.Subscriber('/cmd_vel',Twist,self.teleop_callback,queue_size=1)


    def pubimu(self):
        try:
            imu_values = self.right_w_dev.read_imu_measurement()
            right_w_x = imu_values['accel_x']
            right_w_y = imu_values['accel_y']
            right_w_z = imu_values['accel_z']
            right_w_gyro_x = imu_values['gyro_x']
            right_w_gyro_y = imu_values['gyro_y']
            right_w_gyro_z = imu_values['gyro_z']
            self.imu.linear_acceleration.x = - 9.80665 * (right_w_x)
            self.imu.linear_acceleration.y = 9.80665 * (right_w_z)
            self.imu.linear_acceleration.z = 9.80665 * (right_w_y)
            self.imu.orientation_covariance[0]=-1
            gyro =np.array([right_w_gyro_x,right_w_gyro_y,right_w_gyro_z])
            gyro -= GYRO_OFFSET
            self.imu_val += gyro
            #self.imu_count += 1
            #print "IMU:", self.imu_val/self.imu_count
            if np.abs(gyro[0])<0.001:
                self.imu.angular_velocity.x = 0
            else:
                self.imu.angular_velocity.x = -gyro[0]
            if np.abs(gyro[2])<0.001:
                self.imu.angular_velocity.y = 0
            else:
                self.imu.angular_velocity.y = gyro[2]
            if np.abs(gyro[1])<0.001:
                self.imu.angular_velocity.z = 0
            else:
                self.imu.angular_velocity.z = gyro[1]

            self.imu.header.stamp = rospy.Time.now()
            self.pub_imu.publish(self.imu)
        except:
            import traceback
            traceback.print_exc()

    def pubodo(self):
        try:
            self.odocount+=1
            self.current_time = time.time()
            dt=self.current_time-self.last_time

            lvalues = self.left_w_dev.read_motor_measurement()
            left_velocity = lvalues['velocity']
            left_position = lvalues['position']

            rvalues = self.right_w_dev.read_motor_measurement()
            right_velocity=rvalues['velocity']
            right_position=rvalues['position']

            u=calc_input(-right_velocity,left_velocity)#right wheel rotates backwards
            old_theta = self.x[2]
            self.x=MotionModel(self.x,u,dt)
            self.odo.header.seq=self.odocount
            self.odo.header.stamp=rospy.Time.now()
            self.odo.pose.pose.position.x=self.x[0]
            self.odo.pose.pose.position.y=self.x[1]

            q = tf.transformations.quaternion_from_euler(0, 0, self.x[2])
            self.odo.pose.pose.orientation=Quaternion(*q)
            self.odo.twist.twist.linear.x=u[0]
            self.odo.twist.twist.angular.z=u[1]

            self.pub_odo.publish(self.odo)

            self.odom_trans.header.stamp = rospy.Time.now()
            self.odom_trans.transform.translation.x = self.x[0]
            self.odom_trans.transform.translation.y = self.x[1]
            self.odom_trans.transform.translation.z = 0
            self.odom_trans.transform.rotation = Quaternion(*q)
            self.tf_broadcaster.sendTransform(self.odom_trans)

            self.left_angle_trans.header.stamp = rospy.Time.now()
            self.left_angle_trans.transform.translation.x = -0.03
            self.left_angle_trans.transform.translation.y = 0
            self.left_angle_trans.transform.translation.z = 0
            q = tf.transformations.quaternion_from_euler(-left_position,0,0)
            self.left_angle_trans.transform.rotation = Quaternion(*q)
            self.tf_broadcaster.sendTransform(self.left_angle_trans)

            self.right_angle_trans.header.stamp = rospy.Time.now()
            self.right_angle_trans.transform.translation.x = 0.03
            self.right_angle_trans.transform.translation.y = 0
            self.right_angle_trans.transform.translation.z = 0
            q = tf.transformations.quaternion_from_euler(right_position,0,0)
            self.right_angle_trans.transform.rotation = Quaternion(*q)
            self.tf_broadcaster.sendTransform(self.right_angle_trans)

            self.last_time = time.time()
        except:
            import traceback
            traceback.print_exc()

    def teleop_callback(self,data):
        linear_speed = LINEAR_RATIO*data.linear.x
        angular_speed = ANGULAR_RATIO*data.angular.z
        new_right_velocity = linear_speed / D_RIGHT + angular_speed / (TREAD/D_RIGHT/2)
        new_left_velocity = linear_speed / D_LEFT - angular_speed / (TREAD/D_LEFT/2)
        if self.right_velocity != new_right_velocity or self.left_velocity != new_left_velocity:
            self.right_velocity = new_right_velocity
            self.left_velocity = new_left_velocity
            self.run_ctrl_cmd()

    def run_ctrl_cmd(self):
        if self.left_velocity!=0:
            self.left_w_dev.set_speed(np.abs(self.left_velocity))
            if self.left_velocity>0:
                self.left_w_dev.run_forward()
            elif self.left_velocity<0:
                self.left_w_dev.run_reverse()
        else:
            self.left_w_dev.free_motor()
        if self.right_velocity!=0:
            self.right_w_dev.set_speed(np.abs(self.right_velocity))
            if self.right_velocity>0:
                self.right_w_dev.run_reverse()
            elif self.right_velocity<0:
                self.right_w_dev.run_forward()
        else:
            self.right_w_dev.free_motor()

    def shutdown(self):
        rospy.sleep(1)
        self.right_w_dev.finish_auto_serial_reading()  # 切断前にモーターの通知を停止
        self.left_w_dev.finish_auto_serial_reading()  # 切断前にモーターの通知を停止
        self.right_w_dev.set_led(1, 100, 100, 100)
        self.left_w_dev.set_led(1, 100, 100, 100)
        rospy.sleep(1)
        # atexit.register()#修了イベントバインド

def km_wheels_main():
    rospy.init_node('km_wheels', anonymous = True)
    for k in range(10):
        try:
            km_wheels = KMWheels()
        except:
            print "Initializing ERROR"
            sleep(1)
        else:
            break

    while not rospy.is_shutdown():
        km_wheels.pubimu()
        km_wheels.pubodo()
        sleep(SLEEP_TIME)

if __name__=='__main__':
    km_wheels_main()
