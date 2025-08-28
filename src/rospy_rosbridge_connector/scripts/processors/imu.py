from bridge.rosmsg_util import rosmsg_to_dict
from processors.processor_base import BaseProcessor, processor_decorator

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import numpy as np
import rospy

from tf.transformations import *

class ImuProcessor(BaseProcessor):
    def __init__(self):
        self.imu_data_ = Imu()
        self.gyro_variance_ = 5e-4
        self.acceleration_variance_ = 0.05

        self.imu_data_.angular_velocity_covariance = [
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, self.gyro_variance_
        ]

        self.imu_data_.linear_acceleration_covariance = [
            self.acceleration_variance_, 0.0, 0.0,
            0.0, self.acceleration_variance_, 0.0,
            0.0, 0.0, self.acceleration_variance_
        ]

        self.mean_ = 0.0
        self.stddev_vel_ = np.sqrt(self.gyro_variance_)
        self.stddev_accel_ = np.sqrt(self.acceleration_variance_)

        self.prev_time_ = rospy.Time.now()
        self.start_flag_ = True

        # drift noise
        # self.drift_bias_z_ = 0.001           
        self.drift_bias_z_ = 0.003       #rad/s   
        self.drift_random_walk_z_ = 0.0001  #
        self.drift_accum_z_ = 0.0

    @processor_decorator
    def process(self, msg, publisher, output_topic):
        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time_).to_sec()
        self.prev_time_ = current_time
        
        drift_noise = np.random.normal(0.0, self.drift_random_walk_z_* np.sqrt(dt))
        self.drift_accum_z_ += drift_noise 

        bias_z = self.drift_bias_z_ * dt
        wz_with_noise = (msg.angular_velocity.z +
                         bias_z +
                         self.drift_accum_z_ +
                         np.random.normal(self.mean_, self.stddev_vel_)) # gausian noize and drift noise

        self.imu_data_.angular_velocity.x = msg.angular_velocity.x
        self.imu_data_.angular_velocity.y = msg.angular_velocity.y
        # self.imu_data_.angular_velocity.z = msg.angular_velocity.z + np.random.normal(self.mean_, self.stddev_vel_) # just gausian noize
        self.imu_data_.angular_velocity.z = wz_with_noise

        self.imu_data_.linear_acceleration.x = msg.linear_acceleration.x + np.random.normal(self.mean_, self.stddev_accel_)
        self.imu_data_.linear_acceleration.y = msg.linear_acceleration.y + np.random.normal(self.mean_, self.stddev_accel_)
        self.imu_data_.linear_acceleration.z = msg.linear_acceleration.z + np.random.normal(self.mean_, self.stddev_accel_)

        self.imu_data_.header.frame_id = msg.header.frame_id
        self.imu_data_.header.stamp = rospy.Time.now()

        if self.start_flag_:
            self.imu_data_.orientation.x = 0.0
            self.imu_data_.orientation.y = 0.0
            self.imu_data_.orientation.z = 0.0
            self.imu_data_.orientation.w = 1.0

            self.q_orig = np.array([
                        0.0,
                        0.0,
                        0.0,
                        1.0
                                    ])
            
            self.prev_time_ = rospy.Time.now()

            self.start_flag_ = False
        else:
            wx = self.imu_data_.angular_velocity.x
            wy = self.imu_data_.angular_velocity.y
            wz = self.imu_data_.angular_velocity.z

            norm_w = np.linalg.norm([wx, wy, wz])

            if norm_w > 1e-6:
                angle = norm_w * dt
                axis = np.array([wx, wy, wz]) / norm_w
 
                q_rot = quaternion_about_axis(angle, axis)
 
                q_new = quaternion_multiply(self.q_orig, q_rot)
                q_new /= np.linalg.norm(q_new)
                self.q_orig = q_new

                self.imu_data_.orientation.x = q_new[0]
                self.imu_data_.orientation.y = q_new[1]
                self.imu_data_.orientation.z = q_new[2]
                self.imu_data_.orientation.w = q_new[3]

            orientation_var_yaw = self.gyro_variance_  * dt * dt * 10
            self.imu_data_.orientation_covariance = [
                1e-6, 0.0, 0.0,
                0.0, 1e-6, 0.0,
                0.0, 0.0, orientation_var_yaw
            ]
            
        data = rosmsg_to_dict(self.imu_data_)
        return data