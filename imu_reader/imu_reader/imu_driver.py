# 230516 chson
# imu node for ebimu
# ros2 foxy

import threading
import time

import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from builtin_interfaces.msg import Time


## Prerequisites of EBIMU Settings: 
# quaternion mode
# linacc
# batt enable
# temp enable


class ImuNode(Node, threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self.daemon = True

        Node.__init__(self,'imu_driver')

        self.declare_parameter('imu_port', '/dev/ttyUSB0')
        self.declare_parameter('imu_baud', '921600')
        
        self.publisher_ = self.create_publisher(Imu, 'imu', 10) # declare publisher
        pub_rate = 50  # seconds, 50hz

        self.lg = self.get_logger() # logger

        self.timer = self.create_timer(1/pub_rate, self.timer_callback)
        self.i = 0

        self.imu_msg = Imu()

        self.run()

    def timer_callback(self):
        self.publisher_.publish(self.imu_msg)

    def run(self):
        while True:
            port = self.get_parameter('imu_port').get_parameter_value().string_value
            baud = self.get_parameter('imu_baud').get_parameter_value().string_value

            try:
                self.lg.debug('ebimu_node')
                ser = serial.Serial(port, baud)
                self.lg.info('trying to connect IMU')

                while True:
                    # Read data from the serial port

                    data = ser.readline().decode().strip().split(',')

                    # # Unpack the data into the IMU message
                    self.imu_msg.header.frame_id = 'imu_link'
                    self.imu_msg.header.stamp = self.get_clock().now().to_msg()
                    self.imu_msg.orientation.x, self.imu_msg.orientation.y, self.imu_msg.orientation.z, self.imu_msg.orientation.w, \
                    self.imu_msg.angular_velocity.x, self.imu_msg.angular_velocity.y, self.imu_msg.angular_velocity.z, \
                    self.imu_msg.linear_acceleration.x, self.imu_msg.linear_acceleration.y, self.imu_msg.linear_acceleration.z, \
                    temp, battery = map(float, data[1:])

                    self.imu_msg.linear_acceleration.x = self.imu_msg.linear_acceleration.x *9.81 # g unit to m/s^2
                    self.imu_msg.linear_acceleration.y = self.imu_msg.linear_acceleration.y *9.81 # g unit to m/s^2
                    self.imu_msg.linear_acceleration.z = self.imu_msg.linear_acceleration.z *9.81 # g unit to m/s^2

                    # os.system('cls' if os.name == 'nt' else 'clear')
                    # print(f.renderText('LOCAL'))

                    # print('/////////////esbimu_driver/////////////\n\n')
                    # print(f'Quat : z:{self.imu_msg.orientation.x:.3f} | y:{self.imu_msg.orientation.y:.3f} | x:{self.imu_msg.orientation.z:.3f} | w:{self.imu_msg.orientation.w:.3f}',end='\n\n')
                    # from tf.transformations import euler_from_quaternion

                    # e = euler_from_quaternion([self.imu_msg.orientation.w,self.imu_msg.orientation.x,self.imu_msg.orientation.y,self.imu_msg.orientation.z])
                    # r = np.rad2deg(e[0])
                    # p = np.rad2deg(e[1])
                    # y = np.rad2deg(e[2])

                    # print(f'{r=}  {p=} {y=}')
                    
                    print(f'Accel: x:{self.imu_msg.linear_acceleration.x:.3f} | y:{self.imu_msg.linear_acceleration.y:.3f} | z:{self.imu_msg.linear_acceleration.z:.3f} m/s^2',end='\n\n')
                    print(f'Gyro: x:{self.imu_msg.angular_velocity.x:.3f} | y:{self.imu_msg.angular_velocity.y:.3f} | z:{self.imu_msg.angular_velocity.z:.3f}deg/s',end='\n\n')
                    print(f'imu_temp : {temp}C | battery : {battery}%',end='\n\n')


            except serial.SerialException:
                self.lg.logwarn(f'Unable to connect to serial port:{port}, baud:{baud}, retrying...')
                time.sleep(1.0)        

            

def main(args=None):
    rclpy.init(args=args)

    imu_node = ImuNode()

    rclpy.spin(imu_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
