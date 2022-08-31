import socket
import threading
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float64

from scipy.spatial.transform import Rotation
import numpy as np



IPADDR = "127.0.0.1"
PORT = 49152

sock = socket.socket(socket.AF_INET)
sock.connect((IPADDR, PORT))



class MinimalSubscriber(Node):

    Gyro_Rate = 0


    def __init__(self):
        super().__init__('SocketReceiveAndSend')
        #self.subscription = self.create_subscription(String, 'To42', self.listener_callback, 10)
        self.subscription = self.create_subscription(Float64, 'T_rw_command', self.listener_callback, 10)        
        self.subscription  # prevent unused variable warning

        #self.publisher_ST = self.create_publisher(Float64, 'ST_From42', 10)
        #self.publisher_Gyro = self.create_publisher(Float64, 'Gyro_From42', 10)
        self.publisher_ST = self.create_publisher(Float64, 'theta_raw', 10)
        self.publisher_Gyro = self.create_publisher(Float64, 'omega_raw', 10)
        
        #self.omega_raw_publish_timer = self.create_timer(0.01, self.callback_publish_omega_raw)
        #self.theta_raw_publish_timer = self.create_timer(1.0, self.callback_publish_theta_raw)


    def listener_callback(self, msg):
        self.get_logger().info('Subscribed: "%s"' % msg.data)
        
        data = msg.data
        data_send = '     SC[0].AC.Whl[0].Tcmd = {}'.format(data)

        #TorqueValue = "-12.3"
        #data_send = '     SC[0].AC.Whl[0].Tcmd = {}'.format(TorqueValue)
        #print(data_send)

        sock.send(data_send.encode("utf-8"))

        self.get_logger().info('Send Through Socket: "%s"' % data_send)


    '''
    def callback_publish_omega_raw(self):
        msg_Gyro = Float64()
        msg_Gyro.data = float(Gyro_Rate)
    
        self.publisher_Gyro.publish(msg_Gyro)        
        self.get_logger().info("omega_raw: '%f'", % msg_Gyro.data)


    def callback_publish_theta_raw(self):
        msg_ST = Float64()
        msg_ST.data = float(Gyro_Rate)
    
        self.publisher_Gyro.publish(msg_Gyro)        
        self.get_logger().info("omega_raw: '%f'", % msg_Gyro.data)
    '''

    # データ受信関数

    def recv_data(self, sock):
        while True:
            try:
                #data = sock.recv(32768)
                data = sock.recv(1024)
                data_str = data.decode("utf-8")

                if data == b"":
                    break

                print("ssssssssssssssssssssssssssssssssssssssssssssssssss")
                print("a_" + data_str + "_a")

                if data_str != "Ack\n":

                    print("vvv")

                    parameters = data_str.split("\n")
                    ST_str = parameters[1]
                    ST_x = float(ST_str.split(" ")[2])
                    ST_y = float(ST_str.split(" ")[3])
                    ST_z = float(ST_str.split(" ")[4])
                    ST_w = float(ST_str.split(" ")[5])

                    # クォータニオンをオイラー角度へ
                    quat = np.array([ST_x, ST_y, ST_z, ST_w])
                    rot = Rotation.from_quat(quat)
                    euler = rot.as_euler('XYX')



                    Gyro_str = parameters[2]
                    Gyro_Rate = Gyro_str.split(" ")[2]
                    #print(Gyro_str)
                    #print(Gyro_Rate)

                    msg_ST = Float64()
                    msg_ST.data = quat[0]
                    msg_ST.data = euler[0] / np.pi * 180 + 180
                    
                    msg_Gyro = Float64()
                    msg_Gyro.data = float(Gyro_Rate) / np.pi * 180

                    self.publisher_ST.publish(msg_ST)
                    self.publisher_Gyro.publish(msg_Gyro)

                    #minimal_subscriber.publisher_ST.publish(msg_ST)
                    #minimal_subscriber.publisher_Gyro.publish(msg_Gyro)

                    #minimal_subscriber.get_logger().info('Publishing ST : "%s"' % msg_ST.data)
                    #minimal_subscriber.get_logger().info('Publishing Gyro : "%s"' % msg_Gyro.data)


                print("eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee")


            except ConnectionResetError:
                break
                

        sock.shutdown(socket.SHUT_RDWR)
        sock.close()



def main(args=None):





    
    # データ受信関数
    '''
    def recv_data(sock):
        while True:
            try:
                #data = sock.recv(32768)
                data = sock.recv(1024)
                data_str = data.decode("utf-8")

                if data == b"":
                    break

                print("ssssssssssssssssssssssssssssssssssssssssssssssssss")
                print("a_" + data_str + "_a")

                if data_str != "Ack\n":

                    print("vvv")

                    parameters = data_str.split("\n")
                    ST_str = parameters[1]
                    ST_x = ST_str.split(" ")[2]
                    ST_y = ST_str.split(" ")[3]
                    ST_z = ST_str.split(" ")[4]
                    ST_w = ST_str.split(" ")[5]
                    #print(ST_str)
                    #print(ST_x, ST_y, ST_z, ST_w)

                    Gyro_str = parameters[2]
                    Gyro_Rate = Gyro_str.split(" ")[2]
                    #print(Gyro_str)
                    #print(Gyro_Rate)

                    msg_ST = String()
                    msg_ST.data = ST_x

                    msg_Gyro = String()
                    msg_Gyro.data = Gyro_Rate

                    publisher_ST.publish(msg_ST)

                    #minimal_subscriber.publisher_ST.publish(msg_ST)
                    #minimal_subscriber.publisher_Gyro.publish(msg_Gyro)

                    #minimal_subscriber.get_logger().info('Publishing ST : "%s"' % msg_ST.data)
                    #minimal_subscriber.get_logger().info('Publishing Gyro : "%s"' % msg_Gyro.data)


                print("eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee")


            except ConnectionResetError:
                break
                

        sock.shutdown(socket.SHUT_RDWR)
        sock.close()
    '''
    
    
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    
    
    # データ受信をサブスレッドで実行
    thread = threading.Thread(target=minimal_subscriber.recv_data, args=(sock,))
    thread.start()



    rclpy.spin(minimal_subscriber)
    

    # データ入力ループ

    #i = 0
    #while True:



    #while True:
        #data = input("> ")
        #minimal_subscriber = MinimalSubscriber()

        # データ受信をサブスレッドで実行
        #thread = threading.Thread(target=recv_data, args=(sock,))
        #thread.start()
        

    sock.shutdown(socket.SHUT_RDWR)
    sock.close()

    #rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


'''
# データ受信関数
def recv_data(sock):
    while True:
        try:
            #data = sock.recv(32768)
            data = sock.recv(1024)

            if data == b"":
                break
            print(data.decode("utf-8"))
        except ConnectionResetError:
            break

    sock.shutdown(socket.SHUT_RDWR)
    sock.close()

# データ受信をサブスレッドで実行
thread = threading.Thread(target=recv_data, args=(sock,))
thread.start()



# データ入力ループ

i = 0
#while True:
while True:
    data = input("> ")

    if data == "exit":
        break
    else:
        try:
            sock.send(data.encode("utf-8"))
        except ConnectionResetError:
            break

sock.shutdown(socket.SHUT_RDWR)
sock.close()
'''