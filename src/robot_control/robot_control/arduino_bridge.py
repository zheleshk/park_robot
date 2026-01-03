#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial
import math
import time

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        # --- НАСТРОЙКИ РОБОТА (ИЗМЕНИТЬ ПОД СЕБЯ!) ---
        self.WHEEL_RADIUS = 0.08 / 2.0  # Радиус колеса в метрах (если диаметр 80мм, то радиус 0.04)
        self.WHEEL_SEP_WIDTH = 0.27     # Расстояние между левыми и правыми колесами (метры)
        self.WHEEL_SEP_LENGTH = 0.26    # Расстояние между передней и задней осью (метры)
        self.TICKS_PER_REV = 1024       # Сколько тиков энкодера на 1 полный оборот колеса
        
        # Настройки Serial
        self.port = '/dev/arduino'
        self.baudrate = 115200

        # --- Инициализация переменных одометрии ---
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()
        
        # Храним предыдущие значения энкодеров
        self.prev_ticks = [0, 0, 0, 0] 
        self.first_run = True

        # --- ROS Коммуникации ---
        # Подписка на команды скорости
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        # Публикация одометрии
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # Публикация TF (трансформации координат)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Подключение к Arduino
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.get_logger().info(f'Connected to Arduino on {self.port}')
            time.sleep(2) # Ждем перезагрузки Arduino
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Arduino: {e}')
            return

        # Таймер для чтения данных с Arduino (50 Гц)
        self.create_timer(0.02, self.update_odometry)

    def cmd_vel_callback(self, msg):
        """Отправка команд скорости на Arduino"""
        lx = msg.linear.x
        ly = msg.linear.y
        az = msg.angular.z
        
        # Формат команды: <v,lx,ly,az>
        command = f"<v,{lx:.3f},{ly:.3f},{az:.3f}>"
        if self.ser.is_open:
            try:
                self.ser.write(command.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f'Serial write error: {e}')

    def update_odometry(self):
        """Чтение энкодеров и расчет положения"""
        if not self.ser.is_open: return

        try:
            raw_data = self.ser.readline()
            line = raw_data.decode('utf-8', errors='replace').strip()
            if not line.startswith('<o,') or not line.endswith('>'):
                return 
            data = line[3:-1].split(',')
            if len(data) != 4: return
            ticks = [int(x) for x in data]

            if self.first_run:
                self.prev_ticks = ticks
                self.first_run = False
                return
               
           
            d_ticks = [curr - prev for curr, prev in zip(ticks, self.prev_ticks)]
            self.prev_ticks = ticks

                # --- КИНЕМАТИКА MECANUM (ПРЯМАЯ) ---
                # 1. Переводим тики в метры для каждого колеса
            meters_per_tick = (2 * math.pi * self.WHEEL_RADIUS) / self.TICKS_PER_REV
            d_wheels = [dt * meters_per_tick for dt in d_ticks]
                
                # Порядок моторов: 1=FL, 2=FR, 3=BL, 4=BR
                # Формулы для Mecanum:
                # dx = (fl + fr + bl + br) / 4
                # dy = (-fl + fr + bl - br) / 4
                # dth = (-fl + fr - bl + br) / (4 * (L + W))
                
            d1, d2, d3, d4 = d_wheels
                
            geom_factor = self.WHEEL_SEP_WIDTH/2 + self.WHEEL_SEP_LENGTH/2

            dx = (d1 + d2 + d3 + d4) / 4.0
            dy = (-d1 + d2 + d3 - d4) / 4.0
            dth = (-d1 + d2 - d3 + d4) / (4.0 * geom_factor)

                # --- ИНТЕГРАЦИЯ ОДОМЕТРИИ ---
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.last_time = current_time

                # Поворот вектора движения на текущий угол робота
            delta_x = (dx * math.cos(self.th) - dy * math.sin(self.th))
            delta_y = (dx * math.sin(self.th) + dy * math.cos(self.th))
                
            self.x += delta_x
            self.y += delta_y
            self.th += dth

                # --- ПУБЛИКАЦИЯ ---
            self.publish_odom_and_tf(current_time, dx/dt, dy/dt, dth/dt)

        except Exception as e:
            # self.get_logger().warn(f'Read error: {e}') # Можно раскомментировать для отладки
            pass

    def publish_odom_and_tf(self, current_time, vx, vy, vth):
        # Кватернион из угла yaw (th)
        q = self.euler_to_quaternion(0, 0, self.th)

        # 1. Публикация TF (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        # 2. Публикация Odometry сообщения
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Позиция
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Скорость (в системе координат робота)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth

        self.odom_pub.publish(odom)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Вспомогательная функция для перевода углов в кватернионы"""
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
