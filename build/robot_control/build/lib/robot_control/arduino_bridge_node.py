#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial
import threading
import time
import math

# Вспомогательная функция для преобразования углов Эйлера в кватернион
def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    q = [0.0] * 4
    q[0] = sr * cp * cy - cr * sp * sy  # x
    q[1] = cr * sp * cy + sr * cp * sy  # y
    q[2] = cr * cp * sy - sr * sp * cy  # z
    q[3] = cr * cp * cy + sr * sp * sy  # w
    return q

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge_node')
        
        # --- Параметры ноды и робота ---
        self.declare_parameter('port', '/dev/arduino')
        self.declare_parameter('baudrate', 115200)
        
        # !!! ВАЖНО: Укажите ваши точные значения !!!
        self.declare_parameter('wheel_radius', 0.08) # Радиус колеса в метрах
        self.declare_parameter('wheel_base_width', 0.135) # Расстояние W в метрах
        self.declare_parameter('wheel_base_length', 0.13) # Расстояние L в метрах
        self.declare_parameter('ticks_per_revolution', 1024) # Количество тиков энкодера на оборот
        
        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        
        self.WHEEL_RADIUS = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.WHEEL_BASE_WIDTH = self.get_parameter('wheel_base_width').get_parameter_value().double_value
        self.WHEEL_BASE_LENGTH = self.get_parameter('wheel_base_length').get_parameter_value().double_value
        self.TICKS_PER_REVOLUTION = self.get_parameter('ticks_per_revolution').get_parameter_value().integer_value
        
        self.get_logger().info(f"Connecting to port {port} at {baudrate} baud...")

        # --- Подключение к Serial порту ---
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=1.0)
            time.sleep(2)
            self.get_logger().info('Successfully connected to Arduino.')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Arduino: {e}')
            rclpy.shutdown()
            return

        # --- ROS-интерфейсы ---
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Переменные для одометрии ---
        self.last_ticks = [0, 0, 0, 0]
        self.last_time = self.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # --- Поток для чтения ---
        self.read_thread = threading.Thread(target=self.read_from_arduino)
        self.read_thread.daemon = True
        self.read_thread.start()

    def cmd_vel_callback(self, msg):
        # ... (эта функция остается без изменений) ...
        lx = msg.linear.x
        ly = msg.linear.y
        wz = msg.angular.z
        command = f"<v,{lx:.2f},{ly:.2f},{wz:.2f}>"
        try:
            self.serial_port.write(command.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f'Failed to send command to Arduino: {e}')

    def read_from_arduino(self):
        """Читает данные от Arduino и вызывает обработчик одометрии."""
        while rclpy.ok():
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                if line.startswith('<o,') and line.endswith('>'):
                    data = line[3:-1]
                    ticks_str = data.split(',')
                    if len(ticks_str) == 4:
                        current_ticks = [int(t) for t in ticks_str]
                        # Вызываем функцию расчета и публикации одометрии
                        self.publish_odometry(current_ticks)
            except rclpy.executors.ExternalShutdownException:
                break
            except Exception as e:
                self.get_logger().warn(f'Error reading or parsing from Arduino: {e}')

    def publish_odometry(self, current_ticks):
        """Рассчитывает и публикует одометрию на основе тиков энкодеров."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt == 0: return

        # Рассчитываем изменение количества тиков
        # Нумерация: 0-FL, 1-FR, 2-BL, 3-BR
        delta_ticks = [current_ticks[i] - self.last_ticks[i] for i in range(4)]
        self.last_ticks = current_ticks

        # Преобразуем тики в радианы
        ticks_to_rad = (2 * math.pi) / self.TICKS_PER_REVOLUTION
        
        # Угловое смещение каждого колеса в радианах
        w1_delta_rad = delta_ticks[0] * ticks_to_rad # FL
        w2_delta_rad = delta_ticks[1] * ticks_to_rad # FR
        w4_delta_rad = delta_ticks[2] * ticks_to_rad # BL (Arduino шлет его третьим)
        w3_delta_rad = delta_ticks[3] * ticks_to_rad # BR (Arduino шлет его четвертым)

        # Угловая скорость каждого колеса (рад/с)
        w1 = w1_delta_rad / dt
        w2 = w2_delta_rad / dt
        w3 = w3_delta_rad / dt
        w4 = w4_delta_rad / dt

        # --- Прямая кинематика ---
        L_plus_W = self.WHEEL_BASE_LENGTH + self.WHEEL_BASE_WIDTH
        
        vx = (self.WHEEL_RADIUS / 4) * (-w1 + w2 + w3 - w4) # Ошибка в предыдущей формуле, здесь правильная
        vy = (self.WHEEL_RADIUS / 4) * ( w1 + w2 + w3 + w4)
        wz = (self.WHEEL_RADIUS / (4 * L_plus_W)) * (-w1 + w2 - w3 + w4)

        # --- Интегрирование для получения позиции ---
        delta_x_robot = vx * dt
        delta_y_robot = vy * dt
        delta_theta = wz * dt

        self.x += delta_x_robot * math.cos(self.theta) - delta_y_robot * math.sin(self.theta)
        self.y += delta_x_robot * math.sin(self.theta) + delta_y_robot * math.cos(self.theta)
        self.theta += delta_theta

        # --- Публикация сообщения Odometry ---
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        q = euler_to_quaternion(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = wz
        
        self.odom_pub.publish(odom_msg)

        # --- Публикация трансформации (TF) odom -> base_link ---
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = odom_msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

        self.last_time = current_time

    def destroy_node(self):
        """Корректно закрываем порт при выключении ноды."""
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info('Serial port closed.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    #finally:
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
