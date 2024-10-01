import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from vesc_msgs.msg import ServoCommand

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        
        # LiDAR 데이터 구독
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',  # LiDAR 데이터 토픽
            self.scan_callback,
            10
        )
        
        # VESC 조향 명령 발행
        self.steer_publisher = self.create_publisher(
            ServoCommand,
            '/vesc/steering',
            10
        )

        # 속도는 0으로 고정
        self.speed_publisher = self.create_publisher(
            ServoCommand,
            '/vesc/speed',
            10
        )
        self.set_speed(0.0)

        # 조향 각도 초기값
        self.current_steering_angle = 0.0
    
    def scan_callback(self, msg: LaserScan):
        # LiDAR 데이터 처리
        # 범위를 180도씩 양쪽으로 나누어 장애물 확인
        left_range = min(msg.ranges[:len(msg.ranges)//2])
        right_range = min(msg.ranges[len(msg.ranges)//2:])
        
        # 조향 방향 결정
        if left_range < 1.0 and right_range < 1.0:
            # 양쪽 모두 장애물이 있을 경우, 조향 중지 (중립)
            self.set_steering(0.0)
        elif left_range < 1.0:
            # 왼쪽에 장애물이 있을 경우 오른쪽으로 조향
            self.set_steering(1.0)
        elif right_range < 1.0:
            # 오른쪽에 장애물이 있을 경우 왼쪽으로 조향
            self.set_steering(-1.0)
        else:
            # 장애물이 없을 경우 직진 방향 (중립)
            self.set_steering(0.0)

    def set_steering(self, angle):
        # VESC에 조향 명령 발행
        steering_msg = ServoCommand()
        steering_msg.data = angle  # -1.0 (좌회전) ~ 1.0 (우회전)
        self.steer_publisher.publish(steering_msg)
    
    def set_speed(self, speed):
        # VESC에 속도 명령 발행 (속도는 0으로 고정)
        speed_msg = ServoCommand()
        speed_msg.data = speed  # 속도 설정 (여기서는 0)
        self.speed_publisher.publish(speed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
