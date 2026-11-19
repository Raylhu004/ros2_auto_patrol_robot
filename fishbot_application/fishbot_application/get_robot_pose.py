import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer # TF 监听器 + 缓存
from tf_transformations import euler_from_quaternion # 四元数转换为欧拉角


class TfListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.buffer_, self)
        self.timer_ = self.create_timer(1, self.listen_transform)

    def listen_transform(self):
        """查询机器人在地图上的位姿: map→base_footprint"""
        try:
            # 超时用 Duration()（不等待）：TransformListener 和定时器共用一个线程，
            # 带超时会在这里忙等、堵住 /tf 回调，可能一直收不到 map→odom（11-18 实测）
            result = self.buffer_.lookup_transform('map', 'base_footprint', rclpy.time.Time(seconds=0), rclpy.duration.Duration())
            transform = result.transform
            self.get_logger().info(f'平移：{transform.translation}')
            self.get_logger().info(f'旋转：{transform.rotation}')
            rotation_euler = euler_from_quaternion([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w])
            self.get_logger().info(f'欧拉角：{rotation_euler}')
            

        except Exception as e:
            self.get_logger().error(f'监听动态TF失败：,原因{str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = TfListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
            pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

