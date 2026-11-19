from geometry_msgs.msg import PoseStamped, Pose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer  # TF 监听器 + 缓存
from tf_transformations import euler_from_quaternion, quaternion_from_euler  # 四元数转
from autopatrol_interfaces.srv import SpeechText
from sensor_msgs.msg import Image #图像格式接口
from cv_bridge import CvBridge # 转换图像格式
import cv2 # 保存图像
import os

class PatrolNode(BasicNavigator):
    def __init__(self, node_name='patrol_node'):
        super().__init__(node_name)
        self.buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.buffer_, self)
        self.speech_client_ = self.create_client(SpeechText, 'speech_text')
        self.cv_bridge_= CvBridge()
        self.latest_img_=None
        self.img_sub_=self.create_subscription(Image,'/camera_sensor/image_raw',self.img_callback,11)
        # 声明参数
        self.declare_parameter('initial_point', [0.0, 0.0, 0.0])
        self.declare_parameter('target_points', [0.0, 0.0, 0.0, 1.0, 1.0, 1.57])
        self.declare_parameter('img_save_path','')

        self.initial_point_ = self.get_parameter('initial_point').value
        self.target_points_ = self.get_parameter('target_points').value
        self.img_save_path_ = self.get_parameter('img_save_path').value

    def img_callback(self,msg):
        self.latest_img_=msg

    def record_img(self):
        if self.latest_img_ is None:
            self.get_logger().warn('还没收到相机图像，跳过拍照')
            return
        pose = self.get_current_pose()
        # 相机格式是 R8G8B8（rgb8），而 cv2.imwrite 按 BGR 解释数组，所以转换时指定 bgr8
        cv_image = self.cv_bridge_.imgmsg_to_cv2(self.latest_img_, 'bgr8')
        # os.path.join 不在乎路径结尾有没有 '/'；路径为空时存到当前工作目录
        save_dir = self.img_save_path_ or os.getcwd()
        os.makedirs(save_dir, exist_ok=True)
        file_name = os.path.join(
            save_dir, f'img_{pose.translation.x:3.2f}_{pose.translation.y:3.2f}.png')
        # 目录不存在等情况下 imwrite 不抛异常，只返回 False，所以要检查返回值
        if cv2.imwrite(file_name, cv_image):
            self.get_logger().info(f'照片已保存：{file_name}')
        else:
            self.get_logger().error(f'照片保存失败：{file_name}')

    def get_pose_by_xyyaw(self, x, y, yaw):
        """
        return PoseStamped对象
        """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        quat = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        return pose

    def init_robot_pose(self):
        """
        初始化机器人位姿
        """
        self.initial_point_ = self.get_parameter('initial_point').value
        init_pose = self.get_pose_by_xyyaw(self.initial_point_[0], self.initial_point_[
            1], self.initial_point_[2])
        self.setInitialPose(init_pose)
        self.waitUntilNav2Active()

    def get_target_points(self):
        """
        通过参数值获取目标点的集合
        """
        points = []
        self.target_points_ = self.get_parameter('target_points').value
        for index in range(int(len(self.target_points_)/3)):
            x = self.target_points_[index*3]
            y = self.target_points_[index*3+1]
            yaw = self.target_points_[index*3+2]
            points.append([x, y, yaw])
            self.get_logger().info(f"获取到目标点{index}->{x},{y},{yaw}")

        return points

    def nav_to_pose(self, target_point):
        """
        导航到目标点
        """
        self.goToPose(target_point)
        while not self.isTaskComplete():
            feedback = self.getFeedback()
            # 还没收到第一条反馈时 getFeedback() 返回 None
            if feedback:
                self.get_logger().info(f'剩余距离：{feedback.distance_remaining}')
        # nav.cancelTask()
        result = self.getResult()
        self.get_logger().info(f'导航结果：{result}')
        return result

    def get_current_pose(self):
        """
        获取机器人的当前位姿
        """
        while rclpy.ok():
            try:
                # 超时用 Duration()（不等待）：TransformListener 和定时器共用一个线程，
                # 带超时会在这里忙等、堵住 /tf 回调，可能一直收不到 map→odom（11-18 实测）
                result = self.buffer_.lookup_transform(
                    'map', 'base_footprint', rclpy.time.Time(seconds=0), rclpy.duration.Duration())
                transform = result.transform
                self.get_logger().info(f'平移：{transform.translation}')
                self.get_logger().info(f'旋转：{transform.rotation}')
                rotation_euler = euler_from_quaternion([
                    transform.rotation.x, transform.rotation.y,
                    transform.rotation.z, transform.rotation.w])
                self.get_logger().info(f'欧拉角：{rotation_euler}')

                return transform
            except Exception as e:
                self.get_logger().error(f'监听动态TF失败:,原因{str(e)}')
                # 不 spin 的话 TF 回调永远不会执行，缓存不会更新，会一直失败下去
                rclpy.spin_once(self, timeout_sec=0.1)

    def speech_text(self, text):
        """
        调用服务合成语音
        """
        while not self.speech_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('语音服务未上线，等待中')

        request = SpeechText.Request()
        request.text = text
        future = self.speech_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            if response.result == True:
                self.get_logger().info(f'语音合成成功{text}')
            else:
                self.get_logger().warn(f'语音合成失败{text}')
        else:
            self.get_logger().warn(f'语音合成服务响应失败')


def main():
    rclpy.init()
    patrol = PatrolNode()
    patrol.speech_text('正在准备初始化位置')
    patrol.init_robot_pose()
    patrol.speech_text('初始化位置完成')

    while rclpy.ok():
        points = patrol.get_target_points()
        for point in points:
            x, y, yaw = point[0], point[1], point[2]
            target_pose = patrol.get_pose_by_xyyaw(x, y, yaw)
            patrol.speech_text(f'正在准备前往{x},{y}目标点')
            result = patrol.nav_to_pose(target_pose)
            if result != TaskResult.SUCCEEDED:
                patrol.speech_text(f'前往{x},{y}目标点失败，跳过拍照')
                continue
            patrol.speech_text(f'已经到达{x},{y}目标点，准备拍照')
            patrol.record_img()
            patrol.speech_text('拍照完成')

    rclpy.shutdown()
