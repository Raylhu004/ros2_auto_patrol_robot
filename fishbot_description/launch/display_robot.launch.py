import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_package_path =get_package_share_directory('fishbot_description')
    default_urdf_path=os.path.join(urdf_package_path,'urdf','first_robot.urdf')
    default_rviz_path=os.path.join(urdf_package_path,'config','displayrobotmodel.rviz')
    #声明一个urdf目录的参数
    action_declare_arg_mode_path=launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(default_urdf_path),
        description='加载的模型文件路径'
    )
    #通过文件路径，获取内容，并转换成参数值对象，以供传入robot_state_publisher
    #cmd_result=launch.substitutions.Command(['cat ',launch.substitutions.LaunchConfiguration('model')]) #把cat的文件内容以字符串格式传给cmd_result
    cmd_result=launch.substitutions.Command(['xacro ',launch.substitutions.LaunchConfiguration('model')])
    robot_description_value=launch_ros.parameter_descriptions.ParameterValue(cmd_result,value_type=str) #把cmd_result的字符串内容包成一个字符串参数，字符串参数被robot_state_publisher节点的参数robot_description调用

    action_robot_state_publisher=launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':robot_description_value}]

    )

    action_joint_state_publisher=launch_ros.actions.Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
        )

    action_rviz_node = launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', default_rviz_path]
        )

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_robot_state_publisher,
        action_joint_state_publisher,
        action_rviz_node
        ])
