import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    #获取功能包的share路径
    urdf_package_path =get_package_share_directory('fishbot_description')
    default_xacro_path=os.path.join(urdf_package_path,'urdf/fishbot','fishbot.urdf.xacro')
    default_gazebo_world_path=os.path.join(urdf_package_path,'world','custom_room.world')

    #声明一个urdf目录的参数
    action_declare_arg_mode_path=launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(default_xacro_path),
        description='加载的模型文件路径'
    )
    #声明一个world文件路径的参数
    action_declare_arg_world_path=launch.actions.DeclareLaunchArgument(
        name='world',
        default_value=str(default_gazebo_world_path),
        description='加载的world文件路径'
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

    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
           launch.launch_description_sources.PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'),'/launch','/gazebo.launch.py']),
           launch_arguments=[('world',launch.substitutions.LaunchConfiguration('world')),('verbose','true')]
        )

    #/spawn_entity 服务可能在 world 加载完之前就出现，直接生成会偶发超时失败，所以用 TimerAction 延时 5 秒
    #Node 单独存成变量，OnProcessExit 只能监听进程（Node/ExecuteProcess），不能监听 TimerAction
    action_spawn_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'fishbot']
    )
    action_spawn_entity = launch.actions.TimerAction(
        period=5.0,
        actions=[action_spawn_node]
    )



    action_load_joint_state_controller= launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'fishbot_joint_state_broadcaster','--set-state', 'active'],
        output='screen'
    )

    action_load_effort_controller= launch.actions.ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', 'fishbot_effort_controller','--set-state', 'active'],
            output='screen'
        )

    action_load_diff_drive_controller= launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'fishbot_diff_drive_controller','--set-state', 'active'],
        output='screen'
        )

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_declare_arg_world_path,
        action_robot_state_publisher,
        action_launch_gazebo,
        action_spawn_entity,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_spawn_node,
                on_exit=[action_load_joint_state_controller],
            )
        ),
        launch.actions.RegisterEventHandler(
                    event_handler=launch.event_handlers.OnProcessExit(
                        target_action=action_load_joint_state_controller,
                        on_exit=[ action_load_diff_drive_controller],
                    )
                )

        ])
