import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import TimerAction
from launch.actions import ExecuteProcess
def generate_launch_description():
    pkg_name = 'eval_test'
    tag_launch_dir = get_package_share_directory('apriltag_ros')
    pkg_dec = get_package_share_directory(pkg_name)
    world_file = os.path.join(get_package_share_directory('eval_test'), 'world', 'warehouse.world')
    ns = "robot_1"
    ns_2 = "robot_2"
    image_topic = LaunchConfiguration('image_topic', default="image_raw")
    camera_name = LaunchConfiguration('camera_name', default="/robot_1/depthcam")
    no_bots = 1

    xacro_file = os.path.join(pkg_dec, 'urdf', 'turtlebot_burger.urdf.xacro')
    nav2_params_file = os.path.join(get_package_share_directory('apriltag_docking'), 'param', 'nav2_params.yaml')
    doc_params = os.path.join(tag_launch_dir, 'cfg', 'tags_36h11.yaml')
    map_file = os.path.abspath(os.path.join(get_package_share_directory('apriltag_docking'), 'maps', 'map_warehouse.yaml'))

    # if not os.path.exists(map_file):
    #     raise Exception(f"Map file not found: {map_file}")
    actions = []



    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', "gazebo.launch.py")]),
        launch_arguments={'world':world_file}.items()
    )
    actions.append(gazebo)

    for i in range(1,no_bots+1):
        ns = f"robot_{i}"
        doc_robo = xacro.process_file(xacro_file,  mappings={"ns": f"{ns}"}).toxml()
        params = {'robot_description': doc_robo, 'use_sim_time': True}

        actions.append(Node(package='robot_state_publisher',
                                    executable='robot_state_publisher',
                                    namespace=f"{ns}",
                                    output='both',
                                    parameters=[params],
                                    # remappings=[
                                    #     ('/tf', f'/{ns}/tf'),
                                    #     ('/tf_static', f'/{ns}/tf_static'),
                                    # ]
                                    ))
        actions.append(Node(
                        package='gazebo_ros',
                        executable='spawn_entity.py',
                        arguments=['-topic', f'{ns}/robot_description', '-entity', f'{ns}_tortoise_bot', 
                                    "-x", str(int(i) * 1.5),  # spread them out
                                    "-y", "0.0",
                                    "-z", "0.1"],
                        output = 'screen'
                    ))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(pkg_name), 'launch', 'localization_launch.py')),
            launch_arguments={
                'namespace': ns,
                # 'x_val': 'int(i) * 1.5',
                'map': map_file,#'/home/adithya-si3510/ws_bot/install/multiple_bot/map/map.yaml',
                'params_file': nav2_params_file,
                'use_sim_time': 'true',
            }.items()
        ))
        #navigation
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(pkg_name), 'launch', 'navigation_launch.py')),
            launch_arguments={
                'namespace': ns,
                'use_sim_time': 'true',
                'params_file': nav2_params_file
            }.items()
        ))

    actions.append(Node(
        package= 'apriltag_ros',
        executable= "apriltag_node",
        output = "both",
        remappings=[('image_rect', f'/{camera_name}{image_topic}'),
                    ('camera_info', f'/{camera_name}/camera_info')],
        parameters=[doc_params]
                

    ))
    # actions.append(IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(os.path.join(tag_launch_dir,'tag_gazebo.launch.py')),
    #         launch_arguments={'image_topic': image_topic,
    #                           'camera_name': camera_name}.items()))
    return LaunchDescription(actions)