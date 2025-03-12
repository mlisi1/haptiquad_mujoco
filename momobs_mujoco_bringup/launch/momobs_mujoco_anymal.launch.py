import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro


def generate_launch_description():

    force_arg = DeclareLaunchArgument('force', default_value='false', description='Launch extra nodes if true')
    residual_arg = DeclareLaunchArgument('residuals', default_value='false', description='Launch extra nodes if true')


    description_pkg = get_package_share_directory('anymal_c_simple_description')
    momobs_ros_pkg = get_package_share_directory('momobs_ros2')
    mujoco_pkg = get_package_share_directory('mujoco')
    config_pkg_share = get_package_share_directory('anymal_c_config')
    self_pkg = get_package_share_directory('momobs_mujoco_bringup')


    description_launch_file = os.path.join(description_pkg, 'launch', 'floating_base_description.launch.py')   
    momobs_launch_file = os.path.join(momobs_ros_pkg, 'launch', 'mujoco_wrapper.launch.py')
    mujoco_launch_file = os.path.join(mujoco_pkg, 'launch', 'anymal_simulation.launch.py')

    default_model_path = os.path.join(description_pkg, "urdf/anymal_main.xacro")
    xacro_content = xacro.process_file(default_model_path)

    momobs_config = os.path.join(self_pkg, 'config', 'momobs_anymal.yaml')

    momobs = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(momobs_launch_file),
            launch_arguments={'config_file': momobs_config}.items()
    ) 
    description = IncludeLaunchDescription(PythonLaunchDescriptionSource(description_launch_file))
    mujoco = IncludeLaunchDescription(PythonLaunchDescriptionSource(mujoco_launch_file))

    joints_config = os.path.join(config_pkg_share, "config/joints/joints.yaml")
    gait_config = os.path.join(config_pkg_share, "config/gait/gait.yaml")
    links_config = os.path.join(config_pkg_share, "config/links/links.yaml")


    force_plotter = Node(
        package='momobs_plot',
        executable='force_plotter.py',
        condition=IfCondition(LaunchConfiguration('force')),
        emulate_tty=True,
        parameters=[{
            'autoscale':True,
            'listening':True,
            'x_lim': 5.0,
            'memory_limit': 1000,
            'legs_prefix': ["LF", "LH", "RF", "RH"],
            'foot_suffix': 'FOOT'
        }]
    )

    residual_plotter = Node(
        package='momobs_plot',
        executable='residual_plotter.py',
        condition=IfCondition(LaunchConfiguration('residuals')),
        emulate_tty = True,
        parameters=[{
            'autoscale':True,
            'listening':True,
            'x_lim':5.0,
            'memory_limit': 1000,
            'legs_prefix': ["LF", "LH", "RF", "RH"],
        }]
    )

    quadruped_controller_node = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            {"gazebo": False},
            {"publish_joint_states": False},
            {"publish_joint_control": True},
            {"publish_foot_contacts": True},
            {"joint_controller_topic": '/simulation/joint_trajectory'},
            {"loop_rate": 500.0},
            {"urdf": xacro_content.toxml()},
            {joints_config},
            {links_config},
            {gait_config}
        ],
        remappings=[("/cmd_vel/smooth", "/cmd_vel"), ("/joint_states", "/simulation/joint_states")],
    )




    return LaunchDescription(
        [
            force_arg,
            residual_arg,
            description,
            momobs,
            force_plotter,
            residual_plotter,
            mujoco,
            quadruped_controller_node
        ]
    )