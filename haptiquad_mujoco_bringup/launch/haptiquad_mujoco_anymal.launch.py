import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetRemap
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro


def generate_launch_description():

    force_arg = DeclareLaunchArgument('force', default_value='false', description='Launch extra nodes if true')
    residual_arg = DeclareLaunchArgument('residuals', default_value='false', description='Launch extra nodes if true')
    use_nvidia_arg = DeclareLaunchArgument('use_nvidia', default_value='false', description='Uses nVidia GPU for Mujoco rendering if true')
    estimate_contacts_arg = DeclareLaunchArgument('estimate_contacts', default_value='false', description='Wether to launch or not the contact estimation package')
    mass_scale = DeclareLaunchArgument('mass_scale', default_value='1.0', description="Mass scale factor")
    inertia_scale = DeclareLaunchArgument('inertia_scale', default_value='1.0', description="Inertia scale factor")
    debug = DeclareLaunchArgument('debug', default_value='false', description="Debug mode")

    use_nvidia = LaunchConfiguration("use_nvidia")
    estimate_contacts = LaunchConfiguration("estimate_contacts")

    gpu1 = SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1')
    gpu2 = SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia')

    gpu1 = SetEnvironmentVariable(
        '__NV_PRIME_RENDER_OFFLOAD', '1',
        condition=IfCondition(use_nvidia)
    )
    gpu2 = SetEnvironmentVariable(
        '__GLX_VENDOR_LIBRARY_NAME', 'nvidia',
        condition=IfCondition(use_nvidia)
    )


    description_pkg = get_package_share_directory('anymal_c_simple_description')
    haptiquad_ros_pkg = get_package_share_directory('haptiquad_ros2')
    mujoco_pkg = get_package_share_directory('mujoco')
    config_pkg_share = get_package_share_directory('anymal_c_config')
    self_pkg = get_package_share_directory('haptiquad_mujoco_bringup')
    haptiquad_contacts_pkg = get_package_share_directory('haptiquad_contacts')


    description_launch_file = os.path.join(description_pkg, 'launch', 'floating_base_description.launch.py')   
    haptiquad_launch_file = os.path.join(haptiquad_ros_pkg, 'launch', 'mujoco_wrapper.launch.py')
    mujoco_launch_file = os.path.join(mujoco_pkg, 'launch', 'anymal_simulation.launch.py')
    estimate_contacts_launch_file = os.path.join(haptiquad_contacts_pkg, 'launch', 'haptiquad_estimator_mujoco.launch.py')

    default_model_path = os.path.join(description_pkg, "urdf/anymal_main.xacro")
    xacro_content = xacro.process_file(default_model_path)

    haptiquad_config = os.path.join(self_pkg, 'config', 'haptiquad_anymal.yaml')
    contact_estimator_config = os.path.join(self_pkg, 'config', 'contact_estimator.yaml')

    haptiquad = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(haptiquad_launch_file),
            launch_arguments={
                'config_file': haptiquad_config,
                'mass_scale': LaunchConfiguration('mass_scale'),
                'inertia_scale': LaunchConfiguration('inertia_scale'),
                'debug': LaunchConfiguration('debug')
            }.items()
    ) 

    estimate_contacts_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(estimate_contacts_launch_file),
        launch_arguments={'rviz': str(True), "plot": str(True)}.items(),
        condition=IfCondition(estimate_contacts)
    )

    description_ = IncludeLaunchDescription(PythonLaunchDescriptionSource(description_launch_file))
    description = GroupAction([
        SetRemap('joint_states', 'simulation/joint_states'),
        description_
    ])

    mujoco = IncludeLaunchDescription(PythonLaunchDescriptionSource(mujoco_launch_file))

    joints_config = os.path.join(config_pkg_share, "config/joints/joints.yaml")
    gait_config = os.path.join(config_pkg_share, "config/gait/gait.yaml")
    links_config = os.path.join(config_pkg_share, "config/links/links.yaml")


    force_plotter = Node(
        package='haptiquad_plot',
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
        package='haptiquad_plot',
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
            use_nvidia_arg,
            debug,
            mass_scale,
            inertia_scale,
            gpu1,
            gpu2,
            force_arg,
            residual_arg,
            description,
            haptiquad,
            force_plotter,
            residual_plotter,
            mujoco,
            quadruped_controller_node,
            estimate_contacts_arg,     
            estimate_contacts_launch       
        ]
    )