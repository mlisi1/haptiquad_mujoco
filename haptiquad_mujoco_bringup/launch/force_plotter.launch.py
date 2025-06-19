from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():

    force_plotter = Node(
        package='haptiquad_plot',
        executable='force_plotter.py',
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


    return LaunchDescription(
        [
          force_plotter
        ]
    )